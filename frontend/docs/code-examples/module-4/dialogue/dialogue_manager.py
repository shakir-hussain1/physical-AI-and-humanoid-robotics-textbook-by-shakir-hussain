#!/usr/bin/env python3
"""
Dialogue Manager for Conversational Robot
Module 4: VLA Systems - Chapter 16

Manages multi-turn conversation state, context tracking,
and reference resolution for humanoid robots.
"""

from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any, Callable
from datetime import datetime
import json


class DialogueState(Enum):
    """States in the dialogue state machine."""
    IDLE = auto()
    LISTENING = auto()
    UNDERSTANDING = auto()
    CLARIFYING = auto()
    PLANNING = auto()
    CONFIRMING = auto()
    EXECUTING = auto()
    ERROR_RECOVERY = auto()
    GOODBYE = auto()


@dataclass
class EntityMention:
    """Tracking information for an entity mentioned in conversation."""
    name: str
    entity_type: str  # object, location, person, action
    attributes: Dict[str, Any] = field(default_factory=dict)
    first_mentioned: datetime = field(default_factory=datetime.now)
    last_mentioned: datetime = field(default_factory=datetime.now)
    mention_count: int = 1
    resolved_id: Optional[str] = None  # Physical object ID if grounded

    def update(self, attributes: Dict = None):
        """Update mention with new information."""
        self.last_mentioned = datetime.now()
        self.mention_count += 1
        if attributes:
            self.attributes.update(attributes)


@dataclass
class ConversationTurn:
    """Single turn in conversation."""
    speaker: str  # "user" or "robot"
    text: str
    intent: Optional[str] = None
    entities: List[str] = field(default_factory=list)
    timestamp: datetime = field(default_factory=datetime.now)

    def to_dict(self) -> Dict:
        return {
            "speaker": self.speaker,
            "text": self.text,
            "intent": self.intent,
            "entities": self.entities,
            "timestamp": self.timestamp.isoformat()
        }


class ConversationMemory:
    """Long-term memory for conversation context."""

    def __init__(self, max_turns: int = 50, max_entities: int = 100):
        self.max_turns = max_turns
        self.max_entities = max_entities
        self.turns: List[ConversationTurn] = []
        self.entities: Dict[str, EntityMention] = {}
        self.current_focus: Optional[str] = None
        self.task_stack: List[Dict] = []

    def add_turn(
        self,
        speaker: str,
        text: str,
        intent: str = None,
        entities: List[str] = None
    ):
        """Add a conversation turn."""
        turn = ConversationTurn(
            speaker=speaker,
            text=text,
            intent=intent,
            entities=entities or []
        )
        self.turns.append(turn)

        # Update entity mentions
        for entity in (entities or []):
            self.update_entity(entity)

        # Update focus
        if entities:
            self.current_focus = entities[-1]

        # Trim old turns
        if len(self.turns) > self.max_turns:
            self.turns = self.turns[-self.max_turns:]

    def update_entity(
        self,
        entity_name: str,
        entity_type: str = "object",
        attributes: Dict = None
    ):
        """Update or create entity tracking."""
        if entity_name in self.entities:
            self.entities[entity_name].update(attributes)
        else:
            self.entities[entity_name] = EntityMention(
                name=entity_name,
                entity_type=entity_type,
                attributes=attributes or {}
            )

        # Enforce entity limit
        if len(self.entities) > self.max_entities:
            # Remove oldest entities
            sorted_entities = sorted(
                self.entities.items(),
                key=lambda x: x[1].last_mentioned
            )
            for name, _ in sorted_entities[:len(self.entities) - self.max_entities]:
                del self.entities[name]

    def get_recent_entity(self, entity_type: str = None) -> Optional[str]:
        """Get most recently mentioned entity of given type."""
        candidates = list(self.entities.values())
        if entity_type:
            candidates = [e for e in candidates if e.entity_type == entity_type]

        if not candidates:
            return None

        return max(candidates, key=lambda e: e.last_mentioned).name

    def get_context_for_llm(self, num_turns: int = 5) -> str:
        """Generate context string for LLM."""
        recent = self.turns[-num_turns:]

        lines = ["Recent conversation:"]
        for turn in recent:
            lines.append(f"{turn.speaker}: {turn.text}")

        if self.current_focus:
            lines.append(f"\nCurrent focus: {self.current_focus}")

        if self.task_stack:
            lines.append(f"Active task: {self.task_stack[-1].get('description', 'unknown')}")

        return "\n".join(lines)

    def push_task(self, task: Dict):
        """Start a new task."""
        self.task_stack.append(task)

    def pop_task(self) -> Optional[Dict]:
        """Complete current task."""
        return self.task_stack.pop() if self.task_stack else None

    def clear(self):
        """Reset memory."""
        self.turns.clear()
        self.entities.clear()
        self.current_focus = None
        self.task_stack.clear()


class ReferenceResolver:
    """Resolve linguistic references to physical entities."""

    PRONOUNS = ["it", "them", "this", "that", "these", "those"]
    SPATIAL_WORDS = ["left", "right", "front", "back", "near", "far"]
    TEMPORAL_WORDS = ["previous", "last", "before", "earlier"]
    CONTRASTIVE_WORDS = ["other", "another", "different"]

    def __init__(self, memory: ConversationMemory):
        self.memory = memory
        self.scene: Dict = {"objects": []}

    def set_scene(self, scene: Dict):
        """Update current scene understanding."""
        self.scene = scene

    def resolve(self, reference: str) -> Optional[str]:
        """Resolve reference to entity name."""
        reference = reference.lower().strip()

        # Pronoun resolution
        if reference in self.PRONOUNS:
            return self._resolve_pronoun(reference)

        # Spatial resolution
        for word in self.SPATIAL_WORDS:
            if word in reference:
                return self._resolve_spatial(reference, word)

        # Temporal resolution
        for word in self.TEMPORAL_WORDS:
            if word in reference:
                return self._resolve_temporal()

        # Contrastive resolution
        for word in self.CONTRASTIVE_WORDS:
            if word in reference:
                return self._resolve_contrastive()

        # Definite description
        return self._resolve_definite(reference)

    def _resolve_pronoun(self, pronoun: str) -> Optional[str]:
        """Resolve pronouns like 'it', 'them'."""
        if self.memory.current_focus:
            return self.memory.current_focus
        return self.memory.get_recent_entity("object")

    def _resolve_spatial(self, reference: str, direction: str) -> Optional[str]:
        """Resolve spatial references."""
        objects = self.scene.get("objects", [])
        if not objects:
            return None

        # Sort by spatial dimension
        if direction == "left":
            sorted_objs = sorted(objects, key=lambda o: o.get("y", 0), reverse=True)
        elif direction == "right":
            sorted_objs = sorted(objects, key=lambda o: o.get("y", 0))
        elif direction in ["front", "near"]:
            sorted_objs = sorted(objects, key=lambda o: o.get("distance", float('inf')))
        elif direction in ["back", "far"]:
            sorted_objs = sorted(objects, key=lambda o: -o.get("distance", 0))
        else:
            return None

        return sorted_objs[0]["name"] if sorted_objs else None

    def _resolve_temporal(self) -> Optional[str]:
        """Resolve temporal references like 'the previous one'."""
        # Get entities in mention order
        mentioned = []
        for turn in reversed(self.memory.turns):
            mentioned.extend(turn.entities)

        # Remove duplicates preserving order
        seen = set()
        unique = []
        for entity in mentioned:
            if entity not in seen:
                seen.add(entity)
                unique.append(entity)

        # Return second most recent
        return unique[1] if len(unique) > 1 else None

    def _resolve_contrastive(self) -> Optional[str]:
        """Resolve 'the other one'."""
        current = self.memory.current_focus
        if not current:
            return None

        # Find same-type object
        current_type = self._get_type(current)
        for obj in self.scene.get("objects", []):
            if self._get_type(obj["name"]) == current_type and obj["name"] != current:
                return obj["name"]

        return None

    def _resolve_definite(self, reference: str) -> Optional[str]:
        """Resolve definite descriptions like 'the red ball'."""
        words = reference.replace("the", "").strip().split()

        for obj in self.scene.get("objects", []):
            obj_str = f"{obj.get('color', '')} {obj.get('type', '')} {obj['name']}".lower()
            if all(w in obj_str for w in words):
                return obj["name"]

        return None

    def _get_type(self, name: str) -> str:
        """Extract type from object name."""
        return name.split("_")[-1] if "_" in name else name


class DialogueStateMachine:
    """Manage dialogue state and transitions."""

    def __init__(self):
        self.state = DialogueState.IDLE
        self.history: List[Dict] = []
        self.context: Dict[str, Any] = {}

        self.transitions = {
            DialogueState.IDLE: {
                "user_speaks": DialogueState.LISTENING,
                "timeout": DialogueState.IDLE,
                "goodbye": DialogueState.GOODBYE
            },
            DialogueState.LISTENING: {
                "speech_complete": DialogueState.UNDERSTANDING,
                "silence": DialogueState.IDLE,
                "error": DialogueState.ERROR_RECOVERY
            },
            DialogueState.UNDERSTANDING: {
                "understood": DialogueState.PLANNING,
                "needs_clarification": DialogueState.CLARIFYING,
                "not_understood": DialogueState.ERROR_RECOVERY
            },
            DialogueState.CLARIFYING: {
                "clarification_received": DialogueState.UNDERSTANDING,
                "user_cancels": DialogueState.IDLE,
                "timeout": DialogueState.ERROR_RECOVERY
            },
            DialogueState.PLANNING: {
                "plan_ready": DialogueState.CONFIRMING,
                "simple_task": DialogueState.EXECUTING,
                "plan_failed": DialogueState.ERROR_RECOVERY
            },
            DialogueState.CONFIRMING: {
                "user_confirms": DialogueState.EXECUTING,
                "user_rejects": DialogueState.CLARIFYING,
                "user_modifies": DialogueState.PLANNING
            },
            DialogueState.EXECUTING: {
                "task_complete": DialogueState.IDLE,
                "task_failed": DialogueState.ERROR_RECOVERY,
                "user_interrupts": DialogueState.LISTENING
            },
            DialogueState.ERROR_RECOVERY: {
                "recovered": DialogueState.PLANNING,
                "user_helps": DialogueState.UNDERSTANDING,
                "give_up": DialogueState.IDLE
            }
        }

    def transition(self, event: str) -> bool:
        """Attempt state transition."""
        valid_events = self.transitions.get(self.state, {})

        if event in valid_events:
            old_state = self.state
            self.state = valid_events[event]

            self.history.append({
                "from": old_state.name,
                "to": self.state.name,
                "event": event,
                "timestamp": datetime.now().isoformat()
            })

            return True
        return False

    def set_context(self, key: str, value: Any):
        """Set context value."""
        self.context[key] = value

    def get_context(self, key: str, default: Any = None) -> Any:
        """Get context value."""
        return self.context.get(key, default)

    def reset(self):
        """Reset to idle state."""
        self.state = DialogueState.IDLE
        self.context.clear()


class ErrorRecoveryHandler:
    """Handle errors in conversation."""

    def __init__(self):
        self.attempt_counts: Dict[str, int] = {}
        self.max_attempts = 3

    def handle_error(self, error_type: str, context: Dict = None) -> Dict:
        """Handle error and suggest recovery."""
        self.attempt_counts[error_type] = self.attempt_counts.get(error_type, 0) + 1
        attempts = self.attempt_counts[error_type]

        strategies = {
            "speech_not_recognized": {
                "message": "I didn't catch that. Could you say it again?",
                "action": "retry_speech"
            },
            "intent_unclear": {
                "message": "I'm not sure what you'd like me to do. Could you rephrase?",
                "action": "retry_understanding"
            },
            "object_not_found": {
                "message": "I can't find that object. Could you point to it?",
                "action": "search"
            },
            "action_failed": {
                "message": "That didn't work. Let me try again.",
                "action": "retry_action"
            }
        }

        strategy = strategies.get(error_type, {
            "message": "I'm having trouble. Could you help?",
            "action": "ask_help"
        })

        if attempts > self.max_attempts:
            return {
                "should_continue": False,
                "message": "I'm having persistent trouble. Let's try something else.",
                "action": "give_up"
            }

        return {
            "should_continue": True,
            "message": strategy["message"],
            "action": strategy["action"],
            "attempt": attempts
        }

    def reset(self, error_type: str = None):
        """Reset attempt counts."""
        if error_type:
            self.attempt_counts.pop(error_type, None)
        else:
            self.attempt_counts.clear()


class DialogueManager:
    """Complete dialogue management system."""

    def __init__(self):
        self.memory = ConversationMemory()
        self.state_machine = DialogueStateMachine()
        self.resolver = ReferenceResolver(self.memory)
        self.error_handler = ErrorRecoveryHandler()

        # Callbacks
        self.on_response: Optional[Callable[[str], None]] = None
        self.on_action: Optional[Callable[[Dict], None]] = None

    def process_input(self, text: str) -> Dict:
        """Process user input and generate response."""
        # Log turn
        self.memory.add_turn("user", text)

        # Transition from idle to listening
        if self.state_machine.state == DialogueState.IDLE:
            self.state_machine.transition("user_speaks")

        # Process based on state
        self.state_machine.transition("speech_complete")

        # Try to understand
        result = self._understand(text)

        return result

    def _understand(self, text: str) -> Dict:
        """Understand user intent."""
        # Check for simple commands
        text_lower = text.lower()

        if any(w in text_lower for w in ["stop", "cancel", "nevermind"]):
            self.state_machine.transition("user_cancels")
            return {"action": "cancel", "response": "Okay, cancelled."}

        if any(w in text_lower for w in ["yes", "yeah", "okay", "sure"]):
            if self.state_machine.state == DialogueState.CONFIRMING:
                self.state_machine.transition("user_confirms")
                return {"action": "confirm", "response": "Got it, proceeding."}

        if any(w in text_lower for w in ["no", "nope"]):
            if self.state_machine.state == DialogueState.CONFIRMING:
                self.state_machine.transition("user_rejects")
                return {"action": "reject", "response": "Okay, what would you like instead?"}

        # Regular command processing
        self.state_machine.transition("understood")
        return {
            "action": "process",
            "text": text,
            "context": self.memory.get_context_for_llm()
        }

    def resolve_reference(self, reference: str) -> Optional[str]:
        """Resolve a reference in current context."""
        return self.resolver.resolve(reference)

    def set_scene(self, scene: Dict):
        """Update scene for reference resolution."""
        self.resolver.set_scene(scene)

    def add_response(self, text: str):
        """Log robot response."""
        self.memory.add_turn("robot", text)
        if self.on_response:
            self.on_response(text)

    def handle_error(self, error_type: str, context: Dict = None) -> Dict:
        """Handle error during dialogue."""
        self.state_machine.transition("error")
        recovery = self.error_handler.handle_error(error_type, context)

        if not recovery["should_continue"]:
            self.state_machine.transition("give_up")
        else:
            self.add_response(recovery["message"])

        return recovery

    def reset(self):
        """Reset dialogue state."""
        self.state_machine.reset()
        self.error_handler.reset()


# Example usage
if __name__ == "__main__":
    print("Dialogue Manager Demo")
    print("=" * 40)

    dm = DialogueManager()

    # Set up scene
    scene = {
        "objects": [
            {"name": "red_ball", "color": "red", "type": "ball", "y": 0.5},
            {"name": "blue_ball", "color": "blue", "type": "ball", "y": -0.3},
            {"name": "cup", "color": "white", "type": "cup", "y": 0.1}
        ]
    }
    dm.set_scene(scene)

    # Simulate conversation
    conversation = [
        "Pick up the red ball",
        "Now put it on the table",
        "Get the other one",
        "The one on the left"
    ]

    print("\nSimulated conversation:")
    for user_input in conversation:
        print(f"\nUser: {user_input}")

        result = dm.process_input(user_input)
        print(f"State: {dm.state_machine.state.name}")

        # Try reference resolution
        for word in user_input.lower().split():
            if word in ["it", "one"]:
                resolved = dm.resolve_reference(word)
                print(f"Resolved '{word}' -> {resolved}")

    print("\n" + "=" * 40)
    print("Memory context:")
    print(dm.memory.get_context_for_llm())
