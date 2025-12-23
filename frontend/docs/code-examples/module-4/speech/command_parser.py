#!/usr/bin/env python3
"""
Command Parser for Voice-Controlled Robot
Module 4: VLA Systems - Chapter 14

Parses transcribed voice commands into structured robot actions.
"""

import re
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Tuple
from enum import Enum


class Intent(Enum):
    """Robot command intents."""
    GRASP = "grasp"
    PLACE = "place"
    NAVIGATE = "navigate"
    LOOK = "look"
    POINT = "point"
    STOP = "stop"
    HELP = "help"
    CONFIRM = "confirm"
    CANCEL = "cancel"
    UNKNOWN = "unknown"


@dataclass
class ParsedCommand:
    """Structured representation of a voice command."""
    intent: Intent
    action_phrase: str = ""
    target_object: Optional[str] = None
    target_location: Optional[str] = None
    modifiers: List[str] = field(default_factory=list)
    confidence: float = 1.0
    raw_text: str = ""

    def is_complete(self) -> bool:
        """Check if command has required information."""
        if self.intent in [Intent.STOP, Intent.HELP, Intent.CANCEL]:
            return True
        if self.intent in [Intent.GRASP, Intent.LOOK, Intent.POINT]:
            return self.target_object is not None
        if self.intent == Intent.NAVIGATE:
            return self.target_location is not None
        if self.intent == Intent.PLACE:
            return self.target_location is not None
        return False

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return {
            "intent": self.intent.value,
            "action": self.action_phrase,
            "target_object": self.target_object,
            "target_location": self.target_location,
            "modifiers": self.modifiers,
            "confidence": self.confidence,
            "raw_text": self.raw_text
        }

    def describe(self) -> str:
        """Generate natural language description."""
        parts = [self.intent.value]
        if self.modifiers:
            parts.append(" ".join(self.modifiers))
        if self.target_object:
            parts.append(self.target_object)
        if self.target_location:
            parts.append(f"to/at {self.target_location}")
        return " ".join(parts)


class RuleBasedCommandParser:
    """Parse voice commands using pattern matching."""

    # Intent patterns (verb phrases that indicate intent)
    INTENT_PATTERNS = {
        Intent.GRASP: [
            r"pick\s+up", r"grab", r"grasp", r"get", r"take",
            r"fetch", r"bring\s+me", r"hand\s+me", r"give\s+me"
        ],
        Intent.PLACE: [
            r"put", r"place", r"set", r"drop", r"leave",
            r"put\s+down", r"set\s+down"
        ],
        Intent.NAVIGATE: [
            r"go\s+to", r"move\s+to", r"walk\s+to", r"navigate\s+to",
            r"come\s+to", r"head\s+to", r"come\s+here"
        ],
        Intent.LOOK: [
            r"look\s+at", r"look\s+for", r"find", r"locate",
            r"search\s+for", r"where\s+is", r"show\s+me"
        ],
        Intent.POINT: [
            r"point\s+at", r"point\s+to", r"indicate"
        ],
        Intent.STOP: [
            r"stop", r"halt", r"freeze", r"cancel", r"abort",
            r"don't", r"wait"
        ],
        Intent.HELP: [
            r"help", r"what\s+can\s+you\s+do", r"commands",
            r"how\s+do\s+i", r"options"
        ],
        Intent.CONFIRM: [
            r"yes", r"yeah", r"yep", r"okay", r"ok", r"sure",
            r"correct", r"right", r"go\s+ahead", r"do\s+it"
        ],
        Intent.CANCEL: [
            r"no", r"nope", r"cancel", r"never\s*mind", r"forget\s+it"
        ]
    }

    # Color vocabulary
    COLORS = [
        "red", "blue", "green", "yellow", "orange", "purple",
        "pink", "black", "white", "brown", "gray", "grey"
    ]

    # Size modifiers
    SIZES = ["big", "large", "small", "little", "tiny", "huge"]

    # Common objects
    OBJECTS = [
        "ball", "cube", "box", "cup", "mug", "bottle", "can",
        "book", "pen", "pencil", "phone", "remote", "keys",
        "plate", "bowl", "spoon", "fork", "knife", "glass",
        "toy", "block", "paper", "bag", "tool", "hammer"
    ]

    # Locations
    LOCATIONS = [
        "table", "desk", "floor", "shelf", "counter", "chair",
        "couch", "sofa", "bed", "kitchen", "living room", "bedroom",
        "bathroom", "office", "garage", "fridge", "cabinet",
        "drawer", "box", "bin", "here", "there"
    ]

    # Spatial prepositions
    PREPOSITIONS = [
        "on", "in", "at", "near", "by", "next to", "beside",
        "under", "below", "above", "over", "behind", "in front of",
        "to the left of", "to the right of", "from"
    ]

    def __init__(self):
        # Compile regex patterns
        self._compile_patterns()

    def _compile_patterns(self):
        """Compile regex patterns for efficiency."""
        self.intent_regexes = {}
        for intent, patterns in self.INTENT_PATTERNS.items():
            combined = "|".join(f"({p})" for p in patterns)
            self.intent_regexes[intent] = re.compile(combined, re.IGNORECASE)

    def parse(self, text: str) -> ParsedCommand:
        """Parse transcribed text into structured command."""
        text = text.lower().strip()
        text = self._normalize_text(text)

        # Extract intent
        intent, action_phrase = self._extract_intent(text)

        # Extract modifiers (colors, sizes)
        modifiers = self._extract_modifiers(text)

        # Extract object
        target_object = self._extract_object(text)

        # Extract location
        target_location = self._extract_location(text)

        # Build result
        return ParsedCommand(
            intent=intent,
            action_phrase=action_phrase,
            target_object=target_object,
            target_location=target_location,
            modifiers=modifiers,
            raw_text=text
        )

    def _normalize_text(self, text: str) -> str:
        """Normalize text for parsing."""
        # Remove filler words
        fillers = ["um", "uh", "like", "please", "could you", "can you", "would you"]
        for filler in fillers:
            text = text.replace(filler, " ")

        # Normalize whitespace
        text = " ".join(text.split())
        return text

    def _extract_intent(self, text: str) -> Tuple[Intent, str]:
        """Extract intent from text."""
        for intent, regex in self.intent_regexes.items():
            match = regex.search(text)
            if match:
                return intent, match.group(0)

        return Intent.UNKNOWN, ""

    def _extract_modifiers(self, text: str) -> List[str]:
        """Extract color and size modifiers."""
        modifiers = []

        for color in self.COLORS:
            if color in text:
                modifiers.append(color)

        for size in self.SIZES:
            if size in text:
                modifiers.append(size)

        return modifiers

    def _extract_object(self, text: str) -> Optional[str]:
        """Extract target object from text."""
        # Look for known objects
        for obj in self.OBJECTS:
            if obj in text:
                return obj

        # Try to extract noun after "the"
        the_pattern = r"the\s+(\w+)"
        match = re.search(the_pattern, text)
        if match:
            word = match.group(1)
            # Filter out prepositions and common words
            if word not in self.PREPOSITIONS and word not in ["one", "thing"]:
                return word

        return None

    def _extract_location(self, text: str) -> Optional[str]:
        """Extract target location from text."""
        # Look for preposition + location patterns
        for prep in self.PREPOSITIONS:
            pattern = rf"{prep}\s+(?:the\s+)?(\w+(?:\s+\w+)?)"
            match = re.search(pattern, text)
            if match:
                candidate = match.group(1)
                # Check if it's a known location
                for loc in self.LOCATIONS:
                    if loc in candidate or candidate in loc:
                        return loc

        # Direct location match
        for loc in self.LOCATIONS:
            if loc in text:
                return loc

        return None

    def get_clarification_question(self, command: ParsedCommand) -> Optional[str]:
        """Generate clarification question for incomplete command."""
        if command.intent == Intent.UNKNOWN:
            return "What would you like me to do?"

        if command.intent in [Intent.GRASP, Intent.LOOK, Intent.POINT]:
            if command.target_object is None:
                return "Which object should I work with?"

        if command.intent == Intent.NAVIGATE:
            if command.target_location is None:
                return "Where should I go?"

        if command.intent == Intent.PLACE:
            if command.target_location is None:
                return "Where should I put it?"

        return None


class SlotFillingParser:
    """Parser with multi-turn slot filling support."""

    def __init__(self, base_parser: RuleBasedCommandParser = None):
        self.parser = base_parser or RuleBasedCommandParser()
        self.current_slots: Optional[ParsedCommand] = None

    def process_utterance(self, text: str) -> ParsedCommand:
        """Process utterance, filling slots incrementally."""
        parsed = self.parser.parse(text)

        if self.current_slots is None:
            # Start new command
            self.current_slots = parsed
        else:
            # Update slots with new information
            if parsed.intent != Intent.UNKNOWN:
                self.current_slots.intent = parsed.intent
                self.current_slots.action_phrase = parsed.action_phrase

            if parsed.target_object:
                self.current_slots.target_object = parsed.target_object

            if parsed.target_location:
                self.current_slots.target_location = parsed.target_location

            if parsed.modifiers:
                # Add new modifiers, avoiding duplicates
                for mod in parsed.modifiers:
                    if mod not in self.current_slots.modifiers:
                        self.current_slots.modifiers.append(mod)

        return self.current_slots

    def get_clarification_question(self) -> Optional[str]:
        """Get clarification question for current slots."""
        if self.current_slots is None:
            return "What would you like me to do?"

        return self.parser.get_clarification_question(self.current_slots)

    def is_complete(self) -> bool:
        """Check if current command is complete."""
        return self.current_slots is not None and self.current_slots.is_complete()

    def reset(self):
        """Clear current slots for new command."""
        self.current_slots = None

    def get_command(self) -> Optional[ParsedCommand]:
        """Get current command if complete."""
        if self.is_complete():
            cmd = self.current_slots
            self.reset()
            return cmd
        return None


# Example usage and testing
if __name__ == "__main__":
    print("Command Parser Demo")
    print("=" * 40)

    parser = RuleBasedCommandParser()

    test_commands = [
        "Pick up the red ball",
        "Put it on the table",
        "Go to the kitchen",
        "Find my keys",
        "Stop",
        "What can you do?",
        "Get the big blue cube from the shelf",
        "Place the bottle near the fridge",
        "Look at the door",
        "Point to the chair",
        "blah blah random words"
    ]

    print("\nParsing test commands:\n")
    for cmd in test_commands:
        result = parser.parse(cmd)
        print(f"Input: \"{cmd}\"")
        print(f"  Intent: {result.intent.value}")
        print(f"  Object: {result.target_object}")
        print(f"  Location: {result.target_location}")
        print(f"  Modifiers: {result.modifiers}")
        print(f"  Complete: {result.is_complete()}")
        if not result.is_complete():
            question = parser.get_clarification_question(result)
            print(f"  Clarification: {question}")
        print()

    # Test slot filling
    print("\nSlot Filling Demo:")
    print("-" * 40)

    slot_parser = SlotFillingParser()

    utterances = [
        "Pick something up",
        "The red ball",
        "And put it on the table"
    ]

    for utterance in utterances:
        print(f"\nUser: \"{utterance}\"")
        result = slot_parser.process_utterance(utterance)
        print(f"  Current: {result.describe()}")
        print(f"  Complete: {slot_parser.is_complete()}")
        if not slot_parser.is_complete():
            print(f"  Ask: {slot_parser.get_clarification_question()}")
