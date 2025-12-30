"""Build system and user prompts for LLM."""

from typing import List, Optional
from .types import Message
from .config import AgentConfig


class PromptBuilder:
    """Build system and user prompts with grounding constraints."""

    def __init__(self, config: AgentConfig):
        """Initialize prompt builder.

        Args:
            config: AgentConfig for role-based prompts.
        """
        self.config = config

    def build_system_prompt(self, user_role: str = "student") -> str:
        """Build system prompt for role.

        Args:
            user_role: Role (student, teacher, researcher).

        Returns:
            System prompt string.
        """
        return """You are a helpful assistant answering questions about Physical AI, Robotics, and Humanoid Robots based on the provided textbook content.

Be direct and concise. Answer only what is asked. Only use information from the provided context."""

    def build_user_prompt(
        self,
        query: str,
        context: str,
        conversation_history: Optional[List[Message]] = None,
    ) -> str:
        """Build user prompt combining query, context, and history.

        Args:
            query: User question.
            context: Retrieved context from knowledge base.
            conversation_history: Optional conversation history.

        Returns:
            User prompt string.
        """
        parts = []

        # Add context only
        parts.append("Context:")
        parts.append(context)
        parts.append("")

        # Add question
        parts.append("Question: " + query)
        parts.append("")

        # Add instruction - simple and direct
        parts.append("Answer briefly and directly from the context above.")

        return "\n".join(parts)
