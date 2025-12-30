"""Interface to OpenAI GPT-4 with retry logic."""

import time
from typing import Optional
from openai import OpenAI, RateLimitError, APIError
from .config import AgentConfig


class LLMInterface:
    """Interface to OpenAI API with exponential backoff retry logic."""

    def __init__(self, config: AgentConfig):
        """Initialize LLM interface.

        Args:
            config: AgentConfig with OpenAI credentials.
        """
        self.config = config
        self.has_api_key = bool(config.openai_api_key)

        if self.has_api_key:
            self.client = OpenAI(api_key=config.openai_api_key)
        else:
            self.client = None

        self.model = config.openai_model
        self.temperature = config.temperature

    def generate_response(
        self, system_prompt: str, user_prompt: str, max_retries: int = 3
    ) -> tuple[str, float]:
        """Generate response from LLM with exponential backoff retry.

        Args:
            system_prompt: System prompt defining behavior.
            user_prompt: User query and context.
            max_retries: Maximum retry attempts (default 3).

        Returns:
            Tuple of (response_text, latency_ms).
        """
        start_time = time.time()

        # Check if API key is available
        if not self.has_api_key:
            latency_ms = (time.time() - start_time) * 1000
            raise RuntimeError(
                "OpenAI API key not configured. Set OPENAI_API_KEY environment variable to enable full RAG. Running in demo mode."
            )

        last_error = None

        for attempt in range(max_retries):
            try:
                response = self.client.chat.completions.create(
                    model=self.model,
                    messages=[
                        {"role": "system", "content": system_prompt},
                        {"role": "user", "content": user_prompt},
                    ],
                    temperature=self.temperature,
                    max_tokens=500,
                )

                latency_ms = (time.time() - start_time) * 1000
                return response.choices[0].message.content, latency_ms

            except (RateLimitError, APIError) as e:
                last_error = e
                if attempt < max_retries - 1:
                    # Exponential backoff: 1s, 2s, 4s
                    wait_time = 2 ** attempt
                    time.sleep(wait_time)
                continue

        # All retries exhausted
        latency_ms = (time.time() - start_time) * 1000
        raise RuntimeError(
            f"Failed to generate response after {max_retries} attempts: {last_error}"
        )

    def parse_response(self, response_text: str) -> str:
        """Parse LLM response (basic pass-through for now).

        Args:
            response_text: Raw response from LLM.

        Returns:
            Parsed response.
        """
        return response_text.strip()
