"""Agent orchestrator coordinating all agent operations."""

from typing import Optional, List
from .types import Query, AgentResponse, Message
from .config import AgentConfig
from .intent_parser import IntentParser
from .context_constructor import ContextConstructor
from .prompt_builder import PromptBuilder
from .llm_interface import LLMInterface
from .grounding_validator import GroundingValidator
from .response_formatter import ResponseFormatter
from .monitoring import AgentMonitor
from .error_handler import ErrorHandler


class AgentOrchestrator:
    """Orchestrates agent workflow: intent → retrieval → generation → validation → response."""

    def __init__(self, config: Optional[AgentConfig] = None):
        """Initialize orchestrator with configuration and components.

        Args:
            config: Optional AgentConfig. If None, loads from environment.
        """
        self.config = config or AgentConfig.load_from_env()

        # Initialize components
        self.intent_parser = IntentParser()
        self.context_constructor = ContextConstructor()
        self.prompt_builder = PromptBuilder(self.config)
        self.llm_interface = LLMInterface(self.config)
        self.grounding_validator = GroundingValidator()
        self.response_formatter = ResponseFormatter()
        self.monitor = AgentMonitor()
        self.error_handler = ErrorHandler()

    def query(
        self,
        query_text: str,
        conversation_history: Optional[List[Message]] = None,
        user_role: str = "student",
    ) -> AgentResponse:
        """Process user query and return grounded response.

        Args:
            query_text: User question or command.
            conversation_history: Optional list of previous messages.
            user_role: Role for system prompt (student, teacher, researcher).

        Returns:
            AgentResponse with answer, sources, confidence, and metadata.
        """
        try:
            # Create query object
            query = Query(
                text=query_text,
                conversation_history=conversation_history or [],
                user_role=user_role,
            )

            # Parse intent
            intent = self.intent_parser.parse_intent(query.text)

            # Check for out-of-domain early
            if intent.query_type == "out_of_scope":
                return self.error_handler.handle_out_of_domain(query.text)

            # === FULL RAG ORCHESTRATION WORKFLOW ===

            # Step 1: Retrieve relevant documents from knowledge base
            retrieved_chunks = []
            try:
                retrieval_tool = self._initialize_retrieval_tool()
                retrieved_chunks = retrieval_tool.search_knowledge_base(
                    query=query.text,
                    k=5
                )
                # Handle error response from retrieval tool
                if isinstance(retrieved_chunks, dict) and "error" in retrieved_chunks:
                    retrieved_chunks = []
            except Exception as retrieval_error:
                # Log retrieval failure but continue with LLM-only response
                import logging
                logger = logging.getLogger(__name__)
                logger.warning(f"Retrieval failed, using LLM-only mode: {str(retrieval_error)}")
                retrieved_chunks = []

            # Step 2: Build context from retrieved chunks
            context = self._build_context_from_chunks(retrieved_chunks)

            # Step 3: Build system and user prompts
            system_prompt = self.prompt_builder.build_system_prompt(user_role)
            user_prompt = self.prompt_builder.build_user_prompt(
                query=query.text,
                context=context,
                conversation_history=query.conversation_history,
            )

            # Step 4: Generate response using LLM
            answer, latency_ms = self.llm_interface.generate_response(
                system_prompt=system_prompt,
                user_prompt=user_prompt,
            )

            # Step 5: Validate grounding in retrieved context
            is_grounded, grounding_score, grounding_reason = self.grounding_validator.validate_grounding(
                answer=answer,
                context=context,
            )

            # Step 6: Determine confidence level based on grounding and retrieval
            confidence = self._compute_confidence(
                is_grounded=is_grounded,
                grounding_score=grounding_score,
                retrieval_quality=len(retrieved_chunks) > 0,
            )

            # Step 7: Format sources from retrieved chunks
            sources = self._format_sources(retrieved_chunks)

            # Step 8: Build metadata
            metadata = {
                "grounding": is_grounded,
                "grounding_score": grounding_score,
                "grounding_reason": grounding_reason,
                "intent_type": intent.query_type,
                "intent_topic": intent.primary_topic,
                "llm_latency_ms": latency_ms,
                "follow_ups": self._generate_follow_up_questions(query.text, answer),
            }

            # Step 9: Format final response
            response = self.response_formatter.format_response(
                answer=answer,
                sources=sources,
                confidence=confidence,
                metadata=metadata,
            )

            return response

        except Exception as e:
            # Pass query for context-aware demo responses
            return self.error_handler.handle_error(str(e), query=query_text)

    def _initialize_retrieval_tool(self):
        """Initialize retrieval tool lazily."""
        if not hasattr(self, '_retrieval_tool'):
            from .retrieval_tool import RetrievalTool
            self._retrieval_tool = RetrievalTool()
        return self._retrieval_tool

    def _build_context_from_chunks(self, chunks: List) -> str:
        """Build context string from retrieved chunks.

        Args:
            chunks: List of retrieved chunk dicts.

        Returns:
            Formatted context string.
        """
        if not chunks:
            return "[No relevant information found in knowledge base]"

        context_parts = []
        has_text_content = False

        for i, chunk in enumerate(chunks, 1):
            # Extract text from chunk
            if isinstance(chunk, dict):
                # Check if metadata is nested
                metadata = chunk.get("metadata", {})
                text = chunk.get("text", chunk.get("chunk_text", ""))

                # Try to get title from metadata, then top level
                if isinstance(metadata, dict):
                    title = metadata.get("page_title", chunk.get("page_title", chunk.get("title", f"Source {i}")))
                else:
                    title = chunk.get("page_title", chunk.get("title", f"Source {i}"))

                # Check if we have actual text content
                if text and len(text.strip()) > 10:
                    has_text_content = True
            else:
                text = str(chunk)
                title = f"Source {i}"
                if len(text.strip()) > 10:
                    has_text_content = True

            # Format chunk in context
            if text:
                context_parts.append(f"[{title}]\n{text}\n")
            else:
                # If no text, just include the title as a hint
                context_parts.append(f"[Relevant source: {title}]")

        context_str = "\n".join(context_parts)

        # If we don't have actual text content, add a note for the LLM
        if not has_text_content:
            context_str += "\n\n[Note: The above sources are relevant to the query. Provide an answer based on your knowledge, citing these sources.]"

        return context_str

    def _format_sources(self, chunks: List) -> List:
        """Format retrieved chunks as SourceInfo objects.

        Args:
            chunks: List of retrieved chunks.

        Returns:
            List of SourceInfo objects.
        """
        sources = []
        for chunk in chunks[:3]:  # Top 3 sources
            if isinstance(chunk, dict):
                # Extract metadata - could be nested or at top level
                metadata = chunk.get("metadata", {})

                # Get values from metadata if nested, otherwise from top level
                url = metadata.get("url", "") or chunk.get("url", "")
                page_title = metadata.get("page_title", "") or chunk.get("page_title", chunk.get("title", ""))
                chunk_index = metadata.get("chunk_index", 0) or chunk.get("chunk_index", 0)

                source = {
                    "url": url,
                    "page_title": page_title,
                    "relevance_score": chunk.get("similarity_score", chunk.get("relevance_score", 0.8)),
                    "chunk_index": chunk_index,
                }
            else:
                source = {
                    "url": "",
                    "page_title": "",
                    "relevance_score": 0.8,
                    "chunk_index": 0,
                }
            sources.append(source)

        return sources

    def _compute_confidence(self, is_grounded: bool, grounding_score: float, retrieval_quality: bool) -> str:
        """Compute confidence level based on grounding and retrieval.

        Args:
            is_grounded: Whether response is grounded in context.
            grounding_score: Grounding confidence score [0-1].
            retrieval_quality: Whether retrieval found results.

        Returns:
            Confidence level: "high", "medium", or "low".
        """
        # If we have good grounding and retrieval, confidence is high
        if is_grounded and grounding_score >= 0.7 and retrieval_quality:
            return "high"
        # If we have good grounding or good retrieval quality, confidence is medium
        elif (is_grounded and grounding_score >= 0.5) or retrieval_quality:
            return "medium"
        # Otherwise confidence is low
        else:
            return "low"

    def _generate_follow_up_questions(self, query: str, answer: str) -> List[str]:
        """Generate suggested follow-up questions.

        Args:
            query: Original user query.
            answer: Generated answer.

        Returns:
            List of follow-up question suggestions.
        """
        # Basic follow-ups - can be enhanced with LLM in future
        follow_ups = []

        keywords = ["ROS", "robotics", "humanoid", "AI", "perception", "control", "middleware"]
        for keyword in keywords:
            if keyword.lower() in query.lower() or keyword.lower() in answer.lower():
                follow_ups.append(f"Tell me more about {keyword.lower()}")

        follow_ups.append("Can you explain this in simpler terms?")
        follow_ups.append("What are the key applications?")

        return follow_ups[:3]  # Return top 3 follow-ups
