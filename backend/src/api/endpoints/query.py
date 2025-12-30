"""Query and context query endpoints."""

import time
import uuid
from typing import Optional

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse

from ..errors import (
    AgentError,
    InvalidHistoryError,
    LLMError,
    OutOfDomainError,
    QueryTooLongError,
    RetrievalError,
    RequestTimeoutError,
    ServiceUnavailableError,
    ValidationError,
)
from ..models import ContextQueryRequest, QueryRequest, QueryResponse, SourceInfo
from ..validators import validate_history_format, validate_query_length, validate_selected_text, validate_user_role
from ...utils.logging import get_logger

router = APIRouter(prefix="/api", tags=["query"])
logger = get_logger()


@router.post("/query", response_model=QueryResponse, status_code=200)
async def query_endpoint(request: Request, query_request: QueryRequest) -> QueryResponse:
    """
    Process user query and return answer with sources.

    POST /api/query
    - Accepts: user query, conversation history, user role
    - Returns: answer, sources, confidence, metadata
    - Errors: 400 (invalid input), 502 (LLM), 503 (retrieval), 504 (timeout)
    """
    request_id = getattr(request.state, "request_id", "unknown")
    start_time = time.time()

    try:
        # Import agent dynamically (initialized in lifespan)
        from ..main import agent

        if not agent:
            logger.error("Agent not initialized", extra={"request_id": request_id})
            raise ServiceUnavailableError("RAG Agent")

        # Validate request
        valid, error_msg = validate_query_length(query_request.query)
        if not valid:
            logger.warning(
                f"Query validation failed: {error_msg}",
                extra={"request_id": request_id},
            )
            raise QueryTooLongError()

        valid, error_msg = validate_history_format(query_request.conversation_history)
        if not valid:
            logger.warning(
                f"History validation failed: {error_msg}",
                extra={"request_id": request_id},
            )
            raise InvalidHistoryError(error_msg)

        valid, error_msg = validate_user_role(query_request.user_role)
        if not valid:
            logger.warning(
                f"User role validation failed: {error_msg}",
                extra={"request_id": request_id},
            )
            raise ValidationError(error_msg, field="user_role")

        # Convert conversation history to agent format
        history = []
        if query_request.conversation_history:
            for msg in query_request.conversation_history:
                history.append({"role": msg.role, "content": msg.content})

        logger.info(
            "Query processing started",
            extra={
                "request_id": request_id,
                "query_length": len(query_request.query),
                "history_length": len(history),
                "user_role": query_request.user_role,
            },
        )

        # Call agent
        agent_response = agent.query(
            query_text=query_request.query,
            conversation_history=history,
            user_role=query_request.user_role,
        )

        # Check for out-of-domain response
        if agent_response.confidence == "low" and "out of domain" in agent_response.answer.lower():
            logger.info(
                "Out-of-domain query detected",
                extra={"request_id": request_id},
            )
            raise OutOfDomainError()

        # Format response
        latency_ms = int((time.time() - start_time) * 1000)

        # Convert sources to SourceInfo objects (handle both dict and object formats)
        sources_list = []
        for src in agent_response.sources:
            if hasattr(src, 'url'):
                # It's a SourceInfo object
                sources_list.append(
                    SourceInfo(
                        url=src.url,
                        page_title=src.page_title,
                        relevance_score=src.relevance_score,
                        chunk_index=src.chunk_index,
                    )
                )
            else:
                # It's a dict
                sources_list.append(
                    SourceInfo(
                        url=src.get("url", ""),
                        page_title=src.get("page_title", ""),
                        relevance_score=src.get("relevance_score", 0.0),
                        chunk_index=src.get("chunk_index", 0),
                    )
                )

        response = QueryResponse(
            answer=agent_response.answer,
            sources=sources_list,
            confidence=agent_response.confidence,
            metadata={
                "latency_ms": latency_ms,
                "grounding": agent_response.metadata.get("grounding", True),
                "follow_ups": agent_response.metadata.get("follow_ups", []),
                "request_id": request_id,
            },
        )

        logger.info(
            "Query processing completed",
            extra={
                "request_id": request_id,
                "latency_ms": latency_ms,
                "sources_count": len(response.sources),
                "confidence": response.confidence,
            },
        )

        return response

    except QueryTooLongError:
        raise
    except InvalidHistoryError:
        raise
    except ValidationError:
        raise
    except OutOfDomainError:
        raise
    except TimeoutError:
        logger.error(
            "Query timeout",
            extra={"request_id": request_id},
        )
        raise RequestTimeoutError(timeout_seconds=30)
    except Exception as e:
        error_type = type(e).__name__
        logger.error(
            f"Query processing failed: {str(e)}",
            extra={
                "request_id": request_id,
                "error_type": error_type,
            },
            exc_info=True,
        )

        # Map errors to HTTP responses
        if "timeout" in str(e).lower():
            raise RequestTimeoutError(timeout_seconds=30)
        elif "retrieval" in error_type.lower():
            raise RetrievalError(f"Failed to retrieve documents: {str(e)}")
        elif "llm" in error_type.lower() or "openai" in str(e).lower():
            raise LLMError(f"Unable to generate response: {str(e)}")
        else:
            raise AgentError(f"Unable to process query: {str(e)}")


@router.post("/query/with-context", response_model=QueryResponse, status_code=200)
async def context_query_endpoint(request: Request, context_request: ContextQueryRequest) -> QueryResponse:
    """
    Process query with selected text context.

    POST /api/query/with-context
    - Accepts: selected text, query, conversation history
    - Returns: answer with enhanced grounding from selected text
    - Errors: 400 (invalid input), 502 (LLM), 503 (retrieval), 504 (timeout)
    """
    request_id = getattr(request.state, "request_id", "unknown")
    start_time = time.time()

    try:
        # Import agent dynamically
        from ..main import agent

        if not agent:
            logger.error("Agent not initialized", extra={"request_id": request_id})
            raise ServiceUnavailableError("RAG Agent")

        # Validate request
        valid, error_msg = validate_selected_text(context_request.selected_text)
        if not valid:
            logger.warning(
                f"Selected text validation failed: {error_msg}",
                extra={"request_id": request_id},
            )
            raise ValidationError(error_msg, field="selected_text")

        valid, error_msg = validate_query_length(context_request.query)
        if not valid:
            logger.warning(
                f"Query validation failed: {error_msg}",
                extra={"request_id": request_id},
            )
            raise QueryTooLongError()

        valid, error_msg = validate_history_format(context_request.conversation_history)
        if not valid:
            logger.warning(
                f"History validation failed: {error_msg}",
                extra={"request_id": request_id},
            )
            raise InvalidHistoryError(error_msg)

        # Convert conversation history to agent format
        history = []
        if context_request.conversation_history:
            for msg in context_request.conversation_history:
                history.append({"role": msg.role, "content": msg.content})

        # Prepend selected text to query as context
        enhanced_query = f"Given this text:\n\n{context_request.selected_text}\n\n{context_request.query}"

        logger.info(
            "Context query processing started",
            extra={
                "request_id": request_id,
                "selected_text_length": len(context_request.selected_text),
                "query_length": len(context_request.query),
                "history_length": len(history),
            },
        )

        # Call agent with enhanced query
        agent_response = agent.query(
            query_text=enhanced_query,
            conversation_history=history,
            user_role="student",  # Default role for context queries
        )

        # Format response
        latency_ms = int((time.time() - start_time) * 1000)

        response = QueryResponse(
            answer=agent_response.answer,
            sources=[
                {
                    "url": src["url"],
                    "page_title": src.get("page_title", ""),
                    "relevance_score": src.get("relevance_score", 0.0),
                    "chunk_index": src.get("chunk_index", 0),
                }
                for src in agent_response.sources
            ],
            confidence=agent_response.confidence,
            metadata={
                "latency_ms": latency_ms,
                "grounding": True,  # Selected text provides strong grounding
                "context_enhanced": True,
                "follow_ups": agent_response.metadata.get("follow_ups", []),
                "request_id": request_id,
            },
        )

        logger.info(
            "Context query processing completed",
            extra={
                "request_id": request_id,
                "latency_ms": latency_ms,
                "sources_count": len(response.sources),
                "confidence": response.confidence,
            },
        )

        return response

    except ValidationError:
        raise
    except QueryTooLongError:
        raise
    except InvalidHistoryError:
        raise
    except TimeoutError:
        logger.error(
            "Context query timeout",
            extra={"request_id": request_id},
        )
        raise RequestTimeoutError(timeout_seconds=30)
    except Exception as e:
        error_type = type(e).__name__
        logger.error(
            f"Context query processing failed: {str(e)}",
            extra={
                "request_id": request_id,
                "error_type": error_type,
            },
            exc_info=True,
        )

        # Map errors to HTTP responses
        if "timeout" in str(e).lower():
            raise RequestTimeoutError(timeout_seconds=30)
        elif "retrieval" in error_type.lower():
            raise RetrievalError(f"Failed to retrieve documents: {str(e)}")
        elif "llm" in error_type.lower() or "openai" in str(e).lower():
            raise LLMError(f"Unable to generate response: {str(e)}")
        else:
            raise AgentError(f"Unable to process context query: {str(e)}")
