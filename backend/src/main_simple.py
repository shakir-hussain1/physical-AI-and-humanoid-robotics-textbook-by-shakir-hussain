"""Simplified FastAPI application for hackathon demonstration."""
import os
import logging
from fastapi import FastAPI, HTTPException, UploadFile, File, Form
from pydantic import BaseModel
from typing import Optional, List
from dotenv import load_dotenv
import uuid
from datetime import datetime

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot Backend - Hackathon Demo",
    description="Simplified version for demonstration purposes",
    version="1.0.0"
)

# Import models from existing code
from src.models.schemas import QueryRequest, QueryResponse, ContextRestrictedQueryRequest, ContextRestrictedQueryResponse, IngestRequest, IngestResponse, HealthResponse

# Mock services (since we can't install all dependencies)
class MockRetrievalService:
    @staticmethod
    async def retrieve_chunks(query: str, top_k: int = 5):
        # Return mock chunks for demonstration
        return [
            {
                "chunk_id": f"mock_chunk_{i}",
                "text": f"This is a mock chunk related to your query: '{query}' - Result {i}",
                "page_number": i,
                "section": f"Section {i}",
                "score": 0.85 - (i * 0.05)  # Decreasing relevance
            }
            for i in range(1, top_k + 1)
        ]

    @staticmethod
    async def rank_by_confidence(chunks, threshold: float = 0.60):
        # Filter and rank mock chunks
        filtered = [c for c in chunks if c["score"] >= threshold]
        ranked = sorted(filtered, key=lambda x: x["score"], reverse=True)
        avg_confidence = sum(c["score"] for c in ranked) / len(ranked) if ranked else 0.0
        return ranked, avg_confidence

class MockGenerationService:
    @staticmethod
    async def generate_answer(chunks, query: str):
        # Generate a mock answer based on query
        answer = f"Based on the Physical AI textbook, the answer to '{query}' is related to the concepts in the provided materials. The key points are covered in sections {[c['section'] for c in chunks[:2]]}."
        return {
            "answer": answer,
            "confidence": 0.82,
            "raw_response": "This is a mock response for demonstration."
        }

    @staticmethod
    async def estimate_confidence(retrieval_confidence: float, generation_result: dict):
        return min(1.0, (retrieval_confidence * 0.6) + (generation_result["confidence"] * 0.4))

class MockCitationService:
    @staticmethod
    async def build_citation_objects(chunks):
        from src.models.schemas import Citation
        citations = []
        for chunk in chunks:
            citation = Citation(
                chunk_id=chunk["chunk_id"],
                excerpt=chunk["text"][:100] + "..." if len(chunk["text"]) > 100 else chunk["text"],
                page_number=chunk["page_number"],
                section_heading=chunk["section"],
                source_confidence=chunk["score"]
            )
            citations.append(citation)
        return citations

    @staticmethod
    async def inject_citations(answer: str, chunks):
        citation_text = "\n\nSources:\n"
        for i, chunk in enumerate(chunks[:3]):  # Limit to top 3 sources
            citation_text += f"- {chunk['section']} (Page {chunk['page_number']})\n"
        return answer + citation_text

# Initialize mock services
retrieval_service = MockRetrievalService()
generation_service = MockGenerationService()
citation_service = MockCitationService()

@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint."""
    return HealthResponse(
        status="healthy",
        timestamp=datetime.utcnow().isoformat(),
        services={
            "api": "operational",
            "database": "mocked",
            "vector_store": "mocked"
        }
    )

@app.post("/chat/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """Handle natural language queries and return answers with citations."""
    try:
        logger.info(f"Processing query: {request.query}")

        # Retrieve relevant chunks (mock)
        chunks_data = await retrieval_service.retrieve_chunks(request.query)
        ranked_chunks, retrieval_confidence = await retrieval_service.rank_by_confidence(chunks_data)

        # Generate answer (mock)
        generation_result = await generation_service.generate_answer(ranked_chunks, request.query)

        # Calculate final confidence
        final_confidence = await generation_service.estimate_confidence(
            retrieval_confidence,
            generation_result
        )

        # Create citations
        citations = await citation_service.build_citation_objects(ranked_chunks)

        # Inject citations into answer
        answer_with_citations = await citation_service.inject_citations(
            generation_result["answer"],
            ranked_chunks
        )

        # Create response
        response = QueryResponse(
            answer_id=str(uuid.uuid4()),
            answer=answer_with_citations,
            sources=citations,
            confidence=final_confidence,
            confidence_level="high" if final_confidence >= 0.8 else "medium" if final_confidence >= 0.6 else "low",
            status="success"
        )

        logger.info(f"Query processed successfully, confidence: {final_confidence}")
        return response

    except Exception as e:
        logger.error(f"Error processing query: {e}")
        raise HTTPException(status_code=500, detail="Error processing query")

@app.post("/chat/context-restricted", response_model=ContextRestrictedQueryResponse)
async def context_restricted_query_endpoint(request: ContextRestrictedQueryRequest):
    """Handle queries restricted to a specific passage context."""
    try:
        logger.info(f"Processing context-restricted query: {request.query}")

        # For context-restricted, we use the provided passage
        passage_chunk = [{
            "chunk_id": "provided_passage",
            "text": request.selected_passage,
            "page_number": 1,
            "section": "Selected Passage",
            "score": 1.0
        }]

        # Generate answer from the passage
        generation_result = await generation_service.generate_answer(passage_chunk, request.query)

        # Create response
        response = ContextRestrictedQueryResponse(
            answer_id=str(uuid.uuid4()),
            answer=generation_result["answer"],
            sources=await citation_service.build_citation_objects(passage_chunk),
            confidence=generation_result["confidence"],
            confidence_level="high" if generation_result["confidence"] >= 0.8 else "medium",
            status="success",
            source_mode="context-restricted"
        )

        logger.info("Context-restricted query processed successfully")
        return response

    except Exception as e:
        logger.error(f"Error processing context-restricted query: {e}")
        raise HTTPException(status_code=500, detail="Error processing query")

@app.post("/content/ingest", response_model=IngestResponse)
async def ingest_content_endpoint(
    file: UploadFile = File(...),
    chapter_id: str = Form(...)
):
    """Handle content ingestion for indexing in the RAG system."""
    try:
        logger.info(f"Ingesting content for chapter: {chapter_id}")

        # Read file content
        content = await file.read()
        text_content = content.decode('utf-8') if isinstance(content, bytes) else str(content)

        # Mock processing
        response = IngestResponse(
            ingest_id=str(uuid.uuid4()),
            status="completed",
            message=f"File {file.filename} processed successfully for chapter {chapter_id}",
            chunks_created=min(10, len(text_content) // 500 + 1)  # Estimate chunks
        )

        logger.info(f"Content ingestion completed for chapter {chapter_id}")
        return response

    except Exception as e:
        logger.error(f"Error ingesting content: {e}")
        raise HTTPException(status_code=500, detail="Error ingesting content")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "src.main_simple:app",
        host="0.0.0.0",
        port=8000,
        reload=True
    )