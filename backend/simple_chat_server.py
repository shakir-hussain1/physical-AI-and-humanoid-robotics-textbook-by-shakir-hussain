from fastapi import FastAPI
from pydantic import BaseModel
from typing import List, Optional
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI()

# Add CORS middleware to allow requests from the frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify the exact frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Define request/response models
class QueryRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None

class Source(BaseModel):
    id: str
    content: str
    source: str
    score: float

class QueryResponse(BaseModel):
    answer: str
    sources: List[Source]
    confidence: float
    confidence_level: str
    status: str

@app.post("/chat/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """Handle natural language queries and return answers with citations."""
    logger.info(f"Received query: {request.query}")

    # Mock response based on the query
    query_lower = request.query.lower()

    if "hello" in query_lower or "hi" in query_lower:
        answer = "Hello! I'm your Physical AI & Humanoid Robotics textbook assistant. How can I help you with the content?"
    elif "ros" in query_lower:
        answer = "ROS (Robot Operating System) is a flexible framework for writing robot software. It provides services like hardware abstraction, device drivers, libraries, visualizers, message-passing, and package management. In the context of Physical AI, ROS is used as the middleware for communication between different robot components."
    elif "humanoid" in query_lower:
        answer = "Humanoid robots are robots with human-like form and capabilities. In Physical AI, humanoid robotics combines perception, decision-making, and action to create robots that can operate in human environments. The textbook covers various aspects of humanoid robotics including kinematics, control systems, and AI integration."
    elif "physical ai" in query_lower:
        answer = "Physical AI is an interdisciplinary field that combines artificial intelligence with physical systems. It focuses on creating intelligent systems that can interact with the physical world, including robots that can perceive, reason, and act in real environments. The field integrates machine learning, robotics, computer vision, and control theory."
    else:
        answer = f"I understand you're asking about '{request.query}'. This is a mock response from the Physical AI & Humanoid Robotics textbook assistant. In the full implementation, this would retrieve relevant information from the textbook content and provide an accurate answer with citations."

    # Create mock response
    response = QueryResponse(
        answer=answer,
        sources=[
            Source(id="mock-source-1", content="Mock content related to your query", source="Physical AI & Humanoid Robotics Textbook", score=0.9)
        ],
        confidence=0.85,
        confidence_level="high",
        status="success"
    )

    return response

@app.get("/health")
async def health_check():
    return {"status": "healthy", "service": "chatbot-backend"}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)