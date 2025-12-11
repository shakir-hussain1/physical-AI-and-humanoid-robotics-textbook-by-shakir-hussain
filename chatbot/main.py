"""RAG Chatbot API - FastAPI Backend with OpenAI"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from openai import OpenAI
import os
import hashlib

app = FastAPI()
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

# Setup OpenAI
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# Qdrant
qdrant = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))
COLLECTION = "book_chunks"

class Query(BaseModel):
    question: str
    selected_text: str = ""

class IndexRequest(BaseModel):
    text: str
    source: str

def get_embedding(text: str) -> list:
    result = client.embeddings.create(input=text, model="text-embedding-3-small")
    return result.data[0].embedding

@app.on_event("startup")
def startup():
    try:
        qdrant.delete_collection(COLLECTION)
    except: pass
    try:
        qdrant.create_collection(COLLECTION, vectors_config=VectorParams(size=1536, distance=Distance.COSINE))
    except: pass

@app.post("/index")
def index_content(req: IndexRequest):
    chunks = [req.text[i:i+500] for i in range(0, len(req.text), 400)]
    points = []
    for i, chunk in enumerate(chunks):
        vec = get_embedding(chunk)
        pid = hashlib.md5(f"{req.source}_{i}".encode()).hexdigest()
        points.append(PointStruct(id=pid, vector=vec, payload={"text": chunk, "source": req.source}))
    qdrant.upsert(COLLECTION, points)
    return {"indexed": len(points)}

@app.post("/chat")
def chat(query: Query):
    context_text = query.selected_text if query.selected_text else ""
    vec = get_embedding(query.question)
    results = qdrant.query_points(COLLECTION, query=vec, limit=3).points
    retrieved = "\n".join([r.payload["text"] for r in results])
    full_context = f"{context_text}\n\n{retrieved}" if context_text else retrieved

    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "system", "content": "You are a helpful assistant for a Physical AI & Humanoid Robotics textbook. Answer based only on the provided context. If unsure, say so."},
            {"role": "user", "content": f"Context:\n{full_context}\n\nQuestion: {query.question}"}
        ]
    )
    answer = response.choices[0].message.content

    return {"answer": answer, "sources": [r.payload["source"] for r in results]}

@app.get("/health")
def health():
    return {"status": "ok"}