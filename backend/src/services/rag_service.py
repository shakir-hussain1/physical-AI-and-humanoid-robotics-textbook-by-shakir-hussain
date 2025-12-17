"""RAG Service - Core retrieval and generation logic."""

import os
import json
from typing import List, Dict, Any, Optional
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import logging

logger = logging.getLogger(__name__)

class RAGService:
    """Retrieval-Augmented Generation Service using OpenAI and Qdrant."""

    def __init__(self):
        self.openai_client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
        self.qdrant_client = QdrantClient(
            url=os.getenv('QDRANT_URL'),
            api_key=os.getenv('QDRANT_API_KEY')
        )
        self.embedding_model = os.getenv('OPENAI_EMBED_MODEL', 'text-embedding-3-small')
        self.chat_model = os.getenv('OPENAI_CHAT_MODEL', 'gpt-4o-mini')
        self.collection_name = os.getenv('QDRANT_COLLECTION', 'textbook_embeddings')
        self.top_k = int(os.getenv('RETRIEVAL_TOP_K', 5))
        self.temperature = float(os.getenv('RAG_TEMPERATURE', 0.7))
        self.confidence_threshold = float(os.getenv('CONFIDENCE_THRESHOLD', 0.6))

        # Initialize Qdrant collection if needed
        self._initialize_collection()

        # Sample textbook content for initial data
        self.sample_content = {
            'ros': {
                'title': 'Robot Operating System (ROS)',
                'content': 'ROS (Robot Operating System) is a flexible framework for writing robot software. It is a collection of tools and libraries that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. ROS provides: hardware abstraction, device drivers, message-passing between processes, package management, visualization tools, and more.',
                'source': 'Chapter 2: Foundations of Physical AI',
                'category': 'robotics'
            },
            'humanoid': {
                'title': 'Humanoid Robots',
                'content': 'Humanoid robots are designed to interact with human environments and work alongside humans. They typically have a torso, two arms, two legs, and a head. Humanoid robots combine several fields: mechanics (kinematics and dynamics), control theory, computer vision, natural language processing, and artificial intelligence. Notable examples include Boston Dynamics Atlas, Tesla Optimus, and ASIMO.',
                'source': 'Chapter 3: Humanoid Robot Architectures',
                'category': 'robotics'
            },
            'physical_ai': {
                'title': 'Physical AI Overview',
                'content': 'Physical AI is the intersection of artificial intelligence and robotics, focusing on creating intelligent systems that can perceive, reason, and act in the physical world. It combines machine learning, computer vision, natural language processing, and control theory to enable robots to understand and interact with their environment in real-time.',
                'source': 'Chapter 1: Introduction to Physical AI',
                'category': 'fundamentals'
            },
            'kinematics': {
                'title': 'Robot Kinematics',
                'content': 'Kinematics is the study of motion without considering the forces that cause it. Forward kinematics calculates the position and orientation of the end-effector given joint angles. Inverse kinematics solves the opposite problem: finding joint angles that achieve a desired end-effector position. Understanding kinematics is fundamental for robot control and motion planning.',
                'source': 'Chapter 4: Kinematics and Dynamics',
                'category': 'control'
            },
            'perception': {
                'title': 'Robot Perception',
                'content': 'Robot perception involves using sensors like cameras, LiDAR, and tactile sensors to understand the environment. Computer vision algorithms process visual data to detect objects, recognize gestures, and build 3D maps. Sensor fusion combines multiple sensor inputs for robust perception. Deep learning models enable robots to recognize complex patterns and objects.',
                'source': 'Chapter 5: Perception and Sensing',
                'category': 'sensing'
            },
            'control': {
                'title': 'Robot Control Systems',
                'content': 'Control systems ensure robots execute desired behaviors accurately. Common control approaches include PID control for simple tasks, model predictive control for complex dynamics, and reinforcement learning for adaptive behavior. Real-time control loops typically run at 50-1000 Hz depending on the application and system requirements.',
                'source': 'Chapter 6: Control Theory for Robotics',
                'category': 'control'
            }
        }

    def _initialize_collection(self):
        """Initialize Qdrant collection for embeddings."""
        try:
            collections = self.qdrant_client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if self.collection_name not in collection_names:
                self.qdrant_client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=1536,  # OpenAI embedding size
                        distance=Distance.COSINE
                    )
                )
                logger.info(f"Created Qdrant collection: {self.collection_name}")
                self._populate_initial_data()
        except Exception as e:
            logger.error(f"Error initializing Qdrant collection: {e}")

    def _populate_initial_data(self):
        """Populate Qdrant with sample textbook content."""
        try:
            for idx, (key, doc) in enumerate(self.sample_content.items()):
                embedding = self._get_embedding(doc['content'])
                point = PointStruct(
                    id=idx,
                    vector=embedding,
                    payload={
                        'title': doc['title'],
                        'content': doc['content'],
                        'source': doc['source'],
                        'category': doc['category'],
                        'key': key
                    }
                )
                self.qdrant_client.upsert(
                    collection_name=self.collection_name,
                    points=[point]
                )
            logger.info(f"Populated {len(self.sample_content)} documents to Qdrant")
        except Exception as e:
            logger.error(f"Error populating initial data: {e}")

    def _get_embedding(self, text: str) -> List[float]:
        """Get embedding for text using OpenAI."""
        try:
            response = self.openai_client.embeddings.create(
                model=self.embedding_model,
                input=text
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Error getting embedding: {e}")
            raise

    def _retrieve_context(self, query: str, user_context: Optional[str] = None) -> List[Dict[str, Any]]:
        """Retrieve relevant documents from Qdrant."""
        try:
            # Get embedding for the query
            query_embedding = self._get_embedding(query)

            # Search Qdrant
            search_result = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=self.top_k
            )

            # Format results
            contexts = []
            for point in search_result:
                if point.score >= self.confidence_threshold:
                    contexts.append({
                        'content': point.payload.get('content', ''),
                        'title': point.payload.get('title', ''),
                        'source': point.payload.get('source', ''),
                        'score': float(point.score),
                        'category': point.payload.get('category', '')
                    })

            return contexts
        except Exception as e:
            logger.error(f"Error retrieving context: {e}")
            return []

    def generate_answer(
        self,
        query: str,
        user_context: Optional[str] = None,
        conversation_history: Optional[List[Dict]] = None
    ) -> Dict[str, Any]:
        """Generate answer using RAG pipeline."""
        try:
            # Retrieve relevant documents
            retrieved_docs = self._retrieve_context(query, user_context)

            if not retrieved_docs:
                return {
                    'answer': 'I could not find relevant information about your query in the textbook. Could you rephrase your question?',
                    'sources': [],
                    'confidence': 0.0,
                    'status': 'no_context'
                }

            # Prepare context
            context_text = "\n\n".join([
                f"Source: {doc['source']}\nTitle: {doc['title']}\nContent: {doc['content']}"
                for doc in retrieved_docs
            ])

            # Build messages
            system_message = """You are an expert assistant for a Physical AI and Humanoid Robotics textbook.
Your role is to answer questions based strictly on the provided textbook content.
If the answer is not in the provided context, clearly state that the information is not available in the textbook.
Be accurate, concise, and helpful. Cite the relevant sections when appropriate."""

            messages = []

            # Add conversation history if available
            if conversation_history:
                for msg in conversation_history:
                    messages.append({
                        'role': msg.get('role', 'user'),
                        'content': msg.get('content', '')
                    })

            # Add context and current query
            messages.append({
                'role': 'user',
                'content': f"""Based on the following textbook content, please answer this question:

TEXTBOOK CONTENT:
{context_text}

{f'USER SELECTED TEXT: {user_context}' if user_context else ''}

QUESTION: {query}"""
            })

            # Get response from OpenAI
            response = self.openai_client.chat.completions.create(
                model=self.chat_model,
                messages=[
                    {'role': 'system', 'content': system_message},
                    *messages
                ],
                temperature=self.temperature,
                max_tokens=500
            )

            answer = response.choices[0].message.content

            # Calculate confidence
            avg_score = sum(doc['score'] for doc in retrieved_docs) / len(retrieved_docs)
            confidence = min(avg_score, 1.0)

            return {
                'answer': answer,
                'sources': [
                    {
                        'title': doc['title'],
                        'source': doc['source'],
                        'category': doc['category'],
                        'score': doc['score']
                    }
                    for doc in retrieved_docs
                ],
                'confidence': float(confidence),
                'confidence_level': 'high' if confidence > 0.8 else 'medium' if confidence > 0.6 else 'low',
                'status': 'success'
            }

        except Exception as e:
            logger.error(f"Error generating answer: {e}")
            return {
                'answer': f'An error occurred while processing your question: {str(e)}',
                'sources': [],
                'confidence': 0.0,
                'status': 'error'
            }

# Global RAG service instance
_rag_service = None

def get_rag_service() -> RAGService:
    """Get or create RAG service instance."""
    global _rag_service
    if _rag_service is None:
        _rag_service = RAGService()
    return _rag_service
