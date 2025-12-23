"""RAG Service using Cohere API - Enhanced with actual chapter content."""

import os
import json
from typing import List, Dict, Any, Optional
import cohere
import logging
from pathlib import Path
import re

logger = logging.getLogger(__name__)

class RAGService:
    """Retrieval-Augmented Generation using Cohere and comprehensive chapter content."""

    def __init__(self):
        self.cohere_api_key = os.getenv('COHERE_API_KEY')
        self.cohere_client = None

        if self.cohere_api_key:
            try:
                self.cohere_client = cohere.ClientV2(api_key=self.cohere_api_key)
                logger.info("Cohere client initialized successfully")
            except Exception as e:
                logger.error(f"Error initializing Cohere client: {e}")
                self.cohere_client = None
        else:
            logger.warning("COHERE_API_KEY not set - will use local responses with Claude")

        self.top_k = int(os.getenv('RETRIEVAL_TOP_K', 5))
        self.temperature = float(os.getenv('RAG_TEMPERATURE', 0.7))

        # Load actual chapter content from files
        self.sample_content = self._load_chapter_content()
        logger.info(f"Loaded {len(self.sample_content)} chapters into RAG service")

    def _load_chapter_content(self) -> Dict[str, Dict[str, str]]:
        """Load actual chapter content from markdown files."""
        chapters = {}

        # Try multiple possible paths for chapter files
        possible_paths = [
            Path('/app/frontend/docs/modules'),  # Production
            Path('./frontend/docs/modules'),      # Local dev
            Path('../frontend/docs/modules'),     # Alternative local
        ]

        chapter_dir = None
        for path in possible_paths:
            if path.exists():
                chapter_dir = path
                logger.info(f"Found chapters at {chapter_dir}")
                break

        if not chapter_dir:
            logger.warning("Chapter directory not found, using default sample content")
            return self._get_default_content()

        try:
            # Find all chapter markdown files
            chapter_files = sorted(chapter_dir.glob('**/chapter-*.md'))
            logger.info(f"Found {len(chapter_files)} chapter files")

            for chapter_file in chapter_files:
                try:
                    with open(chapter_file, 'r', encoding='utf-8') as f:
                        content = f.read()

                    # Extract title from markdown
                    title_match = re.search(r'title:\s*"([^"]+)"', content)
                    title = title_match.group(1) if title_match else chapter_file.stem

                    # Extract first 2000 chars of meaningful content (skip frontmatter)
                    parts = content.split('---')
                    body = parts[2] if len(parts) > 2 else content

                    # Remove markdown headers and clean up
                    body = re.sub(r'^#+\s+', '', body, flags=re.MULTILINE)
                    body = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', body)  # Remove links

                    # Get first meaningful paragraphs
                    text_content = ' '.join([p.strip() for p in body.split('\n') if p.strip() and not p.startswith('-')])[:2000]

                    module_name = chapter_file.parent.name
                    chapters[chapter_file.stem] = {
                        'title': title,
                        'content': text_content,
                        'source': f'{module_name}/{chapter_file.stem}',
                        'category': module_name.replace('module-', 'Module ')
                    }
                    logger.info(f"Loaded chapter: {title}")
                except Exception as e:
                    logger.warning(f"Error loading chapter {chapter_file}: {e}")

            if chapters:
                return chapters
            else:
                return self._get_default_content()
        except Exception as e:
            logger.error(f"Error loading chapters: {e}")
            return self._get_default_content()

    def _get_default_content(self) -> Dict[str, Dict[str, str]]:
        """Fallback to default sample content."""
        return {
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

    def _retrieve_context(self, query: str, user_context: Optional[str] = None) -> List[Dict[str, Any]]:
        """Retrieve relevant documents using phrase matching and synonyms."""
        try:
            query_lower = query.lower()

            # Synonym mapping for better matching
            synonyms = {
                'robot operating system': 'ros',
                'ros2': 'ros',
                'ros 2': 'ros',
                'kinematics': ['forward kinematics', 'inverse kinematics'],
                'perception': ['sensor', 'vision', 'camera'],
                'control': ['controller', 'actuator'],
                'humanoid': ['biped', 'bipedal'],
            }

            # Expand query with synonyms
            expanded_query = query_lower
            for key, value in synonyms.items():
                if key in query_lower:
                    if isinstance(value, list):
                        expanded_query += ' ' + ' '.join(value)
                    else:
                        expanded_query += ' ' + value

            query_words = [w.strip('?.,!;:') for w in expanded_query.split() if len(w.strip('?.,!;:')) > 2]

            if not query_words:
                logger.warning("Query has no meaningful words")
                return []

            matched_docs = []

            # Score each document
            for key, doc in self.sample_content.items():
                doc_text = (doc['title'] + ' ' + doc['content']).lower()
                doc_title_lower = doc['title'].lower()

                score = 0.0
                title_matches = 0

                # CRITICAL: Check exact phrase match in title first
                if 'robot operating system' in query_lower or 'ros' in query_lower:
                    if 'introduction' in doc_title_lower and 'ros' in doc_title_lower:
                        score += 2.0  # Highest priority for ROS chapters
                        title_matches += 5

                # Check for phrase match in title (not just words)
                for word in query_words:
                    if word in doc_title_lower:
                        title_matches += 1
                        score += 0.5  # Higher weight for title matches

                # Content match (lower weight)
                for word in query_words:
                    word_count = doc_text.count(word)
                    if word_count > 0:
                        score += min(0.1, word_count * 0.01)

                # Add document if there's relevant match
                if score > 0:
                    matched_docs.append({
                        'content': doc['content'],
                        'title': doc['title'],
                        'source': doc['source'],
                        'score': min(score, 1.0),
                        'category': doc['category'],
                        'title_matches': title_matches
                    })

            if not matched_docs:
                logger.warning(f"No good matches for query: {query}")
                return []

            # Sort by title_matches first (most important), then by score
            matched_docs.sort(key=lambda x: (x['title_matches'], x['score']), reverse=True)

            # Remove temporary field
            for doc in matched_docs:
                doc.pop('title_matches', None)

            return matched_docs[:self.top_k]

        except Exception as e:
            logger.error(f"Error retrieving context: {e}")
            return []

    def _is_greeting(self, query: str) -> bool:
        """Detect if query is a greeting."""
        greetings = ['hi', 'hello', 'hey', 'greetings', 'good morning', 'good afternoon',
                     'good evening', 'howdy', 'what\'s up', 'sup', 'namaste', 'salam']
        query_lower = query.lower().strip()
        return any(greeting in query_lower for greeting in greetings)

    def _get_greeting_response(self) -> Dict[str, Any]:
        """Generate a friendly greeting response."""
        responses = [
            "Hello! I'm your Physical AI & Humanoid Robotics textbook assistant. How can I help you today? Feel free to ask me questions about ROS 2, humanoid robots, kinematics, perception, or any other topic from the textbook.",
            "Hi there! Welcome to the Physical AI & Humanoid Robotics textbook assistant. What would you like to know about robotics, AI, or humanoid systems?",
            "Greetings! I'm here to help you learn about Physical AI and Humanoid Robotics. Ask me anything about the textbook chapters!",
        ]
        import random
        return {
            'answer': random.choice(responses),
            'sources': [],
            'confidence': 1.0,
            'confidence_level': 'high',
            'status': 'greeting'
        }

    def generate_answer(
        self,
        query: str,
        user_context: Optional[str] = None,
        conversation_history: Optional[List[Dict]] = None
    ) -> Dict[str, Any]:
        """Generate answer using Cohere and local retrieval."""
        try:
            # Check if it's a greeting first
            if self._is_greeting(query):
                return self._get_greeting_response()

            # Retrieve relevant documents
            retrieved_docs = self._retrieve_context(query, user_context)

            if not retrieved_docs:
                return {
                    'answer': 'I could not find relevant information about your query in the textbook. Could you rephrase your question?',
                    'sources': [],
                    'confidence': 0.0,
                    'confidence_level': 'low',
                    'status': 'no_context'
                }

            # Prepare context
            context_text = "\n\n".join([
                f"Source: {doc['source']}\nTitle: {doc['title']}\nContent: {doc['content']}"
                for doc in retrieved_docs
            ])

            # Build prompt - optimized for concise answers
            system_prompt = """You are a concise expert assistant for a Physical AI and Humanoid Robotics textbook.
IMPORTANT - Keep answers SHORT and TO-THE-POINT:
- Answer in 2-4 sentences maximum
- Only include essential information
- Be direct and clear
- Cite the source chapter if relevant
- Use simple language
If information is not in the provided context, say so clearly."""

            user_prompt = f"""Based on the following textbook content, answer this question BRIEFLY (2-4 sentences):

TEXTBOOK CONTENT:
{context_text}

{f'USER CONTEXT: {user_context}' if user_context else ''}

QUESTION: {query}

Answer (keep it SHORT and TO-THE-POINT):"""

            # Use Cohere if available
            if self.cohere_client:
                try:
                    response = self.cohere_client.chat(
                        model="command-r-plus",
                        messages=[
                            {"role": "user", "content": user_prompt}
                        ],
                        system=system_prompt,
                        temperature=self.temperature,
                        max_tokens=500
                    )
                    answer = response.message.content[0].text
                except Exception as e:
                    logger.warning(f"Cohere API failed: {e}, using fallback")
                    answer = self._generate_fallback_answer(query, retrieved_docs, context_text)
            else:
                logger.warning("Cohere client not available, using fallback")
                answer = self._generate_fallback_answer(query, retrieved_docs, context_text)

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
                'confidence_level': 'low',
                'status': 'error'
            }

    def _clean_text(self, text: str) -> str:
        """Clean markdown and metadata from text."""
        import re
        # Remove markdown headers
        text = re.sub(r'^#+\s+', '', text, flags=re.MULTILINE)
        # Remove bold/italic markdown
        text = re.sub(r'\*\*([^*]+)\*\*', r'\1', text)
        text = re.sub(r'\*([^*]+)\*', r'\1', text)
        # Remove metadata lines like "**Week 3 of 13 | Estimated Time..."
        text = re.sub(r'\*\*Week.*?\*\*', '', text)
        text = re.sub(r'Week \d+ of \d+.*?hours', '', text, flags=re.IGNORECASE)
        # Remove extra whitespace
        text = ' '.join(text.split())
        return text.strip()

    def _generate_fallback_answer(self, query: str, docs: List[Dict], context: str) -> str:
        """Generate a concise fallback answer - clean, to-the-point, no raw content."""
        if not docs:
            return f"I don't have information about '{query}' in the textbook. Try asking about ROS 2, Humanoid Robots, Kinematics, Perception, or Control."

        # Try using Claude API first
        try:
            from anthropic import Anthropic

            client = Anthropic()

            # Use only top document for conciseness
            context_text = "\n\n".join([
                f"{doc['title']}: {doc['content']}"
                for doc in docs[:2]
            ])

            system_prompt = """You are a concise robotics tutor. Answer questions using provided textbook content.
STRICT RULES:
- Answer in 2-3 sentences ONLY
- No fluff, no extra details
- Be direct and clear
- Cite chapter name if helpful"""

            user_message = f"""Answer this question in 2-3 sentences maximum:

TEXTBOOK CONTENT:
{context_text}

QUESTION: {query}"""

            response = client.messages.create(
                model="claude-3-5-sonnet-20241022",
                max_tokens=150,
                system=system_prompt,
                messages=[
                    {"role": "user", "content": user_message}
                ]
            )

            return response.content[0].text

        except Exception as e:
            logger.warning(f"Claude API not available: {e}")

            # Fallback: Clean extraction from top document - concise and readable
            if docs:
                first_doc = docs[0]
                # Clean the content
                cleaned = self._clean_text(first_doc['content'])
                # Get just first 2 sentences (roughly 100 words)
                sentences = cleaned.split('.')
                answer_sentences = []
                for sent in sentences[:3]:  # Take up to 3 sentence fragments
                    sent = sent.strip()
                    if sent and len(sent) > 10:  # Skip very short fragments
                        answer_sentences.append(sent)
                    if len(' '.join(answer_sentences)) > 150:  # Stop if we have enough
                        break

                if answer_sentences:
                    answer = '. '.join(answer_sentences[:2]) + '.'
                    return answer
                else:
                    return cleaned[:150] + '...'
            else:
                return "I don't have information about that topic in the textbook."


# Global RAG service instance
_rag_service = None

def get_rag_service() -> RAGService:
    """Get or create RAG service instance."""
    global _rag_service
    if _rag_service is None:
        _rag_service = RAGService()
    return _rag_service
