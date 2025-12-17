"""Personalization service - Customize content based on user background."""

import logging
from typing import Dict, Any, List

logger = logging.getLogger(__name__)

class PersonalizationService:
    """Service to personalize RAG responses based on user background."""

    def __init__(self):
        self.software_level_prompts = {
            "beginner": "Explain in simple terms with basic concepts and examples",
            "intermediate": "Provide moderate level explanation with some technical details",
            "advanced": "Use technical terminology and dive into implementation details",
            "expert": "Provide cutting-edge insights and research-level details"
        }

        self.learning_style_guidance = {
            "visual": "Use diagrams, flowcharts, and visual representations",
            "text": "Provide comprehensive text-based explanations",
            "interactive": "Include code examples and hands-on exercises",
            "hands-on": "Focus on practical implementation and real-world examples"
        }

        # Content complexity mapping
        self.complexity_mapping = {
            "beginner": {
                "depth": "surface",
                "examples": 3,
                "code_snippets": 0,
                "mathematical_rigor": "minimal"
            },
            "intermediate": {
                "depth": "moderate",
                "examples": 4,
                "code_snippets": 1,
                "mathematical_rigor": "basic"
            },
            "advanced": {
                "depth": "deep",
                "examples": 5,
                "code_snippets": 2,
                "mathematical_rigor": "rigorous"
            },
            "expert": {
                "depth": "comprehensive",
                "examples": 6,
                "code_snippets": 3,
                "mathematical_rigor": "highly_rigorous"
            }
        }

    def build_personalized_prompt(self, user_profile: Dict[str, Any], base_query: str) -> str:
        """Build a personalized system prompt based on user background."""

        software_level = user_profile.get("software_background", "beginner")
        hardware_level = user_profile.get("hardware_background", "beginner")
        learning_style = user_profile.get("preferred_learning_style", "text")
        interest_areas = user_profile.get("interest_areas", [])
        profession = user_profile.get("profession", "")

        # Build personalized instruction
        personalized_prompt = f"""You are a Physical AI & Humanoid Robotics textbook assistant, customized for this user:

User Profile:
- Software Background: {software_level.title()}
- Hardware Background: {hardware_level.title()}
- Learning Style: {learning_style.title()}
- Interest Areas: {", ".join(interest_areas) if interest_areas else "General"}
- Professional Background: {profession if profession else "Not specified"}

Instructions:
1. {self.software_level_prompts.get(software_level, self.software_level_prompts["beginner"])}
2. {self.learning_style_guidance.get(learning_style, self.learning_style_guidance["text"])}
3. If user is interested in specific areas ({", ".join(interest_areas) if interest_areas else "general topics"}), prioritize those
4. Answer based strictly on the provided textbook content
5. Use appropriate depth of explanation based on the user's level
6. If the answer is not in the textbook, clearly state that

Current Query: {base_query}"""

        return personalized_prompt

    def get_recommended_content(self, user_profile: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Get recommended content based on user background."""

        recommendations = []
        interest_areas = user_profile.get("interest_areas", [])
        software_level = user_profile.get("software_background", "beginner")

        # Recommend chapters based on interests
        chapter_recommendations = {
            "ROS": {
                "chapter": "Chapter 2: Foundations of Physical AI",
                "reason": "Perfect for learning Robot Operating System fundamentals",
                "difficulty": "intermediate"
            },
            "Kinematics": {
                "chapter": "Chapter 4: Kinematics and Dynamics",
                "reason": "Essential for understanding robot motion and control",
                "difficulty": "advanced"
            },
            "Control": {
                "chapter": "Chapter 6: Control Theory for Robotics",
                "reason": "Learn control systems for robot movements",
                "difficulty": "advanced"
            },
            "Perception": {
                "chapter": "Chapter 5: Perception and Sensing",
                "reason": "Understand how robots see and sense the world",
                "difficulty": "intermediate"
            },
            "Humanoid": {
                "chapter": "Chapter 3: Humanoid Robot Architectures",
                "reason": "Deep dive into humanoid robot design and control",
                "difficulty": "advanced"
            }
        }

        for area in interest_areas:
            if area in chapter_recommendations:
                recommendations.append(chapter_recommendations[area])

        return recommendations

    def get_personalization_context(self, user_profile: Dict[str, Any]) -> Dict[str, Any]:
        """Get full personalization context for RAG system."""

        software_level = user_profile.get("software_background", "beginner")
        complexity_config = self.complexity_mapping.get(software_level, self.complexity_mapping["beginner"])

        return {
            "user_id": user_profile.get("id"),
            "software_level": software_level,
            "hardware_level": user_profile.get("hardware_background", "beginner"),
            "learning_style": user_profile.get("preferred_learning_style", "text"),
            "complexity_config": complexity_config,
            "interest_areas": user_profile.get("interest_areas", []),
            "recommended_content": self.get_recommended_content(user_profile),
            "personalized_system_prompt": self.build_personalized_prompt(user_profile, ""),
            "learning_pace": user_profile.get("learning_pace", "moderate")
        }

# Global service instance
_personalization_service = None

def get_personalization_service() -> PersonalizationService:
    """Get or create personalization service instance."""
    global _personalization_service
    if _personalization_service is None:
        _personalization_service = PersonalizationService()
    return _personalization_service
