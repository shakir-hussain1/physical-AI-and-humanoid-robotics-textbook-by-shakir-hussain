"""
Translation Service - Core translation logic and caching

Features:
- HTML-aware translation (preserves structure)
- Multiple language support
- Translation caching with expiry
- Confidence scoring
- Graceful fallback on errors
"""

import re
import logging
import asyncio
from typing import Dict, Any, List, Optional
from datetime import datetime, timedelta
from bs4 import BeautifulSoup
import json
import hashlib

logger = logging.getLogger(__name__)

# Supported languages
SUPPORTED_LANGUAGES = {
    'urdu': {'name': 'Urdu', 'native_name': 'اردو', 'code': 'ur'},
    'spanish': {'name': 'Spanish', 'native_name': 'Español', 'code': 'es'},
    'french': {'name': 'French', 'native_name': 'Français', 'code': 'fr'},
    'arabic': {'name': 'Arabic', 'native_name': 'العربية', 'code': 'ar'},
    'hindi': {'name': 'Hindi', 'native_name': 'हिंदी', 'code': 'hi'},
}

# In-memory cache - can be replaced with Redis
translation_cache = {}


class TranslationService:
    """
    Translation service for converting chapter content to different languages

    Responsibilities:
    - Parse HTML content while preserving structure
    - Translate text portions
    - Cache translations
    - Handle errors gracefully
    """

    def __init__(self):
        """Initialize translation service"""
        self.cache_expiry_hours = 24
        self.confidence_threshold = 0.8

    async def translate(
        self,
        content: str,
        target_language: str,
        chapter_id: str,
        user_id: int
    ) -> Dict[str, Any]:
        """
        Translate HTML content to target language

        Process:
        1. Check cache first
        2. Parse HTML to preserve structure
        3. Extract translatable text
        4. Translate text portions
        5. Reconstruct HTML with translations
        6. Cache result
        7. Return translated content

        Args:
            content: HTML content to translate
            target_language: Target language code
            chapter_id: Chapter identifier for caching
            user_id: User ID for logging

        Returns:
            Dict with translated_content, confidence_score, from_cache
        """
        try:
            # Check cache first
            cached_result = self._get_cached_translation(chapter_id, target_language)
            if cached_result:
                logger.info(
                    f"Translation cache hit for chapter {chapter_id} "
                    f"to {target_language}"
                )
                return {
                    **cached_result,
                    'from_cache': True
                }

            logger.info(f"Translating content for chapter {chapter_id} to {target_language}")

            # Validate target language
            if target_language not in SUPPORTED_LANGUAGES:
                raise ValueError(f"Unsupported language: {target_language}")

            # Parse HTML and translate
            translated_content = await self._translate_html_content(
                content,
                target_language
            )

            # Calculate confidence score
            confidence_score = await self._calculate_confidence(
                content,
                translated_content
            )

            result = {
                'translated_content': translated_content,
                'confidence_score': confidence_score,
                'from_cache': False
            }

            # Cache the result
            self._cache_translation(chapter_id, target_language, result)

            logger.info(
                f"Successfully translated chapter {chapter_id} "
                f"with confidence score {confidence_score}"
            )

            return result

        except Exception as e:
            logger.error(f"Error translating content: {e}")
            raise

    async def _translate_html_content(
        self,
        html_content: str,
        target_language: str
    ) -> str:
        """
        Parse HTML and translate text portions while preserving structure

        Process:
        1. Parse HTML using BeautifulSoup
        2. Identify text nodes to translate
        3. Translate each text portion
        4. Reconstruct HTML
        5. Return translated HTML

        Args:
            html_content: HTML content to translate
            target_language: Target language code

        Returns:
            Translated HTML content
        """
        try:
            # Parse HTML
            soup = BeautifulSoup(html_content, 'html.parser')

            # Translate all text nodes
            self._translate_soup_recursive(soup, target_language)

            # Return translated HTML
            return str(soup)

        except Exception as e:
            logger.error(f"Error translating HTML: {e}")
            raise

    def _translate_soup_recursive(
        self,
        element,
        target_language: str
    ):
        """
        Recursively translate text nodes in BeautifulSoup element tree

        Args:
            element: BeautifulSoup element
            target_language: Target language code
        """
        for child in element.children:
            # Handle text nodes
            if isinstance(child, str):
                # Translate text while preserving whitespace
                text = child.strip()
                if text and len(text) > 3:  # Only translate meaningful text
                    translated = self._translate_text(text, target_language)
                    # Replace text in soup (complex due to soup structure)
                    child.replace_with(translated)
            # Recursively handle elements
            elif hasattr(child, 'children'):
                # Skip code blocks, scripts, etc.
                if child.name not in ['code', 'pre', 'script', 'style']:
                    self._translate_soup_recursive(child, target_language)

    def _translate_text(self, text: str, target_language: str) -> str:
        """
        Translate a text portion to target language

        This is the main translation function that can be replaced
        with actual translation API (Google Translate, Azure, etc.)

        Args:
            text: Text to translate
            target_language: Target language code

        Returns:
            Translated text
        """
        try:
            # Mock translation using simple dictionary lookup
            # In production, replace with actual translation API
            translations = self._get_mock_translations(target_language)

            # Simple word-by-word translation (mock)
            words = text.split()
            translated_words = []

            for word in words:
                # Clean word (remove punctuation)
                clean_word = re.sub(r'[^\w\s]', '', word.lower())

                # Look up translation
                if clean_word in translations:
                    # Replace word with translation, preserving punctuation
                    translated = translations[clean_word]
                    # Re-add punctuation
                    if word != clean_word:
                        punct = word.replace(clean_word, '')
                        translated = translated + punct
                    translated_words.append(translated)
                else:
                    # Keep original word if no translation found
                    translated_words.append(word)

            return ' '.join(translated_words)

        except Exception as e:
            logger.error(f"Error translating text: {e}")
            return text  # Return original text on error

    def _get_mock_translations(self, target_language: str) -> Dict[str, str]:
        """
        Get mock translations for demo purposes

        In production, replace with actual translation API call
        Examples: Google Translate, Azure Translator, AWS Translate, etc.

        Args:
            target_language: Target language code

        Returns:
            Dictionary of word translations
        """
        mock_translations = {
            'urdu': {
                # Technical terms
                'chapter': 'باب',
                'introduction': 'تعارف',
                'physics': 'طبیعیات',
                'robotics': 'روبوٹکس',
                'artificial': 'مصنوعی',
                'intelligence': 'ذہانت',
                'learning': 'سیکھنا',
                'machine': 'مشین',
                'system': 'نظام',
                'computer': 'کمپیوٹر',
                'algorithm': 'الگورتھم',
                'program': 'پروگرام',
                'code': 'کوڈ',
                'function': 'فنکشن',
                'variable': 'متغیر',
                'data': 'ڈیٹا',
                'network': 'نیٹ ورک',
                'sensor': 'سینسر',
                'motor': 'موٹر',
                'control': 'کنٹرول',
                'robot': 'روبوٹ',
                'hardware': 'سخت ویئر',
                'software': 'سافٹ ویئر',
                'process': 'عمل',
                'method': 'طریقہ',
                'design': 'ڈیزائن',
                'framework': 'ڈھانچہ',
                'interface': 'رابطہ',
                'model': 'ماڈل',
                'test': 'ٹیسٹ',

                # Common words
                'the': 'یہ',
                'a': 'ایک',
                'an': 'ایک',
                'is': 'ہے',
                'are': 'ہیں',
                'be': 'ہو',
                'was': 'تھا',
                'were': 'تھے',
                'been': 'گیا',
                'being': 'جا رہا',
                'have': 'ہے',
                'has': 'ہے',
                'do': 'کریں',
                'does': 'کرتا',
                'did': 'کیا',
                'will': 'ہوگا',
                'would': 'ہوتا',
                'could': 'سکتے',
                'should': 'چاہیے',
                'can': 'سکتے',
                'in': 'میں',
                'on': 'پر',
                'at': 'میں',
                'to': 'کو',
                'for': 'کے لیے',
                'of': 'کا',
                'with': 'کے ساتھ',
                'by': 'کی طرف',
                'from': 'سے',
                'and': 'اور',
                'or': 'یا',
                'but': 'لیکن',
                'as': 'جیسے',
                'if': 'اگر',
                'then': 'پھر',
                'what': 'کیا',
                'which': 'کون',
                'who': 'کون',
                'when': 'کب',
                'where': 'کہاں',
                'why': 'کیوں',
                'how': 'کیسے',
                'not': 'نہیں',
                'no': 'نہیں',
                'yes': 'جی ہاں',
                'all': 'تمام',
                'each': 'ہر',
                'every': 'ہر',
                'some': 'کچھ',
                'any': 'کوئی',

                # Verbs
                'create': 'بنائیں',
                'make': 'بنائیں',
                'build': 'تعمیر کریں',
                'develop': 'تیار کریں',
                'implement': 'نافذ کریں',
                'use': 'استعمال کریں',
                'apply': 'لاگو کریں',
                'run': 'چلائیں',
                'start': 'شروع کریں',
                'stop': 'روکیں',
                'define': 'متعین کریں',
                'describe': 'بیان کریں',
                'explain': 'وضاحت کریں',
                'understand': 'سمجھیں',
                'learn': 'سیکھیں',
                'teach': 'سکھائیں',
                'show': 'دکھائیں',
                'work': 'کام کریں',
                'perform': 'انجام دیں',
                'execute': 'نافذ کریں',
                'handle': 'سنبھالیں',
                'manage': 'منظم کریں',
                'monitor': 'نگرانی کریں',
                'check': 'دیکھیں',
                'verify': 'تصدیق کریں',

                # Adjectives
                'new': 'نیا',
                'old': 'پرانا',
                'good': 'اچھا',
                'bad': 'برا',
                'large': 'بڑا',
                'small': 'چھوٹا',
                'high': 'اونچا',
                'low': 'نیچا',
                'simple': 'سادہ',
                'complex': 'پیچیدہ',
                'different': 'مختلف',
                'same': 'ایک جیسا',
                'important': 'اہم',
                'main': 'مرکزی',
                'basic': 'بنیادی',
                'advanced': 'ترقی یافتہ',
                'first': 'پہلا',
                'last': 'آخری',
                'next': 'اگلا',
                'previous': 'پچھلا',
                'real': 'اصل',
                'virtual': 'مجازی',
                'physical': 'جسمانی',
                'digital': 'ڈیجیٹل',

                # Nouns
                'result': 'نتیجہ',
                'example': 'مثال',
                'input': 'ان پٹ',
                'output': 'آؤٹ پٹ',
                'process': 'عمل',
                'step': 'قدم',
                'way': 'طریقہ',
                'time': 'وقت',
                'year': 'سال',
                'day': 'دن',
                'hour': 'گھنٹہ',
                'minute': 'منٹ',
                'second': 'سیکنڈ',
                'number': 'نمبر',
                'value': 'قیمت',
                'type': 'قسم',
                'kind': 'قسم',
                'set': 'سیٹ',
                'list': 'فہرست',
                'table': 'ٹیبل',
                'form': 'شکل',
                'line': 'لائن',
                'point': 'نقطہ',
                'area': 'علاقہ',
                'part': 'حصہ',
                'piece': 'ٹکڑا',
                'section': 'حصہ',
                'page': 'صفحہ',
                'book': 'کتاب',
                'text': 'متن',
                'content': 'مواد',
                'information': 'معلومات',
                'knowledge': 'علم',
                'message': 'پیغام',
                'error': 'خرابی',
                'warning': 'انتباہ',
                'success': 'کامیابی',
                'failure': 'ناکامی',
                'problem': 'مسئلہ',
                'solution': 'حل',
                'question': 'سوال',
                'answer': 'جواب',
                'task': 'کام',
                'job': 'کام',
                'goal': 'مقصد',
                'purpose': 'مقصد',
                'reason': 'وجہ',
                'cause': 'وجہ',
                'effect': 'اثر',
                'change': 'تبدیلی',
                'difference': 'فرق',
                'similarity': 'مماثلت',
                'connection': 'تعلق',
                'relationship': 'رشتہ',
                'link': 'لنک',
                'path': 'راستہ',
                'direction': 'سمت',
                'position': 'پوزیشن',
                'location': 'مقام',
                'state': 'حالت',
                'condition': 'شرط',
                'situation': 'صورتحال',
                'case': 'معاملہ',
                'scenario': 'منظر نامہ',
                'context': 'سیاق',
                'background': 'پس منظر',
                'source': 'ذریعہ',
                'target': 'ہدف',
                'reference': 'حوالہ',
                'resource': 'وسیلہ',
                'tool': 'آلہ',
                'method': 'طریقہ',
                'technique': 'تکنیک',
                'approach': 'نقطہ نظر',
                'strategy': 'حکمت عملی',
                'plan': 'منصوبہ',
                'schedule': 'شیڈول',
                'timeline': 'ٹائم لائن',
                'version': 'ورژن',
                'release': 'رہائی',
                'update': 'اپ ڈیٹ',
                'feature': 'خصوصیت',
                'option': 'اختیار',
                'setting': 'ترتیب',
                'parameter': 'پیرامیٹر',
                'configuration': 'ترتیب',
                'requirement': 'ضرورت',
                'specification': 'تفصیل',
                'documentation': 'دستاویز',
                'guide': 'رہنما',
                'manual': 'دستی',
                'tutorial': 'ٹیوٹوریل',
                'example': 'مثال',
                'demonstration': 'مظاہرہ',
                'sample': 'نمونہ',
                'template': 'سانچہ',
                'pattern': 'نمونہ',
                'rule': 'اصول',
                'standard': 'معیار',
                'convention': 'روایت',
                'practice': 'رویہ',
                'habit': 'عادت',
                'behavior': 'رویہ',
                'activity': 'سرگرمی',
                'action': 'عمل',
                'reaction': 'رد عمل',
                'interaction': 'تعامل',
                'communication': 'رابطہ',
                'exchange': 'تبادلہ',
                'transfer': 'منتقلی',
                'flow': 'بہاؤ',
                'stream': 'بہاؤ',
                'channel': 'چینل',
                'medium': 'ذریعہ',
                'format': 'فارمیٹ',
                'structure': 'ڈھانچہ',
                'organization': 'تنظیم',
                'arrangement': 'ترتیب',
                'layout': 'بندوبست',
                'design': 'ڈیزائن',
                'pattern': 'نمونہ',
                'style': 'انداز',
            },
            'spanish': {
                'chapter': 'capítulo',
                'introduction': 'introducción',
                'physics': 'física',
                'robotics': 'robótica',
                'artificial': 'artificial',
                'intelligence': 'inteligencia',
                'learning': 'aprendizaje',
                'machine': 'máquina',
                'system': 'sistema',
                'computer': 'computadora',
                'algorithm': 'algoritmo',
                'program': 'programa',
                'code': 'código',
                'function': 'función',
                'variable': 'variable',
                'data': 'datos',
                'network': 'red',
                'sensor': 'sensor',
                'motor': 'motor',
                'control': 'control',
            }
        }

        return mock_translations.get(target_language, {})

    async def _calculate_confidence(
        self,
        original: str,
        translated: str
    ) -> float:
        """
        Calculate confidence score for translation quality

        In production, use actual translation quality metrics

        Args:
            original: Original text
            translated: Translated text

        Returns:
            Confidence score (0-1)
        """
        try:
            # Simple heuristics for mock confidence
            if not translated or len(translated) == 0:
                return 0.0

            # If translation length is reasonable (0.5x to 2x original)
            ratio = len(translated) / max(len(original), 1)
            if 0.5 <= ratio <= 2.0:
                confidence = 0.95
            else:
                confidence = 0.7

            return confidence

        except Exception as e:
            logger.error(f"Error calculating confidence: {e}")
            return 0.8  # Default confidence on error

    def _get_cached_translation(
        self,
        chapter_id: str,
        target_language: str
    ) -> Optional[Dict[str, Any]]:
        """
        Get cached translation if available and not expired

        Args:
            chapter_id: Chapter identifier
            target_language: Target language

        Returns:
            Cached translation dict or None
        """
        try:
            cache_key = f"{chapter_id}_{target_language}"

            if cache_key in translation_cache:
                cached = translation_cache[cache_key]

                # Check expiry
                cached_time = datetime.fromisoformat(cached['timestamp'])
                if datetime.utcnow() - cached_time < timedelta(hours=self.cache_expiry_hours):
                    return cached['data']
                else:
                    # Remove expired cache
                    del translation_cache[cache_key]
                    logger.info(f"Removed expired cache for {cache_key}")

        except Exception as e:
            logger.error(f"Error retrieving cache: {e}")

        return None

    def _cache_translation(
        self,
        chapter_id: str,
        target_language: str,
        result: Dict[str, Any]
    ):
        """
        Cache translation result

        Args:
            chapter_id: Chapter identifier
            target_language: Target language
            result: Translation result to cache
        """
        try:
            cache_key = f"{chapter_id}_{target_language}"
            translation_cache[cache_key] = {
                'data': result,
                'timestamp': datetime.utcnow().isoformat()
            }
            logger.info(f"Cached translation for {cache_key}")

        except Exception as e:
            logger.error(f"Error caching translation: {e}")

    async def clear_cache(self, chapter_id: str):
        """
        Clear cached translations for a chapter

        Args:
            chapter_id: Chapter identifier
        """
        try:
            keys_to_delete = [
                k for k in translation_cache.keys()
                if k.startswith(chapter_id)
            ]

            for key in keys_to_delete:
                del translation_cache[key]

            logger.info(f"Cleared {len(keys_to_delete)} cache entries for {chapter_id}")

        except Exception as e:
            logger.error(f"Error clearing cache: {e}")
            raise

    async def get_stats(self, chapter_id: str) -> Dict[str, Any]:
        """
        Get translation statistics for a chapter

        Args:
            chapter_id: Chapter identifier

        Returns:
            Translation statistics
        """
        try:
            languages = [
                k.split('_')[1] for k in translation_cache.keys()
                if k.startswith(chapter_id)
            ]

            return {
                'languages_translated': list(set(languages)),
                'total_translations': len(languages),
                'cache_hit_rate': len(set(languages)) / len(SUPPORTED_LANGUAGES) if SUPPORTED_LANGUAGES else 0,
                'last_translated_at': datetime.utcnow().isoformat()
            }

        except Exception as e:
            logger.error(f"Error getting stats: {e}")
            return {
                'languages_translated': [],
                'total_translations': 0,
                'cache_hit_rate': 0,
                'last_translated_at': None
            }

    def get_supported_languages(self) -> List[Dict[str, str]]:
        """
        Get list of supported languages

        Returns:
            List of language info dicts
        """
        return [
            {
                'code': code,
                'name': info['name'],
                'native_name': info['native_name']
            }
            for code, info in SUPPORTED_LANGUAGES.items()
        ]

    def get_supported_languages_codes(self) -> List[str]:
        """
        Get list of supported language codes

        Returns:
            List of language codes
        """
        return list(SUPPORTED_LANGUAGES.keys())
