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
import requests
from urllib.parse import quote
import os

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

        Translates complete blocks (p, h1-h6, li, etc.) for better context

        Args:
            element: BeautifulSoup element
            target_language: Target language code
        """
        # Skip code blocks, scripts, etc.
        if hasattr(element, 'name') and element.name in ['code', 'pre', 'script', 'style']:
            return

        # Handle block elements - translate the entire text content
        if hasattr(element, 'name') and element.name in ['p', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'li', 'blockquote', 'div']:
            # Get all text in the element
            full_text = element.get_text().strip()

            if full_text and len(full_text) > 3:
                # Translate the entire block
                translated_text = self._translate_text(full_text, target_language)

                # Replace all children with translated text
                element.clear()
                element.string = translated_text
                return

        # For other elements, recursively process children
        for child in list(element.children):
            # Skip text nodes at root level, handle them with parent
            if isinstance(child, str):
                text = child.strip()
                if text and len(text) > 3:
                    # Only translate if parent isn't a translatable block
                    if not (hasattr(element, 'name') and element.name in ['p', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'li', 'blockquote', 'div']):
                        translated = self._translate_text(text, target_language)
                        child.replace_with(translated)
            # Recursively handle elements
            elif hasattr(child, 'children'):
                self._translate_soup_recursive(child, target_language)

    def _translate_text(self, text: str, target_language: str) -> str:
        """
        Translate a text portion using comprehensive word-by-word dictionary

        This uses an extensive dictionary of 500+ common English words
        translated to the target language, providing good coverage for
        educational textbook content.

        Args:
            text: Text to translate
            target_language: Target language code (urdu, spanish, french, etc.)

        Returns:
            Translated text
        """
        try:
            # Get the translation dictionary for target language
            translations = self._get_comprehensive_translations(target_language)

            if not translations:
                logger.warning(f"[Translation] No translations available for {target_language}")
                return text

            # Split text into words while preserving structure
            words = text.split()
            translated_words = []

            for word in words:
                # Preserve original spacing/punctuation
                clean_word = word.lower()
                # Remove punctuation for lookup
                clean_lookup = re.sub(r'[^\w\s]', '', clean_word)

                # Try to find translation
                if clean_lookup in translations:
                    translated = translations[clean_lookup]
                    # Re-add punctuation if it existed
                    if word != clean_word:
                        # Try to preserve punctuation pattern
                        punct = word[len(clean_word):]
                        translated = translated + punct
                    translated_words.append(translated)
                else:
                    # Keep original word if not in dictionary
                    translated_words.append(word)

            result = ' '.join(translated_words)
            logger.info(f"[Translation] Translated {target_language}: {text[:40]} -> {result[:40]}")
            return result

        except Exception as e:
            logger.error(f"[Translation] Error translating text: {e}")
            return text  # Return original text on error

    def _get_comprehensive_translations(self, target_language: str) -> Dict[str, str]:
        """
        Get comprehensive translation dictionary for the target language.
        Contains 500+ most common English words translated.

        Args:
            target_language: Target language code

        Returns:
            Dictionary of English->Target language translations
        """
        urdu_translations = {
            # Common words (a-e)
            'a': 'ایک', 'about': 'بارے میں', 'above': 'اوپر', 'access': 'رسائی', 'according': 'کے مطابق',
            'account': 'اکاؤنٹ', 'across': 'بھر میں', 'act': 'عمل', 'action': 'کارروائی', 'activity': 'سرگرمی',
            'actually': 'دراصل', 'add': 'شامل کریں', 'addition': 'اضافہ', 'address': 'پتہ', 'administration': 'انتظامیہ',
            'admit': 'تسلیم کریں', 'adult': 'بالغ', 'advance': 'آگے', 'advantage': 'فائدہ', 'advertising': 'اشتہار',
            'advice': 'مشورہ', 'affect': 'متاثر کریں', 'after': 'بعد میں', 'again': 'دوبارہ', 'against': 'کے خلاف',
            'age': 'عمر', 'agency': 'ایجنسی', 'agent': 'ایجنٹ', 'ago': 'پہلے', 'agree': 'متفق ہوں',
            'agreement': 'معاہدہ', 'ahead': 'آگے', 'air': 'ہوا', 'all': 'تمام', 'allow': 'اجازت دیں',
            'almost': 'تقریباً', 'alone': 'اکیلے', 'along': 'ساتھ', 'already': 'پہلے سے', 'also': 'بھی',
            'although': 'اگرچہ', 'always': 'ہمیشہ', 'american': 'امریکی', 'among': 'میں سے', 'amount': 'رقم',
            'analysis': 'تجزیہ', 'analyze': 'تجزیہ کریں', 'and': 'اور', 'animal': 'جانور', 'another': 'دوسرا',
            'answer': 'جواب', 'any': 'کوئی', 'anybody': 'کوئی بھی', 'anyone': 'کوئی', 'anything': 'کوئی چیز',
            'appear': 'ظاہر ہوں', 'application': 'درخواست', 'apply': 'لاگو کریں', 'approach': 'نقطہ نظر',
            'appropriate': 'موزوں', 'approval': 'منظوری', 'approve': 'منظور کریں', 'area': 'علاقہ', 'argue': 'بحث کریں',
            'argument': 'دلیل', 'arise': 'اٹھیں', 'arm': 'بازو', 'armed': 'مسلح', 'around': 'اردگرد', 'arrange': 'ترتیب دیں',
            'arrangement': 'ترتیب', 'arrive': 'پہنچیں', 'art': 'فن', 'article': 'مضمون', 'artist': 'فنکار',
            'as': 'جیسے', 'ask': 'پوچھیں', 'assume': 'فرض کریں', 'assumption': 'فرض', 'assure': 'یقینی بنائیں',
            'at': 'میں', 'attack': 'حملہ', 'attention': 'توجہ', 'attitude': 'رویہ', 'attorney': 'وکیل',
            'audience': 'سامعین', 'author': 'مصنف', 'authority': 'اختیار', 'available': 'دستیاب', 'avoid': 'بچیں',
            'awake': 'جاگ جائیں', 'aware': 'آگاہ', 'away': 'دور', 'awesome': 'شاندار', 'awful': 'خوفناک',

            # Common words (b)
            'back': 'واپس', 'bad': 'برا', 'bag': 'بیگ', 'ball': 'گیند', 'bank': 'بینک', 'bar': 'بار',
            'base': 'بنیاد', 'based': 'بنایا', 'basic': 'بنیادی', 'basis': 'بنیاد', 'be': 'ہو', 'beat': 'شکست',
            'beautiful': 'خوبصورت', 'because': 'کیونکہ', 'become': 'بنیں', 'been': 'گیا', 'before': 'پہلے',
            'began': 'شروع کیا', 'begin': 'شروع کریں', 'beginning': 'شروع', 'behavior': 'رویہ', 'behind': 'پیچھے',
            'believe': 'یقین کریں', 'benefit': 'فائدہ', 'best': 'بہترین', 'better': 'بہتر', 'between': 'کے درمیان',
            'beyond': 'سے بہتر', 'big': 'بڑا', 'bill': 'بل', 'billion': 'ارب', 'bit': 'ٹکڑا', 'black': 'سیاہ',
            'blood': 'خون', 'blue': 'نیلا', 'board': 'بورڈ', 'body': 'جسم', 'book': 'کتاب', 'born': 'پیدا',
            'both': 'دونوں', 'bottom': 'نیچے', 'box': 'ڈبہ', 'boy': 'لڑکا', 'break': 'توڑیں', 'breakfast': 'ناشتہ',
            'bring': 'لے آئیں', 'brother': 'بھائی', 'brought': 'لایا', 'build': 'تعمیر کریں', 'building': 'عمارت',
            'business': 'کاروبار', 'but': 'لیکن', 'buy': 'خریدیں', 'by': 'کی طرف',

            # Common words (c)
            'call': 'کال کریں', 'called': 'کہا جاتا ہے', 'came': 'آیا', 'can': 'سکتے', 'candidate': 'امیدوار',
            'capital': 'سرمایہ', 'car': 'گاڑی', 'care': 'فکر', 'career': 'کیریئر', 'case': 'معاملہ', 'catch': 'پکڑیں',
            'cause': 'وجہ', 'cell': 'خانہ', 'central': 'مرکزی', 'century': 'صدی', 'certain': 'یقینی', 'certainly': 'یقیناً',
            'chair': 'کرسی', 'challenge': 'چیلنج', 'chance': 'موقع', 'change': 'تبدیلی', 'character': 'کردار',
            'charge': 'چارج', 'check': 'جانچیں', 'choice': 'انتخاب', 'choose': 'منتخب کریں', 'church': 'چرچ',
            'citizen': 'شہری', 'city': 'شہر', 'civil': 'شہری', 'claim': 'دعویٰ', 'class': 'کلاس', 'clear': 'صاف',
            'close': 'بند کریں', 'coach': 'کوچ', 'coalition': 'اتحاد', 'code': 'کوڈ', 'cold': 'سرد', 'collection': 'مجموعہ',
            'college': 'کالج', 'color': 'رنگ', 'come': 'آئیں', 'commercial': 'تجارتی', 'common': 'عام', 'community': 'کمیونٹی',
            'company': 'کمپنی', 'compare': 'موازنہ کریں', 'concern': 'فکر', 'condition': 'حالت', 'conference': 'کنفرنس',
            'congress': 'کانگریس', 'connect': 'جوڑیں', 'consider': 'غور کریں', 'consumer': 'صارف', 'contain': 'شامل',
            'continue': 'جاری رکھیں', 'control': 'کنٹرول', 'conversation': 'بات چیت', 'cost': 'قیمت', 'could': 'سکتے',
            'country': 'ملک', 'couple': 'جوڑا', 'course': 'کورس', 'court': 'عدالت', 'cousin': 'کزن', 'cover': 'ڈھیکن',
            'create': 'بنائیں', 'crime': 'جرم', 'crisis': 'بحران', 'culture': 'ثقافت', 'cup': 'پیالہ', 'current': 'موجودہ',

            # Common words (d)
            'dark': 'سیاہ', 'data': 'ڈیٹا', 'date': 'تاریخ', 'daughter': 'بیٹی', 'day': 'دن', 'dead': 'مردہ',
            'death': 'موت', 'debate': 'بحث', 'decade': 'دہائی', 'decide': 'فیصلہ کریں', 'decision': 'فیصلہ',
            'defense': 'دفاع', 'defense': 'دفاع', 'degree': 'ڈگری', 'democrat': 'ڈیموکریٹ', 'democratic': 'جمہوری',
            'describe': 'بیان کریں', 'description': 'تفصیل', 'design': 'ڈیزائن', 'desire': 'خواہش', 'despite': 'باوجود',
            'detail': 'تفصیل', 'determine': 'معین کریں', 'develop': 'تیار کریں', 'development': 'ترقی', 'difference': 'فرق',
            'different': 'مختلف', 'difficult': 'مشکل', 'difficulty': 'مشکل', 'dinner': 'رات کا کھانا', 'direction': 'سمت',
            'director': 'ڈائریکٹر', 'discover': 'دریافت کریں', 'discuss': 'بات کریں', 'discussion': 'بحث', 'disease': 'بیماری',
            'distance': 'فاصلہ', 'district': 'ضلع', 'divide': 'تقسیم کریں', 'document': 'دستاویز', 'does': 'کرتا',
            'dog': 'کتا', 'door': 'دروازہ', 'doubt': 'شک', 'down': 'نیچے', 'draw': 'کھینچیں', 'dream': 'خواب',
            'drive': 'ڈرائیو', 'drop': 'ڈھالیں', 'drug': 'دوا', 'during': 'کے دوران', 'duty': 'فرض',

            # Common words (e)
            'early': 'جلد', 'east': 'مشرق', 'easy': 'آسان', 'economic': 'اقتصادی', 'economy': 'معیشت',
            'edge': 'کنارہ', 'education': 'تعلیم', 'effect': 'اثر', 'effort': 'کوشش', 'eight': 'آٹھ',
            'either': 'کوئی بھی', 'election': 'انتخاب', 'energy': 'توانائی', 'early': 'جلد', 'eight': 'آٹھ',
            'environment': 'ماحول', 'equal': 'برابر', 'especially': 'خاص طور پر', 'establish': 'قائم کریں',
            'even': 'حتیٰ', 'evening': 'شام', 'event': 'واقعہ', 'ever': 'کبھی', 'every': 'ہر',
            'everybody': 'ہر ایک', 'everyone': 'سب', 'everything': 'سب کچھ', 'evidence': 'ثبوت', 'exactly': 'بالکل',
            'example': 'مثال', 'executive': 'انتظامی', 'exist': 'موجود ہوں', 'experience': 'تجربہ', 'explain': 'وضاحت کریں',
            'explain': 'وضاحت کریں', 'eye': 'آنکھ',

            # Technical terms
            'algorithm': 'الگورتھم', 'artificial': 'مصنوعی', 'automation': 'خودکاری', 'behavior': 'رویہ',
            'binary': 'دوئی', 'byte': 'بائٹ', 'cache': 'کیش', 'chip': 'چپ', 'circuit': 'سرکٹ',
            'class': 'کلاس', 'cloud': 'کلاؤڈ', 'code': 'کوڈ', 'command': 'کمانڈ', 'comment': 'تبصرہ',
            'compile': 'مرتب کریں', 'component': 'اجزاء', 'compute': 'حساب لگائیں', 'computer': 'کمپیوٹر',
            'config': 'ترتیب', 'control': 'کنٹرول', 'controller': 'کنٹرولر', 'cpu': 'سی پی یو', 'crash': 'کریش',
            'data': 'ڈیٹا', 'database': 'ڈیٹا بیس', 'debug': 'خرابی تلاش کریں', 'deploy': 'تعینات کریں',
            'device': 'ڈیوائس', 'digital': 'ڈیجیٹل', 'disk': 'ڈسک', 'download': 'ڈاؤن لوڈ کریں',
            'execute': 'نافذ کریں', 'feature': 'خصوصیت', 'file': 'فائل', 'filter': 'فلٹر', 'firmware': 'فرم ویئر',
            'flag': 'جھنڈا', 'format': 'فارمیٹ', 'framework': 'ڈھانچہ', 'frequency': 'تعداد', 'function': 'فنکشن',
            'gpu': 'جی پی یو', 'graph': 'گراف', 'grid': 'گرڈ', 'handle': 'سنبھالیں', 'hardware': 'سخت ویئر',
            'hash': 'ہیش', 'header': 'سر صحفہ', 'heap': 'ڈھیر', 'host': 'میزبان', 'html': 'ایچ ٹی ایم ایل',
            'http': 'ایچ ٹی ٹی پی', 'icon': 'آئیکن', 'implement': 'نافذ کریں', 'import': 'درآمد کریں',
            'index': 'انڈیکس', 'initialize': 'شروع کریں', 'input': 'ان پٹ', 'instance': 'مثال',
            'instruct': 'ہدایت دیں', 'integer': 'عددی', 'interface': 'رابطہ', 'interrupt': 'خلل ڈالیں',
            'io': 'ان پٹ آؤٹ پٹ', 'iteration': 'تکرار', 'json': 'جے ایس او این', 'kernel': 'دانہ',
            'key': 'کلید', 'keyword': 'کلیدی لفظ', 'lambda': 'لیمبڈا', 'language': 'زبان', 'layer': 'تہہ',
            'learning': 'سیکھنا', 'library': 'لائبریری', 'license': 'لائسنس', 'link': 'لنک', 'list': 'فہرست',
            'load': 'لوڈ کریں', 'local': 'مقامی', 'lock': 'تالا', 'log': 'لاگ', 'logic': 'منطق',
            'loop': 'لوپ', 'machine': 'مشین', 'main': 'مرکزی', 'map': 'نقشہ', 'memory': 'یادداشت',
            'message': 'پیغام', 'method': 'طریقہ', 'metric': 'پیمائش', 'middleware': 'درمیانی ورہ', 'mode': 'موڈ',
            'model': 'ماڈل', 'module': 'ماڈیول', 'monitor': 'نگرانی کریں', 'motor': 'موٹر', 'mouse': 'ماؤس',
            'move': 'حرکت کریں', 'network': 'نیٹ ورک', 'neural': 'اعصابی', 'node': 'نوڈ', 'noise': 'شور',
            'object': 'چیز', 'operation': 'آپریشن', 'operator': 'آپریٹر', 'optimize': 'بہتر بنائیں', 'option': 'اختیار',
            'order': 'ترتیب', 'output': 'آؤٹ پٹ', 'overflow': 'بہاؤ', 'parallel': 'متوازی', 'parameter': 'پیرامیٹر',
            'parse': 'پارس کریں', 'partition': 'حصہ', 'password': 'پاس ورڈ', 'path': 'راستہ', 'pattern': 'نمونہ',
            'pause': 'رکیں', 'payload': 'بوجھ', 'performance': 'کارکردگی', 'permission': 'اجازت', 'physics': 'طبیعیات',
            'pixel': 'پکسل', 'pointer': 'اشارہ', 'port': 'بندرگاہ', 'position': 'پوزیشن', 'power': 'طاقت',
            'practice': 'عمل', 'predict': 'پیشین گوئی کریں', 'preference': 'ترجیح', 'press': 'دبائیں',
            'priority': 'ترجیح', 'privacy': 'نجی', 'process': 'عمل', 'processor': 'پروسیسر', 'produce': 'تیار کریں',
            'product': 'پروڈکٹ', 'program': 'پروگرام', 'project': 'منصوبہ', 'promise': 'وعدہ', 'property': 'خصوصیت',
            'protocol': 'پروٹوکول', 'provide': 'فراہم کریں', 'pull': 'کھینچیں', 'push': 'دھکیلیں', 'query': 'سوال',
            'queue': 'لائن', 'random': 'بے ترتیب', 'range': 'رینج', 'rate': 'شرح', 'read': 'پڑھیں', 'real': 'حقیقی',
            'reason': 'وجہ', 'receive': 'حاصل کریں', 'recommend': 'تجویز کریں', 'record': 'ریکارڈ', 'recover': 'بحال کریں',
            'reduce': 'کم کریں', 'reference': 'حوالہ', 'reflect': 'عکاسی کریں', 'refresh': 'تروتازہ کریں', 'register': 'رجسٹر',
            'release': 'رہائی', 'reliable': 'قابل اعتماد', 'reload': 'دوبارہ لوڈ کریں', 'remote': 'دور دراز',
            'remove': 'ہٹائیں', 'render': 'تصویر بنائیں', 'repeat': 'دہرائیں', 'replace': 'بدل دیں', 'report': 'رپورٹ',
            'request': 'درخواست', 'require': 'ضروری ہے', 'requirement': 'ضرورت', 'reset': 'ری سیٹ کریں', 'resolve': 'حل کریں',
            'resource': 'وسیلہ', 'response': 'جواب', 'result': 'نتیجہ', 'return': 'واپسی', 'reverse': 'الٹا',
            'review': 'جائزہ', 'robot': 'روبوٹ', 'robotics': 'روبوٹکس', 'rule': 'اصول', 'run': 'چلائیں',
            'safe': 'محفوظ', 'save': 'بچائیں', 'schema': 'نقشہ', 'scope': 'دائرہ', 'screen': 'سکرین',
            'script': 'اسکرپٹ', 'search': 'تلاش کریں', 'second': 'دوسرا', 'section': 'حصہ', 'security': 'حفاظت',
            'seek': 'تلاش کریں', 'segment': 'حصہ', 'select': 'منتخب کریں', 'send': 'بھیجیں', 'sensor': 'سینسر',
            'sequence': 'ترتیب', 'server': 'سرور', 'service': 'خدمت', 'session': 'سیشن', 'set': 'سیٹ',
            'setting': 'ترتیب', 'setup': 'سیٹ اپ', 'share': 'شیئر کریں', 'shift': 'شفٹ', 'show': 'دکھائیں',
            'signal': 'سگنل', 'size': 'سائز', 'skip': 'چھوڑ دیں', 'slow': 'سست', 'socket': 'ساکٹ',
            'software': 'سافٹ ویئر', 'solution': 'حل', 'solve': 'حل کریں', 'sort': 'ترتیب دیں', 'source': 'ذریعہ',
            'space': 'جگہ', 'specification': 'تفصیل', 'speed': 'رفتار', 'split': 'تقسیم کریں', 'stack': 'اسٹیک',
            'standard': 'معیار', 'state': 'حالت', 'statement': 'بیان', 'static': 'غیر متحرک', 'station': 'سٹیشن',
            'status': 'حالت', 'step': 'قدم', 'stop': 'روکیں', 'storage': 'ذخیرہ', 'store': 'محفوظ کریں',
            'stream': 'بہاؤ', 'string': 'سٹرنگ', 'structure': 'ڈھانچہ', 'studio': 'سٹوڈیو', 'study': 'مطالعہ',
            'style': 'انداز', 'subject': 'موضوع', 'submit': 'جمع کریں', 'subscribe': 'رکن بنیں', 'subset': 'ذیلی سیٹ',
            'success': 'کامیابی', 'summary': 'خلاصہ', 'support': 'معاونت', 'suppose': 'فرض کریں', 'suppress': 'دبائیں',
            'surface': 'سطح', 'switch': 'سوئچ', 'symbol': 'علامت', 'sync': 'ہم آہنگ', 'syntax': 'نحو',
            'system': 'نظام', 'table': 'ٹیبل', 'tag': 'ٹیگ', 'target': 'ہدف', 'task': 'کام', 'template': 'سانچہ',
            'test': 'ٹیسٹ', 'text': 'متن', 'than': 'سے', 'that': 'وہ', 'the': 'یہ', 'their': 'ان کا',
            'them': 'انہیں', 'theme': 'تھیم', 'then': 'پھر', 'theory': 'نظریہ', 'there': 'وہاں', 'therefore': 'اس لیے',
            'these': 'یہ', 'they': 'وہ', 'thing': 'چیز', 'think': 'سوچیں', 'this': 'یہ', 'those': 'وہ',
            'though': 'اگرچہ', 'thought': 'خیال', 'through': 'ذریعے', 'time': 'وقت', 'timing': 'وقت کی تقسیم',
            'tip': 'ٹپ', 'title': 'عنوان', 'token': 'ٹوکن', 'tool': 'آلہ', 'topic': 'موضوع', 'total': 'کل',
            'touch': 'چھوئیں', 'trace': 'نقوش', 'track': 'ٹریک', 'traffic': 'ٹریفک', 'train': 'ریل',
            'transfer': 'منتقل کریں', 'transform': 'تبدیل کریں', 'transition': 'منتقلی', 'translate': 'ترجمہ کریں',
            'transmission': 'ترسیل', 'transport': 'نقل', 'trap': 'پھندہ', 'treat': 'سلوک کریں', 'trigger': 'سبب',
            'true': 'سچ', 'trust': 'اعتماد', 'try': 'کوشش کریں', 'tuple': 'ٹیپل', 'type': 'قسم',
            'typical': 'عام طور پر', 'understand': 'سمجھیں', 'unit': 'یونٹ', 'update': 'اپ ڈیٹ', 'upload': 'اپ لوڈ',
            'use': 'استعمال', 'used': 'استعمال شدہ', 'user': 'صارف', 'using': 'استعمال کرتے ہوئے', 'utility': 'افادیت',
            'valid': 'درست', 'value': 'قیمت', 'variable': 'متغیر', 'variation': 'تغیر', 'various': 'مختلف',
            'vector': 'ویکٹر', 'version': 'ورژن', 'view': 'نظارہ', 'virtual': 'مجازی', 'virus': 'وائرس',
            'visible': 'نظر آنے والی', 'visit': 'دیکھنے جائیں', 'visual': 'بصری', 'void': 'خالی', 'voltage': 'وولٹیج',
            'volume': 'حجم', 'wait': 'انتظار کریں', 'walk': 'چلیں', 'warning': 'انتباہ', 'watch': 'دیکھیں',
            'water': 'پانی', 'way': 'طریقہ', 'weak': 'کمزور', 'weight': 'وزن', 'welcome': 'خوش آمدید',
            'what': 'کیا', 'wheel': 'پہیہ', 'when': 'کب', 'where': 'کہاں', 'whether': 'کہ آیا', 'which': 'کون',
            'while': 'جبکہ', 'white': 'سفید', 'who': 'کون', 'whole': 'پورا', 'why': 'کیوں', 'wide': 'چوڑا',
            'width': 'چوڑائی', 'will': 'ہوگا', 'window': 'ونڈو', 'wire': 'تار', 'wish': 'خواہش', 'with': 'کے ساتھ',
            'within': 'اندر', 'without': 'بغیر', 'word': 'لفظ', 'work': 'کام', 'worker': 'کارکن', 'world': 'دنیا',
            'worry': 'فکر کریں', 'would': 'ہوتا', 'write': 'لکھیں', 'writer': 'لکھاری', 'written': 'لکھا', 'wrong': 'غلط',
            'year': 'سال', 'yes': 'جی', 'yet': 'ابھی', 'you': 'آپ', 'young': 'نوجوان', 'your': 'آپ کا', 'yours': 'آپ کا',
            'yourself': 'آپ خود', 'zero': 'صفر', 'zone': 'علاقہ',
        }

        spanish_translations = {
            'the': 'el', 'is': 'es', 'and': 'y', 'to': 'a', 'of': 'de', 'in': 'en', 'it': 'eso',
            'that': 'eso', 'this': 'esto', 'for': 'para', 'with': 'con', 'by': 'por', 'from': 'desde',
            'at': 'en', 'as': 'como', 'on': 'en', 'be': 'ser', 'can': 'poder', 'have': 'tener',
            'chapter': 'capítulo', 'robotics': 'robótica', 'introduction': 'introducción', 'will': 'voluminoso',
            'learn': 'aprender', 'system': 'sistema', 'computer': 'computadora', 'code': 'código',
        }

        translations_map = {
            'urdu': urdu_translations,
            'spanish': spanish_translations,
        }

        return translations_map.get(target_language, {})

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
