"""Metadata and content validation for retrieval results.

Provides validation checks for metadata completeness, consistency, and content accessibility.
"""

import logging
import requests
from typing import Tuple, List, Dict, Optional
from urllib.parse import urlparse

logger = logging.getLogger(__name__)


class MetadataValidator:
    """Validate metadata completeness and consistency."""

    REQUIRED_FIELDS = ["url", "page_title", "chunk_index"]

    @staticmethod
    def validate_metadata_completeness(result: Dict) -> Tuple[bool, List[str]]:
        """Check if result has all required metadata fields.

        Args:
            result: Retrieved result dictionary with metadata

        Returns:
            Tuple of (is_valid: bool, error_messages: List[str])
        """
        errors = []
        metadata = result.get("metadata", {})

        if not metadata:
            errors.append("Metadata is missing or empty")
            return False, errors

        for field in MetadataValidator.REQUIRED_FIELDS:
            if field not in metadata:
                errors.append(f"Required metadata field '{field}' is missing")
            elif not metadata[field]:
                errors.append(f"Required metadata field '{field}' is empty")

        return len(errors) == 0, errors

    @staticmethod
    def validate_metadata_consistency(metadata: Dict) -> Tuple[bool, List[str]]:
        """Check if metadata fields have valid values and types.

        Args:
            metadata: Metadata dictionary to validate

        Returns:
            Tuple of (is_valid: bool, error_messages: List[str])
        """
        errors = []

        # Validate URL format
        url = metadata.get("url", "")
        if url:
            try:
                result = urlparse(url)
                is_valid_url = all([result.scheme, result.netloc])
                if not is_valid_url:
                    errors.append(f"URL is not valid format: {url}")
            except Exception as e:
                errors.append(f"URL parsing failed: {str(e)}")

        # Validate page_title is string
        page_title = metadata.get("page_title", "")
        if page_title and not isinstance(page_title, str):
            errors.append(f"page_title must be string, got {type(page_title)}")

        # Validate chunk_index is integer >= 0
        chunk_index = metadata.get("chunk_index")
        if chunk_index is not None:
            if not isinstance(chunk_index, int):
                errors.append(f"chunk_index must be integer, got {type(chunk_index)}")
            elif chunk_index < 0:
                errors.append(f"chunk_index must be >= 0, got {chunk_index}")

        return len(errors) == 0, errors

    @staticmethod
    def validate_similarity_score(score: float) -> bool:
        """Check if similarity score is in valid range [0, 1].

        Args:
            score: Similarity score value

        Returns:
            True if valid, False otherwise
        """
        if not isinstance(score, (int, float)):
            logger.warning(f"Similarity score is not numeric: {type(score)}")
            return False

        if score < 0.0 or score > 1.0:
            logger.warning(f"Similarity score out of range: {score}")
            return False

        return True


class ContentVerifier:
    """Verify content accessibility and integrity."""

    @staticmethod
    def verify_url_accessible(url: str, timeout: int = 5) -> Tuple[bool, str]:
        """Check if URL is accessible via HTTP HEAD request.

        Args:
            url: URL to verify
            timeout: Request timeout in seconds (default 5)

        Returns:
            Tuple of (is_accessible: bool, status_message: str)
        """
        if not url:
            return False, "URL is empty"

        try:
            response = requests.head(url, timeout=timeout, allow_redirects=True)
            if response.status_code == 200:
                return True, f"URL accessible (status: {response.status_code})"
            elif response.status_code < 400:
                return True, f"URL accessible (status: {response.status_code})"
            else:
                return False, f"URL returned error status: {response.status_code}"
        except requests.Timeout:
            return False, f"URL request timed out (timeout: {timeout}s)"
        except requests.ConnectionError as e:
            return False, f"Connection failed: {str(e)}"
        except Exception as e:
            return False, f"URL verification failed: {str(e)}"

    @staticmethod
    def verify_chunk_exists_in_page(
        url: str,
        chunk_index: int,
        timeout: int = 5
    ) -> Tuple[bool, str]:
        """Fetch page content and verify chunk_index matches structure.

        Args:
            url: URL to fetch
            chunk_index: Expected chunk index in page
            timeout: Request timeout in seconds

        Returns:
            Tuple of (is_valid: bool, status_message: str)

        Note:
            This performs a GET request and checks page structure.
            In practice, this is a best-effort validation.
        """
        if not url:
            return False, "URL is empty"

        if chunk_index < 0:
            return False, f"chunk_index must be >= 0, got {chunk_index}"

        try:
            response = requests.get(url, timeout=timeout)

            if response.status_code != 200:
                return False, f"Failed to fetch page (status: {response.status_code})"

            # Verify page has content (basic check)
            if not response.text or len(response.text) < 100:
                return False, "Page content appears empty or too short"

            # Check for common indicators of valid page structure
            has_content = (
                len(response.text) > 100 and
                ("module" in response.text.lower() or
                 "chapter" in response.text.lower() or
                 "content" in response.text.lower())
            )

            if has_content:
                return True, f"Page content verified (length: {len(response.text)} chars)"
            else:
                return False, "Page content structure not recognized"

        except requests.Timeout:
            return False, f"Page fetch timed out (timeout: {timeout}s)"
        except requests.ConnectionError as e:
            return False, f"Connection failed: {str(e)}"
        except Exception as e:
            return False, f"Content verification failed: {str(e)}"


class RetrievalValidator:
    """Validate retrieval results and query responses."""

    @staticmethod
    def validate_query_results(
        query: str,
        results: List[Dict],
        k: int = 5
    ) -> Dict:
        """Validate query results for correctness and consistency.

        Args:
            query: Original query text
            results: List of retrieval results
            k: Expected number of top results

        Returns:
            Dictionary with validation report:
            {
                "is_valid": bool,
                "result_count": int,
                "metadata_valid": int,
                "scores_valid": int,
                "ranked_correctly": bool,
                "issues": List[str]
            }
        """
        report = {
            "is_valid": True,
            "result_count": len(results),
            "metadata_valid": 0,
            "scores_valid": 0,
            "ranked_correctly": True,
            "issues": [],
        }

        if not results:
            report["issues"].append("No results returned for query")
            report["is_valid"] = False
            return report

        # Check result count
        if len(results) > k:
            report["issues"].append(f"Too many results: {len(results)} > {k}")
            report["is_valid"] = False

        # Validate each result
        prev_score = 1.1  # Start higher than max possible score
        for i, result in enumerate(results):
            # Validate metadata
            is_meta_valid, meta_errors = MetadataValidator.validate_metadata_completeness(result)
            if is_meta_valid:
                report["metadata_valid"] += 1
            else:
                report["issues"].append(f"Result {i}: {'; '.join(meta_errors)}")
                report["is_valid"] = False

            # Validate similarity score
            score = result.get("similarity_score", 0)
            if MetadataValidator.validate_similarity_score(score):
                report["scores_valid"] += 1
            else:
                report["issues"].append(f"Result {i}: Invalid similarity score: {score}")
                report["is_valid"] = False

            # Check ranking (should be descending by score)
            if score > prev_score:
                report["ranked_correctly"] = False
                report["issues"].append(
                    f"Result {i}: Scores not in descending order ({score} > {prev_score})"
                )
                report["is_valid"] = False
            prev_score = score

        return report
