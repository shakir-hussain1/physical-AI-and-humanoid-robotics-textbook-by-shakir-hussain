"""Command-line interface for retrieval service.

Provides CLI commands for searching, validation, and reporting.
"""

import argparse
import sys
import logging
from typing import Optional
from .config import RetrievalConfig
from .retrieval_service import RetrievalService
from .test_queries import get_test_queries, validate_test_query_suite
from .report_generator import ReportGenerator
from .logging import setup_retrieval_logging

logger = logging.getLogger(__name__)


class RetrievalCLI:
    """Command-line interface for retrieval operations."""

    def __init__(self):
        """Initialize CLI with configuration and service."""
        try:
            self.config = RetrievalConfig.load_from_env()
            self.service = RetrievalService(self.config)
            logger.info("Retrieval CLI initialized")
        except Exception as e:
            logger.error(f"Failed to initialize CLI: {str(e)}")
            sys.exit(1)

    def run_validation(self) -> int:
        """Execute full validation suite with all test queries.

        Returns:
            0 if successful, 1 if failed
        """
        print("\n" + "=" * 80)
        print("RUNNING FULL VALIDATION SUITE")
        print("=" * 80 + "\n")

        # Validate test query suite
        is_valid, issues = validate_test_query_suite()
        if not is_valid:
            print("Test query suite validation failed:")
            for issue in issues:
                print(f"  - {issue}")
            return 1

        # Get test queries
        test_queries = get_test_queries()
        print(f"Executing {len(test_queries)} test queries...\n")

        # Generate report
        try:
            report_generator = ReportGenerator(self.service)
            report = report_generator.generate_validation_report(test_queries)

            # Display report
            formatted_report = report_generator.format_report_for_display(report)
            print(formatted_report)

            # Write report to file
            with open("backend/VALIDATION_RESULTS.txt", "w") as f:
                f.write(formatted_report)
            print(f"\nReport saved to: backend/VALIDATION_RESULTS.txt")

            # Return success/failure based on pass rate
            return 0 if report.get("pass_rate", 0) >= 80.0 else 1

        except Exception as e:
            print(f"\nERROR: Validation failed: {str(e)}")
            logger.error(f"Validation execution failed: {str(e)}")
            return 1

    def search(self, query: str, k: int = 5) -> int:
        """Execute a similarity search query.

        Args:
            query: Query text to search for
            k: Number of results to return

        Returns:
            0 if successful, 1 if failed
        """
        print(f"\nSearching for: {query}")
        print(f"Requested results: {k}\n")

        try:
            result = self.service.search(query, k=k)

            if result["status"] != "success":
                print(f"Search failed: {result.get('error', 'Unknown error')}")
                return 1

            results = result.get("results", [])
            print(f"Results: {len(results)} returned ({result.get('latency_ms', 0)}ms)\n")

            for i, res in enumerate(results, 1):
                metadata = res.get("metadata", {})
                print(f"{i}. Similarity Score: {res.get('similarity_score', 0):.3f}")
                print(f"   URL: {metadata.get('url', 'N/A')}")
                print(f"   Title: {metadata.get('page_title', 'N/A')}")
                print(f"   Chunk: {metadata.get('chunk_index', 'N/A')}\n")

            return 0

        except Exception as e:
            print(f"Search failed: {str(e)}")
            logger.error(f"Search execution failed: {str(e)}")
            return 1

    def get_collection_info(self) -> int:
        """Display collection statistics and health information.

        Returns:
            0 if successful, 1 if failed
        """
        print("\n" + "=" * 80)
        print("COLLECTION INFORMATION")
        print("=" * 80 + "\n")

        try:
            stats = self.service.get_stats()

            if stats.get("service_status") == "error":
                print(f"Error retrieving stats: {stats.get('error', 'Unknown error')}")
                return 1

            print(f"Collection: {stats.get('collection', 'N/A')}")
            print(f"Total Vectors: {stats.get('total_vectors', 0)}")
            print(f"Vector Dimension: {stats.get('vector_dimension', 0)}")
            print(f"Memory Usage: {stats.get('memory_usage_bytes', 0)} bytes")
            print(f"Collection Status: {stats.get('status', 'N/A')}")
            print(f"Qdrant Health: {stats.get('qdrant_health', 'N/A')}")
            print(f"Service Status: {stats.get('service_status', 'N/A')}\n")

            return 0

        except Exception as e:
            print(f"Failed to get collection info: {str(e)}")
            logger.error(f"Collection info retrieval failed: {str(e)}")
            return 1

    def test_query_set(self, custom_query: Optional[str] = None) -> int:
        """Run specific test queries or custom query.

        Args:
            custom_query: Optional custom query to run instead of test set

        Returns:
            0 if successful, 1 if failed
        """
        if custom_query:
            print(f"\nExecuting custom query: {custom_query}\n")
            return self.search(custom_query, k=5)

        print("\n" + "=" * 80)
        print("TEST QUERY SET")
        print("=" * 80 + "\n")

        test_queries = get_test_queries()
        for i, test_query in enumerate(test_queries, 1):
            print(f"Test Query {i}/{len(test_queries)}: {test_query.text}")
            result = self.service.validate_retrieval(test_query.text, k=5)
            if result["status"] == "success":
                validation = result.get("validation", {})
                print(f"  Status: {'PASS' if validation.get('is_valid') else 'FAIL'}")
                print(f"  Results: {result.get('result_count', 0)}")
                print(f"  Latency: {result.get('latency_ms', 0)}ms\n")
            else:
                print(f"  Error: {result.get('error', 'Unknown')}\n")

        return 0


def main():
    """Main CLI entry point."""
    # Setup logging
    setup_retrieval_logging(
        log_level="INFO",
        log_format="json",
        log_file="backend/logs/retrieval.log",
    )

    parser = argparse.ArgumentParser(
        description="Retrieval Service CLI - Vector similarity search and validation"
    )

    subparsers = parser.add_subparsers(dest="command", help="Available commands")

    # run-validation command
    subparsers.add_parser(
        "run-validation",
        help="Execute full validation suite with all test queries",
    )

    # search command
    search_parser = subparsers.add_parser("search", help="Execute similarity search")
    search_parser.add_argument("query", help="Query text to search for")
    search_parser.add_argument("-k", type=int, default=5, help="Number of results (default: 5)")

    # get-info command
    subparsers.add_parser(
        "get-info",
        help="Display collection statistics and health information",
    )

    # test-query command
    test_parser = subparsers.add_parser(
        "test-query",
        help="Run test query set or custom query",
    )
    test_parser.add_argument(
        "-q", "--query",
        type=str,
        default=None,
        help="Custom query to run (if not provided, runs test suite)",
    )

    args = parser.parse_args()

    cli = RetrievalCLI()

    if args.command == "run-validation":
        return cli.run_validation()
    elif args.command == "search":
        return cli.search(args.query, k=args.k)
    elif args.command == "get-info":
        return cli.get_collection_info()
    elif args.command == "test-query":
        return cli.test_query_set(custom_query=args.query)
    else:
        parser.print_help()
        return 1


if __name__ == "__main__":
    sys.exit(main())
