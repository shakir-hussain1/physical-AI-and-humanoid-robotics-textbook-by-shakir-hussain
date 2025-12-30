"""Validation report generation for retrieval pipeline.

Generates comprehensive reports from test query execution.
"""

import logging
from typing import List, Dict, Any, Optional
from datetime import datetime
from .test_queries import TestQuery
from .retrieval_service import RetrievalService

logger = logging.getLogger(__name__)


class ReportGenerator:
    """Generate validation reports from retrieval test execution."""

    def __init__(self, retrieval_service: RetrievalService):
        """Initialize report generator.

        Args:
            retrieval_service: RetrievalService instance for query execution
        """
        self.retrieval_service = retrieval_service

    def generate_validation_report(
        self, test_queries: List[TestQuery]
    ) -> Dict[str, Any]:
        """Execute test queries and generate validation report.

        Args:
            test_queries: List of TestQuery objects to execute

        Returns:
            Comprehensive validation report dictionary
        """
        report = {
            "timestamp": datetime.utcnow().isoformat(),
            "total_queries": len(test_queries),
            "successful_queries": 0,
            "failed_queries": 0,
            "total_results": 0,
            "average_latency_ms": 0.0,
            "pass_rate": 0.0,
            "query_results": [],
            "issues": [],
        }

        latencies = []
        valid_count = 0

        # Execute each test query
        for query_obj in test_queries:
            try:
                result = self.retrieval_service.validate_retrieval(query_obj.text)

                if result["status"] != "success":
                    report["failed_queries"] += 1
                    report["issues"].append({
                        "query": query_obj.text,
                        "error": result.get("error", "Unknown error"),
                    })
                    continue

                # Extract validation info
                validation = result.get("validation", {})
                is_valid = validation.get("is_valid", False)

                if is_valid:
                    valid_count += 1
                    report["successful_queries"] += 1
                else:
                    report["failed_queries"] += 1

                latency = result.get("latency_ms", 0)
                latencies.append(latency)

                query_result = {
                    "query": query_obj.text,
                    "category": query_obj.category,
                    "expected_primary_domain": query_obj.expected_primary_domain,
                    "expected_modules": query_obj.expected_modules,
                    "result_count": result.get("result_count", 0),
                    "is_valid": is_valid,
                    "latency_ms": latency,
                    "validation_issues": validation.get("issues", []),
                    "results": result.get("results", [])[:3],  # Keep top-3 for report
                }

                report["query_results"].append(query_result)
                report["total_results"] += result.get("result_count", 0)

            except Exception as e:
                logger.error(f"Error executing test query '{query_obj.text}': {str(e)}")
                report["failed_queries"] += 1
                report["issues"].append({
                    "query": query_obj.text,
                    "error": str(e),
                })

        # Calculate statistics
        if latencies:
            report["average_latency_ms"] = round(sum(latencies) / len(latencies), 2)

        if len(test_queries) > 0:
            report["pass_rate"] = round((valid_count / len(test_queries)) * 100, 2)

        logger.info(
            f"Validation report generated: {valid_count}/{len(test_queries)} passed "
            f"({report['pass_rate']}%)"
        )

        return report

    @staticmethod
    def format_report_for_display(report: Dict[str, Any]) -> str:
        """Format validation report as human-readable text.

        Args:
            report: Validation report dictionary

        Returns:
            Formatted text report
        """
        output = []
        output.append("=" * 80)
        output.append("DATA RETRIEVAL & PIPELINE VALIDATION REPORT")
        output.append("=" * 80)
        output.append("")

        # Summary statistics
        output.append("SUMMARY")
        output.append("-" * 80)
        output.append(f"Generated: {report.get('timestamp', 'N/A')}")
        output.append(f"Total Queries Executed: {report.get('total_queries', 0)}")
        output.append(f"Successful: {report.get('successful_queries', 0)}")
        output.append(f"Failed: {report.get('failed_queries', 0)}")
        output.append(f"Pass Rate: {report.get('pass_rate', 0.0)}%")
        output.append(f"Total Results Returned: {report.get('total_results', 0)}")
        output.append(f"Average Latency: {report.get('average_latency_ms', 0.0):.2f}ms")
        output.append("")

        # Query results
        output.append("QUERY RESULTS")
        output.append("-" * 80)

        for i, query_result in enumerate(report.get("query_results", []), 1):
            output.append(f"\nQuery {i}: {query_result.get('query', '')}")
            output.append(f"  Category: {query_result.get('category', 'N/A')}")
            output.append(f"  Expected Domain: {query_result.get('expected_primary_domain', 'N/A')}")
            output.append(f"  Expected Modules: {', '.join(query_result.get('expected_modules', []))}")
            output.append(f"  Results Returned: {query_result.get('result_count', 0)}")
            output.append(f"  Validation Status: {'PASS' if query_result.get('is_valid') else 'FAIL'}")
            output.append(f"  Latency: {query_result.get('latency_ms', 0)}ms")

            # Show top results
            results = query_result.get("results", [])
            if results:
                output.append("  Top Results:")
                for j, result in enumerate(results[:3], 1):
                    metadata = result.get("metadata", {})
                    output.append(
                        f"    {j}. Score: {result.get('similarity_score', 0):.3f} | "
                        f"URL: {metadata.get('url', 'N/A')[:60]}"
                    )

            # Show validation issues if any
            issues = query_result.get("validation_issues", [])
            if issues:
                output.append("  Validation Issues:")
                for issue in issues:
                    output.append(f"    - {issue}")

        # Overall issues
        if report.get("issues", []):
            output.append("\n" + "=" * 80)
            output.append("ISSUES AND ERRORS")
            output.append("-" * 80)
            for issue in report["issues"]:
                output.append(f"Query: {issue.get('query', 'N/A')}")
                output.append(f"  Error: {issue.get('error', 'Unknown')}")
                output.append("")

        # Footer
        output.append("=" * 80)
        output.append("END OF REPORT")
        output.append("=" * 80)

        return "\n".join(output)
