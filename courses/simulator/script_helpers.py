"""Command-line helpers shared by Day 4 entry points."""

from __future__ import annotations

import argparse
from pathlib import Path


def common_parser(description: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="create output without opening a Matplotlib window",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="optional PNG output path",
    )
    return parser


def finish_figure(figure, arguments) -> None:
    """Save if requested and show unless the caller selected headless mode."""

    import matplotlib.pyplot as plt

    if arguments.output:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(arguments.output, dpi=150, bbox_inches="tight")
        print(f"Saved figure: {arguments.output}")
    if not arguments.no_show:
        plt.show()
    else:
        plt.close(figure)


# Day 1 command-line compatibility

def plot_arguments(default_filename: str) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help=f"Optional output image path (for example {default_filename}).",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open the interactive Matplotlib window.",
    )
    return parser.parse_args()
