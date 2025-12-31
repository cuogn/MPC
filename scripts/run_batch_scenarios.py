from __future__ import annotations

import argparse
from pathlib import Path

import os, sys
# Allow running this script from any working directory
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from app.sim.batch_scenarios import run_batch


def main() -> None:
    parser = argparse.ArgumentParser(description="Run PID vs MPC batch scenarios and export tables/logs/plots")
    parser.add_argument(
        "--catalog",
        type=str,
        default=str(Path("scenarios") / "scenarios_catalog.json"),
        help="Path to scenarios_catalog.json",
    )
    parser.add_argument(
        "--out",
        type=str,
        default=None,
        help="Output directory (default: outputs/batch_YYYYmmdd_HHMMSS)",
    )
    args = parser.parse_args()

    out_dir = run_batch(args.catalog, out_dir=args.out)
    print("=== Batch scenarios finished ===")
    print(f"Catalog: {args.catalog}")
    print(f"Output:  {out_dir}")
    print("Files: metrics_tables.xlsx, metrics_pivot.csv, metrics_all.csv, csv/*.csv, plots/*.png")


if __name__ == "__main__":
    main()
