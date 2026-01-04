#!/usr/bin/env python3
from pathlib import Path

import yaml


def build_pgm(width: int, height: int) -> str:
    border_row = " ".join(["0"] * width)
    interior_values = ["0"] + ["254"] * (width - 2) + ["0"]
    interior_row = " ".join(interior_values)
    rows = [border_row] + [interior_row] * (height - 2) + [border_row]
    header = ["P2", f"{width} {height}", "255"]
    return "\n".join(header + rows) + "\n"


def main() -> None:
    here = Path(__file__).resolve().parent
    width = 150
    height = 100
    resolution = 0.02

    pgm_path = here / "field.pgm"
    yaml_path = here / "field.yaml"

    pgm_path.write_text(build_pgm(width, height))
    yaml_payload = {
        "image": "field.pgm",
        "resolution": resolution,
        "origin": [0.0, 0.0, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    yaml_path.write_text(yaml.safe_dump(yaml_payload, sort_keys=False))

    print(f"Wrote {pgm_path} and {yaml_path}")


if __name__ == "__main__":
    main()
