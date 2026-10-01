"""Convert an ASCII OpenSCAD STL (millimetres) to KiCad-compatible VRML2."""

from __future__ import annotations

import argparse
from pathlib import Path


MM_PER_KICAD_VRML_UNIT = 2.54


def read_vertices(path: Path, flip_y: bool = False) -> list[tuple[float, float, float]]:
    vertices: list[tuple[float, float, float]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = line.strip().split()
        if len(fields) == 4 and fields[0] == "vertex":
            x, y, z = (float(value) for value in fields[1:])
            if flip_y:
                y = -y
            vertices.append((
                x / MM_PER_KICAD_VRML_UNIT,
                y / MM_PER_KICAD_VRML_UNIT,
                z / MM_PER_KICAD_VRML_UNIT,
            ))
    if not vertices or len(vertices) % 3:
        raise ValueError(f"Expected an ASCII triangle STL, got {len(vertices)} vertices")
    return vertices


def write_vrml(
    path: Path,
    vertices: list[tuple[float, float, float]],
    color: tuple[float, float, float],
) -> None:
    points = "\n".join(f"          {x:.7f} {y:.7f} {z:.7f}," for x, y, z in vertices)
    faces = "\n".join(
        f"        {index} {index + 1} {index + 2} -1," for index in range(0, len(vertices), 3)
    )
    red, green, blue = color
    path.write_text(
        f"""#VRML V2.0 utf8
Shape {{
  appearance Appearance {{
    material Material {{
      diffuseColor {red:.4f} {green:.4f} {blue:.4f}
      specularColor 0.22 0.22 0.22
      shininess 0.30
    }}
  }}
  geometry IndexedFaceSet {{
    solid TRUE
    creaseAngle 0.55
    coord Coordinate {{
      point [
{points}
      ]
    }}
    coordIndex [
{faces}
    ]
  }}
}}
""",
        encoding="utf-8",
        newline="\n",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--color", nargs=3, type=float, default=(0.16, 0.28, 0.18))
    parser.add_argument(
        "--flip-y",
        action="store_true",
        help="Mirror STL Y coordinates to match KiCad's legacy VRML convention.",
    )
    args = parser.parse_args()
    write_vrml(args.destination, read_vertices(args.source, args.flip_y), tuple(args.color))


if __name__ == "__main__":
    main()
