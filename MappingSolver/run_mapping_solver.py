#!/usr/bin/env python3
import os
import sys
import time

import numpy as np
from ortools.linear_solver import pywraplp

from garment import Garment
from mipsolver import mipGarment_solver
from patchRep import ApproxPolygon, Polygon
from polygon_matching_solver import matching_polygon
from quad_polygon_approx_solver import fit_polygon_to_polygon_pairs


def load_garment(path):
    g = Garment()
    try:
        g.build_from_txt(path, Polygon)
    except Exception:
        try:
            g.build_from_txt(path, ApproxPolygon)
        except Exception as e:
            print(f"Failed to load {path}: {e}", file=sys.stderr)
            sys.exit(1)
    return g


def main():
    if len(sys.argv) != 4:
        print(
            "Usage: python run_mapping_solver.py <garment1_polygons.txt> "
            "<garment2_polygons.txt> <output_mapping.txt>",
            file=sys.stderr,
        )
        sys.exit(1)

    garment1_path = sys.argv[1]
    garment2_path = sys.argv[2]
    output_path = sys.argv[3]

    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    g1 = load_garment(garment1_path)
    g2 = load_garment(garment2_path)

    solver = mipGarment_solver(g1, g2)
    mapping_tic = time.time()
    status = solver.solve()
    mapping_solve_seconds = time.time() - mapping_tic
    print(f"[mapping_solver] mapping_solve_seconds={mapping_solve_seconds:.3f}")

    if status not in (pywraplp.Solver.OPTIMAL, pywraplp.Solver.FEASIBLE):
        print("Segment-mapping solver found no feasible solution.", file=sys.stderr)
        sys.exit(2)

    pairs = []
    if solver.map:
        for i, row in enumerate(solver.map):
            for j, var in enumerate(row):
                if hasattr(var, "solution_value") and var.solution_value() > 0.5:
                    score = solver.distMat[i][j]
                    src_poly = g1.get_segments()[i].get_poly()
                    tgt_poly = g2.get_segments()[j].get_poly()
                    _, flip, trans, rot = matching_polygon(src_poly, tgt_poly)
                    pairs.append((score, i, j, flip, trans, rot))

    pairs.sort(key=lambda t: t[0])

    with open(output_path, "w") as f:
        f.write(f"{len(pairs)}\n")
        for score, i, j, flip, trans, rot in pairs:
            f.write(f"{i} {j} {score:.6f} {flip} {trans[0]:.2f} {trans[1]:.2f} {rot:.2f}\n")

    print(f"Wrote {len(pairs)} segment pairs to {output_path}")

    quad_path = output_path[:-4] + "_quads.txt" if output_path.endswith(".txt") else output_path + "_quads.txt"
    poly_pairs = [
        (
            np.array(g1.get_segments()[i].get_poly(), dtype=float),
            np.array(g2.get_segments()[j].get_poly(), dtype=float),
        )
        for _, i, j, _, _, _ in pairs
    ]

    n_workers = int(os.environ.get("PARAFASHION_QUAD_WORKERS", "0")) or None
    backend = os.environ.get("PARAFASHION_QUAD_BACKEND", "process")
    vertex_counts_env = os.environ.get("PARAFASHION_APPROX_VERTICES", "4,5")
    try:
        vertex_counts = tuple(int(v.strip()) for v in vertex_counts_env.split(",") if v.strip())
    except ValueError:
        vertex_counts = (4, 5)

    if len(poly_pairs) > 1:
        print(
            f"Fitting {len(poly_pairs)} approx polygons (candidate verts={vertex_counts}) "
            f"(backend={backend}, workers={n_workers or 'auto'})..."
        )

    polygon_fit_tic = time.time()
    fit_results = fit_polygon_to_polygon_pairs(
        poly_pairs,
        n_workers=n_workers,
        backend=backend,
        on_error="none",
        vertex_counts=vertex_counts,
    )
    polygon_fit_seconds = time.time() - polygon_fit_tic
    print(f"[mapping_solver] polygon_fit_seconds={polygon_fit_seconds:.3f}")

    approx_records = []
    for (_, i, j, _, _, _), fit in zip(pairs, fit_results):
        if fit is None:
            print(f"Warning: polygon fit failed for pair ({i},{j})", file=sys.stderr)
            continue
        approx_records.append((i, j, fit))

    with open(quad_path, "w") as f:
        f.write(f"{len(approx_records)}\n")
        for i, j, fit in approx_records:
            n = fit.n_vertices
            poly = fit.poly
            f.write(f"{i} {j} {n}\n")
            f.write(f"{fit.score:.6f}\n")
            f.write(" ".join(f"{poly[k, 0]:.6f} {poly[k, 1]:.6f}" for k in range(n)) + "\n")
            f.write(
                f"{int(fit.placement_a.flip)} {fit.placement_a.trans[0]:.6f} "
                f"{fit.placement_a.trans[1]:.6f} {fit.placement_a.rot:.6f}\n"
            )
            f.write(
                f"{int(fit.placement_b.flip)} {fit.placement_b.trans[0]:.6f} "
                f"{fit.placement_b.trans[1]:.6f} {fit.placement_b.rot:.6f}\n"
            )

    print(f"Wrote {len(approx_records)} polygon approximations to {quad_path}")


if __name__ == "__main__":
    main()
