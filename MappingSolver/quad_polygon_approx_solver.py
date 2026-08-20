from __future__ import annotations

import os
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

MIN_VERTICES = 4
MAX_VERTICES = 5


@dataclass(frozen=True)
class Placement:
    flip: bool
    trans: np.ndarray
    rot: float


@dataclass(frozen=True)
class ApproxFitResult:
    poly: np.ndarray
    placement_a: Placement
    placement_b: Placement
    n_vertices: int
    score: float


def _as_open_poly(poly):
    poly = np.asarray(poly, dtype=float)
    if poly.ndim != 2 or poly.shape[1] != 2:
        raise ValueError("Polygon must have shape (N, 2)")
    if poly.shape[0] >= 2 and np.allclose(poly[0], poly[-1]):
        poly = poly[:-1]
    return poly


def _to_approx(poly):
    from patchRep import ApproxPolygon

    p = ApproxPolygon()
    p.verts2D = [[float(x), float(y)] for x, y in _as_open_poly(poly)]
    p.init_metric()
    p.create_flippedCopy()
    return p


def _min_area_obb_quad(poly):
    verts = _as_open_poly(poly)
    if verts.shape[0] < 3:
        raise ValueError("Need at least 3 points to build OBB.")

    c = verts.mean(axis=0)
    X = verts - c
    cov = (X.T @ X) / max(1, X.shape[0])
    evals, evecs = np.linalg.eigh(cov)
    v = evecs[:, int(np.argmax(evals))]
    angle = float(np.arctan2(v[1], v[0]))

    ca, sa = np.cos(-angle), np.sin(-angle)
    R = np.array([[ca, -sa], [sa, ca]], dtype=float)
    Xr = X @ R.T
    mn = Xr.min(axis=0)
    mx = Xr.max(axis=0)
    rect_r = np.array([[mn[0], mn[1]], [mx[0], mn[1]], [mx[0], mx[1]], [mn[0], mx[1]]], dtype=float)

    ca, sa = np.cos(angle), np.sin(angle)
    Rb = np.array([[ca, -sa], [sa, ca]], dtype=float)
    return rect_r @ Rb.T + c


def _cross2(a, b):
    return float(a[0] * b[1] - a[1] * b[0])


def _segment_intersect(p1, p2, q1, q2):
    def orient(a, b, c):
        return _cross2(b - a, c - a)

    def on_segment(a, b, c):
        return (
            min(a[0], b[0]) - 1e-12 <= c[0] <= max(a[0], b[0]) + 1e-12
            and min(a[1], b[1]) - 1e-12 <= c[1] <= max(a[1], b[1]) + 1e-12
        )

    o1 = orient(p1, p2, q1)
    o2 = orient(p1, p2, q2)
    o3 = orient(q1, q2, p1)
    o4 = orient(q1, q2, p2)
    if (o1 * o2 < 0) and (o3 * o4 < 0):
        return True
    if abs(o1) < 1e-12 and on_segment(p1, p2, q1):
        return True
    if abs(o2) < 1e-12 and on_segment(p1, p2, q2):
        return True
    if abs(o3) < 1e-12 and on_segment(q1, q2, p1):
        return True
    if abs(o4) < 1e-12 and on_segment(q1, q2, p2):
        return True
    return False


def _polygon_signed_area(poly):
    x = poly[:, 0]
    y = poly[:, 1]
    return float(0.5 * (np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def _polygon_area(poly):
    return float(abs(_polygon_signed_area(_as_open_poly(poly))))


def _log_area_mismatch_penalty(poly_area, area_a, area_b):
    q = max(float(poly_area), 1e-12)
    a = max(float(area_a), 1e-12)
    b = max(float(area_b), 1e-12)
    return abs(np.log(q) - np.log(a)) + abs(np.log(q) - np.log(b))


def _is_simple_polygon(poly):
    poly = np.asarray(poly, dtype=float)
    n = poly.shape[0]
    if n < 3 or poly.shape[1] != 2:
        return False
    if abs(_polygon_signed_area(poly)) <= 1e-12:
        return False
    for i in range(n):
        for j in range(i + 1, n):
            if j == i or j == (i + 1) % n or i == (j + 1) % n:
                continue
            if _segment_intersect(poly[i], poly[(i + 1) % n], poly[j], poly[(j + 1) % n]):
                return False
    return True


def _regularization_penalty(poly, n_vertices):
    poly = np.asarray(poly, dtype=float)
    if poly.shape != (n_vertices, 2):
        return 1e6
    if not _is_simple_polygon(poly):
        return 1e6
    area = float(abs(_polygon_signed_area(poly)))
    if area < 1e-10:
        return 1e6 + (1e-10 - area) * 1e10
    edges = np.roll(poly, -1, axis=0) - poly
    lens = np.linalg.norm(edges, axis=1)
    short = np.maximum(0.0, (np.min(lens) + 1e-12) ** -1 - 1e2)
    return float(short * 1e2)


def _init_n_gon(poly, n_vertices):
    if n_vertices < 4:
        raise ValueError("n_vertices must be >= 4")
    poly_init = _min_area_obb_quad(poly)
    while poly_init.shape[0] < n_vertices:
        edges = np.roll(poly_init, -1, axis=0) - poly_init
        i = int(np.argmax(np.linalg.norm(edges, axis=1)))
        mid = 0.5 * (poly_init[i] + poly_init[(i + 1) % poly_init.shape[0]])
        poly_init = np.insert(poly_init, i + 1, mid, axis=0)
    return poly_init


def _point_segment_distance(p, a, b):
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-16:
        return float(np.linalg.norm(p - a))
    t = min(1.0, max(0.0, float(np.dot(p - a, ab) / denom)))
    return float(np.linalg.norm(p - (a + t * ab)))


def _point_in_polygon(p, poly):
    inside = False
    n = poly.shape[0]
    x, y = float(p[0]), float(p[1])
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            x_int = (x2 - x1) * (y - y1) / ((y2 - y1) + 1e-16) + x1
            if x < x_int:
                inside = not inside
    return inside


def _outside_distance_to_polygon(p, poly):
    if _point_in_polygon(p, poly):
        return 0.0
    n = poly.shape[0]
    return float(min(_point_segment_distance(p, poly[i], poly[(i + 1) % n]) for i in range(n)))


def _principal_axis_angle(poly):
    c = poly.mean(axis=0)
    X = poly - c
    cov = (X.T @ X) / max(1, X.shape[0])
    evals, evecs = np.linalg.eigh(cov)
    v = evecs[:, int(np.argmax(evals))]
    return float(np.arctan2(v[1], v[0]))


def _rotate_translate(poly, theta, t):
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]], dtype=float)
    return poly @ R.T + t


def _align_poly_to_target_fast(poly, target, flip):
    src = np.asarray(poly, dtype=float)
    if flip:
        cx = src[:, 0].mean()
        src = np.column_stack([2.0 * cx - src[:, 0], src[:, 1]])
    theta = _principal_axis_angle(target) - _principal_axis_angle(src)
    t = target.mean(axis=0) - _rotate_translate(src, theta, np.zeros(2, dtype=float)).mean(axis=0)
    return _rotate_translate(src, theta, t)


def _coverage_penalty(poly, target):
    aligned_0 = _align_poly_to_target_fast(poly, target, flip=False)
    aligned_1 = _align_poly_to_target_fast(poly, target, flip=True)
    diag = float(np.linalg.norm(target.max(axis=0) - target.min(axis=0)) + 1e-12)

    def score(aligned_poly):
        d2 = 0.0
        for p in target:
            d = _outside_distance_to_polygon(p, aligned_poly)
            d2 += (d / diag) ** 2
        return d2 / max(1, target.shape[0])

    return float(min(score(aligned_0), score(aligned_1)))


def _evaluate_objective(
    poly,
    target_a,
    target_b,
    target_a_raw,
    target_b_raw,
    area_a,
    area_b,
    balance_weight,
    area_weight,
    cover_weight,
    oversize_weight,
    hard_coverage,
    coverage_tol,
    n_vertices,
):
    reg = _regularization_penalty(poly, n_vertices)
    if reg >= 1e6:
        return float(reg), float("inf"), float("inf"), 0.0, 0.0, 0.0, float(reg)

    poly_rep = _to_approx(poly)
    da = float(poly_rep.distance(target_a))
    db = float(poly_rep.distance(target_b))
    poly_area = abs(_polygon_signed_area(poly))
    area_pen = _log_area_mismatch_penalty(poly_area, area_a, area_b)
    cover_pen = _coverage_penalty(poly, target_a_raw) + _coverage_penalty(poly, target_b_raw)
    if hard_coverage and cover_pen > float(coverage_tol):
        return 1e6 + cover_pen * 1e6, da, db, area_pen, cover_pen, 0.0, float(reg)

    max_input_area = max(area_a, area_b)
    oversize_pen = max(0.0, np.log(max(poly_area, 1e-12) / max(max_input_area, 1e-12))) ** 2
    imbalance = (da - db) ** 2
    total = float(
        da
        + db
        + float(balance_weight) * imbalance
        + float(area_weight) * area_pen
        + float(cover_weight) * cover_pen
        + float(oversize_weight) * oversize_pen
        + reg
    )
    return total, da, db, area_pen, cover_pen, oversize_pen, reg


def _fit_n_gon_to_polygon_pair(
    poly_a,
    poly_b,
    n_vertices,
    *,
    maxiter=800,
    xatol=1e-6,
    fatol=1e-6,
    random_restarts=6,
    restart_sigma=0.02,
    balance_weight=1.0,
    area_weight=1.0,
    cover_weight=4.0,
    oversize_weight=1.5,
    hard_coverage=True,
    coverage_tol=1e-4,
    verbose=False,
    print_every=25,
):
    from scipy.optimize import minimize
    from polygon_matching_solver import matching_polygon

    if n_vertices < MIN_VERTICES or n_vertices > MAX_VERTICES:
        raise ValueError(f"n_vertices must be in [{MIN_VERTICES}, {MAX_VERTICES}]")

    poly_a = _as_open_poly(poly_a)
    poly_b = _as_open_poly(poly_b)
    if poly_a.shape[0] < 3 or poly_b.shape[0] < 3:
        raise ValueError("Both polygons must have at least 3 vertices.")

    target_a = _to_approx(poly_a)
    target_b = _to_approx(poly_b)
    area_a = _polygon_area(poly_a)
    area_b = _polygon_area(poly_b)

    init_poly = poly_a if area_a >= area_b else poly_b
    x0 = _init_n_gon(init_poly, n_vertices).reshape(-1)
    bbox_min = np.min(init_poly, axis=0)
    bbox_max = np.max(init_poly, axis=0)
    diag = float(np.linalg.norm(bbox_max - bbox_min) + 1e-12)

    best_x = x0.copy()
    best_f = float("inf")
    rng = np.random.default_rng(0)
    restart_seeds = [x0]
    if random_restarts > 0:
        jitter_scale = restart_sigma * diag
        restart_seeds.extend(x0 + rng.normal(scale=jitter_scale, size=x0.shape) for _ in range(random_restarts))

    for seed in restart_seeds:
        iter_state = {"n": 0, "best": float("inf")}

        def obj_wrapped(x, a, b):
            f, _, _, _, _, _, _ = _evaluate_objective(
                x.reshape(n_vertices, 2),
                a,
                b,
                poly_a,
                poly_b,
                area_a,
                area_b,
                balance_weight,
                area_weight,
                cover_weight,
                oversize_weight,
                hard_coverage,
                coverage_tol,
                n_vertices,
            )
            if verbose:
                iter_state["n"] += 1
                if np.isfinite(f) and f < iter_state["best"]:
                    iter_state["best"] = float(f)
                if iter_state["n"] % max(1, int(print_every)) == 0:
                    print(f"[approx-fit] eval={iter_state['n']} f={float(f):.6g} best={iter_state['best']:.6g}")
            return f

        res = minimize(
            obj_wrapped,
            seed,
            args=(target_a, target_b),
            method="Nelder-Mead",
            options={"maxiter": int(maxiter), "xatol": float(xatol), "fatol": float(fatol), "adaptive": True},
        )
        f = float(res.fun)
        if f < best_f and np.isfinite(f):
            candidate = res.x.reshape(n_vertices, 2)
            if _is_simple_polygon(candidate):
                best_f = f
                best_x = res.x.copy()

    poly = best_x.reshape(n_vertices, 2)
    score, _, _, _, _, _, _ = _evaluate_objective(
        poly,
        target_a,
        target_b,
        poly_a,
        poly_b,
        area_a,
        area_b,
        balance_weight,
        area_weight,
        cover_weight,
        oversize_weight,
        hard_coverage,
        coverage_tol,
        n_vertices,
    )

    _, flip_a, trans_a, rot_a = matching_polygon(poly, poly_a)
    _, flip_b, trans_b, rot_b = matching_polygon(poly, poly_b)
    placement_a = Placement(bool(flip_a), np.asarray(trans_a, dtype=float), float(rot_a))
    placement_b = Placement(bool(flip_b), np.asarray(trans_b, dtype=float), float(rot_b))
    return poly, placement_a, placement_b, float(score)


def fit_polygon_to_polygon_pair(poly_a, poly_b, *, vertex_counts=(4, 5), **fit_kwargs):
    best = None
    errors = []
    for n in vertex_counts:
        if n < MIN_VERTICES or n > MAX_VERTICES:
            raise ValueError(f"vertex_counts entries must be in [{MIN_VERTICES}, {MAX_VERTICES}]")
        try:
            poly, pa, pb, score = _fit_n_gon_to_polygon_pair(poly_a, poly_b, n, **fit_kwargs)
            cand = ApproxFitResult(poly=poly, placement_a=pa, placement_b=pb, n_vertices=n, score=score)
            if best is None or cand.score < best.score:
                best = cand
        except Exception as exc:
            errors.append(f"n={n}: {exc}")

    if best is None:
        msg = "; ".join(errors) if errors else "unknown error"
        raise RuntimeError(f"Polygon fit failed for all vertex counts: {msg}")
    return best


def _default_parallel_workers():
    return max(1, (os.cpu_count() or 1) - 1)


def _fit_polygon_worker(job):
    idx, poly_a, poly_b, fit_kwargs = job
    try:
        return idx, fit_polygon_to_polygon_pair(poly_a, poly_b, **fit_kwargs), None
    except Exception as exc:
        return idx, None, str(exc)


def fit_polygon_to_polygon_pairs(
    pairs,
    *,
    n_workers=None,
    backend="process",
    on_error="none",
    **fit_kwargs,
):
    if on_error not in ("none", "raise"):
        raise ValueError('on_error must be "none" or "raise"')

    n = len(pairs)
    if n == 0:
        return []

    kwargs = dict(fit_kwargs)
    kwargs.setdefault("verbose", False)
    jobs = [
        (i, np.asarray(poly_a, dtype=float), np.asarray(poly_b, dtype=float), kwargs)
        for i, (poly_a, poly_b) in enumerate(pairs)
    ]

    workers = n_workers if n_workers is not None else _default_parallel_workers()
    workers = max(1, min(int(workers), n))
    use_parallel = backend != "serial" and n > 1 and workers > 1
    results = [None] * n

    if not use_parallel:
        for job in jobs:
            idx, res, err = _fit_polygon_worker(job)
            if err is not None and on_error == "raise":
                raise RuntimeError(f"polygon fit failed for pair index {idx}: {err}") from None
            if err is None:
                results[idx] = res
        return results

    if backend not in ("process", "thread"):
        raise ValueError('backend must be "process", "thread", or "serial"')

    Executor = ProcessPoolExecutor if backend == "process" else ThreadPoolExecutor
    with Executor(max_workers=workers) as executor:
        for idx, res, err in executor.map(_fit_polygon_worker, jobs, chunksize=1):
            if err is not None and on_error == "raise":
                raise RuntimeError(f"polygon fit failed for pair index {idx}: {err}") from None
            if err is None:
                results[idx] = res
    return results
