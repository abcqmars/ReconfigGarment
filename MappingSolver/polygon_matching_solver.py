import os
import numpy as np
from concurrent.futures import ThreadPoolExecutor
from shapely.geometry import Polygon as ShapelyPolygon

import turning_function


def _centroid(poly):
    if poly.shape[0] == 0:
        return np.zeros(2, dtype=float)
    p = ShapelyPolygon(poly)
    if not p.is_valid:
        p = p.buffer(0)
    c = p.centroid
    return np.array([c.x, c.y], dtype=float)


def _make_ccw(poly):
    if poly.shape[0] < 3:
        return poly.copy()

    is_closed = np.allclose(poly[0], poly[-1])
    verts = poly[:-1] if is_closed else poly
    x = verts[:, 0]
    y = verts[:, 1]
    area2 = np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))

    if area2 > 0:
        ccw = verts.copy()
    else:
        ccw = verts[::-1].copy()

    if is_closed:
        ccw = np.vstack([ccw, ccw[0]])
    return ccw


def _flipped_polygon(poly):
    if poly.shape[0] < 3:
        return poly.copy()

    is_closed = np.allclose(poly[0], poly[-1])
    verts = poly[:-1] if is_closed else poly
    center_x = verts[:, 0].mean()
    flipped_verts = np.column_stack([2 * center_x - verts[:, 0], verts[:, 1]])

    if is_closed:
        flipped_verts = np.vstack([flipped_verts, flipped_verts[0]])
    return _make_ccw(flipped_verts)


def _best_overlap_translation(src_poly, tgt_poly, n_steps=100, center_drift_weight=0.10, max_workers=None):
    if src_poly.shape[0] < 3 or tgt_poly.shape[0] < 3:
        return src_poly.copy(), np.zeros(2, dtype=float), 0.0

    cA = _centroid(src_poly)
    cB = _centroid(tgt_poly)
    A0 = src_poly - cA + cB

    polyB = ShapelyPolygon(tgt_poly)
    if not polyB.is_valid:
        polyB = polyB.buffer(0)

    best_overlap = 0.0
    best_score = -np.inf
    best_t = np.zeros(2, dtype=float)

    U_extend = np.max(tgt_poly[:, 0]) - np.min(tgt_poly[:, 0])
    V_extend = np.max(tgt_poly[:, 1]) - np.min(tgt_poly[:, 1])
    search_radius = max(U_extend, V_extend)
    if search_radius <= 1e-12:
        return A0.copy(), np.zeros(2, dtype=float), 0.0

    tx_vals = np.linspace(-search_radius, search_radius, n_steps)
    ty_vals = np.linspace(-search_radius, search_radius, n_steps)
    target_area = max(polyB.area, 1e-12)
    inv_search_radius = 1.0 / search_radius

    def _score(overlap_area, tx, ty):
        overlap_norm = overlap_area / target_area
        drift_norm = np.hypot(tx, ty) * inv_search_radius
        return overlap_norm - center_drift_weight * drift_norm

    def _evaluate_tx(tx):
        local_best_overlap = 0.0
        local_best_score = -np.inf
        local_best_t = np.zeros(2, dtype=float)
        for ty in ty_vals:
            moved = A0 + np.array([tx, ty], dtype=float)
            polyA = ShapelyPolygon(moved)
            if not polyA.is_valid:
                polyA = polyA.buffer(0)
            overlap = polyA.intersection(polyB).area
            score = _score(overlap, tx, ty)
            if score > local_best_score or (
                np.isclose(score, local_best_score) and overlap > local_best_overlap
            ):
                local_best_score = score
                local_best_overlap = overlap
                local_best_t = np.array([tx, ty], dtype=float)
        return local_best_score, local_best_overlap, local_best_t

    if max_workers is None:
        max_workers = min(len(tx_vals), os.cpu_count() or 1)
    max_workers = max(1, int(max_workers))

    if max_workers == 1:
        row_results = (_evaluate_tx(tx) for tx in tx_vals)
    else:
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            row_results = executor.map(_evaluate_tx, tx_vals)

    for row_score, row_overlap, row_t in row_results:
        if row_score > best_score or (
            np.isclose(row_score, best_score) and row_overlap > best_overlap
        ):
            best_score = row_score
            best_overlap = row_overlap
            best_t = row_t

    return A0 + best_t, best_t, best_overlap


def matching_polygon(src_poly, tgt_poly):
    src_poly = np.asarray(src_poly, dtype=float)
    tgt_poly = np.asarray(tgt_poly, dtype=float)
    if src_poly.ndim != 2 or src_poly.shape[1] != 2:
        raise ValueError("src_poly must be of shape (N, 2)")
    if tgt_poly.ndim != 2 or tgt_poly.shape[1] != 2:
        raise ValueError("tgt_poly must be of shape (M, 2)")
    if src_poly.shape[0] < 3 or tgt_poly.shape[0] < 3:
        return float("inf"), False, np.zeros(2, dtype=float), 0.0

    poly_src = _make_ccw(src_poly)
    poly_tgt = _make_ccw(tgt_poly)
    dist_orig, theta_orig, _, _ = turning_function.distance(
        poly_src, poly_tgt, brute_force_updates=False
    )

    poly_src_flip = _flipped_polygon(poly_src)
    src_flip_open = (
        poly_src_flip[:-1]
        if np.allclose(poly_src_flip[0], poly_src_flip[-1])
        else poly_src_flip
    )
    dist_flip, theta_flip, _, _ = turning_function.distance(
        src_flip_open, poly_tgt, brute_force_updates=False
    )

    if dist_flip < dist_orig:
        score = float(dist_flip)
        theta = float(theta_flip)
        poly_src_used = poly_src_flip
        flip_used = True
    else:
        score = float(dist_orig)
        theta = float(theta_orig)
        poly_src_used = poly_src
        flip_used = False

    R = np.array(
        [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]],
        dtype=float,
    )
    A_rot = poly_src_used @ R.T
    cA = _centroid(A_rot)
    cB = _centroid(poly_tgt)
    t_centroid = cB - cA
    _, t_refine, _ = _best_overlap_translation(
        A_rot + t_centroid, poly_tgt, n_steps=200, max_workers=8
    )
    return score, flip_used, t_centroid + t_refine, theta
