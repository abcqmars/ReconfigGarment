#ifndef UV_BACK_PROJECTION_H
#define UV_BACK_PROJECTION_H
/**
 * Back-project a UV-mapped mesh onto another mesh in 3D using UV barycentric lookup.
 *
 * The caller is responsible for ensuring that:
 *  - Both meshes are TraceMesh-compatible (see tracing/mesh_type.h).
 *  - The source mesh has meaningful per-wedge UVs in WT().P().
 *  - The destination mesh has UVs that are already aligned to the source UV space
 *    (e.g. after applying a 2D rigid transform).
 *
 * This utility does NOT modify UV coordinates; it only overwrites the 3D positions
 * of the destination mesh vertices so that its UV layout is back-projected onto
 * the source surface.
 */

#include <tracing/mesh_type.h>
#include <limits>
#include <tuple>
#include <vector>

namespace uv_back_projection {

using MeshType    = TraceMesh;
using VertexType  = MeshType::VertexType;
using FaceType    = MeshType::FaceType;
using CoordType   = MeshType::CoordType;
using ScalarType  = MeshType::ScalarType;
using UVCoordType = vcg::Point2<ScalarType>;

struct BaryHit {
    int faceIndex = -1;
    ScalarType w0 = 0;
    ScalarType w1 = 0;
    ScalarType w2 = 0;
};

// ----- Point-to-segment and point-to-triangle distance (2D) -----

// Returns squared distance from p to segment [a,b], and parameter t in [0,1]
// such that closest point = a + t*(b - a).
inline std::pair<ScalarType, ScalarType> SquaredDistancePointToSegment2D(
    const UVCoordType &p, const UVCoordType &a, const UVCoordType &b)
{
    UVCoordType ab(b.X() - a.X(), b.Y() - a.Y());
    UVCoordType ap(p.X() - a.X(), p.Y() - a.Y());
    ScalarType ab2 = ab.X() * ab.X() + ab.Y() * ab.Y();
    ScalarType t;
    if (ab2 <= ScalarType(0)) {
        t = ScalarType(0);
    } else {
        t = (ap.X() * ab.X() + ap.Y() * ab.Y()) / ab2;
        if (t < ScalarType(0)) t = ScalarType(0);
        else if (t > ScalarType(1)) t = ScalarType(1);
    }
    UVCoordType q(a.X() + t * ab.X(), a.Y() + t * ab.Y());
    ScalarType dx = p.X() - q.X();
    ScalarType dy = p.Y() - q.Y();
    return { dx * dx + dy * dy, t };
}

// Returns squared distance from p to the triangle (a,b,c), and barycentric
// (w0,w1,w2) of the closest point on the triangle (interior, edge, or vertex).
inline std::pair<ScalarType, std::tuple<ScalarType, ScalarType, ScalarType>>
SquaredDistancePointToTriangle2D(const UVCoordType &p,
                                  const UVCoordType &a,
                                  const UVCoordType &b,
                                  const UVCoordType &c,
                                  ScalarType eps = ScalarType(1e-10))
{
    ScalarType denom =
        (b.Y() - c.Y()) * (a.X() - c.X()) +
        (c.X() - b.X()) * (a.Y() - c.Y());
    if (std::abs(denom) < eps) {
        // Degenerate triangle: treat as segment (a,b) and use segment distance.
        auto [d2ab, tab] = SquaredDistancePointToSegment2D(p, a, b);
        return { d2ab, { ScalarType(1) - tab, tab, ScalarType(0) } };
    }

    ScalarType w0 =
        ( (b.Y() - c.Y()) * (p.X() - c.X()) +
          (c.X() - b.X()) * (p.Y() - c.Y()) ) / denom;
    ScalarType w1 =
        ( (c.Y() - a.Y()) * (p.X() - c.X()) +
          (a.X() - c.X()) * (p.Y() - c.Y()) ) / denom;
    ScalarType w2 = ScalarType(1) - w0 - w1;

    // Inside triangle: closest point is p's projection (barycentric).
    if (w0 >= -eps && w1 >= -eps && w2 >= -eps &&
        w0 <= ScalarType(1) + eps &&
        w1 <= ScalarType(1) + eps &&
        w2 <= ScalarType(1) + eps) {
        UVCoordType q(a.X() * w0 + b.X() * w1 + c.X() * w2,
                      a.Y() * w0 + b.Y() * w1 + c.Y() * w2);
        ScalarType dx = p.X() - q.X();
        ScalarType dy = p.Y() - q.Y();
        return { dx * dx + dy * dy, { w0, w1, w2 } };
    }

    // Outside: closest point is on one of the three edges.
    auto [d2_ab, t_ab] = SquaredDistancePointToSegment2D(p, a, b);
    auto [d2_bc, t_bc] = SquaredDistancePointToSegment2D(p, b, c);
    auto [d2_ca, t_ca] = SquaredDistancePointToSegment2D(p, c, a);

    if (d2_ab <= d2_bc && d2_ab <= d2_ca) {
        ScalarType w0e = ScalarType(1) - t_ab;
        ScalarType w1e = t_ab;
        ScalarType w2e = ScalarType(0);
        return { d2_ab, { w0e, w1e, w2e } };
    }
    if (d2_bc <= d2_ca) {
        ScalarType w0e = ScalarType(0);
        ScalarType w1e = ScalarType(1) - t_bc;
        ScalarType w2e = t_bc;
        return { d2_bc, { w0e, w1e, w2e } };
    }
    ScalarType w0e = t_ca;
    ScalarType w1e = ScalarType(0);
    ScalarType w2e = ScalarType(1) - t_ca;
    return { d2_ca, { w0e, w1e, w2e } };
}

// ----- Helpers used by BackProjectMeshUV -----

// Barycentric coordinates of point p with respect to triangle (a,b,c) in UV.
// Valid when p is inside or outside the triangle (extrapolation).
inline void BarycentricCoordsOfPoint(const UVCoordType &p,
                                      const UVCoordType &a,
                                      const UVCoordType &b,
                                      const UVCoordType &c,
                                      ScalarType &w0, ScalarType &w1, ScalarType &w2,
                                      ScalarType eps = ScalarType(1e-10))
{
    ScalarType denom =
        (b.Y() - c.Y()) * (a.X() - c.X()) +
        (c.X() - b.X()) * (a.Y() - c.Y());
    if (std::abs(denom) < eps) {
        w0 = ScalarType(1);
        w1 = ScalarType(0);
        w2 = ScalarType(0);
        return;
    }
    w0 = ( (b.Y() - c.Y()) * (p.X() - c.X()) +
           (c.X() - b.X()) * (p.Y() - c.Y()) ) / denom;
    w1 = ( (c.Y() - a.Y()) * (p.X() - c.X()) +
           (a.X() - c.X()) * (p.Y() - c.Y()) ) / denom;
    w2 = ScalarType(1) - w0 - w1;
}

inline std::vector<UVCoordType> CollectVertexUV(const MeshType &m)
{
    std::vector<UVCoordType> uvs(m.vert.size(), UVCoordType(0, 0));
    std::vector<bool> has(m.vert.size(), false);

    for (size_t fi = 0; fi < m.face.size(); ++fi) {
        const FaceType &f = m.face[fi];
        if (f.IsD()) continue;
        for (int j = 0; j < 3; ++j) {
            const VertexType *v = f.V(j);
            int vi = vcg::tri::Index(m, v);
            if (vi < 0 || static_cast<size_t>(vi) >= m.vert.size()) continue;
            if (!has[vi]) {
                uvs[vi] = f.WT(j).P();
                has[vi] = true;
            }
        }
    }
    return uvs;
}

inline BaryHit PointInTriangleBarycentric(const UVCoordType &p,
                                          const UVCoordType &a,
                                          const UVCoordType &b,
                                          const UVCoordType &c,
                                          ScalarType eps = ScalarType(1e-6))
{
    ScalarType denom =
        (b.Y() - c.Y()) * (a.X() - c.X()) +
        (c.X() - b.X()) * (a.Y() - c.Y());

    BaryHit hit;
    if (std::abs(denom) < eps)
        return hit;

    ScalarType w0 =
        ( (b.Y() - c.Y()) * (p.X() - c.X()) +
          (c.X() - b.X()) * (p.Y() - c.Y()) ) / denom;
    ScalarType w1 =
        ( (c.Y() - a.Y()) * (p.X() - c.X()) +
          (a.X() - c.X()) * (p.Y() - c.Y()) ) / denom;
    ScalarType w2 = ScalarType(1) - w0 - w1;

    if (w0 >= -eps && w1 >= -eps && w2 >= -eps &&
        w0 <= ScalarType(1) + eps &&
        w1 <= ScalarType(1) + eps &&
        w2 <= ScalarType(1) + eps) {
        hit.faceIndex = 0; // caller overwrites with actual index
        hit.w0 = w0;
        hit.w1 = w1;
        hit.w2 = w2;
    }
    return hit;
}

inline BaryHit FindContainingFaceUV(const MeshType &src, const UVCoordType &p)
{
    const ScalarType eps = ScalarType(1e-5);
    BaryHit best;
    ScalarType bestMargin = -std::numeric_limits<ScalarType>::infinity();

    for (size_t fi = 0; fi < src.face.size(); ++fi) {
        const FaceType &f = src.face[fi];
        if (f.IsD()) continue;

        UVCoordType a = f.WT(0).P();
        UVCoordType b = f.WT(1).P();
        UVCoordType c = f.WT(2).P();

        BaryHit h = PointInTriangleBarycentric(p, a, b, c, eps);
        if (h.faceIndex < 0) continue;

        ScalarType margin = std::min(h.w0, std::min(h.w1, h.w2));
        if (margin > bestMargin) {
            bestMargin = margin;
            best = h;
            best.faceIndex = static_cast<int>(fi);
        }
    }
    return best;
}

struct FaceCandidate {
    ScalarType d2;
    int fi;  // -1 if none found
    ScalarType w0, w1, w2;
};

// Find the nearest face in UV space using point-to-triangle distance (closest
// point may be on an edge or vertex). Returns a candidate with fi == -1 if no faces.
inline FaceCandidate FindNearestFaceInUV(const MeshType &tgtMesh, const UVCoordType &p)
{
    FaceCandidate best{ std::numeric_limits<ScalarType>::max(), -1, 0, 0, 0 };
    const ScalarType eps = ScalarType(1e-10);

    for (size_t fi = 0; fi < tgtMesh.face.size(); ++fi) {
        const FaceType &f = tgtMesh.face[fi];
        if (f.IsD()) continue;

        UVCoordType a = f.WT(0).P();
        UVCoordType b = f.WT(1).P();
        UVCoordType c = f.WT(2).P();

        auto [d2, bary] = SquaredDistancePointToTriangle2D(p, a, b, c, eps);
        if (d2 < best.d2) {
            best.d2 = d2;
            best.fi = static_cast<int>(fi);
            best.w0 = std::get<0>(bary);
            best.w1 = std::get<1>(bary);
            best.w2 = std::get<2>(bary);
        }
    }
    return best;
}

} // namespace uv_back_projection

// Back-project srcMesh vertices onto tgtMesh using UVs.
// tgtMesh is read-only; srcMesh vertex positions are updated in-place.
inline void BackProjectMeshUV(TraceMesh &srcMesh, const TraceMesh &tgtMesh)
{
    using namespace uv_back_projection;

    std::vector<UVCoordType> srcUV = CollectVertexUV(srcMesh);
    if (srcUV.size() != srcMesh.vert.size())
        srcUV.resize(srcMesh.vert.size(), UVCoordType(0, 0));

    for (size_t vi = 0; vi < srcMesh.vert.size(); ++vi) {
        if (vi >= srcUV.size()) break;
        UVCoordType p = srcUV[vi];

        BaryHit hit = FindContainingFaceUV(tgtMesh, p);
        if (hit.faceIndex < 0) {
            FaceCandidate near = FindNearestFaceInUV(tgtMesh, p);
            if (near.fi < 0) continue;
            hit.faceIndex = near.fi;
            const FaceType &nf = tgtMesh.face[near.fi];
            UVCoordType a = nf.WT(0).P(), b = nf.WT(1).P(), c = nf.WT(2).P();
            BarycentricCoordsOfPoint(p, a, b, c, hit.w0, hit.w1, hit.w2);
        }

        const FaceType &f = tgtMesh.face[hit.faceIndex];
        CoordType p0 = f.V(0)->P();
        CoordType p1 = f.V(1)->P();
        CoordType p2 = f.V(2)->P();
        srcMesh.vert[vi].P() = p0 * hit.w0 + p1 * hit.w1 + p2 * hit.w2;
    }
}

#endif // UV_BACK_PROJECTION_H

