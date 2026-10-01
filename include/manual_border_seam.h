#ifndef MANUAL_BORDER_SEAM_H
#define MANUAL_BORDER_SEAM_H

#include "garment_session.h"

#include <cstddef>
#include <vector>

// Snap a 3D pick (from OpenGL unproject) to the closest point on a mesh border edge.
// Border edges are mesh boundaries (IsBorder) and existing seam edges (IsFaceEdgeS).
struct BorderVertexPick
{
    size_t vertexIndex = 0;
    GarmentSession::CoordType position = GarmentSession::CoordType(0, 0, 0);
    bool ok = false;
};

BorderVertexPick SnapPickToBorderVertex(
    GarmentSession::MeshType &mesh,
    const GarmentSession::CoordType &pickPos,
    GarmentSession::ScalarType maxSnapDistance);

// Shortest-path vertex sequence between two mesh vertices (geodesic on triangle mesh).
bool ComputeGeodesicVertexPath(
    GarmentSession::MeshType &mesh,
    size_t vertexIndex0,
    size_t vertexIndex1,
    std::vector<size_t> &outVertexPath);

// Fill non-adjacent gaps with geodesic segments and smooth (PathUI::CompletePath + SmoothPaths).
// Updates vertex positions on the mesh along the path (same as PathUI::AddSharpConstraints).
void RefineAndSmoothVertexPath(
    GarmentSession::MeshType &mesh,
    std::vector<size_t> &vertexPath,
    size_t smoothSteps = 3);

// Re-snap smoothed 3D positions to mesh vertices (PathUI::SnapPathOnVertices).
void ResnapVertexPathFromSmoothedPositions(
    GarmentSession::MeshType &mesh,
    std::vector<size_t> &vertexPath);

// Bridge non-adjacent vertices with geodesic segments (PathUI::CompletePath).
void CompleteVertexPath(
    GarmentSession::MeshType &mesh,
    std::vector<size_t> &vertexPath);

// Mark every edge along a vertex path as a cutting seam (FaceEdgeS + SharpFeatures).
bool ApplyVertexPathAsCuttingSeam(
    GarmentSession::MeshType &mesh,
    const std::vector<size_t> &vertexPath);

// Convert a vertex path to 3D polyline coordinates (for GL drawing).
std::vector<GarmentSession::CoordType> VertexPathToPolyline(
    const GarmentSession::MeshType &mesh,
    const std::vector<size_t> &vertexPath);

BorderVertexPick SnapBorderPickOnMesh(
    GarmentSession &session,
    const GarmentSession::CoordType &pickPos);

#endif // MANUAL_BORDER_SEAM_H
