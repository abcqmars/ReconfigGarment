#ifndef GARMENT_SESSION_H
#define GARMENT_SESSION_H

#include <string>
#include <utility>
#include <vector>
#include <cmath>

#include <tracing/mesh_type.h>
#include <vcg/space/color4.h>

// Forward declarations to avoid pulling heavy implementations
// (and their globals) into every translation unit.
template<class MeshType> class AnimationManager;
template<class MeshType> class Parafashion;

// Lightweight wrapper that groups together all per-garment
// processing state (meshes, animation, parafashion pipeline, etc.).
// This is the building block for supporting multiple garments.
struct GarmentSession
{
    using MeshType    = TraceMesh;
    using ScalarType  = MeshType::ScalarType;
    using CoordType   = MeshType::CoordType;

    // Core meshes for this garment.
    MeshType  deformed_mesh;
    MeshType  reference_mesh;
    MeshType  half_def_mesh;

    // Centers used to re-position meshes around the origin.
    CoordType centerDef;
    CoordType centerRef;

    // Source Path:
    std::string meshPath;

    // Per-garment managers (owned via pointers, constructed elsewhere).
    AnimationManager<MeshType> *AManager = nullptr;
    Parafashion<MeshType>      *PFashion = nullptr;

    bool hasFrames = false;

    // Polylines of manually added border-to-border seams (for GL overlay in dual view).
    std::vector<std::vector<CoordType>> manualBorderSeamPolylines;

    // Space+drag constraint paths for this garment only (dual view uses gPathA / gPathB).
    std::vector<std::vector<CoordType>> constraintPickedPoints;

    // Manual border-seam vertex paths to re-apply after batch (survives RestoreInitMesh).
    std::vector<std::vector<size_t>> manualSeamVertexPaths;

    // Post-batch manual segment candidates: each entry is a patch-id set for one segment.
    std::vector<std::vector<int>> manualSegmentPatchSets;
    // In-progress patch selection for the next manual segment candidate.
    std::vector<int> manualSegmentPickInProgress;

    GarmentSession();

    // Load deformed/reference meshes and optional animation frames.
    // Returns true on success, false on any load error.
    bool loadMeshes(const std::string &defPath,
                    const std::string &refPath,
                    const std::string &framesPath);
    void initMesh();
};

// Run the full parafashion pipeline (field, trace, parametrize, patch graph, segments) for one session.
void DoBatchProcess(GarmentSession &s);
void DoBatchProcessWithPickedPoints(
    GarmentSession &s,
    const std::vector<std::vector<GarmentSession::CoordType>> &pickedPoints);

// Export segment polygons to a text file (parafashion format for segmentMappingSolver).
// basePath: path without extension; writes basePath + "_polygons.txt".
void ExportPolygonsForSession(GarmentSession &s, const std::string &basePath);

// Procedural color for mapping pairs: same palette as segment mapping visualization.
// Used by ApplySegmentMappingColors and by back-projection overlay so colors match.
inline vcg::Color4b ColorForPair(int idx)
{
    const float golden = 0.6180339887f;
    float h = std::fmod(idx * golden, 1.0f);
    int tier = (idx / 32) % 3;
    float s = 0.8f;
    float v = 0.95f - 0.15f * tier;
    if (s <= 0.0f) {
        unsigned char g = (unsigned char)std::round(v * 255.0f);
        return vcg::Color4b(g, g, g, 255);
    }
    h *= 6.0f;
    int i = (int)std::floor(h);
    float f = h - i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - s * f);
    float t = v * (1.0f - s * (1.0f - f));
    float r, g, b;
    switch (i % 6) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
    }
    return vcg::Color4b(
        (unsigned char)std::round(std::max(0.f, std::min(1.f, r)) * 255.0f),
        (unsigned char)std::round(std::max(0.f, std::min(1.f, g)) * 255.0f),
        (unsigned char)std::round(std::max(0.f, std::min(1.f, b)) * 255.0f),
        255);
}

// Apply segment-mapping colors to a garment mesh (for dual view / export).
// pairs: (segment_idx_A, segment_idx_B); isGarmentA: true for left garment.
void ApplySegmentMappingColorsToMesh(GarmentSession &s,
                                     TraceMesh &mesh,
                                     const std::vector<std::pair<int,int>> &pairs,
                                     bool isGarmentA);

// Same as ApplySegmentMappingColorsToMesh on session.deformed_mesh.
void ApplySegmentMappingColors(GarmentSession &s,
                               const std::vector<std::pair<int,int>> &pairs,
                               bool isGarmentA);

// Color session mesh by patch (partition). Use to restore appearance after turning off segment mapping.
void ColorSessionByPatch(GarmentSession &s);

// Neutral grey face color (same as initial mesh load: 220,220,220).
void ColorSessionNeutralGrey(GarmentSession &s);

// When true, batch/recolor uses neutral grey instead of per-patch colors (dual view default).
extern bool gGarmentColorGreyMode;

// Build patch -> garment segment index for segments that appear in mapping pairs only.
// Unmapped patches are left as -1.
void BuildPaidToMappedSegment(GarmentSession &s,
                              const std::vector<std::pair<int, int>> &pairs,
                              bool isGarmentA,
                              std::vector<int> &paid2seg);

// Draw mesh border edges and edges between different segments only (no patch/seam edges).
// When mappingPairs is non-null, segment membership matches ApplySegmentMappingColors.
void GLDrawSegmentEdges(GarmentSession &session,
                        vcg::Color4b col = vcg::Color4b(0, 0, 0, 255),
                        float lineWidth = 5.0f,
                        const std::vector<std::pair<int, int>> *mappingPairs = nullptr,
                        bool isGarmentA = true);

// Check that, for a given garment session, all patches are covered by
// segments that appear in at least one mapping pair.
bool CheckPatchCoverageForSession(GarmentSession &s,
                                  const std::vector<std::pair<int,int>> &pairs,
                                  bool useFirstIndex);

// Compute the mean X coordinate of the ApproxPolygon vertices for a given segment
// in session s. Returns 0 if the segment or polygon is missing.
TraceMesh::ScalarType SegmentApproxPolygonCenterX(GarmentSession &s, int segIndex);

// Lightweight accessors for segment meshes without exposing Parafashion internals.
// Return number of segments for this garment (0 if PFashion/segments are missing).
int SessionNumSegments(GarmentSession &s);
int SessionNumPatches(GarmentSession &s);
bool SessionFacePatchId(GarmentSession &s, size_t faceIndex, int &patchIdOut);
bool SessionSegmentPaids(GarmentSession &s, int segIndex, std::vector<int> &outPaids);
bool SessionSegmentFacePatchId(GarmentSession &s, int segIndex, size_t segFaceIndex, int &patchIdOut);

// Return mapping pair index for segIndex, or -1 if not in any pair (respects gMaxMappingPairs).
int SessionMappingPairIndexForSegment(GarmentSession &s,
                                      const std::vector<std::pair<int, int>> &pairs,
                                      int segIndex,
                                      bool isGarmentA);

// Return pointer to the segment mesh for segIndex, or nullptr on failure.
TraceMesh* SessionSegmentMesh(GarmentSession &s, int segIndex);

// Copy Segment::polygon.polyVs for segIndex. Returns false if unavailable.
bool SessionApproxPolygonVerts(GarmentSession &s,
                               int segIndex,
                               std::vector<vcg::Point2<GarmentSession::ScalarType>> &outVerts);

// Segment approx polygon vertices mapped back to garment UV space.
bool SessionApproxPolygonGarmentUV(GarmentSession &s,
                                   int segIndex,
                                   std::vector<vcg::Point2<GarmentSession::ScalarType>> &outUV);

// Rigid placement of canonical approx polygon into garment UV (matches quad export).
struct ApproxPolygonPlacement
{
    bool flip = false;
    double tx = 0.0;
    double ty = 0.0;
    double rot = 0.0;  // radians
};

void TransformApproxPolygonWithPlacement(
    const std::vector<vcg::Point2<GarmentSession::ScalarType>> &canonical,
    const ApproxPolygonPlacement &placement,
    std::vector<vcg::Point2<GarmentSession::ScalarType>> &outGarmentUV);

void ScalePolygonVertsInPlace(std::vector<vcg::Point2<GarmentSession::ScalarType>> &verts,
                              GarmentSession::ScalarType linearScale);

std::string MappingColorToFileStem(const vcg::Color4b &color, int pairIndex);

double SessionGarmentMeshArea(GarmentSession &s);

bool WriteApproxPolygonTxt(const std::string &path,
                           const std::vector<vcg::Point2<GarmentSession::ScalarType>> &verts);

struct Polygon2DDrawItem
{
    std::vector<vcg::Point2<GarmentSession::ScalarType>> verts;
    vcg::Color4b color = vcg::Color4b(220, 220, 220, 255);
};

void GLDrawArrangedPolygonPatches(const std::vector<Polygon2DDrawItem> &items);

// Same UV packing as GLDrawArrangedPolygonPatches (PatchManager / PolyPacker grid).
void ArrangeTraceMeshesUVLayout(std::vector<TraceMesh *> &meshes);

void DrawArrangedApproxPolygonsFromSession(
    GarmentSession &s,
    const std::vector<std::pair<int, int>> *mappingPairs,
    bool isGarmentA);

// Given a temporary copy of a segment mesh whose per-vertex/per-wedge UVs are in
// "arranged segment space", convert its UVs back to the original garment UV space
// using the segment's Similarity2 transform (Segment::revertUVvert).
void SessionRevertSegmentUVToGarment(GarmentSession &s, int segIndex, TraceMesh &mesh);

// Post-batch manual border seams (dual view); implemented in myglwidget.cpp.
void RebuildGarmentPatchesFromSeams(GarmentSession &session);
bool AddManualBorderSeamToSession(GarmentSession &session,
                                  size_t borderVertex0,
                                  size_t borderVertex1);

// Re-apply manual border seams after a batch run on the same garment.
void MergeManualSeamsIntoMesh(GarmentSession &session);

// Post-batch manual segment candidates (dual view); implemented in myglwidget.cpp.
// Rebuild segments from the current patch graph, including manual patch sets.
void RebuildGarmentSegments(GarmentSession &session);

// Pick a patch at screen coordinates (after mesh draw). Returns patch id or -1.
int PickPatchIdAtScreen(GarmentSession &session, int screenX, int screenY);

// Toggle patch id in the in-progress manual segment selection.
bool TogglePatchInManualSegmentPick(GarmentSession &session, int patchId);

// Commit in-progress patches as a manual segment candidate and rebuild segments.
bool CommitManualSegmentCandidate(GarmentSession &session);

void ClearManualSegmentPickInProgress(GarmentSession &session);

// Remove all manual segment candidates and rebuild auto segments only.
void ClearManualSegmentCandidates(GarmentSession &session);

// Highlight in-progress and committed manual segment patches on the mesh.
void ColorSessionManualSegmentPicks(GarmentSession &session);

void GetGarmentSessionTensionParams(const GarmentSession &session,
                                    double &maxCompression,
                                    double &maxTension);
void SetGarmentSessionTensionParams(GarmentSession &session,
                                    double maxCompression,
                                    double maxTension);

#endif // GARMENT_SESSION_H

