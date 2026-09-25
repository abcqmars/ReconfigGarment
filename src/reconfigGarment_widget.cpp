#include <GL/glew.h>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QProcess>
#include <QMessageBox>
#include <QFile>
#include <QTextStream>
#include <QWidget>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

#include <wrap/gui/trackball.h>
#include <wrap/qt/trackball.h>
#include <wrap/qt/device_to_logical.h>
#include <wrap/qt/anttweakbarMapper.h>

#include <tracing/GL_mesh_drawing.h>
#include <tracing/mesh_type.h>

#include "reconfigGarment_widget.h"
#include "garment_session.h"
#include "trace_path_GL.h"
#include "manual_border_seam.h"
#include "uv_back_projection.h"
#include "experiment_settings.h"

#include <wrap/gl/picking.h>

#include <Eigen/Dense>

#include <vcg/complex/algorithms/update/color.h>
#include <vcg/complex/algorithms/stat.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/position.h>
#include <vcg/complex/allocate.h>
#include <wrap/io_trimesh/export_obj.h>

#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <memory>
#include <unordered_map>

// Mesh paths provided by the application entry point.
std::string pathMeshA;
std::string pathMeshB;
std::string gPendingExperimentJsonPath;

// Active garment session pointer defined in myglwidget.cpp.
extern GarmentSession* gActiveSession;
// Main AntTweakBar instance created in myglwidget.cpp::InitBar.
extern TwBar *barFashion;
// Global rendering flags defined in myglwidget.cpp.
extern bool draw3D;
extern bool drawParam;
extern bool parametrized;
extern bool colored_distortion;
extern bool textured;
extern bool drawArrangedApproxPolygonUV;
extern int gMaxMappingPairs;
extern bool drawConstraints;
extern int  gMaxMappingPairs;

namespace {

using TraceMesh = ::TraceMesh;

static const vcg::Color4b kSeamEdgeColor(0, 0, 0, 255);
static const float kSeamEdgeLineWidth = 5.0f;

static bool SessionHasMeshSeams(const GarmentSession &session)
{
    const TraceMesh &mesh = session.deformed_mesh;
    for (size_t fi = 0; fi < mesh.face.size(); ++fi) {
        if (mesh.face[fi].IsD())
            continue;
        for (int j = 0; j < 3; ++j) {
            if (mesh.face[fi].IsFaceEdgeS(j))
                return true;
        }
    }
    return false;
}

static bool EnsureDir(const std::string &dirPath, const char *tag)
{
    if (dirPath.empty()) {
        std::cout << "[" << tag << "] Empty output directory path." << std::endl;
        return false;
    }
    if (!QDir().mkpath(QString::fromStdString(dirPath))) {
        std::cout << "[" << tag << "] Failed to create directory: " << dirPath << std::endl;
        return false;
    }
    return true;
}

static bool ExportMeshObjWithFaceColors(TraceMesh &meshIn,
                                       const GarmentSession::CoordType &centerOffset,
                                       const std::string &outPath,
                                       const char *tag)
{
    TraceMesh mesh;
    vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(mesh, meshIn);
    // Restore original coordinates (meshes were centered on load).
    for (size_t vi = 0; vi < mesh.vert.size(); ++vi)
        mesh.vert[vi].P() += centerOffset;
    mesh.UpdateAttributes();

    const int mask = vcg::tri::io::Mask::IOM_WEDGTEXCOORD |
                     vcg::tri::io::Mask::IOM_FACECOLOR;
    const int err = vcg::tri::io::ExporterOBJ<TraceMesh>::Save(mesh, outPath.c_str(), mask);
    if (err != 0) {
        std::cout << "[" << tag << "] Failed to write OBJ (" << err << "): " << outPath << std::endl;
        return false;
    }
    return true;
}

static bool BuildPolygonFanMesh(const std::vector<vcg::Point2<TraceMesh::ScalarType>> &poly,
                               const vcg::Color4b &color,
                               TraceMesh &outMesh)
{
    outMesh.Clear();
    if (poly.size() < 3)
        return false;

    vcg::tri::Allocator<TraceMesh>::AddVertices(outMesh, poly.size());
    for (size_t i = 0; i < poly.size(); ++i) {
        outMesh.vert[i].P() = TraceMesh::CoordType(poly[i].X(), poly[i].Y(), TraceMesh::ScalarType(0));
        outMesh.vert[i].T().P() = poly[i];
    }

    const size_t numTris = poly.size() - 2;
    vcg::tri::Allocator<TraceMesh>::AddFaces(outMesh, numTris);
    for (size_t t = 0; t < numTris; ++t) {
        outMesh.face[t].V(0) = &outMesh.vert[0];
        outMesh.face[t].V(1) = &outMesh.vert[t + 1];
        outMesh.face[t].V(2) = &outMesh.vert[t + 2];
        outMesh.face[t].C() = color;
        for (int j = 0; j < 3; ++j)
            outMesh.face[t].WT(j).P() = outMesh.face[t].V(j)->T().P();
    }
    vcg::tri::UpdateColor<TraceMesh>::PerFaceConstant(outMesh, color);
    vcg::tri::UpdateBounding<TraceMesh>::Box(outMesh);
    outMesh.UpdateAttributes();
    return true;
}

static void ApplyVertexUVToPositions(TraceMesh &mesh)
{
    for (size_t vi = 0; vi < mesh.vert.size(); ++vi) {
        const auto uv = mesh.vert[vi].T().P();
        mesh.vert[vi].P() = TraceMesh::CoordType(uv.X(), uv.Y(), TraceMesh::ScalarType(0));
    }
    mesh.UpdateAttributes();
}

static void CenterFlat2DMeshAtOrigin(TraceMesh &mesh)
{
    vcg::tri::UpdateBounding<TraceMesh>::Box(mesh);
    if (mesh.vert.empty())
        return;

    const auto center = mesh.bbox.Center();
    const TraceMesh::ScalarType cx = center[0];
    const TraceMesh::ScalarType cy = center[1];
    for (size_t vi = 0; vi < mesh.vert.size(); ++vi) {
        mesh.vert[vi].P()[0] -= cx;
        mesh.vert[vi].P()[1] -= cy;
        mesh.vert[vi].T().P().X() -= cx;
        mesh.vert[vi].T().P().Y() -= cy;
    }
    for (size_t fi = 0; fi < mesh.face.size(); ++fi) {
        if (mesh.face[fi].IsD())
            continue;
        for (int j = 0; j < 3; ++j) {
            mesh.face[fi].WT(j).P().X() -= cx;
            mesh.face[fi].WT(j).P().Y() -= cy;
        }
    }
    mesh.UpdateAttributes();
}

static bool ExportFlat2DMeshObj(TraceMesh &meshIn,
                                const std::string &outPath,
                                const char *tag)
{
    TraceMesh mesh;
    vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(mesh, meshIn);
    ApplyVertexUVToPositions(mesh);
    CenterFlat2DMeshAtOrigin(mesh);
    const int mask = vcg::tri::io::Mask::IOM_WEDGTEXCOORD |
                     vcg::tri::io::Mask::IOM_FACECOLOR;
    const int err = vcg::tri::io::ExporterOBJ<TraceMesh>::Save(mesh, outPath.c_str(), mask);
    if (err != 0) {
        std::cout << "[" << tag << "] Failed to write flat 2D OBJ (" << err << "): " << outPath
                  << std::endl;
        return false;
    }
    return true;
}

static vcg::Color4b PatchShadeForSeamVisibility(const vcg::Color4b &base, int patchSlot)
{
    static const float factors[] = {0.90f, 0.96f, 1.0f, 0.96f, 0.90f};
    const float f = factors[patchSlot % 5];
    auto clampByte = [](int v) -> unsigned char {
        return static_cast<unsigned char>(std::max(0, std::min(255, v)));
    };
    return vcg::Color4b(
        clampByte(static_cast<int>(std::lround(base[0] * f))),
        clampByte(static_cast<int>(std::lround(base[1] * f))),
        clampByte(static_cast<int>(std::lround(base[2] * f))),
        255);
}

static void ColorFlattenedSegmentMeshByPatch(GarmentSession &session,
                                             int segIndex,
                                             TraceMesh &segMesh,
                                             const vcg::Color4b &defaultGrey,
                                             const std::vector<std::pair<int, int>> *mappingPairs,
                                             bool isGarmentA)
{
    std::vector<int> paids;
    if (!SessionSegmentPaids(session, segIndex, paids))
        return;

    std::unordered_map<int, int> paidToSlot;
    paidToSlot.reserve(paids.size());
    for (size_t i = 0; i < paids.size(); ++i)
        paidToSlot[paids[i]] = static_cast<int>(i);

    int pairIdx = -1;
    if (mappingPairs != nullptr && !mappingPairs->empty())
        pairIdx = SessionMappingPairIndexForSegment(session, *mappingPairs, segIndex, isGarmentA);

    const vcg::Color4b baseColor =
        (pairIdx >= 0) ? ColorForPair(pairIdx) : defaultGrey;

    for (size_t fi = 0; fi < segMesh.face.size(); ++fi) {
        if (segMesh.face[fi].IsD())
            continue;
        int paid = -1;
        if (!SessionSegmentFacePatchId(session, segIndex, fi, paid))
            continue;
        const auto slotIt = paidToSlot.find(paid);
        const int slot = (slotIt != paidToSlot.end()) ? slotIt->second : 0;
        segMesh.face[fi].C() = PatchShadeForSeamVisibility(baseColor, slot);
    }
}

static int ExportFlattenedSegmentMeshes(GarmentSession &session,
                                        const std::string &dir,
                                        const vcg::Color4b &defaultGrey,
                                        const std::vector<std::pair<int, int>> *mappingPairs,
                                        bool isGarmentA)
{
    const int numSeg = SessionNumSegments(session);
    if (numSeg <= 0)
        return 0;

    int exported = 0;
    for (int si = 0; si < numSeg; ++si) {
        TraceMesh *src = SessionSegmentMesh(session, si);
        if (src == nullptr || src->face.empty())
            continue;

        TraceMesh segMesh;
        vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(segMesh, *src);
        ColorFlattenedSegmentMeshByPatch(session, si, segMesh, defaultGrey, mappingPairs, isGarmentA);
        segMesh.UpdateAttributes();

        char fname[64];
        std::snprintf(fname, sizeof(fname), "segment_%03d.obj", si);
        const std::string out = QDir(QString::fromStdString(dir))
                                    .filePath(QString::fromLatin1(fname))
                                    .toStdString();
        if (ExportFlat2DMeshObj(segMesh, out, "DualExport"))
            ++exported;
    }
    return exported;
}

static bool BuildPatchSubmeshByPatchId(GarmentSession &session,
                                       int patchId,
                                       const vcg::Color4b &color,
                                       TraceMesh &outMesh)
{
    outMesh.Clear();
    auto &mesh = session.deformed_mesh;

    std::vector<int> oldToNew(mesh.vert.size(), -1);
    std::vector<size_t> faceIndices;
    faceIndices.reserve(mesh.face.size());
    for (size_t fi = 0; fi < mesh.face.size(); ++fi) {
        if (mesh.face[fi].IsD())
            continue;
        int facePatchId = -1;
        if (SessionFacePatchId(session, fi, facePatchId) && facePatchId == patchId)
            faceIndices.push_back(fi);
    }
    if (faceIndices.empty())
        return false;

    for (size_t fi : faceIndices) {
        for (int j = 0; j < 3; ++j) {
            TraceMesh::VertexType *v = mesh.face[fi].V(j);
            if (!v)
                continue;
            const int oldIdx = static_cast<int>(vcg::tri::Index(mesh, v));
            if (oldIdx >= 0 && static_cast<size_t>(oldIdx) < oldToNew.size() && oldToNew[oldIdx] < 0)
                oldToNew[oldIdx] = 0;
        }
    }

    int newVertCount = 0;
    for (int &idx : oldToNew) {
        if (idx >= 0)
            idx = newVertCount++;
    }
    if (newVertCount <= 0)
        return false;

    vcg::tri::Allocator<TraceMesh>::AddVertices(outMesh, static_cast<size_t>(newVertCount));
    for (size_t oldIdx = 0; oldIdx < oldToNew.size(); ++oldIdx) {
        const int newIdx = oldToNew[oldIdx];
        if (newIdx < 0)
            continue;
        outMesh.vert[static_cast<size_t>(newIdx)].P() = mesh.vert[oldIdx].P();
        outMesh.vert[static_cast<size_t>(newIdx)].N() = mesh.vert[oldIdx].N();
        outMesh.vert[static_cast<size_t>(newIdx)].T() = mesh.vert[oldIdx].T();
    }

    vcg::tri::Allocator<TraceMesh>::AddFaces(outMesh, faceIndices.size());
    for (size_t k = 0; k < faceIndices.size(); ++k) {
        auto &srcF = mesh.face[faceIndices[k]];
        auto &dstF = outMesh.face[k];
        dstF.C() = color;
        for (int j = 0; j < 3; ++j) {
            const int oldIdx = static_cast<int>(vcg::tri::Index(mesh, srcF.V(j)));
            const int newIdx = (oldIdx >= 0 && static_cast<size_t>(oldIdx) < oldToNew.size())
                                   ? oldToNew[oldIdx]
                                   : -1;
            if (newIdx < 0)
                return false;
            dstF.V(j) = &outMesh.vert[static_cast<size_t>(newIdx)];
            dstF.WT(j).P() = srcF.WT(j).P();
        }
    }

    outMesh.UpdateAttributes();
    return true;
}

static bool ExportPolygonMeshObj(const std::vector<vcg::Point2<TraceMesh::ScalarType>> &poly,
                                const vcg::Color4b &color,
                                const std::string &outPath,
                                const char *tag)
{
    TraceMesh mesh;
    if (!BuildPolygonFanMesh(poly, color, mesh))
        return false;
    const int mask = vcg::tri::io::Mask::IOM_FACECOLOR;
    const int err = vcg::tri::io::ExporterOBJ<TraceMesh>::Save(mesh, outPath.c_str(), mask);
    if (err != 0) {
        std::cout << "[" << tag << "] Failed to write polygon OBJ (" << err << "): " << outPath << std::endl;
        return false;
    }
    return true;
}

// One session per viewport.
GarmentSession sessionA;
GarmentSession sessionB;

// One trackball per viewport (left/right).
vcg::Trackball trackLeft;
vcg::Trackball trackRight;

vcg::GlTrimesh<TraceMesh> glWrap;

vcg::GLW::DrawMode drawmode = vcg::GLW::DMSmooth;

// 0 = left garment (A), 1 = right garment (B)
int activeGarmentIndex = 0;
bool activeGarmentControlAdded = false;

static constexpr int kMinApproxVerts = 4;
static constexpr int kMaxApproxVerts = 64;

// Rigid placement approx polygon -> garment UV (matches polygon_matching_solver).
struct QuadPlacement
{
    bool   flip = false;
    double tx = 0.0;
    double ty = 0.0;
    double rot = 0.0;  // radians
};

// Segment mapping state for the dual-view widget.
struct SegmentMappingEntry
{
    int srcIndex = -1;   // segment index on garment A
    int dstIndex = -1;   // segment index on garment B

    double score = 0.0;  // turning-function distance (lower is better)
    bool   flip  = false;
    double tx    = 0.0;
    double ty    = 0.0;
    double rot   = 0.0;  // radians

    // Optional approximated polygon (N verts, canonical frame) and placements.
    bool hasQuad = false;
    int approxNumVerts = 0;
    std::vector<Eigen::Vector2d> approxVerts;
    QuadPlacement placementA;
    QuadPlacement placementB;
};

struct SegmentMappingState
{
    GarmentSession* sessionA = nullptr;
    GarmentSession* sessionB = nullptr;

    // Full per-pair mapping information.
    std::vector<SegmentMappingEntry> entries;

    bool hasMapping = false;
    bool drawMapping = false;

    // Convenience accessor for legacy APIs that only need (i,j) pairs.
    std::vector<std::pair<int,int>> pairs() const
    {
        std::vector<std::pair<int,int>> out;
        out.reserve(entries.size());
        for (const auto &e : entries)
            out.emplace_back(e.srcIndex, e.dstIndex);
        return out;
    }
};

SegmentMappingState gSegmentMappingState { &sessionA, &sessionB };
bool dualButtonsAdded = false;
bool gGreyGarmentColorToggleAdded = false;

// Physical garment surface areas for fabrication export (square units, e.g. m^2).
// Must use TW_TYPE_CSSTRING (fixed buffer), not TW_TYPE_CDSTRING (heap char*).
static const int kRealGarmentAreaStringSize = 64;
static char gRealGarmentAreaA[kRealGarmentAreaStringSize] = "1.0";
static char gRealGarmentAreaB[kRealGarmentAreaStringSize] = "1.0";

// Back-projection visualization mode (AntTweakBar enum).
enum BackProjectionVizMode {
    BP_VizNone = 0,
    BP_VizSegment = 1,
    BP_VizQuad = 2
};
int gBackProjectionVizMode = BP_VizNone;
enum BackProjectionSourceAreaMode {
    BP_SourceAreaFull = 0,
    BP_SourceAreaOverlapOnly = 1
};
int gBackProjectionSourceAreaMode = BP_SourceAreaFull;
// Subdivision resolution per quad edge for quad back-projection mesh.
int gQuadSubdivisions = 32;

enum BackProjectionDirection {
    BP_DirAontoB = 0,
    BP_DirBontoA = 1
};

enum BackProjectionGeometryKind {
    BP_GeomSegment = 0,
    BP_GeomApprox = 1
};

struct BackProjectionCacheKey {
    int pairIdx = -1;
    int direction = 0;
    int geometryKind = 0;
    int sourceAreaMode = 0;
    int quadSubdivisions = 0;

    bool operator==(const BackProjectionCacheKey &o) const
    {
        return pairIdx == o.pairIdx &&
               direction == o.direction &&
               geometryKind == o.geometryKind &&
               sourceAreaMode == o.sourceAreaMode &&
               quadSubdivisions == o.quadSubdivisions;
    }
};

struct BackProjectionCacheKeyHash {
    std::size_t operator()(const BackProjectionCacheKey &k) const
    {
        std::size_t h = 1469598103934665603ULL;
        auto mix = [&h](std::size_t v) {
            h ^= v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        };
        mix(static_cast<std::size_t>(k.pairIdx));
        mix(static_cast<std::size_t>(k.direction));
        mix(static_cast<std::size_t>(k.geometryKind));
        mix(static_cast<std::size_t>(k.sourceAreaMode));
        mix(static_cast<std::size_t>(k.quadSubdivisions));
        return h;
    }
};

std::unordered_map<BackProjectionCacheKey, std::shared_ptr<TraceMesh>, BackProjectionCacheKeyHash>
    gBackProjectionMeshCache;

static void InvalidateBackProjectionCache()
{
    gBackProjectionMeshCache.clear();
}
PathGL<TraceMesh> gPathA;
PathGL<TraceMesh> gPathB;
bool gSpacebarPressed = false;
bool gUserIsPicking = false;
int gPickingViewport = -1; // 0 = left(A), 1 = right(B)
int gPickX = 0;
int gPickY = 0;
bool gHasDoubleClick = false;
int gDoubleClickViewport = -1;

// Manual border-to-border seam tool (post-batch).
bool gBorderSeamMode = false;
bool gBorderSeamSymmetricMode = false;
int gBorderSeamPickStage = 0; // 0 = need first border point, 1 = need second
size_t gBorderSeamVertexA = 0;
TraceMesh::CoordType gBorderSeamPosA(0, 0, 0);
bool gPendingBorderSeamPick = false;
int gPendingBorderSeamViewport = -1;
int gPendingBorderSeamPixelX = 0;
int gPendingBorderSeamPixelY = 0;
bool gBorderSeamControlsAdded = false;

// When true, draw only mesh border edges (as on initial load), not batch-traced internal seams.
bool gInputGarmentSeamViz = false;
bool gInputGarmentSeamVizControlAdded = false;

// Post-batch manual segment candidate tool (patch picking).
bool gManualSegmentMode = false;
bool gPendingManualSegmentPick = false;
int gPendingManualSegmentViewport = -1;
int gPendingManualSegmentPixelX = 0;
int gPendingManualSegmentPixelY = 0;
bool gManualSegmentControlsAdded = false;
bool gExperimentControlsAdded = false;
bool gExperimentListVarAdded = false;

std::vector<std::string> gSavedExperimentNames;
std::vector<std::string> gSavedExperimentEnumLabels;
std::vector<TwEnumVal> gSavedExperimentEnumVals;
TwType gSavedExperimentTwType = TW_TYPE_UNDEF;
int gSelectedSavedExperimentIndex = 0;

static void SyncConstraintPathsToGLPaths()
{
    gPathA.PickedPoints = sessionA.constraintPickedPoints;
    gPathB.PickedPoints = sessionB.constraintPickedPoints;
}

static void ApplyDualGarmentColoring(GarmentSession &session, bool isGarmentA)
{
    // Grey mode is a visualization override: keep mapping/back-projection active,
    // but render garments in neutral grey instead of pair colors.
    if (gGarmentColorGreyMode) {
        ColorSessionNeutralGrey(session);
        return;
    }
    if (gSegmentMappingState.drawMapping && gSegmentMappingState.hasMapping) {
        const auto ijPairs = gSegmentMappingState.pairs();
        ApplySegmentMappingColors(session, ijPairs, isGarmentA);
        return;
    }
    ColorSessionByPatch(session);
}

static void ApplyDualGarmentColoringBoth()
{
    ApplyDualGarmentColoring(sessionA, true);
    ApplyDualGarmentColoring(sessionB, false);
}

static void GLDrawConstraintPaths(const PathGL<TraceMesh> &paths,
                                  const GarmentSession &session,
                                  const vcg::Color4b &col = kSeamEdgeColor)
{
    if (paths.PickedPoints.empty())
        return;
    // After batch, traced paths are mesh seam edges; skip overlay to avoid thick double lines.
    if (SessionHasMeshSeams(session) && !gUserIsPicking)
        return;

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glDisable(GL_LIGHTING);
    glLineWidth(kSeamEdgeLineWidth);
    glDepthRange(0, 0.9999f);
    vcg::glColor(col);
    for (const auto &path : paths.PickedPoints) {
        if (path.size() < 2)
            continue;
        glBegin(GL_LINE_STRIP);
        for (const auto &p : path)
            vcg::glVertex(p);
        glEnd();
    }
    glPopAttrib();
}

static bool InitializeDualGarmentSessions()
{
    ClearActiveExperimentDirectory();

    if (!sessionA.loadMeshes(pathMeshA, pathMeshA, std::string())) {
        std::cout << "Error initializing session A with mesh: " << pathMeshA << std::endl;
        return false;
    }
    if (!sessionB.loadMeshes(pathMeshB, pathMeshB, std::string())) {
        std::cout << "Error initializing session B with mesh: " << pathMeshB << std::endl;
        return false;
    }

    NormalizeDualGarmentAreas(sessionA, sessionB);

    sessionA.initMesh();
    sessionB.initMesh();
    gActiveSession = &sessionA;
    gGarmentColorGreyMode = true;
    return true;
}
int gLastManualSegmentViewport = 0;

// --------------------------------------------------------------------
// Manual border seam drawing / picking helpers
// --------------------------------------------------------------------

static void GLDrawPolyline3D(const std::vector<TraceMesh::CoordType> &pts,
                             const vcg::Color4b &col,
                             float lineWidth = 6.f)
{
    if (pts.size() < 2)
        return;

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glDisable(GL_LIGHTING);
    glDepthRange(0, 0.9999);
    glLineWidth(lineWidth);
    vcg::glColor(col);
    glBegin(GL_LINE_STRIP);
    for (const auto &p : pts)
        vcg::glVertex(p);
    glEnd();
    glPopAttrib();
}

static void GLDrawPickMarker(const TraceMesh::CoordType &p,
                             const vcg::Color4b &col,
                             float size = 0.01f)
{
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glDisable(GL_LIGHTING);
    glDepthRange(0, 0.9999);
    vcg::glColor(col);
    glLineWidth(4.f);
    glBegin(GL_LINES);
    vcg::glVertex(p + TraceMesh::CoordType(size, 0, 0));
    vcg::glVertex(p - TraceMesh::CoordType(size, 0, 0));
    vcg::glVertex(p + TraceMesh::CoordType(0, size, 0));
    vcg::glVertex(p - TraceMesh::CoordType(0, size, 0));
    vcg::glVertex(p + TraceMesh::CoordType(0, 0, size));
    vcg::glVertex(p - TraceMesh::CoordType(0, 0, size));
    glEnd();
    glPopAttrib();
}

static void GLDrawManualBorderSeams(GarmentSession &session)
{
    // Committed border seams are on the mesh (IsFaceEdgeS) and drawn by GLDrawGarmentSeamEdges.

    if (gBorderSeamPickStage == 1) {
        const float markerSize = session.reference_mesh.bbox.Diag() * 0.02f;
        GLDrawPickMarker(gBorderSeamPosA, vcg::Color4b(255, 255, 0, 255), markerSize);
    }
}

// Must run after the mesh is drawn (same as GPath.GLAddPoint in MyGLWidget::paintGL).
static void ProcessPendingBorderSeamPick(ReconfigGarmentWidget *widget,
                                       int viewportIndex,
                                       GarmentSession &session)
{
    if (!gPendingBorderSeamPick || gPendingBorderSeamViewport != viewportIndex)
        return;

    gPendingBorderSeamPick = false;

    TraceMesh::CoordType pickPos;
    if (!vcg::Pick(gPendingBorderSeamPixelX, gPendingBorderSeamPixelY, pickPos)) {
        std::cout << "Border seam pick missed the mesh." << std::endl;
        return;
    }

    const BorderVertexPick snap = SnapBorderPickOnMesh(session, pickPos);
    if (!snap.ok) {
        std::cout << "Pick is too far from a border/seam edge." << std::endl;
        return;
    }

    if (gBorderSeamPickStage == 0) {
        gBorderSeamVertexA = snap.vertexIndex;
        gBorderSeamPosA = snap.position;
        gBorderSeamPickStage = 1;
        std::cout << "Border seam: first point on vertex " << gBorderSeamVertexA
                  << ". Pick second border point." << std::endl;
        return;
    }

    if (snap.vertexIndex == gBorderSeamVertexA) {
        std::cout << "Border seam: choose a different second point." << std::endl;
        return;
    }

    const size_t v0 = gBorderSeamVertexA;
    const size_t v1 = snap.vertexIndex;

    bool okMain = AddManualBorderSeamToSession(session, v0, v1);
    bool okMirror = true;

    if (okMain && gBorderSeamSymmetricMode) {
        // Mirror the two picked points across the symmetry plane and snap them to
        // border/seam vertices, then apply the same seam tool again.
        auto mirrorAcrossX0 = [](const TraceMesh::CoordType &p) {
            TraceMesh::CoordType out = p;
            out.X() = -out.X();
            return out;
        };
        const TraceMesh::CoordType p0m = mirrorAcrossX0(session.deformed_mesh.vert[v0].P());
        const TraceMesh::CoordType p1m = mirrorAcrossX0(session.deformed_mesh.vert[v1].P());

        const BorderVertexPick s0 = SnapBorderPickOnMesh(session, p0m);
        const BorderVertexPick s1 = SnapBorderPickOnMesh(session, p1m);
        if (!s0.ok || !s1.ok || s0.vertexIndex == s1.vertexIndex) {
            okMirror = false;
            std::cout << "Symmetric seam: could not snap mirrored points to border/seam vertices."
                      << std::endl;
        } else {
            okMirror = AddManualBorderSeamToSession(session, s0.vertexIndex, s1.vertexIndex);
            if (!okMirror)
                std::cout << "Symmetric seam: failed to add mirrored seam." << std::endl;
        }
    }

    if (okMain && okMirror) {
        InvalidateBackProjectionCache();
        std::cout << "Border seam added between vertices " << v0
                  << " and " << v1
                  << (gBorderSeamSymmetricMode ? " (with symmetric seam)" : "")
                  << std::endl;
        gBorderSeamPickStage = 0;
        if (widget)
            widget->update();
    } else {
        std::cout << "Failed to add border seam." << std::endl;
    }
}

static void ProcessPendingManualSegmentPick(int viewportIndex, GarmentSession &session)
{
    if (!gPendingManualSegmentPick || gPendingManualSegmentViewport != viewportIndex)
        return;

    gPendingManualSegmentPick = false;

    const int patchId = PickPatchIdAtScreen(session, gPendingManualSegmentPixelX, gPendingManualSegmentPixelY);
    if (patchId < 0) {
        std::cout << "Manual segment: pick missed the mesh or patch graph." << std::endl;
        return;
    }

    if (TogglePatchInManualSegmentPick(session, patchId)) {
        gLastManualSegmentViewport = viewportIndex;
        std::cout << "Manual segment: toggled patch " << patchId
                  << " on garment " << (viewportIndex == 0 ? "A" : "B")
                  << " (in progress: " << session.manualSegmentPickInProgress.size() << " patches)"
                  << std::endl;
    }
}

static GarmentSession &activeGarmentSession()
{
    return (activeGarmentIndex == 0) ? sessionA : sessionB;
}

// --------------------------------------------------------------------
// Back-projected UV mesh visualization helpers
// --------------------------------------------------------------------

static void DrawArrangedApproxPolygonsForDualGarment(bool isGarmentA)
{
    GarmentSession &session = isGarmentA ? sessionA : sessionB;
    std::vector<Polygon2DDrawItem> items;

    if (gSegmentMappingState.hasMapping) {
        const auto &entries = gSegmentMappingState.entries;
        items.reserve(entries.size());
        for (size_t idx = 0; idx < entries.size(); ++idx) {
            if (gMaxMappingPairs >= 0 && static_cast<int>(idx) >= gMaxMappingPairs)
                break;

            const auto &e = entries[idx];
            const int segIdx = isGarmentA ? e.srcIndex : e.dstIndex;
            if (segIdx < 0)
                continue;

            Polygon2DDrawItem item;
            item.color = ColorForPair(static_cast<int>(idx));

            if (e.hasQuad && e.approxVerts.size() >= 3) {
                item.verts.reserve(e.approxVerts.size());
                for (const auto &v : e.approxVerts)
                    item.verts.emplace_back(
                        TraceMesh::ScalarType(v.x()),
                        TraceMesh::ScalarType(v.y()));
            } else {
                SessionApproxPolygonVerts(session, segIdx, item.verts);
            }

            if (item.verts.size() < 3)
                continue;
            items.push_back(std::move(item));
        }
    }

    if (items.empty()) {
        const auto pairs = gSegmentMappingState.hasMapping
            ? gSegmentMappingState.pairs()
            : std::vector<std::pair<int, int>>{};
        const std::vector<std::pair<int, int>> *pairPtr =
            pairs.empty() ? nullptr : &pairs;
        DrawArrangedApproxPolygonsFromSession(session, pairPtr, isGarmentA);
        return;
    }

    GLDrawArrangedPolygonPatches(items);
}

// Apply (flip, rot, trans) mapping canonical approx polygon -> garment UV.
static void ApplyPlacementToApproxUVs(const std::vector<Eigen::Vector2d> &canonical,
                                      const QuadPlacement &placement,
                                      std::vector<Eigen::Vector2d> &outUV)
{
    std::vector<Eigen::Vector2d> pts = canonical;
    const int numVerts = static_cast<int>(pts.size());
    if (placement.flip && numVerts > 0) {
        double cx = 0.0;
        for (int i = 0; i < numVerts; ++i)
            cx += pts[static_cast<size_t>(i)].x();
        cx /= static_cast<double>(numVerts);
        for (int i = 0; i < numVerts; ++i)
            pts[static_cast<size_t>(i)].x() = 2.0 * cx - pts[static_cast<size_t>(i)].x();
    }

    const double c = std::cos(placement.rot);
    const double s = std::sin(placement.rot);
    Eigen::Matrix2d R;
    R << c, -s,
         s,  c;
    const Eigen::Vector2d t(placement.tx, placement.ty);

    outUV.resize(static_cast<size_t>(numVerts));
    for (int i = 0; i < numVerts; ++i)
        outUV[static_cast<size_t>(i)] = R * pts[static_cast<size_t>(i)] + t;
}

// Bilinear point inside a quad: corners 0..3 at (0,0), (1,0), (1,1), (0,1).
static Eigen::Vector2d BilinearQuadPoint(double u, double v,
                                        const std::array<Eigen::Vector2d, 4> &corners)
{
    const double s = 1.0 - u;
    const double t = 1.0 - v;
    return s * t * corners[0] + u * t * corners[1] + u * v * corners[2] + s * v * corners[3];
}

// Build a subdivided quad mesh in UV space (for back-projection overlay).
static void BuildTriangulatedQuadMesh(TraceMesh &mesh,
                                      const std::array<Eigen::Vector2d, 4> &uvs,
                                      int subdivisions = -1)
{
    using Scalar = TraceMesh::ScalarType;
    using Point2 = vcg::Point2<Scalar>;

    int subdiv = (subdivisions > 0) ? subdivisions : gQuadSubdivisions;
    subdiv = std::max(1, subdiv);

    const int nvSide = subdiv + 1;
    const size_t numVerts = static_cast<size_t>(nvSide) * static_cast<size_t>(nvSide);
    const size_t numFaces = static_cast<size_t>(2 * subdiv * subdiv);

    mesh.Clear();
    vcg::tri::Allocator<TraceMesh>::AddVertices(mesh, numVerts);
    vcg::tri::Allocator<TraceMesh>::AddFaces(mesh, numFaces);

    auto vertIndex = [nvSide](int i, int j) -> size_t {
        return static_cast<size_t>(j * nvSide + i);
    };

    for (int j = 0; j < nvSide; ++j) {
        for (int i = 0; i < nvSide; ++i) {
            const double u = static_cast<double>(i) / static_cast<double>(subdiv);
            const double v = static_cast<double>(j) / static_cast<double>(subdiv);
            const Eigen::Vector2d uv = BilinearQuadPoint(u, v, uvs);
            auto &vert = mesh.vert[vertIndex(i, j)];
            vert.P() = TraceMesh::CoordType(0, 0, 0);
            vert.T().P() = Point2(static_cast<Scalar>(uv.x()),
                                  static_cast<Scalar>(uv.y()));
        }
    }

    size_t fi = 0;
    for (int j = 0; j < subdiv; ++j) {
        for (int i = 0; i < subdiv; ++i) {
            auto *v00 = &mesh.vert[vertIndex(i, j)];
            auto *v10 = &mesh.vert[vertIndex(i + 1, j)];
            auto *v01 = &mesh.vert[vertIndex(i, j + 1)];
            auto *v11 = &mesh.vert[vertIndex(i + 1, j + 1)];

            mesh.face[fi].V(0) = v00;
            mesh.face[fi].V(1) = v10;
            mesh.face[fi].V(2) = v11;
            ++fi;
            mesh.face[fi].V(0) = v00;
            mesh.face[fi].V(1) = v11;
            mesh.face[fi].V(2) = v01;
            ++fi;
        }
    }

    for (size_t fidx = 0; fidx < mesh.face.size(); ++fidx) {
        auto &f = mesh.face[fidx];
        for (int j = 0; j < 3; ++j) {
            if (auto *v = f.V(j))
                f.WT(j).P() = v->T().P();
        }
    }

    mesh.UpdateAttributes();
}

// Build a subdivided mesh for a convex n-gon (n=4 uses bilinear quad; n>=3 fans each
// triangular sector into a (subdiv x subdiv) grid, matching quad resolution).
static void BuildTriangulatedApproxMesh(TraceMesh &mesh,
                                        const std::vector<Eigen::Vector2d> &uvs,
                                        int subdivisions = -1)
{
    const int numVerts = static_cast<int>(uvs.size());
    if (numVerts == 4) {
        std::array<Eigen::Vector2d, 4> quadUV;
        for (int i = 0; i < 4; ++i)
            quadUV[static_cast<size_t>(i)] = uvs[static_cast<size_t>(i)];
        BuildTriangulatedQuadMesh(mesh, quadUV, subdivisions);
        return;
    }
    if (numVerts < 3)
        return;

    using Scalar = TraceMesh::ScalarType;
    using Point2 = vcg::Point2<Scalar>;

    // Match quad resolution: each triangular sector gets (subdiv+1)*(subdiv+2)/2 verts
    // and subdiv*subdiv triangles, same as one row/col slice of the quad grid.
    int subdiv = (subdivisions > 0) ? subdivisions : gQuadSubdivisions;
    subdiv = std::max(1, subdiv);

    Eigen::Vector2d center(0.0, 0.0);
    for (int i = 0; i < numVerts; ++i)
        center += uvs[static_cast<size_t>(i)];
    center /= static_cast<double>(numVerts);

    // For each edge of the n-gon we build a uniformly-subdivided triangle fan sector:
    // vertices are bilinear-interpolated over (center, cornerA, cornerB).
    // Grid coordinates: u in [0,1] toward the edge, v in [0,1] from center to edge.
    // At grid point (iu, iv): lerp from center toward (lerp(A,B,iu/subdiv)) by iv/subdiv.

    const int nSide = subdiv + 1;            // points per side
    const size_t vertsPerSector = static_cast<size_t>(nSide * nSide);
    const size_t facesPerSector = static_cast<size_t>(2 * subdiv * subdiv);
    const size_t totalVerts = static_cast<size_t>(numVerts) * vertsPerSector;
    const size_t totalFaces = static_cast<size_t>(numVerts) * facesPerSector;

    mesh.Clear();
    vcg::tri::Allocator<TraceMesh>::AddVertices(mesh, totalVerts);
    vcg::tri::Allocator<TraceMesh>::AddFaces(mesh, totalFaces);

    size_t vi = 0;
    size_t fi = 0;

    for (int e = 0; e < numVerts; ++e) {
        const Eigen::Vector2d &A = uvs[static_cast<size_t>(e)];
        const Eigen::Vector2d &B = uvs[static_cast<size_t>((e + 1) % numVerts)];
        const size_t vertBase = vi;

        // Fill vertex grid for this sector.
        for (int iv = 0; iv <= subdiv; ++iv) {
            const double vt = static_cast<double>(iv) / static_cast<double>(subdiv);
            for (int iu = 0; iu <= subdiv; ++iu) {
                const double ut = static_cast<double>(iu) / static_cast<double>(subdiv);
                // Point on the A-B edge.
                const Eigen::Vector2d edgePt = (1.0 - ut) * A + ut * B;
                // Lerp from center to edgePt.
                const Eigen::Vector2d p = (1.0 - vt) * center + vt * edgePt;
                auto &vert = mesh.vert[vi++];
                vert.P() = TraceMesh::CoordType(0, 0, 0);
                vert.T().P() = Point2(static_cast<Scalar>(p.x()), static_cast<Scalar>(p.y()));
            }
        }

        // Build two triangles per grid quad.
        auto vIdx = [&](int iu, int iv) -> size_t {
            return vertBase + static_cast<size_t>(iv * nSide + iu);
        };

        for (int iv = 0; iv < subdiv; ++iv) {
            for (int iu = 0; iu < subdiv; ++iu) {
                mesh.face[fi].V(0) = &mesh.vert[vIdx(iu,     iv    )];
                mesh.face[fi].V(1) = &mesh.vert[vIdx(iu + 1, iv    )];
                mesh.face[fi].V(2) = &mesh.vert[vIdx(iu + 1, iv + 1)];
                ++fi;
                mesh.face[fi].V(0) = &mesh.vert[vIdx(iu,     iv    )];
                mesh.face[fi].V(1) = &mesh.vert[vIdx(iu + 1, iv + 1)];
                mesh.face[fi].V(2) = &mesh.vert[vIdx(iu,     iv + 1)];
                ++fi;
            }
        }
    }

    for (size_t fidx = 0; fidx < mesh.face.size(); ++fidx) {
        auto &f = mesh.face[fidx];
        for (int j = 0; j < 3; ++j) {
            if (auto *v = f.V(j))
                f.WT(j).P() = v->T().P();
        }
    }
    mesh.UpdateAttributes();
}

static void DrawProjectedMeshOverlay(TraceMesh &mesh, const vcg::Color4b &col)
{
    TraceMesh drawMesh;
    vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(drawMesh, mesh);
    for (size_t fi = 0; fi < drawMesh.face.size(); ++fi) {
        if (drawMesh.face[fi].IsD()) continue;
        drawMesh.face[fi].C() = col;
    }
    drawMesh.UpdateAttributes();
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(-1.0f, -1.0f);
    glWrap.m = &drawMesh;
    glWrap.Draw(drawmode, vcg::GLW::CMPerFace, vcg::GLW::TMNone);
    glDisable(GL_POLYGON_OFFSET_FILL);
}

// Draw a mesh as a colored, filled overlay (positions assumed in world space).
static void GLDrawProjectedMeshFilled(const TraceMesh &mesh,
                                      const vcg::Color4b &col,
                                      float alpha = 0.5f)
{
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glDisable(GL_LIGHTING);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Slightly offset polygons to reduce z-fighting with the base mesh.
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(-1.0f, -1.0f);

    vcg::Color4b c = col;
    c[3] = static_cast<unsigned char>(std::round(std::max(0.f, std::min(1.f, alpha)) * 255.0f));
    vcg::glColor(c);

    glBegin(GL_TRIANGLES);
    for (size_t fi = 0; fi < mesh.face.size(); ++fi) {
        const auto &f = mesh.face[fi];
        if (f.IsD()) continue;
        for (int j = 0; j < 3; ++j) {
            const auto *v = f.V(j);
            if (!v) continue;
            const auto &p = v->P();
            glVertex3f(p.X(), p.Y(), p.Z());
        }
    }
    glEnd();

    glDisable(GL_POLYGON_OFFSET_FILL);
    glPopAttrib();
}

// Apply a rigid 2D transform (rot, tx, ty) to all UVs of a mesh.
// We transform each vertex UV once (in T().P()) and then propagate to per-wedge WT().P().
static void ApplyRigidTransformToMeshUV(TraceMesh &m,
                                        double rot,
                                        double tx,
                                        double ty)
{
    double c = std::cos(rot);
    double s = std::sin(rot);
    Eigen::Matrix2d R;
    R << c, -s,
         s,  c;
    Eigen::Vector2d t(tx, ty);

    // Transform per-vertex UVs once.
    for (size_t vi = 0; vi < m.vert.size(); ++vi) {
        auto &v = m.vert[vi];
        auto uv = v.T().P();
        Eigen::Vector2d p(uv.X(), uv.Y());
        Eigen::Vector2d q = R * p + t;
        v.T().P().X() = static_cast<TraceMesh::ScalarType>(q[0]);
        v.T().P().Y() = static_cast<TraceMesh::ScalarType>(q[1]);
    }

    // Propagate updated vertex UVs to all wedges.
    for (size_t fi = 0; fi < m.face.size(); ++fi) {
        auto &f = m.face[fi];
        if (f.IsD()) continue;
        for (int j = 0; j < 3; ++j) {
            auto *v = f.V(j);
            if (!v) continue;
            f.WT(j).P() = v->T().P();
        }
    }
}

// Apply the inverse rigid 2D transform (rot, tx, ty)^{-1} to all UVs of a mesh.
// Same pattern as above, but using R^T and (q - t).
static void ApplyInverseRigidTransformToMeshUV(TraceMesh &m,
                                               double rot,
                                               double tx,
                                               double ty)
{
    double c = std::cos(rot);
    double s = std::sin(rot);
    Eigen::Matrix2d R;
    R << c, -s,
         s,  c;
    Eigen::Matrix2d Rinv = R.transpose();
    Eigen::Vector2d t(tx, ty);

    for (size_t vi = 0; vi < m.vert.size(); ++vi) {
        auto &v = m.vert[vi];
        auto uv = v.T().P();
        Eigen::Vector2d q(uv.X(), uv.Y());
        Eigen::Vector2d p = Rinv * (q - t);
        v.T().P().X() = static_cast<TraceMesh::ScalarType>(p[0]);
        v.T().P().Y() = static_cast<TraceMesh::ScalarType>(p[1]);
    }

    for (size_t fi = 0; fi < m.face.size(); ++fi) {
        auto &f = m.face[fi];
        if (f.IsD()) continue;
        for (int j = 0; j < 3; ++j) {
            auto *v = f.V(j);
            if (!v) continue;
            f.WT(j).P() = v->T().P();
        }
    }
}

// Optional horizontal flip of UVs around a given center X (matching Python polygon flip).
// As with the rigid transform, we flip each vertex UV once, then propagate to wedges.
static void ApplyHorizontalFlipToMeshUV(TraceMesh &m, TraceMesh::ScalarType centerU)
{
    using Scalar = TraceMesh::ScalarType;

    // Scalar centerU = 0;
    // // Compute the center:
    // for (size_t vi = 0; vi < m.vert.size(); ++vi) {
    //     auto &v = m.vert[vi];
    //     centerU += v.T().P().X();
    // }
    // centerU = centerU/TraceMesh::ScalarType(m.vert.size());

    // Flip per-vertex UVs.
    for (size_t vi = 0; vi < m.vert.size(); ++vi) {
        auto &v = m.vert[vi];
        Scalar u = v.T().P().X();
        v.T().P().X() = Scalar(2) * centerU - u;
    }

    // Propagate flipped vertex UVs to wedges.
    for (size_t fi = 0; fi < m.face.size(); ++fi) {
        auto &f = m.face[fi];
        if (f.IsD()) continue;
        for (int j = 0; j < 3; ++j) {
            auto *v = f.V(j);
            if (!v) continue;
            f.WT(j).P() = v->T().P();
        }
    }
}

static bool IsUVInsideTargetMesh(const TraceMesh &targetMesh, const vcg::Point2<TraceMesh::ScalarType> &uv)
{
    const uv_back_projection::BaryHit hit =
        uv_back_projection::FindContainingFaceUV(targetMesh, uv_back_projection::UVCoordType(uv.X(), uv.Y()));
    return hit.faceIndex >= 0;
}

static bool FindBoundaryIntersectionOnUVEdge(const TraceMesh &targetMesh,
                                             const vcg::Point2<TraceMesh::ScalarType> &uvA,
                                             const vcg::Point2<TraceMesh::ScalarType> &uvB,
                                             bool insideA,
                                             bool insideB,
                                             TraceMesh::ScalarType &tOut)
{
    if (insideA == insideB)
        return false;
    using Scalar = TraceMesh::ScalarType;
    Scalar lo = Scalar(0);
    Scalar hi = Scalar(1);
    for (int iter = 0; iter < 28; ++iter) {
        const Scalar mid = Scalar(0.5) * (lo + hi);
        const vcg::Point2<Scalar> uvMid(uvA.X() + (uvB.X() - uvA.X()) * mid,
                                        uvA.Y() + (uvB.Y() - uvA.Y()) * mid);
        const bool insideMid = IsUVInsideTargetMesh(targetMesh, uvMid);
        if (insideMid == insideA)
            lo = mid;
        else
            hi = mid;
    }
    tOut = Scalar(0.5) * (lo + hi);
    return true;
}

// Build a temporary source mesh clipped in UV space by the target mesh domain.
// This cuts intersecting triangles and creates new boundary vertices, yielding
// smoother overlap boundaries than dropping full out-of-bound faces.
static bool BuildOverlapOnlySourceMeshInUV(const TraceMesh &sourceMesh,
                                           const TraceMesh &targetMesh,
                                           TraceMesh &outMesh)
{
    using Scalar = TraceMesh::ScalarType;
    using Point2 = vcg::Point2<Scalar>;
    using Point3 = TraceMesh::CoordType;

    struct ClipVertex {
        Point2 uv;
        Point3 pos;
    };

    outMesh.Clear();
    if (sourceMesh.face.empty())
        return false;

    std::vector<ClipVertex> outVerts;
    std::vector<std::array<int, 3>> outTris;
    outVerts.reserve(sourceMesh.face.size() * 3);
    outTris.reserve(sourceMesh.face.size() * 2);

    for (size_t fi = 0; fi < sourceMesh.face.size(); ++fi) {
        const auto &f = sourceMesh.face[fi];
        if (f.IsD())
            continue;
        if (!f.V(0) || !f.V(1) || !f.V(2))
            continue;

        std::array<ClipVertex, 3> tri = {{
            {f.WT(0).P(), f.V(0)->P()},
            {f.WT(1).P(), f.V(1)->P()},
            {f.WT(2).P(), f.V(2)->P()}
        }};
        std::array<bool, 3> inside = {{
            IsUVInsideTargetMesh(targetMesh, tri[0].uv),
            IsUVInsideTargetMesh(targetMesh, tri[1].uv),
            IsUVInsideTargetMesh(targetMesh, tri[2].uv)
        }};

        std::vector<ClipVertex> poly;
        poly.reserve(6);
        for (int i = 0; i < 3; ++i) {
            const int j = (i + 1) % 3;
            const ClipVertex &vi = tri[static_cast<size_t>(i)];
            const ClipVertex &vj = tri[static_cast<size_t>(j)];
            const bool inI = inside[static_cast<size_t>(i)];
            const bool inJ = inside[static_cast<size_t>(j)];

            if (inI)
                poly.push_back(vi);

            if (inI != inJ) {
                Scalar t = Scalar(0.5);
                if (FindBoundaryIntersectionOnUVEdge(targetMesh, vi.uv, vj.uv, inI, inJ, t)) {
                    ClipVertex vInt;
                    vInt.uv = Point2(vi.uv.X() + (vj.uv.X() - vi.uv.X()) * t,
                                     vi.uv.Y() + (vj.uv.Y() - vi.uv.Y()) * t);
                    vInt.pos = vi.pos + (vj.pos - vi.pos) * t;
                    poly.push_back(vInt);
                }
            }
        }

        if (poly.size() < 3)
            continue;

        const int base = static_cast<int>(outVerts.size());
        outVerts.insert(outVerts.end(), poly.begin(), poly.end());
        for (size_t k = 1; k + 1 < poly.size(); ++k) {
            outTris.push_back({base, base + static_cast<int>(k), base + static_cast<int>(k + 1)});
        }
    }

    if (outTris.empty() || outVerts.empty())
        return false;

    vcg::tri::Allocator<TraceMesh>::AddVertices(outMesh, outVerts.size());
    vcg::tri::Allocator<TraceMesh>::AddFaces(outMesh, outTris.size());

    for (size_t i = 0; i < outVerts.size(); ++i) {
        outMesh.vert[i].P() = outVerts[i].pos;
        outMesh.vert[i].T().P() = outVerts[i].uv;
    }

    for (size_t i = 0; i < outTris.size(); ++i) {
        auto &fOut = outMesh.face[i];
        const auto &triIdx = outTris[i];
        for (int j = 0; j < 3; ++j) {
            const int vi = triIdx[static_cast<size_t>(j)];
            fOut.V(j) = &outMesh.vert[static_cast<size_t>(vi)];
            fOut.WT(j).P() = outMesh.vert[static_cast<size_t>(vi)].T().P();
        }
    }

    outMesh.UpdateAttributes();
    return true;
}

static bool ApplyBackProjectionSourceAreaMode(TraceMesh &sourceMesh, const TraceMesh &targetMesh)
{
    if (gBackProjectionSourceAreaMode != BP_SourceAreaOverlapOnly)
        return true;
    TraceMesh overlapMesh;
    if (!BuildOverlapOnlySourceMeshInUV(sourceMesh, targetMesh, overlapMesh))
        return false;
    sourceMesh.Clear();
    vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(sourceMesh, overlapMesh);
    sourceMesh.UpdateAttributes();
    return true;
}

static std::shared_ptr<TraceMesh> LookupBackProjectionMeshCache(const BackProjectionCacheKey &key)
{
    auto it = gBackProjectionMeshCache.find(key);
    if (it == gBackProjectionMeshCache.end())
        return nullptr;
    return it->second;
}

static void StoreBackProjectionMeshCache(const BackProjectionCacheKey &key, const TraceMesh &mesh)
{
    auto cached = std::make_shared<TraceMesh>();
    TraceMesh &meshRef = const_cast<TraceMesh &>(mesh);
    vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(*cached, meshRef);
    cached->UpdateAttributes();
    gBackProjectionMeshCache[key] = cached;
}

// Back-project segments from garment A onto garment B.
static void DrawBackProjectedSegmentsAontoB()
{
    if (gBackProjectionVizMode != BP_VizSegment)
        return;
    if (!gSegmentMappingState.hasMapping || !gSegmentMappingState.drawMapping)
        return;
    if (!gSegmentMappingState.sessionA || !gSegmentMappingState.sessionB)
        return;

    GarmentSession &sessA = *gSegmentMappingState.sessionA;
    GarmentSession &sessB = *gSegmentMappingState.sessionB;
    int numSegA = SessionNumSegments(sessA);
    if (numSegA <= 0)
        return;

    int maxPairs = gMaxMappingPairs;
    const auto &entries = gSegmentMappingState.entries;

    for (size_t idx = 0; idx < entries.size(); ++idx) {
        if (maxPairs >= 0 && static_cast<int>(idx) >= maxPairs)
            break;

        const auto &e = entries[idx];
        if (e.srcIndex < 0 || e.srcIndex >= numSegA)
            continue;

        const BackProjectionCacheKey key {
            static_cast<int>(idx), BP_DirAontoB, BP_GeomSegment,
            gBackProjectionSourceAreaMode, gQuadSubdivisions
        };
        std::shared_ptr<TraceMesh> projectedMesh = LookupBackProjectionMeshCache(key);
        if (!projectedMesh) {
            // 1) Make fresh copies of the source and target segment meshes.
            TraceMesh segMeshA; // source segment on garment A
            TraceMesh segMeshB; // corresponding target segment on garment B
            if (TraceMesh *srcMeshA = SessionSegmentMesh(sessA, e.srcIndex)) {
                vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(segMeshA, *srcMeshA);
            } else {
                continue;
            }
            if (TraceMesh *srcMeshB = SessionSegmentMesh(sessB, e.dstIndex)) {
                vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(segMeshB, *srcMeshB);
            } else {
                continue;
            }

            // 2) Revert both segments' UVs from arranged segment space back to their
            //    respective garment UV spaces.
            SessionRevertSegmentUVToGarment(sessA, e.srcIndex, segMeshA);
            SessionRevertSegmentUVToGarment(sessB, e.dstIndex, segMeshB);

            // 3) On the source segment (A), apply optional horizontal flip
            //    (about ApproxPolygon center in A), then the forward rigid transform
            //    (A -> B) in garment-UV space.
            if (e.flip) {
                TraceMesh::ScalarType centerX = SegmentApproxPolygonCenterX(sessA, e.srcIndex);
                ApplyHorizontalFlipToMeshUV(segMeshA, centerX);
            }
            ApplyRigidTransformToMeshUV(segMeshA, e.rot, e.tx, e.ty);
            if (!ApplyBackProjectionSourceAreaMode(segMeshA, segMeshB))
                continue;

            // 4) Back-project the transformed A segment UV mesh onto the corresponding target segment.
            BackProjectMeshUV(segMeshA, segMeshB);
            StoreBackProjectionMeshCache(key, segMeshA);
            projectedMesh = LookupBackProjectionMeshCache(key);
        }
        if (!projectedMesh)
            continue;

        DrawProjectedMeshOverlay(*projectedMesh, ColorForPair(static_cast<int>(idx)));
    }
}

// Back-project approximated quads from garment A onto garment B.
static void DrawQuadBackProjectedAontoB()
{
    if (gBackProjectionVizMode != BP_VizQuad)
        return;
    if (!gSegmentMappingState.hasMapping || !gSegmentMappingState.drawMapping)
        return;
    if (!gSegmentMappingState.sessionA || !gSegmentMappingState.sessionB)
        return;

    GarmentSession &sessA = *gSegmentMappingState.sessionA;
    GarmentSession &sessB = *gSegmentMappingState.sessionB;
    const int numSegA = SessionNumSegments(sessA);
    const int maxPairs = gMaxMappingPairs;
    const auto &entries = gSegmentMappingState.entries;

    for (size_t idx = 0; idx < entries.size(); ++idx) {
        if (maxPairs >= 0 && static_cast<int>(idx) >= maxPairs)
            break;

        const auto &e = entries[idx];
        if (!e.hasQuad || e.approxNumVerts < 4 || e.srcIndex < 0 || e.srcIndex >= numSegA)
            continue;

        const BackProjectionCacheKey key {
            static_cast<int>(idx), BP_DirAontoB, BP_GeomApprox,
            gBackProjectionSourceAreaMode, gQuadSubdivisions
        };
        std::shared_ptr<TraceMesh> projectedMesh = LookupBackProjectionMeshCache(key);
        if (!projectedMesh) {
            TraceMesh segMeshB;
            if (TraceMesh *srcMeshB = SessionSegmentMesh(sessB, e.dstIndex)) {
                vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(segMeshB, *srcMeshB);
            } else {
                continue;
            }
            SessionRevertSegmentUVToGarment(sessB, e.dstIndex, segMeshB);

            std::vector<Eigen::Vector2d> approxUV;
            ApplyPlacementToApproxUVs(e.approxVerts, e.placementA, approxUV);

            TraceMesh quadMesh;
            BuildTriangulatedApproxMesh(quadMesh, approxUV);

            if (e.flip) {
                const TraceMesh::ScalarType centerX = SegmentApproxPolygonCenterX(sessA, e.srcIndex);
                ApplyHorizontalFlipToMeshUV(quadMesh, centerX);
            }
            ApplyRigidTransformToMeshUV(quadMesh, e.rot, e.tx, e.ty);
            if (!ApplyBackProjectionSourceAreaMode(quadMesh, segMeshB))
                continue;

            BackProjectMeshUV(quadMesh, segMeshB);
            StoreBackProjectionMeshCache(key, quadMesh);
            projectedMesh = LookupBackProjectionMeshCache(key);
        }
        if (!projectedMesh)
            continue;

        DrawProjectedMeshOverlay(*projectedMesh, ColorForPair(static_cast<int>(idx)));
    }
}

// Back-project approximated quads from garment B onto garment A.
static void DrawQuadBackProjectedBontoA()
{
    if (gBackProjectionVizMode != BP_VizQuad)
        return;
    if (!gSegmentMappingState.hasMapping || !gSegmentMappingState.drawMapping)
        return;
    if (!gSegmentMappingState.sessionA || !gSegmentMappingState.sessionB)
        return;

    GarmentSession &sessA = *gSegmentMappingState.sessionA;
    GarmentSession &sessB = *gSegmentMappingState.sessionB;
    const int numSegB = SessionNumSegments(sessB);
    const int maxPairs = gMaxMappingPairs;
    const auto &entries = gSegmentMappingState.entries;

    for (size_t idx = 0; idx < entries.size(); ++idx) {
        if (maxPairs >= 0 && static_cast<int>(idx) >= maxPairs)
            break;

        const auto &e = entries[idx];
        if (!e.hasQuad || e.approxNumVerts < 4 || e.dstIndex < 0 || e.dstIndex >= numSegB)
            continue;

        const BackProjectionCacheKey key {
            static_cast<int>(idx), BP_DirBontoA, BP_GeomApprox,
            gBackProjectionSourceAreaMode, gQuadSubdivisions
        };
        std::shared_ptr<TraceMesh> projectedMesh = LookupBackProjectionMeshCache(key);
        if (!projectedMesh) {
            TraceMesh segMeshA;
            if (TraceMesh *srcMeshA = SessionSegmentMesh(sessA, e.srcIndex)) {
                vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(segMeshA, *srcMeshA);
            } else {
                continue;
            }
            SessionRevertSegmentUVToGarment(sessA, e.srcIndex, segMeshA);

            std::vector<Eigen::Vector2d> approxUV;
            ApplyPlacementToApproxUVs(e.approxVerts, e.placementB, approxUV);

            TraceMesh quadMesh;
            BuildTriangulatedApproxMesh(quadMesh, approxUV);

            ApplyInverseRigidTransformToMeshUV(quadMesh, e.rot, e.tx, e.ty);

            if (e.flip) {
                const TraceMesh::ScalarType centerX = SegmentApproxPolygonCenterX(sessA, e.srcIndex);
                ApplyHorizontalFlipToMeshUV(segMeshA, centerX);
            }
            if (!ApplyBackProjectionSourceAreaMode(quadMesh, segMeshA))
                continue;

            BackProjectMeshUV(quadMesh, segMeshA);
            StoreBackProjectionMeshCache(key, quadMesh);
            projectedMesh = LookupBackProjectionMeshCache(key);
        }
        if (!projectedMesh)
            continue;

        DrawProjectedMeshOverlay(*projectedMesh, ColorForPair(static_cast<int>(idx)));
    }
}

// Back-project segments from garment B onto garment A.
// A is target, B is src.
static void DrawBackProjectedSegmentsBontoA()
{
    if (gBackProjectionVizMode != BP_VizSegment)
        return;
    if (!gSegmentMappingState.hasMapping || !gSegmentMappingState.drawMapping)
        return;
    if (!gSegmentMappingState.sessionA || !gSegmentMappingState.sessionB)
        return;

    GarmentSession &sessA = *gSegmentMappingState.sessionA;
    GarmentSession &sessB = *gSegmentMappingState.sessionB;
    int numSegB = SessionNumSegments(sessB);
    if (numSegB <= 0)
        return;

    int maxPairs = gMaxMappingPairs;
    const auto &entries = gSegmentMappingState.entries;

    for (size_t idx = 0; idx < entries.size(); ++idx) {
        if (maxPairs >= 0 && static_cast<int>(idx) >= maxPairs)
            break;

        const auto &e = entries[idx];
        if (e.dstIndex < 0 || e.dstIndex >= numSegB)
            continue;

        const BackProjectionCacheKey key {
            static_cast<int>(idx), BP_DirBontoA, BP_GeomSegment,
            gBackProjectionSourceAreaMode, gQuadSubdivisions
        };
        std::shared_ptr<TraceMesh> projectedMesh = LookupBackProjectionMeshCache(key);
        if (!projectedMesh) {
            // 1) Make fresh copies of the source and target segment meshes.
            TraceMesh segMeshB; // source segment on garment B
            TraceMesh segMeshA; // corresponding target segment on garment A
            if (TraceMesh *srcMeshB = SessionSegmentMesh(sessB, e.dstIndex)) {
                vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(segMeshB, *srcMeshB);
            } else {
                continue;
            }
            if (TraceMesh *srcMeshA = SessionSegmentMesh(sessA, e.srcIndex)) {
                vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(segMeshA, *srcMeshA);
            } else {
                continue;
            }

            // 2) Revert both segments' UVs from arranged segment space back to their
            //    respective garment UV spaces.
            SessionRevertSegmentUVToGarment(sessB, e.dstIndex, segMeshB);
            SessionRevertSegmentUVToGarment(sessA, e.srcIndex, segMeshA);

            // 3) On the source segment (B), apply the inverse rigid transform (B -> A).
            ApplyInverseRigidTransformToMeshUV(segMeshB, e.rot, e.tx, e.ty);

            // 4) Inverse of A flip for B -> A path.
            if (e.flip) {
                TraceMesh::ScalarType centerX = SegmentApproxPolygonCenterX(sessA, e.srcIndex);
                ApplyHorizontalFlipToMeshUV(segMeshA, centerX);
            }
            if (!ApplyBackProjectionSourceAreaMode(segMeshB, segMeshA))
                continue;

            // 5) Back-project transformed B segment UV mesh onto target A segment mesh.
            BackProjectMeshUV(segMeshB, segMeshA);
            StoreBackProjectionMeshCache(key, segMeshB);
            projectedMesh = LookupBackProjectionMeshCache(key);
        }
        if (!projectedMesh)
            continue;

        DrawProjectedMeshOverlay(*projectedMesh, ColorForPair(static_cast<int>(idx)));
    }
}

inline int viewportIndexFromX(QWidget *w, int mouseXLogical)
{
    const int devW = std::max(1, QTDeviceWidth(w));
    const int devX = QTLogicalToDevice(w, mouseXLogical);
    return (devX < devW / 2) ? 0 : 1;
}

inline vcg::Trackball &activeTrackball(QWidget *w, int mouseXLogical)
{
    if (viewportIndexFromX(w, mouseXLogical) == 0)
        return trackLeft;
    return trackRight;
}

inline PathGL<TraceMesh> &activePathForViewport(int viewportIndex)
{
    return (viewportIndex == 0) ? gPathA : gPathB;
}

inline GarmentSession &activeSessionForViewport(int viewportIndex)
{
    return (viewportIndex == 0) ? sessionA : sessionB;
}

inline void RunBatchForViewport(int viewportIndex)
{
    GarmentSession &s = activeSessionForViewport(viewportIndex);
    PathGL<TraceMesh> &p = activePathForViewport(viewportIndex);
    s.constraintPickedPoints = p.PickedPoints;
    DoBatchProcess(s);
    ApplyDualGarmentColoring(s, viewportIndex == 0);
}

// Draws only open mesh boundary edges (pattern-piece borders), as on initial load.
static void GLDrawMeshBorderEdgesMesh(TraceMesh &mesh,
                                      vcg::Color4b col = kSeamEdgeColor,
                                      float GLSize = kSeamEdgeLineWidth)
{
    using CoordType = TraceMesh::CoordType;

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glDisable(GL_LIGHTING);
    glDepthRange(0, 0.9999);
    glLineWidth(GLSize);
    vcg::glColor(col);
    glBegin(GL_LINES);
    for (size_t i = 0; i < mesh.face.size(); ++i) {
        if (mesh.face[i].IsD())
            continue;
        for (size_t j = 0; j < 3; ++j) {
            if (!vcg::face::IsBorder(mesh.face[i], j))
                continue;
            const CoordType Pos0 = mesh.face[i].P0(j);
            const CoordType Pos1 = mesh.face[i].P1(j);
            vcg::glVertex(Pos0);
            vcg::glVertex(Pos1);
        }
    }
    glEnd();
    glPopAttrib();
}

// Draws patch boundary / seam edges for a given mesh, mirroring MyGLWidget.
void GLDrawPatchEdgesMesh(TraceMesh &mesh,
                          vcg::Color4b col = kSeamEdgeColor,
                          float GLSize = kSeamEdgeLineWidth)
{
    using CoordType = TraceMesh::CoordType;

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glDisable(GL_LIGHTING);
    glDepthRange(0,0.9999);
    glLineWidth(GLSize);
    vcg::glColor(col);
    glBegin(GL_LINES);
    for (size_t i = 0; i < mesh.face.size(); ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            bool drawEdge = false;
            drawEdge |= vcg::face::IsBorder(mesh.face[i], j);
            drawEdge |= mesh.face[i].IsFaceEdgeS(j);
            if (!drawEdge) continue;
            CoordType Pos0 = mesh.face[i].P0(j);
            CoordType Pos1 = mesh.face[i].P1(j);
            vcg::glVertex(Pos0);
            vcg::glVertex(Pos1);
        }
    }
    glEnd();
    glPopAttrib();
}

static void GLDrawGarmentSeamEdges(GarmentSession &session, bool isGarmentA)
{
    if (gInputGarmentSeamViz) {
        GLDrawMeshBorderEdgesMesh(session.deformed_mesh, kSeamEdgeColor, kSeamEdgeLineWidth);
        return;
    }
    if (gSegmentMappingState.hasMapping && gSegmentMappingState.drawMapping) {
        const auto pairs = gSegmentMappingState.pairs();
        GLDrawSegmentEdges(session, kSeamEdgeColor, kSeamEdgeLineWidth, &pairs, isGarmentA);
    } else {
        GLDrawPatchEdgesMesh(session.deformed_mesh, kSeamEdgeColor, kSeamEdgeLineWidth);
    }
}

void TW_CALL GetActiveGarmentCB(void *value, void * /*clientData*/)
{
    *static_cast<int*>(value) = activeGarmentIndex;
}

void TW_CALL SetActiveGarmentCB(const void *value, void * /*clientData*/)
{
    activeGarmentIndex = *static_cast<const int*>(value);
    if (activeGarmentIndex == 0)
        gActiveSession = &sessionA;
    else
        gActiveSession = &sessionB;
}

void TW_CALL ClearManualBorderSeamOverlaysCB(void *)
{
    sessionA.manualBorderSeamPolylines.clear();
    sessionB.manualBorderSeamPolylines.clear();
    gBorderSeamPickStage = 0;
    gPendingBorderSeamPick = false;
}

void TW_CALL CommitManualSegmentCandidateCB(void *clientData)
{
    GarmentSession *target = &activeGarmentSession();
    if (target->manualSegmentPickInProgress.empty()) {
        GarmentSession &last = activeSessionForViewport(gLastManualSegmentViewport);
        if (!last.manualSegmentPickInProgress.empty())
            target = &last;
    }
    if (CommitManualSegmentCandidate(*target)) {
        InvalidateBackProjectionCache();
        std::cout << "Manual segment candidate committed. Total manual candidates: "
                  << target->manualSegmentPatchSets.size()
                  << ", segments: " << SessionNumSegments(*target) << std::endl;
    } else {
        std::cout << "Manual segment: select at least one patch before committing." << std::endl;
    }
    if (QWidget *w = static_cast<QWidget*>(clientData))
        w->update();
}

void TW_CALL ClearManualSegmentPickCB(void *clientData)
{
    ClearManualSegmentPickInProgress(activeGarmentSession());
    if (QWidget *w = static_cast<QWidget*>(clientData))
        w->update();
}

void TW_CALL ClearManualSegmentCandidatesCB(void *clientData)
{
    ClearManualSegmentCandidates(activeGarmentSession());
    InvalidateBackProjectionCache();
    std::cout << "Cleared all manual segment candidates." << std::endl;
    if (QWidget *w = static_cast<QWidget*>(clientData))
        w->update();
}

void TW_CALL RebuildSegmentsCB(void *clientData)
{
    GarmentSession &s = activeGarmentSession();
    if (!s.PFashion) {
        std::cout << "Rebuild segments: run batch first to build the patch graph." << std::endl;
        return;
    }
    RebuildGarmentSegments(s);
    InvalidateBackProjectionCache();
    if (s.manualSegmentPickInProgress.empty() && s.manualSegmentPatchSets.empty())
        ApplyDualGarmentColoring(s, &s == &sessionA);
    if (QWidget *w = static_cast<QWidget*>(clientData))
        w->update();
}

static std::string basePathFromMeshPath(const std::string &path)
{
    size_t pos = path.find_last_of('.');
    if (pos != std::string::npos)
        return path.substr(0, pos);
    return path;
}

void TW_CALL RunActivePipelineCB(void *clientData)
{
    GarmentSession &s = (activeGarmentIndex == 0) ? sessionA : sessionB;
    PathGL<TraceMesh> &p = (activeGarmentIndex == 0) ? gPathA : gPathB;
    s.constraintPickedPoints = p.PickedPoints;
    DoBatchProcess(s);
    InvalidateBackProjectionCache();
    ApplyDualGarmentColoring(s, activeGarmentIndex == 0);
    if (QWidget *w = static_cast<QWidget*>(clientData))
        w->update();
}

void TW_CALL RunBothPipelinesCB(void *clientData)
{
    sessionA.constraintPickedPoints = gPathA.PickedPoints;
    sessionB.constraintPickedPoints = gPathB.PickedPoints;
    DoBatchProcess(sessionA);
    DoBatchProcess(sessionB);
    InvalidateBackProjectionCache();
    ApplyDualGarmentColoringBoth();
    if (QWidget *w = static_cast<QWidget*>(clientData))
        w->update();
}

static void RefreshSavedExperimentList()
{
    gSavedExperimentNames = ListSavedExperimentNames();
    if (gSelectedSavedExperimentIndex >= static_cast<int>(gSavedExperimentNames.size()))
        gSelectedSavedExperimentIndex =
            gSavedExperimentNames.empty() ? 0 : static_cast<int>(gSavedExperimentNames.size()) - 1;
}

static void UpdateSavedExperimentTwEnum()
{
    RefreshSavedExperimentList();

    gSavedExperimentEnumLabels.clear();
    gSavedExperimentEnumVals.clear();
    if (gSavedExperimentNames.empty()) {
        gSavedExperimentEnumLabels.emplace_back("(no saved experiments)");
        gSavedExperimentEnumVals.push_back(
            {0, gSavedExperimentEnumLabels.back().c_str()});
    } else {
        gSavedExperimentEnumLabels.reserve(gSavedExperimentNames.size());
        gSavedExperimentEnumVals.reserve(gSavedExperimentNames.size());
        for (size_t i = 0; i < gSavedExperimentNames.size(); ++i) {
            gSavedExperimentEnumLabels.push_back(gSavedExperimentNames[i]);
            gSavedExperimentEnumVals.push_back(
                {static_cast<int>(i), gSavedExperimentEnumLabels.back().c_str()});
        }
    }

    gSavedExperimentTwType =
        TwDefineEnum("SavedExperimentList",
                     gSavedExperimentEnumVals.data(),
                     static_cast<unsigned int>(gSavedExperimentEnumVals.size()));
}

static void EnsureSavedExperimentListControl()
{
    if (barFashion == nullptr)
        return;

    UpdateSavedExperimentTwEnum();

    if (gExperimentListVarAdded)
        TwRemoveVar(barFashion, "Saved Experiment");

    TwAddVarRW(barFashion,
               "Saved Experiment",
               gSavedExperimentTwType,
               &gSelectedSavedExperimentIndex,
               " label='Saved experiment' "
               "help='Experiments in <project>/exp/.' ");
    gExperimentListVarAdded = true;
}

void TW_CALL ExportExperimentCB(void *)
{
    sessionA.constraintPickedPoints = gPathA.PickedPoints;
    sessionB.constraintPickedPoints = gPathB.PickedPoints;

    double maxCompressionA = -0.05;
    double maxTensionA = 0.03;
    double maxCompressionB = -0.05;
    double maxTensionB = 0.03;
    GetGarmentSessionTensionParams(sessionA, maxCompressionA, maxTensionA);
    GetGarmentSessionTensionParams(sessionB, maxCompressionB, maxTensionB);

    const std::string expRoot = GetExperimentsRootDirectory(true);
    std::string experimentDir;
    if (!ExportExperimentSettings(expRoot,
                                  sessionA,
                                  sessionB,
                                  pathMeshA,
                                  pathMeshB,
                                  maxCompressionA,
                                  maxTensionA,
                                  maxCompressionB,
                                  maxTensionB,
                                  experimentDir)) {
        std::cout << "Export experiment failed." << std::endl;
        return;
    }

    std::cout << "Exported experiment to " << experimentDir << std::endl;
    SetActiveExperimentDirectory(experimentDir);
    EnsureSavedExperimentListControl();

    const std::string folderName = QFileInfo(QString::fromStdString(experimentDir)).fileName().toStdString();
    for (size_t i = 0; i < gSavedExperimentNames.size(); ++i) {
        if (gSavedExperimentNames[i] == folderName) {
            gSelectedSavedExperimentIndex = static_cast<int>(i);
            break;
        }
    }
}

void TW_CALL LoadSavedExperimentCB(void *clientData)
{
    RefreshSavedExperimentList();
    if (gSavedExperimentNames.empty()) {
        std::cout << "No saved experiments in " << GetExperimentsRootDirectory(false) << std::endl;
        return;
    }

    if (gSelectedSavedExperimentIndex < 0 ||
        gSelectedSavedExperimentIndex >= static_cast<int>(gSavedExperimentNames.size())) {
        std::cout << "Invalid experiment selection." << std::endl;
        return;
    }

    const std::string folderPath =
        ResolveExperimentFolderPath(gSavedExperimentNames[gSelectedSavedExperimentIndex]);
    if (folderPath.empty()) {
        std::cout << "Selected experiment folder is missing or invalid." << std::endl;
        EnsureSavedExperimentListControl();
        return;
    }

    ReconfigGarmentWidget *widget = static_cast<ReconfigGarmentWidget*>(clientData);
    if (!widget->loadExperiment(folderPath)) {
        std::cout << "Failed to load experiment. See console for details." << std::endl;
    }
}

static std::string polygonExportBaseFromOutputPath(const std::string &polygonPath)
{
    const std::string suffix = "_polygons.txt";
    if (polygonPath.size() >= suffix.size() &&
        polygonPath.compare(polygonPath.size() - suffix.size(), suffix.size(), suffix) == 0) {
        return polygonPath.substr(0, polygonPath.size() - suffix.size());
    }
    return polygonPath;
}

void TW_CALL ExportBothPolygonsCB(void *)
{
    const SegmentMappingArtifactPaths artifacts =
        SegmentMappingArtifactsForMeshes(pathMeshA, pathMeshB);
    ExportPolygonsForSession(sessionA, polygonExportBaseFromOutputPath(artifacts.polygonsA));
    ExportPolygonsForSession(sessionB, polygonExportBaseFromOutputPath(artifacts.polygonsB));
}

static bool ParseSegmentMappingTimingLine(const QString &line,
                                          const QString &key,
                                          double &secondsOut)
{
    const QString prefix = QString("[reconfigGarment_timing] %1=").arg(key);
    if (!line.startsWith(prefix))
        return false;
    bool ok = false;
    secondsOut = line.mid(prefix.size()).toDouble(&ok);
    return ok;
}

static void ReportSegmentMappingTiming(const QByteArray &stdoutBytes,
                                       double &mappingSecondsOut,
                                       double &polygonFitSecondsOut)
{
    mappingSecondsOut = -1.0;
    polygonFitSecondsOut = -1.0;
    const QString out = QString::fromUtf8(stdoutBytes);
    const QStringList lines = out.split('\n', Qt::SkipEmptyParts);
    for (const QString &line : lines) {
        ParseSegmentMappingTimingLine(line, "mapping_solve_seconds", mappingSecondsOut);
        ParseSegmentMappingTimingLine(line, "polygon_fit_seconds", polygonFitSecondsOut);
    }
    if (mappingSecondsOut >= 0.0) {
        std::cout << "[SegmentMapping] Mapping solve time: "
                  << mappingSecondsOut << " s" << std::endl;
    }
    if (polygonFitSecondsOut >= 0.0) {
        std::cout << "[SegmentMapping] Polygon fit time: "
                  << polygonFitSecondsOut << " s" << std::endl;
    }
}

void TW_CALL RunSegmentMappingCB(void *)
{
    const SegmentMappingArtifactPaths artifacts =
        SegmentMappingArtifactsForMeshes(pathMeshA, pathMeshB);

    ExportPolygonsForSession(sessionA, polygonExportBaseFromOutputPath(artifacts.polygonsA));
    ExportPolygonsForSession(sessionB, polygonExportBaseFromOutputPath(artifacts.polygonsB));

    const std::string &polyA = artifacts.polygonsA;
    const std::string &polyB = artifacts.polygonsB;
    const std::string &outPath = artifacts.mapping;

    QProcess proc;
    QString appDir = QCoreApplication::applicationDirPath();
    QString scriptPath = appDir + "/../segmentMappingSolver/run_mapping_solver.py";
    QStringList args;
    args << scriptPath << QString::fromStdString(polyA) << QString::fromStdString(polyB) << QString::fromStdString(outPath);
    proc.start("python", args);
    if (!proc.waitForStarted(3000)) {
        proc.start("python", args);
        if (!proc.waitForStarted(3000)) {
            QMessageBox::warning(nullptr, "Segment Mapping",
                "Could not start Python. Ensure 'python3' or 'python' is on PATH and run from parafashion project root.");
            return;
        }
    }
    if (!proc.waitForFinished(3000000)) {
        QMessageBox::warning(nullptr, "Segment Mapping", "Python solver timed out.");
        return;
    }
    const QByteArray stdoutBytes = proc.readAllStandardOutput();
    if (!stdoutBytes.isEmpty())
        std::cout << stdoutBytes.constData();

    if (proc.exitStatus() != QProcess::NormalExit || proc.exitCode() != 0) {
        QMessageBox::warning(nullptr, "Segment Mapping",
            QString("Solver failed: %1").arg(QString::fromUtf8(proc.readAllStandardError())));
        return;
    }

    double mappingSeconds = -1.0;
    double polygonFitSeconds = -1.0;
    ReportSegmentMappingTiming(stdoutBytes, mappingSeconds, polygonFitSeconds);

    QString timingMsg;
    if (mappingSeconds >= 0.0) {
        timingMsg += QString("\nMapping solve: %1 s")
                         .arg(mappingSeconds, 0, 'f', 3);
    }
    if (polygonFitSeconds >= 0.0) {
        timingMsg += QString("\nPolygon fit: %1 s")
                         .arg(polygonFitSeconds, 0, 'f', 3);
    }

    const std::string &quadOut = artifacts.quads;
    QMessageBox::information(nullptr, "Segment Mapping",
        QString("Mapping written to %1.\nApprox polygons (if fitted): %2%3\n"
                "Click 'Load Mapping & Color', then use 3D view with Back-Projection Viz = "
                "'Approx polygon (N verts)'.")
            .arg(QString::fromStdString(outPath))
            .arg(QString::fromStdString(quadOut))
            .arg(timingMsg));
}

static bool ParseQuadPlacementLine(const QString &line, QuadPlacement &placement)
{
    const QStringList parts = line.split(' ', Qt::SkipEmptyParts);
    if (parts.size() < 4)
        return false;
    bool ok = false;
    const int flipInt = parts[0].toInt(&ok);
    if (!ok) return false;
    placement.flip = (flipInt != 0);
    placement.tx = parts[1].toDouble(&ok);
    if (!ok) return false;
    placement.ty = parts[2].toDouble(&ok);
    if (!ok) return false;
    placement.rot = parts[3].toDouble(&ok);
    return ok;
}

static std::string QuadPathFromMappingPath(const std::string &mappingPath)
{
    // Must match run_mapping_solver.py: output.txt -> output_quads.txt
    const std::string segSuffix = "_segment_mapping.txt";
    const std::string segQuadSuffix = "_segment_mapping_quads.txt";
    if (mappingPath.size() >= segSuffix.size()
        && mappingPath.compare(mappingPath.size() - segSuffix.size(), segSuffix.size(), segSuffix) == 0) {
        return mappingPath.substr(0, mappingPath.size() - segSuffix.size()) + segQuadSuffix;
    }
    const std::string dotTxt = ".txt";
    if (mappingPath.size() >= dotTxt.size()
        && mappingPath.compare(mappingPath.size() - dotTxt.size(), dotTxt.size(), dotTxt) == 0) {
        return mappingPath.substr(0, mappingPath.size() - dotTxt.size()) + "_quads.txt";
    }
    return mappingPath + "_quads.txt";
}

static bool TryLoadQuadFile(const std::string &quadPath, int &loadedOut)
{
    loadedOut = 0;
    QFile f(QString::fromStdString(quadPath));
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
        return false;

    QTextStream in(&f);
    bool ok = false;
    const int numQuads = in.readLine().trimmed().toInt(&ok);
    if (!ok || numQuads < 0) {
        f.close();
        return false;
    }

    for (int q = 0; q < numQuads; ++q) {
        const QString indexLine = in.readLine().trimmed();
        if (indexLine.isEmpty())
            continue;
        QStringList idxParts = indexLine.split(' ', Qt::SkipEmptyParts);
        if (idxParts.size() < 2)
            continue;
        const int srcIdx = idxParts[0].toInt(&ok);
        if (!ok) continue;
        const int dstIdx = idxParts[1].toInt(&ok);
        if (!ok) continue;

        int nVerts = kMinApproxVerts;
        QString coordLine;
        if (idxParts.size() >= 3) {
            nVerts = idxParts[2].toInt(&ok);
            if (!ok || nVerts < kMinApproxVerts || nVerts > kMaxApproxVerts)
                continue;
            in.readLine();  // score (ignored for viz)
            coordLine = in.readLine().trimmed();
        } else {
            coordLine = in.readLine().trimmed();
            QStringList maybeN = coordLine.split(' ', Qt::SkipEmptyParts);
            if (maybeN.size() == 1) {
                nVerts = maybeN[0].toInt(&ok);
                if (!ok || nVerts < kMinApproxVerts || nVerts > kMaxApproxVerts)
                    continue;
                in.readLine();  // score
                coordLine = in.readLine().trimmed();
            }
        }

        const QString placeALine = in.readLine().trimmed();
        const QString placeBLine = in.readLine().trimmed();
        if (coordLine.isEmpty() || placeALine.isEmpty() || placeBLine.isEmpty())
            continue;

        QStringList coordParts = coordLine.split(' ', Qt::SkipEmptyParts);
        if (idxParts.size() < 3) {
            if (coordParts.size() % 2 != 0)
                continue;
            nVerts = coordParts.size() / 2;
        }
        if (nVerts < kMinApproxVerts || nVerts > kMaxApproxVerts)
            continue;
        if (coordParts.size() < 2 * nVerts)
            continue;

        SegmentMappingEntry *match = nullptr;
        for (auto &entry : gSegmentMappingState.entries) {
            if (entry.srcIndex == srcIdx && entry.dstIndex == dstIdx) {
                match = &entry;
                break;
            }
        }
        if (!match)
            continue;

        match->approxVerts.clear();
        match->approxVerts.reserve(static_cast<size_t>(nVerts));
        bool coordsOk = true;
        for (int k = 0; k < nVerts; ++k) {
            bool okX = false, okY = false;
            const double x = coordParts[2 * k].toDouble(&okX);
            const double y = coordParts[2 * k + 1].toDouble(&okY);
            if (!okX || !okY) {
                coordsOk = false;
                break;
            }
            match->approxVerts.emplace_back(x, y);
        }
        if (!coordsOk)
            continue;
        match->approxNumVerts = nVerts;
        if (!ParseQuadPlacementLine(placeALine, match->placementA))
            continue;
        if (!ParseQuadPlacementLine(placeBLine, match->placementB))
            continue;
        match->hasQuad = true;
        ++loadedOut;
    }
    f.close();
    return true;
}

static void LoadQuadApproximationsForMapping(const std::string &mappingPath,
                                             const std::string &preferredQuadPath = {})
{
    // Clear previous approx data on all entries.
    for (auto &entry : gSegmentMappingState.entries) {
        entry.hasQuad = false;
        entry.approxNumVerts = 0;
        entry.approxVerts.clear();
    }

    std::vector<std::string> candidates;
    if (!preferredQuadPath.empty())
        candidates.push_back(preferredQuadPath);
    const std::string derived = QuadPathFromMappingPath(mappingPath);
    if (derived != preferredQuadPath)
        candidates.push_back(derived);

    int loaded = 0;
    std::string loadedFrom;
    for (const std::string &quadPath : candidates) {
        if (TryLoadQuadFile(quadPath, loaded) && loaded > 0) {
            loadedFrom = quadPath;
            break;
        }
    }

    if (loaded == 0) {
        std::cout << "[QuadMapping] No approx polygons loaded. Tried:";
        for (const std::string &p : candidates)
            std::cout << " " << p;
        std::cout << "\n  Re-run segment mapping (ensure quad export succeeded), then Load Mapping & Color."
                  << std::endl;
        std::cout << "  In the UI set Back-Projection Viz to 'Approx polygon (N verts)' (3D view, not UV)."
                  << std::endl;
        return;
    }

    gBackProjectionVizMode = BP_VizQuad;
    std::cout << "[QuadMapping] Loaded " << loaded << " approx polygon(s) from " << loadedFrom
              << ". Back-Projection Viz set to Approx polygon." << std::endl;
}

static double ParsePositiveAreaString(const char *text, const char *label)
{
    if (text == nullptr || text[0] == '\0') {
        std::cout << "[FabricationExport] " << label << " is empty." << std::endl;
        return -1.0;
    }
    char *end = nullptr;
    const double value = std::strtod(text, &end);
    while (end != nullptr && *end != '\0' && std::isspace(static_cast<unsigned char>(*end)))
        ++end;
    if (end == nullptr || *end != '\0' || !(value > 0.0) || !std::isfinite(value)) {
        std::cout << "[FabricationExport] Invalid " << label << ": \"" << text << "\"." << std::endl;
        return -1.0;
    }
    return value;
}

static bool BuildMappedApproxPolygonGarmentUV(const SegmentMappingEntry &entry,
                                              bool isGarmentA,
                                              GarmentSession &session,
                                              std::vector<vcg::Point2<TraceMesh::ScalarType>> &outUV);

// Polygon vertices for 04 export / Draw Approx Polygon UV (canonical fit, not garment placement).
static bool BuildDualExportPolygonVertsForArrangedUV(const SegmentMappingEntry &entry,
                                                     bool isGarmentA,
                                                     GarmentSession &session,
                                                     std::vector<vcg::Point2<TraceMesh::ScalarType>> &outUV)
{
    outUV.clear();
    const int segIdx = isGarmentA ? entry.srcIndex : entry.dstIndex;
    if (entry.hasQuad && entry.approxVerts.size() >= 3) {
        outUV.reserve(entry.approxVerts.size());
        for (const auto &v : entry.approxVerts)
            outUV.emplace_back(TraceMesh::ScalarType(v.x()), TraceMesh::ScalarType(v.y()));
        return true;
    }
    if (segIdx >= 0 && SessionApproxPolygonVerts(session, segIdx, outUV))
        return outUV.size() >= 3;
    return BuildMappedApproxPolygonGarmentUV(entry, isGarmentA, session, outUV);
}

static bool BuildMappedApproxPolygonGarmentUV(const SegmentMappingEntry &entry,
                                              bool isGarmentA,
                                              GarmentSession &session,
                                              std::vector<vcg::Point2<TraceMesh::ScalarType>> &outUV)
{
    outUV.clear();
    const int segIdx = isGarmentA ? entry.srcIndex : entry.dstIndex;
    if (segIdx < 0)
        return false;

    if (entry.hasQuad && entry.approxVerts.size() >= 3) {
        std::vector<vcg::Point2<TraceMesh::ScalarType>> canonical;
        canonical.reserve(entry.approxVerts.size());
        for (const auto &v : entry.approxVerts)
            canonical.emplace_back(TraceMesh::ScalarType(v.x()), TraceMesh::ScalarType(v.y()));

        ApproxPolygonPlacement placement;
        const QuadPlacement &qp = isGarmentA ? entry.placementA : entry.placementB;
        placement.flip = qp.flip;
        placement.tx = qp.tx;
        placement.ty = qp.ty;
        placement.rot = qp.rot;
        TransformApproxPolygonWithPlacement(canonical, placement, outUV);
        return outUV.size() >= 3;
    }

    return SessionApproxPolygonGarmentUV(session, segIdx, outUV);
}

static bool BuildSegmentSeamEndpointApproxPolygonGarmentUV(const SegmentMappingEntry &entry,
                                                           bool isGarmentA,
                                                           GarmentSession &session,
                                                           std::vector<vcg::Point2<TraceMesh::ScalarType>> &outUV)
{
    const int segIdx = isGarmentA ? entry.srcIndex : entry.dstIndex;
    if (segIdx < 0)
        return false;
    return SessionApproxPolygonGarmentUV(session, segIdx, outUV);
}

using ApproxPolygonBuilderFn = bool (*)(const SegmentMappingEntry &,
                                        bool,
                                        GarmentSession &,
                                        std::vector<vcg::Point2<TraceMesh::ScalarType>> &);

static bool ExportScaledMappingPairPolygons(const std::string &exportRoot,
                                            ApproxPolygonBuilderFn buildPoly,
                                            TraceMesh::ScalarType scaleA,
                                            TraceMesh::ScalarType scaleB,
                                            int &exportedA,
                                            int &exportedB)
{
    exportedA = 0;
    exportedB = 0;

    const std::string dirA = QDir(QString::fromStdString(exportRoot))
                                 .filePath(QString::fromStdString(
                                     GarmentBaseNameFromPath(pathMeshA) + "_garmentA"))
                                 .toStdString();
    const std::string dirB = QDir(QString::fromStdString(exportRoot))
                                 .filePath(QString::fromStdString(
                                     GarmentBaseNameFromPath(pathMeshB) + "_garmentB"))
                                 .toStdString();

    if (!QDir().mkpath(QString::fromStdString(dirA)) ||
        !QDir().mkpath(QString::fromStdString(dirB))) {
        std::cout << "[FabricationExport] Could not create output directories under " << exportRoot
                  << std::endl;
        return false;
    }

    const auto &entries = gSegmentMappingState.entries;
    for (size_t idx = 0; idx < entries.size(); ++idx) {
        if (gMaxMappingPairs >= 0 && static_cast<int>(idx) >= gMaxMappingPairs)
            break;

        const auto &e = entries[idx];
        const std::string stem =
            MappingColorToFileStem(ColorForPair(static_cast<int>(idx)), static_cast<int>(idx));

        std::vector<vcg::Point2<TraceMesh::ScalarType>> polyA;
        if (buildPoly(e, true, sessionA, polyA)) {
            ScalePolygonVertsInPlace(polyA, scaleA);
            const std::string pathA = QDir(QString::fromStdString(dirA))
                                          .filePath(QString::fromStdString(stem + ".txt"))
                                          .toStdString();
            if (WriteApproxPolygonTxt(pathA, polyA))
                ++exportedA;
        }

        std::vector<vcg::Point2<TraceMesh::ScalarType>> polyB;
        if (buildPoly(e, false, sessionB, polyB)) {
            ScalePolygonVertsInPlace(polyB, scaleB);
            const std::string pathB = QDir(QString::fromStdString(dirB))
                                          .filePath(QString::fromStdString(stem + ".txt"))
                                          .toStdString();
            if (WriteApproxPolygonTxt(pathB, polyB))
                ++exportedB;
        }
    }
    return true;
}

static void WritePolygonExportInfo(const std::string &exportRoot,
                                   const char *polygonKind,
                                   double realAreaA,
                                   double realAreaB,
                                   double meshAreaA,
                                   double meshAreaB,
                                   TraceMesh::ScalarType scaleA,
                                   TraceMesh::ScalarType scaleB,
                                   int exportedA,
                                   int exportedB)
{
    const std::string infoPath = QDir(QString::fromStdString(exportRoot))
                                     .filePath(QString::fromLatin1("export_info.txt"))
                                     .toStdString();
    std::ofstream info(infoPath);
    if (!info.is_open())
        return;

    info << "polygon_kind " << polygonKind << "\n";
    info << "real_garment_area_A " << realAreaA << "\n";
    info << "real_garment_area_B " << realAreaB << "\n";
    info << "mesh_area_A " << meshAreaA << "\n";
    info << "mesh_area_B " << meshAreaB << "\n";
    info << "linear_scale_A " << scaleA << "\n";
    info << "linear_scale_B " << scaleB << "\n";
    info << "exported_polygons_A " << exportedA << "\n";
    info << "exported_polygons_B " << exportedB << "\n";
    info << "units note: polygon coordinates are in sqrt(real_area) space; "
            "use the same length unit as real_area (e.g. m if area is m^2).\n";
}

void TW_CALL ExportFabricationApproxPolygonsCB(void *)
{
    if (!gSegmentMappingState.hasMapping || gSegmentMappingState.entries.empty()) {
        QMessageBox::warning(nullptr,
                           "Export Fabrication Polygons",
                           "Load segment mapping first (Run Segment Mapping, then Load Mapping & Color).");
        return;
    }

    const double realAreaA = ParsePositiveAreaString(gRealGarmentAreaA, "Real garment area A");
    const double realAreaB = ParsePositiveAreaString(gRealGarmentAreaB, "Real garment area B");
    if (realAreaA <= 0.0 || realAreaB <= 0.0)
        return;

    const double meshAreaA = SessionGarmentMeshArea(sessionA);
    const double meshAreaB = SessionGarmentMeshArea(sessionB);
    if (meshAreaA <= 0.0 || meshAreaB <= 0.0) {
        std::cout << "[FabricationExport] Garment mesh area must be positive." << std::endl;
        return;
    }

    const TraceMesh::ScalarType scaleA =
        static_cast<TraceMesh::ScalarType>(std::sqrt(realAreaA / meshAreaA));
    const TraceMesh::ScalarType scaleB =
        static_cast<TraceMesh::ScalarType>(std::sqrt(realAreaB / meshAreaB));

    const std::string fittedExportRoot =
        ResolveFabricationApproxExportDirectory(pathMeshA, pathMeshB);
    const std::string segmentExportRoot =
        ResolveSegmentApproxExportDirectory(pathMeshA, pathMeshB);

    int fittedExportedA = 0;
    int fittedExportedB = 0;
    if (!ExportScaledMappingPairPolygons(fittedExportRoot,
                                         BuildMappedApproxPolygonGarmentUV,
                                         scaleA,
                                         scaleB,
                                         fittedExportedA,
                                         fittedExportedB))
        return;

    int segmentExportedA = 0;
    int segmentExportedB = 0;
    if (!ExportScaledMappingPairPolygons(segmentExportRoot,
                                         BuildSegmentSeamEndpointApproxPolygonGarmentUV,
                                         scaleA,
                                         scaleB,
                                         segmentExportedA,
                                         segmentExportedB))
        return;

    WritePolygonExportInfo(fittedExportRoot,
                           "fitted_approx_polygon",
                           realAreaA,
                           realAreaB,
                           meshAreaA,
                           meshAreaB,
                           scaleA,
                           scaleB,
                           fittedExportedA,
                           fittedExportedB);
    WritePolygonExportInfo(segmentExportRoot,
                           "segment_seam_endpoint_approx_polygon",
                           realAreaA,
                           realAreaB,
                           meshAreaA,
                           meshAreaB,
                           scaleA,
                           scaleB,
                           segmentExportedA,
                           segmentExportedB);

    const QString msg =
        QString("Fitted polygons: %1 (A), %2 (B)\n"
                "Segment seam-endpoint approx polygons: %3 (A), %4 (B)\n\n"
                "Fitted:\n%5\n\nSegment approx:\n%6")
            .arg(fittedExportedA)
            .arg(fittedExportedB)
            .arg(segmentExportedA)
            .arg(segmentExportedB)
            .arg(QString::fromStdString(fittedExportRoot))
            .arg(QString::fromStdString(segmentExportRoot));
    QMessageBox::information(nullptr, "Export Fabrication Polygons", msg);
    std::cout << "[FabricationExport] Wrote fitted polygons to " << fittedExportRoot << std::endl;
    std::cout << "[FabricationExport] Wrote segment approx polygons to " << segmentExportRoot
              << std::endl;
}

void TW_CALL LoadMappingAndColorCB(void *clientData)
{
    const SegmentMappingArtifactPaths artifacts =
        SegmentMappingArtifactsForMeshes(pathMeshA, pathMeshB);
    const std::string &path = artifacts.mapping;
    QFile f(QString::fromStdString(path));
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(nullptr, "Load Mapping",
            QString("Could not open %1. Run 'Run Segment Mapping' first.").arg(QString::fromStdString(path)));
        return;
    }
    gSegmentMappingState.entries.clear();
    InvalidateBackProjectionCache();
    QTextStream in(&f);
    bool ok = false;
    int numPairs = in.readLine().trimmed().toInt(&ok);
    if (!ok || numPairs < 0) { f.close(); return; }
    for (int i = 0; i < numPairs; i++) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty())
            continue;

        QStringList parts = line.split(' ', Qt::SkipEmptyParts);
        if (parts.size() < 2)
            continue;

        // Python solver format: i j score flip tx ty rot
        SegmentMappingEntry entry;
        entry.srcIndex = parts[0].toInt(&ok);
        if (!ok) continue;
        entry.dstIndex = parts[1].toInt(&ok);
        if (!ok) continue;

        if (parts.size() >= 3) {
            bool okExtra = false;
            entry.score = parts[2].toDouble(&okExtra);
        }
        if (parts.size() >= 4) {
            const QString flipStr = parts[3].trimmed().toLower();
            entry.flip = (flipStr == "true" || flipStr == "1");
        }
        if (parts.size() >= 6) {
            bool okExtra = false;
            entry.tx = parts[4].toDouble(&okExtra);
            entry.ty = parts[5].toDouble(&okExtra);
        }
        if (parts.size() >= 7) {
            bool okExtra = false;
            entry.rot = parts[6].toDouble(&okExtra);
        }

        gSegmentMappingState.entries.push_back(entry);
    }
    f.close();

    LoadQuadApproximationsForMapping(path, artifacts.quads);

    // Sanity check: ensure mapping covers all patches on both garments.
    const auto ijPairs = gSegmentMappingState.pairs();
    bool okA = CheckPatchCoverageForSession(sessionA, ijPairs, true);
    bool okB = CheckPatchCoverageForSession(sessionB, ijPairs, false);
    std::cout<<"okA: "<<okA<<" okB: "<<okB<<std::endl;
    if (!okA || !okB) {
        QMessageBox::warning(nullptr, "Segment Mapping",
            "Loaded mapping does not cover all patches on one or both garments.\n"
            "Some patches may remain grey.");
    }

    gSegmentMappingState.hasMapping = true;
    gSegmentMappingState.drawMapping = true;
    ApplySegmentMappingColors(sessionA, ijPairs, true);
    ApplySegmentMappingColors(sessionB, ijPairs, false);
    if (QWidget *w = static_cast<QWidget*>(clientData))
        w->update();
}

static bool BuildQuadBackProjectedMeshForExport(const SegmentMappingEntry &e,
                                                bool fromAtoB,
                                                TraceMesh &outMesh,
                                                bool overlapOnly = true)
{
    GarmentSession &sessA = sessionA;
    GarmentSession &sessB = sessionB;
    if (!e.hasQuad || e.approxNumVerts < 4)
        return false;

    if (fromAtoB) {
        if (e.srcIndex < 0 || e.dstIndex < 0)
            return false;
        TraceMesh segMeshB;
        if (TraceMesh *dstMeshB = SessionSegmentMesh(sessB, e.dstIndex))
            vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(segMeshB, *dstMeshB);
        else
            return false;
        SessionRevertSegmentUVToGarment(sessB, e.dstIndex, segMeshB);

        std::vector<Eigen::Vector2d> approxUV;
        ApplyPlacementToApproxUVs(e.approxVerts, e.placementA, approxUV);
        TraceMesh quadMesh;
        BuildTriangulatedApproxMesh(quadMesh, approxUV);
        if (e.flip) {
            const TraceMesh::ScalarType centerX = SegmentApproxPolygonCenterX(sessA, e.srcIndex);
            ApplyHorizontalFlipToMeshUV(quadMesh, centerX);
        }
        ApplyRigidTransformToMeshUV(quadMesh, e.rot, e.tx, e.ty);

        TraceMesh clipped;
        TraceMesh *sourceMesh = &quadMesh;
        if (overlapOnly) {
            vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(clipped, quadMesh);
            if (!BuildOverlapOnlySourceMeshInUV(quadMesh, segMeshB, clipped))
                return false;
            sourceMesh = &clipped;
        }
        BackProjectMeshUV(*sourceMesh, segMeshB);
        outMesh.Clear();
        vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(outMesh, *sourceMesh);
        outMesh.UpdateAttributes();
        return true;
    }

    if (e.srcIndex < 0 || e.dstIndex < 0)
        return false;
    TraceMesh segMeshA;
    if (TraceMesh *dstMeshA = SessionSegmentMesh(sessA, e.srcIndex))
        vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(segMeshA, *dstMeshA);
    else
        return false;
    SessionRevertSegmentUVToGarment(sessA, e.srcIndex, segMeshA);

    std::vector<Eigen::Vector2d> approxUV;
    ApplyPlacementToApproxUVs(e.approxVerts, e.placementB, approxUV);
    TraceMesh quadMesh;
    BuildTriangulatedApproxMesh(quadMesh, approxUV);
    ApplyInverseRigidTransformToMeshUV(quadMesh, e.rot, e.tx, e.ty);
    if (e.flip) {
        const TraceMesh::ScalarType centerX = SegmentApproxPolygonCenterX(sessA, e.srcIndex);
        ApplyHorizontalFlipToMeshUV(segMeshA, centerX);
    }

    TraceMesh clipped;
    TraceMesh *sourceMesh = &quadMesh;
    if (overlapOnly) {
        vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(clipped, quadMesh);
        if (!BuildOverlapOnlySourceMeshInUV(quadMesh, segMeshA, clipped))
            return false;
        sourceMesh = &clipped;
    }
    BackProjectMeshUV(*sourceMesh, segMeshA);
    outMesh.Clear();
    vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(outMesh, *sourceMesh);
    outMesh.UpdateAttributes();
    return true;
}

static bool BuildSegmentBackProjectedMeshForExport(const SegmentMappingEntry &e,
                                                   bool fromAtoB,
                                                   TraceMesh &outMesh,
                                                   bool overlapOnly = false)
{
    GarmentSession &sessA = sessionA;
    GarmentSession &sessB = sessionB;
    if (e.srcIndex < 0 || e.dstIndex < 0)
        return false;

    if (fromAtoB) {
        TraceMesh segMeshA;
        TraceMesh segMeshB;
        if (TraceMesh *srcMeshA = SessionSegmentMesh(sessA, e.srcIndex))
            vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(segMeshA, *srcMeshA);
        else
            return false;
        if (TraceMesh *srcMeshB = SessionSegmentMesh(sessB, e.dstIndex))
            vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(segMeshB, *srcMeshB);
        else
            return false;

        SessionRevertSegmentUVToGarment(sessA, e.srcIndex, segMeshA);
        SessionRevertSegmentUVToGarment(sessB, e.dstIndex, segMeshB);

        if (e.flip) {
            const TraceMesh::ScalarType centerX = SegmentApproxPolygonCenterX(sessA, e.srcIndex);
            ApplyHorizontalFlipToMeshUV(segMeshA, centerX);
        }
        ApplyRigidTransformToMeshUV(segMeshA, e.rot, e.tx, e.ty);

        TraceMesh clipped;
        TraceMesh *sourceMesh = &segMeshA;
        if (overlapOnly) {
            if (!BuildOverlapOnlySourceMeshInUV(segMeshA, segMeshB, clipped))
                return false;
            sourceMesh = &clipped;
        }
        BackProjectMeshUV(*sourceMesh, segMeshB);
        outMesh.Clear();
        vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(outMesh, *sourceMesh);
        outMesh.UpdateAttributes();
        return true;
    }

    TraceMesh segMeshB;
    TraceMesh segMeshA;
    if (TraceMesh *srcMeshB = SessionSegmentMesh(sessB, e.dstIndex))
        vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(segMeshB, *srcMeshB);
    else
        return false;
    if (TraceMesh *srcMeshA = SessionSegmentMesh(sessA, e.srcIndex))
        vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(segMeshA, *srcMeshA);
    else
        return false;

    SessionRevertSegmentUVToGarment(sessB, e.dstIndex, segMeshB);
    SessionRevertSegmentUVToGarment(sessA, e.srcIndex, segMeshA);

    ApplyInverseRigidTransformToMeshUV(segMeshB, e.rot, e.tx, e.ty);
    if (e.flip) {
        const TraceMesh::ScalarType centerX = SegmentApproxPolygonCenterX(sessA, e.srcIndex);
        ApplyHorizontalFlipToMeshUV(segMeshA, centerX);
    }

    TraceMesh clipped;
    TraceMesh *sourceMesh = &segMeshB;
    if (overlapOnly) {
        if (!BuildOverlapOnlySourceMeshInUV(segMeshB, segMeshA, clipped))
            return false;
        sourceMesh = &clipped;
    }
    BackProjectMeshUV(*sourceMesh, segMeshA);
    outMesh.Clear();
    vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(outMesh, *sourceMesh);
    outMesh.UpdateAttributes();
    return true;
}

void TW_CALL ExportDualMeshesAndMappingCB(void *)
{
    const std::string exportRoot =
        ResolveExperimentExportDirectory(pathMeshA, pathMeshB, "dual_export");
    if (!EnsureDir(exportRoot, "DualExport"))
        return;

    const std::string originalsDir = QDir(QString::fromStdString(exportRoot))
                                         .filePath("01_original_garments")
                                         .toStdString();
    const std::string patchDir = QDir(QString::fromStdString(exportRoot))
                                     .filePath("02_patch_meshes_grey")
                                     .toStdString();
    const std::string segDir = QDir(QString::fromStdString(exportRoot))
                                   .filePath("03_segment_meshes_colored")
                                   .toStdString();
    const std::string polyDir = QDir(QString::fromStdString(exportRoot))
                                    .filePath("04_solved_polygons_2d")
                                    .toStdString();
    const std::string backProjDir = QDir(QString::fromStdString(exportRoot))
                                        .filePath("05_segment_meshes_backprojected_approx_overlap")
                                        .toStdString();
    const std::string garmentSegColorDir = QDir(QString::fromStdString(exportRoot))
                                               .filePath("06_complete_garments_segment_colored")
                                               .toStdString();
    const std::string flatSegDir = QDir(QString::fromStdString(exportRoot))
                                       .filePath("07_flattened_segment_meshes_grey")
                                       .toStdString();
    const std::string backProjFullDir = QDir(QString::fromStdString(exportRoot))
                                            .filePath("08_segment_meshes_backprojected_approx_full")
                                            .toStdString();
    const std::string backProjMutualDir = QDir(QString::fromStdString(exportRoot))
                                              .filePath("09_segment_meshes_backprojected_mutual_full")
                                              .toStdString();

    if (!EnsureDir(originalsDir, "DualExport") ||
        !EnsureDir(patchDir, "DualExport") ||
        !EnsureDir(segDir, "DualExport") ||
        !EnsureDir(polyDir, "DualExport") ||
        !EnsureDir(backProjDir, "DualExport") ||
        !EnsureDir(garmentSegColorDir, "DualExport") ||
        !EnsureDir(flatSegDir, "DualExport") ||
        !EnsureDir(backProjFullDir, "DualExport") ||
        !EnsureDir(backProjMutualDir, "DualExport"))
        return;

    const vcg::Color4b kExportGrey(220, 220, 220, 255);

    // 1) Original garment meshes (same neutral grey as 02 patch meshes).
    {
        const std::string nameA = GarmentBaseNameFromPath(pathMeshA);
        const std::string nameB = GarmentBaseNameFromPath(pathMeshB);
        const std::string outA = QDir(QString::fromStdString(originalsDir))
                                     .filePath(QString::fromStdString(nameA + "_original.obj"))
                                     .toStdString();
        const std::string outB = QDir(QString::fromStdString(originalsDir))
                                     .filePath(QString::fromStdString(nameB + "_original.obj"))
                                     .toStdString();
        TraceMesh origA;
        TraceMesh origB;
        vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(origA, sessionA.reference_mesh);
        vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(origB, sessionB.reference_mesh);
        vcg::tri::UpdateColor<TraceMesh>::PerFaceConstant(origA, kExportGrey);
        vcg::tri::UpdateColor<TraceMesh>::PerFaceConstant(origB, kExportGrey);
        origA.UpdateAttributes();
        origB.UpdateAttributes();
        ExportMeshObjWithFaceColors(origA, sessionA.centerRef, outA, "DualExport");
        ExportMeshObjWithFaceColors(origB, sessionB.centerRef, outB, "DualExport");
    }

    // 2) Patch meshes (post-batch + manual seams), one mesh per patch, neutral grey.
    {
        const std::string nameA = GarmentBaseNameFromPath(pathMeshA);
        const std::string nameB = GarmentBaseNameFromPath(pathMeshB);
        const std::string dirA = QDir(QString::fromStdString(patchDir)).filePath(QString::fromStdString(nameA + "_garmentA")).toStdString();
        const std::string dirB = QDir(QString::fromStdString(patchDir)).filePath(QString::fromStdString(nameB + "_garmentB")).toStdString();
        if (!EnsureDir(dirA, "DualExport") || !EnsureDir(dirB, "DualExport"))
            return;

        int patchCountA = 0;
        int patchCountB = 0;
        {
            const int np = SessionNumPatches(sessionA);
            for (int pid = 0; pid < np; ++pid) {
                TraceMesh patchMesh;
                if (!BuildPatchSubmeshByPatchId(sessionA, pid, kExportGrey, patchMesh))
                    continue;
                const std::string out = QDir(QString::fromStdString(dirA))
                                            .filePath(QString::fromStdString("patch_" + std::to_string(pid) + "_grey.obj"))
                                            .toStdString();
                if (ExportMeshObjWithFaceColors(patchMesh, sessionA.centerDef, out, "DualExport"))
                    ++patchCountA;
            }
        }
        {
            const int np = SessionNumPatches(sessionB);
            for (int pid = 0; pid < np; ++pid) {
                TraceMesh patchMesh;
                if (!BuildPatchSubmeshByPatchId(sessionB, pid, kExportGrey, patchMesh))
                    continue;
                const std::string out = QDir(QString::fromStdString(dirB))
                                            .filePath(QString::fromStdString("patch_" + std::to_string(pid) + "_grey.obj"))
                                            .toStdString();
                if (ExportMeshObjWithFaceColors(patchMesh, sessionB.centerDef, out, "DualExport"))
                    ++patchCountB;
            }
        }
        std::cout << "[DualExport] Patch meshes exported: A=" << patchCountA
                  << " B=" << patchCountB << std::endl;
    }

    // 7) Flattened 2D segment meshes (full tri mesh, per-patch shading), all segments per garment
    //    (same segment indices as Export Polygons In txt). Mapped segments use pair colors (04).
    {
        const std::string nameA = GarmentBaseNameFromPath(pathMeshA);
        const std::string nameB = GarmentBaseNameFromPath(pathMeshB);
        const std::string dirA = QDir(QString::fromStdString(flatSegDir))
                                     .filePath(QString::fromStdString(nameA + "_garmentA"))
                                     .toStdString();
        const std::string dirB = QDir(QString::fromStdString(flatSegDir))
                                     .filePath(QString::fromStdString(nameB + "_garmentB"))
                                     .toStdString();
        if (!EnsureDir(dirA, "DualExport") || !EnsureDir(dirB, "DualExport"))
            return;

        const std::vector<std::pair<int, int>> *pairPtr = nullptr;
        std::vector<std::pair<int, int>> ijPairs;
        if (gSegmentMappingState.hasMapping && !gSegmentMappingState.entries.empty()) {
            ijPairs = gSegmentMappingState.pairs();
            pairPtr = &ijPairs;
        }

        const int flatA = ExportFlattenedSegmentMeshes(sessionA, dirA, kExportGrey, pairPtr, true);
        const int flatB = ExportFlattenedSegmentMeshes(sessionB, dirB, kExportGrey, pairPtr, false);
        std::cout << "[DualExport] Flattened segment meshes (all segments, per-patch): A=" << flatA
                  << " B=" << flatB << std::endl;
    }

    // 3) Segment meshes after mapping is loaded (per-pair, per-garment), colored like current mapping viz.
    // 4) Solved 2D polygons for mapping, exported as colored triangulated meshes (one per pair, per garment).
    if (gSegmentMappingState.hasMapping && !gSegmentMappingState.entries.empty()) {
        const std::string nameA = GarmentBaseNameFromPath(pathMeshA) + "_garmentA";
        const std::string nameB = GarmentBaseNameFromPath(pathMeshB) + "_garmentB";
        const std::string segA = QDir(QString::fromStdString(segDir)).filePath(QString::fromStdString(nameA)).toStdString();
        const std::string segB = QDir(QString::fromStdString(segDir)).filePath(QString::fromStdString(nameB)).toStdString();
        const std::string polyA = QDir(QString::fromStdString(polyDir)).filePath(QString::fromStdString(nameA)).toStdString();
        const std::string polyB = QDir(QString::fromStdString(polyDir)).filePath(QString::fromStdString(nameB)).toStdString();
        const std::string bpA = QDir(QString::fromStdString(backProjDir)).filePath(QString::fromStdString(nameA + "_from_garmentB")).toStdString();
        const std::string bpB = QDir(QString::fromStdString(backProjDir)).filePath(QString::fromStdString(nameB + "_from_garmentA")).toStdString();
        const std::string bpFullA = QDir(QString::fromStdString(backProjFullDir)).filePath(QString::fromStdString(nameA + "_from_garmentB")).toStdString();
        const std::string bpFullB = QDir(QString::fromStdString(backProjFullDir)).filePath(QString::fromStdString(nameB + "_from_garmentA")).toStdString();
        const std::string bpMutualA = QDir(QString::fromStdString(backProjMutualDir)).filePath(QString::fromStdString(nameA + "_from_garmentB")).toStdString();
        const std::string bpMutualB = QDir(QString::fromStdString(backProjMutualDir)).filePath(QString::fromStdString(nameB + "_from_garmentA")).toStdString();
        if (!EnsureDir(segA, "DualExport") || !EnsureDir(segB, "DualExport") ||
            !EnsureDir(polyA, "DualExport") || !EnsureDir(polyB, "DualExport") ||
            !EnsureDir(bpA, "DualExport") || !EnsureDir(bpB, "DualExport") ||
            !EnsureDir(bpFullA, "DualExport") || !EnsureDir(bpFullB, "DualExport") ||
            !EnsureDir(bpMutualA, "DualExport") || !EnsureDir(bpMutualB, "DualExport"))
            return;

        int exportedSegA = 0, exportedSegB = 0;
        int exportedPolyA = 0, exportedPolyB = 0;
        int exportedBpA = 0, exportedBpB = 0;
        int exportedBpFullA = 0, exportedBpFullB = 0;
        int exportedMutualA = 0, exportedMutualB = 0;
        std::vector<std::unique_ptr<TraceMesh>> polyMeshesA;
        std::vector<std::unique_ptr<TraceMesh>> polyMeshesB;
        std::vector<std::string> polyStemsA;
        std::vector<std::string> polyStemsB;

        for (size_t idx = 0; idx < gSegmentMappingState.entries.size(); ++idx) {
            if (gMaxMappingPairs >= 0 && static_cast<int>(idx) >= gMaxMappingPairs)
                break;
            const SegmentMappingEntry &e = gSegmentMappingState.entries[idx];
            const vcg::Color4b c = ColorForPair(static_cast<int>(idx));
            const std::string stem = MappingColorToFileStem(c, static_cast<int>(idx));

            if (TraceMesh *mA = SessionSegmentMesh(sessionA, e.srcIndex)) {
                TraceMesh segMesh;
                vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(segMesh, *mA);
                vcg::tri::UpdateColor<TraceMesh>::PerFaceConstant(segMesh, c);
                segMesh.UpdateAttributes();
                const std::string out = QDir(QString::fromStdString(segA))
                                            .filePath(QString::fromStdString(stem + ".obj"))
                                            .toStdString();
                if (ExportMeshObjWithFaceColors(segMesh, sessionA.centerDef, out, "DualExport"))
                    ++exportedSegA;
            }

            if (TraceMesh *mB = SessionSegmentMesh(sessionB, e.dstIndex)) {
                TraceMesh segMesh;
                vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(segMesh, *mB);
                vcg::tri::UpdateColor<TraceMesh>::PerFaceConstant(segMesh, c);
                segMesh.UpdateAttributes();
                const std::string out = QDir(QString::fromStdString(segB))
                                            .filePath(QString::fromStdString(stem + ".obj"))
                                            .toStdString();
                if (ExportMeshObjWithFaceColors(segMesh, sessionB.centerDef, out, "DualExport"))
                    ++exportedSegB;
            }

            // Solved polygons in garment UV space (prefer fitted approx polygon if loaded).
            {
                std::vector<vcg::Point2<TraceMesh::ScalarType>> uvA;
                if (BuildDualExportPolygonVertsForArrangedUV(e, true, sessionA, uvA)) {
                    TraceMesh polyMesh;
                    if (BuildPolygonFanMesh(uvA, c, polyMesh)) {
                        auto m = std::make_unique<TraceMesh>();
                        vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(*m, polyMesh);
                        polyMeshesA.push_back(std::move(m));
                        polyStemsA.push_back(stem);
                    }
                }
            }
            {
                std::vector<vcg::Point2<TraceMesh::ScalarType>> uvB;
                if (BuildDualExportPolygonVertsForArrangedUV(e, false, sessionB, uvB)) {
                    TraceMesh polyMesh;
                    if (BuildPolygonFanMesh(uvB, c, polyMesh)) {
                        auto m = std::make_unique<TraceMesh>();
                        vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(*m, polyMesh);
                        polyMeshesB.push_back(std::move(m));
                        polyStemsB.push_back(stem);
                    }
                }
            }

            // 5) Back-projected segment meshes using approx polygon + overlap-only.
            {
                TraceMesh bpMeshA;
                if (BuildQuadBackProjectedMeshForExport(e, false, bpMeshA, true)) {
                    vcg::tri::UpdateColor<TraceMesh>::PerFaceConstant(bpMeshA, c);
                    const std::string out = QDir(QString::fromStdString(bpA))
                                                .filePath(QString::fromStdString(stem + ".obj"))
                                                .toStdString();
                    if (ExportMeshObjWithFaceColors(bpMeshA, sessionA.centerDef, out, "DualExport"))
                        ++exportedBpA;
                }
            }
            {
                TraceMesh bpMeshB;
                if (BuildQuadBackProjectedMeshForExport(e, true, bpMeshB, true)) {
                    vcg::tri::UpdateColor<TraceMesh>::PerFaceConstant(bpMeshB, c);
                    const std::string out = QDir(QString::fromStdString(bpB))
                                                .filePath(QString::fromStdString(stem + ".obj"))
                                                .toStdString();
                    if (ExportMeshObjWithFaceColors(bpMeshB, sessionB.centerDef, out, "DualExport"))
                        ++exportedBpB;
                }
            }

            // 8) Full approx-polygon back-projection (no overlap clipping).
            {
                TraceMesh bpMeshA;
                if (BuildQuadBackProjectedMeshForExport(e, false, bpMeshA, false)) {
                    vcg::tri::UpdateColor<TraceMesh>::PerFaceConstant(bpMeshA, c);
                    const std::string out = QDir(QString::fromStdString(bpFullA))
                                                .filePath(QString::fromStdString(stem + ".obj"))
                                                .toStdString();
                    if (ExportMeshObjWithFaceColors(bpMeshA, sessionA.centerDef, out, "DualExport"))
                        ++exportedBpFullA;
                }
            }
            {
                TraceMesh bpMeshB;
                if (BuildQuadBackProjectedMeshForExport(e, true, bpMeshB, false)) {
                    vcg::tri::UpdateColor<TraceMesh>::PerFaceConstant(bpMeshB, c);
                    const std::string out = QDir(QString::fromStdString(bpFullB))
                                                .filePath(QString::fromStdString(stem + ".obj"))
                                                .toStdString();
                    if (ExportMeshObjWithFaceColors(bpMeshB, sessionB.centerDef, out, "DualExport"))
                        ++exportedBpFullB;
                }
            }

            // 9) Full segment mutual back-projection (same as Segment viz, full source area).
            {
                TraceMesh bpMeshA;
                if (BuildSegmentBackProjectedMeshForExport(e, false, bpMeshA, false)) {
                    vcg::tri::UpdateColor<TraceMesh>::PerFaceConstant(bpMeshA, c);
                    const std::string out = QDir(QString::fromStdString(bpMutualA))
                                                .filePath(QString::fromStdString(stem + ".obj"))
                                                .toStdString();
                    if (ExportMeshObjWithFaceColors(bpMeshA, sessionA.centerDef, out, "DualExport"))
                        ++exportedMutualA;
                }
            }
            {
                TraceMesh bpMeshB;
                if (BuildSegmentBackProjectedMeshForExport(e, true, bpMeshB, false)) {
                    vcg::tri::UpdateColor<TraceMesh>::PerFaceConstant(bpMeshB, c);
                    const std::string out = QDir(QString::fromStdString(bpMutualB))
                                                .filePath(QString::fromStdString(stem + ".obj"))
                                                .toStdString();
                    if (ExportMeshObjWithFaceColors(bpMeshB, sessionB.centerDef, out, "DualExport"))
                        ++exportedMutualB;
                }
            }
        }

        // Arrange solved polygons with PatchManager (same as Draw Approx Polygon UV).
        auto exportArrangedPolyMeshes = [&](std::vector<std::unique_ptr<TraceMesh>> &meshes,
                                            const std::vector<std::string> &stems,
                                            const std::string &dir,
                                            int &countOut) {
            countOut = 0;
            if (meshes.empty() || meshes.size() != stems.size())
                return;

            std::vector<TraceMesh *> meshPtrs;
            meshPtrs.reserve(meshes.size());
            for (auto &m : meshes)
                meshPtrs.push_back(m.get());
            ArrangeTraceMeshesUVLayout(meshPtrs);
            for (auto &mPtr : meshes)
                ApplyVertexUVToPositions(*mPtr);

            for (size_t i = 0; i < meshes.size(); ++i) {
                const std::string out = QDir(QString::fromStdString(dir))
                                            .filePath(QString::fromStdString(stems[i] + ".obj"))
                                            .toStdString();
                const int mask = vcg::tri::io::Mask::IOM_WEDGTEXCOORD |
                                 vcg::tri::io::Mask::IOM_FACECOLOR;
                const int err = vcg::tri::io::ExporterOBJ<TraceMesh>::Save(*meshes[i], out.c_str(), mask);
                if (err == 0)
                    ++countOut;
            }
        };
        exportArrangedPolyMeshes(polyMeshesA, polyStemsA, polyA, exportedPolyA);
        exportArrangedPolyMeshes(polyMeshesB, polyStemsB, polyB, exportedPolyB);

        std::cout << "[DualExport] Segment back-projected meshes (approx+overlap): A=" << exportedBpA
                  << " B=" << exportedBpB << std::endl;
        std::cout << "[DualExport] Segment back-projected meshes (approx full): A=" << exportedBpFullA
                  << " B=" << exportedBpFullB << std::endl;
        std::cout << "[DualExport] Segment back-projected meshes (mutual full): A=" << exportedMutualA
                  << " B=" << exportedMutualB << std::endl;
        std::cout << "[DualExport] 2D polygon meshes arranged and exported: A=" << exportedPolyA
                  << " B=" << exportedPolyB << std::endl;

        // 6) Full post-batch garment meshes (like 01 scope), face-colored by mapping (like 03).
        {
            const auto ijPairs = gSegmentMappingState.pairs();
            const std::string nameA = GarmentBaseNameFromPath(pathMeshA);
            const std::string nameB = GarmentBaseNameFromPath(pathMeshB);
            const std::string outA = QDir(QString::fromStdString(garmentSegColorDir))
                                         .filePath(QString::fromStdString(nameA + "_garment.obj"))
                                         .toStdString();
            const std::string outB = QDir(QString::fromStdString(garmentSegColorDir))
                                         .filePath(QString::fromStdString(nameB + "_garment.obj"))
                                         .toStdString();

            TraceMesh garmentA;
            TraceMesh garmentB;
            vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(garmentA, sessionA.deformed_mesh);
            vcg::tri::Append<TraceMesh, TraceMesh>::Mesh(garmentB, sessionB.deformed_mesh);
            ApplySegmentMappingColorsToMesh(sessionA, garmentA, ijPairs, true);
            ApplySegmentMappingColorsToMesh(sessionB, garmentB, ijPairs, false);
            garmentA.UpdateAttributes();
            garmentB.UpdateAttributes();
            const bool okA = ExportMeshObjWithFaceColors(garmentA, sessionA.centerDef, outA, "DualExport");
            const bool okB = ExportMeshObjWithFaceColors(garmentB, sessionB.centerDef, outB, "DualExport");
            std::cout << "[DualExport] Complete garments with segment-mapping colors: A="
                      << (okA ? "ok" : "fail") << " B=" << (okB ? "ok" : "fail") << std::endl;
        }
    } else {
        std::cout << "[DualExport] Mapping not loaded; skipped mapping-dependent exports (03–09)."
                  << std::endl;
    }

    QMessageBox::information(nullptr,
                             "Dual Export",
                             QString("Exported artifacts under:\n%1")
                                 .arg(QString::fromStdString(exportRoot)));
}

void TW_CALL GetDrawSegmentMappingCB(void *value, void *)
{
    *static_cast<int*>(value) = gSegmentMappingState.drawMapping ? 1 : 0;
}

void TW_CALL SetDrawSegmentMappingCB(const void *value, void *clientData)
{
    gSegmentMappingState.drawMapping = (*static_cast<const int*>(value) != 0);
    ApplyDualGarmentColoringBoth();
    if (QWidget *w = static_cast<QWidget*>(clientData))
        w->update();
}

void TW_CALL GetGreyGarmentColorCB(void *value, void *)
{
    *static_cast<int*>(value) = gGarmentColorGreyMode ? 1 : 0;
}

void TW_CALL SetGreyGarmentColorCB(const void *value, void *clientData)
{
    gGarmentColorGreyMode = (*static_cast<const int*>(value) != 0);
    ApplyDualGarmentColoringBoth();
    if (QWidget *w = static_cast<QWidget*>(clientData))
        w->update();
}

void TW_CALL GetInputGarmentSeamVizCB(void *value, void *)
{
    *static_cast<int*>(value) = gInputGarmentSeamViz ? 1 : 0;
}

void TW_CALL SetInputGarmentSeamVizCB(const void *value, void *clientData)
{
    gInputGarmentSeamViz = (*static_cast<const int*>(value) != 0);
    if (QWidget *w = static_cast<QWidget*>(clientData))
        w->update();
}

} // namespace

ReconfigGarmentWidget::ReconfigGarmentWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
    if (!gPendingExperimentJsonPath.empty()) {
        const std::string expPath = gPendingExperimentJsonPath;
        gPendingExperimentJsonPath.clear();
        if (!loadExperiment(expPath))
            std::exit(1);
        return;
    }

    if (!InitializeDualGarmentSessions())
        std::exit(1);
}

bool ReconfigGarmentWidget::loadExperiment(const std::string &experimentPath)
{
    const std::string folderPath = ResolveExperimentFolderPath(experimentPath);
    if (folderPath.empty()) {
        std::cout << "Experiment not found: " << experimentPath << std::endl;
        return false;
    }

    ExperimentSettings exp;
    if (!LoadExperimentSettings(folderPath, exp))
        return false;

    std::string meshA;
    std::string meshB;
    if (!ApplyExperimentSettings(sessionA, sessionB, exp, meshA, meshB))
        return false;

    pathMeshA = meshA;
    pathMeshB = meshB;
    SetActiveExperimentDirectory(folderPath);
    gActiveSession = &sessionA;
    SyncConstraintPathsToGLPaths();
    ApplyDualGarmentColoringBoth();

    gBorderSeamPickStage = 0;
    gPendingBorderSeamPick = false;
    InvalidateBackProjectionCache();
    updateGL();
    return true;
}

void ReconfigGarmentWidget::initializeGL()
{
    glewInit();

    glClearColor(1.f, 1.f, 1.f, 1.f);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    // Draw both sides of the mesh.
    glDisable(GL_CULL_FACE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    glEnable(GL_DEPTH_TEST);
}

void ReconfigGarmentWidget::keyPressEvent(QKeyEvent *event)
{
    // Let AntTweakBar handle key presses (needed for editing numeric fields).
    event->ignore();
    if (event->key() == Qt::Key_Space)
        gSpacebarPressed = true;
    if (event->key() == Qt::Key_Escape && gBorderSeamPickStage != 0) {
        gBorderSeamPickStage = 0;
        gPendingBorderSeamPick = false;
        std::cout << "Border seam pick cancelled." << std::endl;
        updateGL();
        return;
    }
    if (event->key() == Qt::Key_Escape && gManualSegmentMode) {
        ClearManualSegmentPickInProgress(activeGarmentSession());
        gPendingManualSegmentPick = false;
        std::cout << "Manual segment pick cleared." << std::endl;
        updateGL();
        return;
    }
    TwKeyPressQt(event);
    updateGL();
}

void ReconfigGarmentWidget::keyReleaseEvent(QKeyEvent *event)
{
    // Do not forward key releases to AntTweakBar; it expects keyPress only.
    event->ignore();
    if (event->key() == Qt::Key_Space)
        gSpacebarPressed = false;
}

void ReconfigGarmentWidget::resizeGL(int w, int h)
{
    Q_UNUSED(w);
    Q_UNUSED(h);
    const int devW = std::max(1, QTDeviceWidth(this));
    const int devH = std::max(1, QTDeviceHeight(this));
    glViewport(0, 0, (GLsizei)devW, (GLsizei)devH);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40.0, devH > 0 ? (float)devW / (float)devH : 1.0f, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);

    // Keep AntTweakBar in sync with the window size and ensure its UI is set up.
    TwWindowSize(devW, devH);
    extern void InitBar(QWidget *w);
    InitBar(this);

    // Add a control to choose which garment is active (only once).
    if (!activeGarmentControlAdded && barFashion != nullptr)
    {
        TwEnumVal garmentEV[2] = { {0, "Left garment"}, {1, "Right garment"} };
        TwType garmentType = TwDefineEnum("GarmentChoice", garmentEV, 2);
        TwAddVarCB(barFashion,
                   "Active Garment",
                   garmentType,
                   SetActiveGarmentCB,
                   GetActiveGarmentCB,
                   nullptr,
                   " label='Active Garment' help='Choose which garment to process.' ");
        activeGarmentControlAdded = true;
    }

    // Dual-only: Run both pipelines, export both, run segment mapping, load mapping.
    if (!dualButtonsAdded && barFashion != nullptr)
    {
        TwAddSeparator(barFashion, nullptr, nullptr);
        TwAddButton(barFashion, "RunActivePipeline", RunActivePipelineCB, this,
                   " label='Run Pipeline On Active' help='Run parafashion on the active garment only (uses its Space+drag paths). Manual border seams on this garment are kept.' ");
        TwAddButton(barFashion, "RunBothPipelines", RunBothPipelinesCB, this,
                   " label='Run Pipeline On Both' help='Run parafashion on both garments (each uses its own paths). Does not affect the garment you are not processing when using Space+drag on one view.' ");
        TwAddButton(barFashion, "ExportBothPolygons", ExportBothPolygonsCB, nullptr, " label='Export Both Polygons' help='Export segment polygons for both garments (_polygons.txt).' ");
        TwAddButton(barFashion, "RunSegmentMapping", RunSegmentMappingCB, nullptr, " label='Run Segment Mapping' help='Call Python solver to compute segment mapping. Run from project root.' ");
        TwAddButton(barFashion, "LoadMappingAndColor", LoadMappingAndColorCB, this, " label='Load Mapping & Color' help='Load mapping result and color matched segments the same.' ");
        TwAddButton(barFashion,
                    "ExportDualMeshesAndMapping",
                    ExportDualMeshesAndMappingCB,
                    nullptr,
                    " label='Export Meshes (Dual)' "
                    "help='Export dual_export/: originals, patches, mapping exports (03–06), flattened segment meshes (07), etc.' ");
        TwAddVarRW(barFashion,
                   "Real Garment Area A",
                   TW_TYPE_CSSTRING(kRealGarmentAreaStringSize),
                   gRealGarmentAreaA,
                   " label='Real area A (sq units)' "
                   "help='Physical total surface area of garment A (e.g. m^2). Used to rescale exported polygons.' ");
        TwAddVarRW(barFashion,
                   "Real Garment Area B",
                   TW_TYPE_CSSTRING(kRealGarmentAreaStringSize),
                   gRealGarmentAreaB,
                   " label='Real area B (sq units)' "
                   "help='Physical total surface area of garment B (e.g. m^2). Used to rescale exported polygons.' ");
        TwAddButton(barFashion,
                    "ExportFabricationApprox",
                    ExportFabricationApproxPolygonsCB,
                    nullptr,
                    " label='Export Fabrication Approx Polygons' "
                    "help='Write one polygon per mapping pair: fitted quads under exp/.../fabrication_approx_polygons/, "
                    "and seam-endpoint segment approx polygons under exp/.../segment_approx_polygons/, "
                    "named by mapping color, scaled to real garment size.' ");
        TwAddVarCB(barFashion, "Draw Segment Mapping", TW_TYPE_BOOL32, SetDrawSegmentMappingCB, GetDrawSegmentMappingCB, this, " label='Draw segment mapping colors' ");
        TwAddVarCB(barFashion,
                   "Grey Garment Color",
                   TW_TYPE_BOOL32,
                   SetGreyGarmentColorCB,
                   GetGreyGarmentColorCB,
                   this,
                   " label='Grey garment (pre-mapping)' "
                   "help='When on, garments stay neutral grey after batch until mapping is loaded. Turn off for per-patch colors.' ");
        TwAddVarCB(barFashion,
                   "Input Garment Seams",
                   TW_TYPE_BOOL32,
                   SetInputGarmentSeamVizCB,
                   GetInputGarmentSeamVizCB,
                   this,
                   " label='Input garment seams only' "
                   "help='Show only mesh border edges, as when garments are first loaded. Hides batch-traced internal seams and back-projection overlays.' ");
        gInputGarmentSeamVizControlAdded = true;
        TwAddVarRW(barFashion, "Max Mapping Pairs", TW_TYPE_INT32, &gMaxMappingPairs,
                   " label='Top K mapping pairs (-1 = all)' ");
        gGreyGarmentColorToggleAdded = true;
        TwEnumVal bpVizEV[3] = {
            {BP_VizNone, "Off"},
            {BP_VizSegment, "Segment (mutual)"},
            {BP_VizQuad, "Approx polygon (N verts)"}
        };
        TwType bpVizType = TwDefineEnum("BackProjectionVizMode", bpVizEV, 3);
        TwAddVarRW(barFashion,
                   "Back-Projection Viz",
                   bpVizType,
                   &gBackProjectionVizMode,
                   " label='Back-projection mode' "
                   "help='Segment: full segment UV mutual back-projection. Quad: triangulated N-vertex approx overlay.' ");
        TwEnumVal bpAreaEV[2] = {
            {BP_SourceAreaFull, "Full source mesh"},
            {BP_SourceAreaOverlapOnly, "Overlap-only source"}
        };
        TwType bpAreaType = TwDefineEnum("BackProjectionSourceAreaMode", bpAreaEV, 2);
        TwAddVarRW(barFashion,
                   "Back-Projection Area",
                   bpAreaType,
                   &gBackProjectionSourceAreaMode,
                   " label='Back-projection area' "
                   "help='Overlap-only hides source vertices/faces whose transformed UVs lie outside the matched target mesh.' ");
        TwAddVarRW(barFashion,
                   "Quad Subdivisions",
                   TW_TYPE_INT32,
                   &gQuadSubdivisions,
                   " label='Quad mesh subdivisions' min=1 max=128 step=1 "
                   "help='Grid resolution per quad edge for quad back-projection (higher = smoother).' ");
        dualButtonsAdded = true;
    }

    if (!gExperimentControlsAdded && barFashion != nullptr)
    {
        TwAddSeparator(barFashion, nullptr, nullptr);
        EnsureSavedExperimentListControl();
        TwAddButton(barFashion,
                    "ExportExperiment",
                    ExportExperimentCB,
                    nullptr,
                    " label='Export Experiment' "
                    "help='Save to <project>/exp/<garmentA>_<garmentB> (appends _1, _2 if needed).' ");
        TwAddButton(barFashion,
                    "LoadSavedExperiment",
                    LoadSavedExperimentCB,
                    this,
                    " label='Load Experiment' "
                    "help='Load selected experiment from <project>/exp/ and run batch + manual edits.' ");
        gExperimentControlsAdded = true;
    }

    if (!gBorderSeamControlsAdded && barFashion != nullptr)
    {
        TwAddSeparator(barFashion, nullptr, nullptr);
        TwAddVarRW(barFashion,
                   "Border Seam Mode",
                   TW_TYPE_BOOL32,
                   &gBorderSeamMode,
                   " label='Add border seam (2 clicks)' "
                   "help='After batch: left-click two border points (mesh border or existing seams) in the active view to add a geodesic cut. Esc cancels.' ");
        TwAddVarRW(barFashion,
                   "Symmetric Border Seam",
                   TW_TYPE_BOOL32,
                   &gBorderSeamSymmetricMode,
                   " label='Mirror seam to other side' "
                   "help='When enabled, adding a border seam will also add the mirrored seam across the symmetry plane (x=0).' ");
        TwAddButton(barFashion,
                    "ClearManualBorderSeamOverlays",
                    ClearManualBorderSeamOverlaysCB,
                    nullptr,
                    " label='Clear seam overlays' help='Clears manual seam overlay polylines only (does not remove seams from mesh).' ");
        gBorderSeamControlsAdded = true;
    }

    if (!gManualSegmentControlsAdded && barFashion != nullptr)
    {
        TwAddSeparator(barFashion, nullptr, nullptr);
        TwAddVarRW(barFashion,
                   "Manual Segment Mode",
                   TW_TYPE_BOOL32,
                   &gManualSegmentMode,
                   " label='Pick patches for segment' "
                   "help='After batch: left-click patches in either 3D view to toggle selection (left=A, right=B). Commit adds a segment candidate and rebuilds segments.' ");
        TwAddButton(barFashion,
                    "CommitManualSegment",
                    CommitManualSegmentCandidateCB,
                    this,
                    " label='Commit manual segment' "
                    "help='Add the selected patches as a manual segment candidate and rebuild BuildSegments.' ");
        TwAddButton(barFashion,
                    "ClearManualSegmentPick",
                    ClearManualSegmentPickCB,
                    this,
                    " label='Clear patch selection' help='Clear in-progress patch picks (Esc).' ");
        TwAddButton(barFashion,
                    "ClearManualSegmentCandidates",
                    ClearManualSegmentCandidatesCB,
                    this,
                    " label='Clear manual segments' help='Remove all manual segment candidates and rebuild auto segments.' ");
        TwAddButton(barFashion,
                    "RebuildSegments",
                    RebuildSegmentsCB,
                    this,
                    " label='Rebuild segments' help='Re-run BuildSegments with current manual candidates.' ");
        gManualSegmentControlsAdded = true;
    }

    // Center the AntTweakBar roughly in the window on resize.
    if (barFashion != nullptr)
    {
        int params[2];
        int devW = QTDeviceWidth(this);
        int barW = devW / 5; // same width as set in InitBar
        params[0] = (devW - barW) / 2;  // center horizontally
        params[1] = 0;                  // align top edge with window top
        TwSetParam(barFashion, nullptr, "position", TW_PARAM_INT32, 2, params);
    }
}

void ReconfigGarmentWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    const int w = std::max(1, QTDeviceWidth(this));
    const int h = std::max(1, QTDeviceHeight(this));
    const int halfW = std::max(1, w / 2);

    glEnable(GL_POLYGON_SMOOTH);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

    // --- Left viewport: session A ---
    glViewport(0, 0, halfW, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40.0, halfW > 0 ? (float)halfW / (float)h : 1.0f, 0.1, 100.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0, 0, 3.5f, 0, 0, 0, 0, 1, 0);

    trackLeft.center = vcg::Point3f(0, 0, 0);
    trackLeft.radius = 1.0f;
    trackLeft.GetView();

    glPushMatrix();
    trackLeft.Apply();
    {
        glPushMatrix();
        // If UV view is requested, draw only the UV mesh for this garment.
        if (drawParam && parametrized)
        {
            glScalef(2.0f, 2.0f, 2.0f);
            glTranslatef(-0.5f, -0.5f, 0.0f);
            if (drawArrangedApproxPolygonUV) {
                DrawArrangedApproxPolygonsForDualGarment(true);
            } else {
                MeshDrawing<TraceMesh>::GLDrawUV(sessionA.deformed_mesh, textured, colored_distortion);
                MeshDrawing<TraceMesh>::GLDrawEdgeUV(sessionA.deformed_mesh);
            }
            if (gPendingBorderSeamPick && gPendingBorderSeamViewport == 0) {
                gPendingBorderSeamPick = false;
                std::cout << "Border seam pick requires 3D view (turn off UV/param display)." << std::endl;
            }
            if (gPendingManualSegmentPick && gPendingManualSegmentViewport == 0) {
                gPendingManualSegmentPick = false;
                std::cout << "Manual segment pick requires 3D view (turn off UV/param display)." << std::endl;
            }
        }
        else
        {
            glDisable(GL_BLEND);
            glDisable(GL_CULL_FACE);
            float diagA = sessionA.reference_mesh.bbox.Diag();
            vcg::Point3f refCenterA = sessionA.reference_mesh.bbox.Center();
            if (diagA > 1e-6f)
                glScalef(2.0f / diagA, 2.0f / diagA, 2.0f / diagA);
            glTranslatef(-refCenterA.X(), -refCenterA.Y(), -refCenterA.Z());
            glWrap.m = &sessionA.deformed_mesh;
            if (draw3D){
                glWrap.Draw(drawmode, vcg::GLW::CMPerFace, vcg::GLW::TMNone);
            }
            // Depth-based pick right after mesh draw (matches MyGLWidget path picking).
            ProcessPendingBorderSeamPick(this, 0, sessionA);
            ProcessPendingManualSegmentPick(0, sessionA);

            if (drawConstraints && !SessionHasMeshSeams(sessionA) && !gInputGarmentSeamViz)
                MeshDrawing<TraceMesh>::GLDrawSharpEdges(
                    sessionA.deformed_mesh, kSeamEdgeColor, kSeamEdgeLineWidth);
            GLDrawGarmentSeamEdges(sessionA, true);
            GLDrawConstraintPaths(gPathA, sessionA);

            if (!gInputGarmentSeamViz) {
                // Draw back-projected UV meshes from garment B onto garment A.
                DrawBackProjectedSegmentsBontoA();
                DrawQuadBackProjectedBontoA();
            }
        }
        if (gUserIsPicking && gPickingViewport == 0)
            gPathA.GLAddPoint(vcg::Point2i(gPickX, gPickY));
        GLDrawManualBorderSeams(sessionA);
        glPopMatrix();
    }
    glPopMatrix();

    // --- Right viewport: session B ---
    glViewport(halfW, 0, w - halfW, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40.0, (w - halfW) > 0 ? (float)(w - halfW) / (float)h : 1.0f, 0.1, 100.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0, 0, 3.5f, 0, 0, 0, 0, 1, 0);

    trackRight.center = vcg::Point3f(0, 0, 0);
    trackRight.radius = 1.0f;
    trackRight.GetView();

    glPushMatrix();
    trackRight.Apply();
    {
        glPushMatrix();
        if (drawParam && parametrized)
        {
            glScalef(2.0f, 2.0f, 2.0f);
            glTranslatef(-0.5f, -0.5f, 0.0f);
            if (drawArrangedApproxPolygonUV) {
                DrawArrangedApproxPolygonsForDualGarment(false);
            } else {
                MeshDrawing<TraceMesh>::GLDrawUV(sessionB.deformed_mesh, textured, colored_distortion);
                MeshDrawing<TraceMesh>::GLDrawEdgeUV(sessionB.deformed_mesh);
            }
            if (gPendingBorderSeamPick && gPendingBorderSeamViewport == 1) {
                gPendingBorderSeamPick = false;
                std::cout << "Border seam pick requires 3D view (turn off UV/param display)." << std::endl;
            }
            if (gPendingManualSegmentPick && gPendingManualSegmentViewport == 1) {
                gPendingManualSegmentPick = false;
                std::cout << "Manual segment pick requires 3D view (turn off UV/param display)." << std::endl;
            }
        }
        else
        {
            glDisable(GL_BLEND);
            glDisable(GL_CULL_FACE);
            float diagB = sessionB.reference_mesh.bbox.Diag();
            vcg::Point3f refCenterB = sessionB.reference_mesh.bbox.Center();
            if (diagB > 1e-6f)
                glScalef(2.0f / diagB, 2.0f / diagB, 2.0f / diagB);
            glTranslatef(-refCenterB.X(), -refCenterB.Y(), -refCenterB.Z());
            glWrap.m = &sessionB.deformed_mesh;
            if (draw3D){
                glWrap.Draw(drawmode, vcg::GLW::CMPerFace, vcg::GLW::TMNone);
            }
            ProcessPendingBorderSeamPick(this, 1, sessionB);
            ProcessPendingManualSegmentPick(1, sessionB);

            if (drawConstraints && !SessionHasMeshSeams(sessionB) && !gInputGarmentSeamViz)
                MeshDrawing<TraceMesh>::GLDrawSharpEdges(
                    sessionB.deformed_mesh, kSeamEdgeColor, kSeamEdgeLineWidth);
            GLDrawGarmentSeamEdges(sessionB, false);
            GLDrawConstraintPaths(gPathB, sessionB);

            if (!gInputGarmentSeamViz) {
                // Draw back-projected UV meshes from garment A onto garment B.
                DrawBackProjectedSegmentsAontoB();
                DrawQuadBackProjectedAontoB();
            }
        }
        if (gUserIsPicking && gPickingViewport == 1)
            gPathB.GLAddPoint(vcg::Point2i(gPickX, gPickY));
        GLDrawManualBorderSeams(sessionB);
        glPopMatrix();
    }
    glPopMatrix();

    // --- Separator line between the two viewports (in screen space) ---
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, w, 0.0, h, -1.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Temporarily adjust depth for separator without touching lighting state.
    glDisable(GL_DEPTH_TEST);

    glColor3f(0.f, 0.f, 0.f);
    glLineWidth(2.f);

    glBegin(GL_LINES);
    glVertex2f(halfW + 0.5f, 0.0f);
    glVertex2f(halfW + 0.5f, (float)h);
    glEnd();

    // Re-enable depth test for subsequent frames.
    glEnable(GL_DEPTH_TEST);

    if (gHasDoubleClick)
    {
        int viewportIndex = gDoubleClickViewport;
        if (viewportIndex == 0 || viewportIndex == 1)
        {
            PathGL<TraceMesh> &path = activePathForViewport(viewportIndex);
            GarmentSession &session = activeSessionForViewport(viewportIndex);
            bool hasRemoved = path.GLRemovePathFromPoint(
                vcg::Point2i(gPickX, gPickY),
                session.half_def_mesh.bbox.Diag() / 10);
            if (hasRemoved)
                RunBatchForViewport(viewportIndex);
        }
        gHasDoubleClick = false;
        gDoubleClickViewport = -1;
    }

    // Draw AntTweakBar UI on top.
    TwDraw();
}

void ReconfigGarmentWidget::mousePressEvent(QMouseEvent *event)
{
    if (!TwMousePressQt(this, event))
    {
        event->accept();
        setFocus();
        if (gManualSegmentMode && event->button() == Qt::LeftButton && !gSpacebarPressed)
        {
            gPendingManualSegmentPick = true;
            gPendingManualSegmentViewport = viewportIndexFromX(this, event->x());
            gPendingManualSegmentPixelX = QT2VCG_X(this, event);
            gPendingManualSegmentPixelY = QT2VCG_Y(this, event);
            updateGL();
            return;
        }
        if (gBorderSeamMode && event->button() == Qt::LeftButton && !gSpacebarPressed)
        {
            gPendingBorderSeamPick = true;
            gPendingBorderSeamViewport = viewportIndexFromX(this, event->x());
            gPendingBorderSeamPixelX = QT2VCG_X(this, event);
            gPendingBorderSeamPixelY = QT2VCG_Y(this, event);
            updateGL();
            return;
        }
        if (gSpacebarPressed)
        {
            gUserIsPicking = true;
            gPickingViewport = viewportIndexFromX(this, event->x());
            gPickX = QT2VCG_X(this, event);
            gPickY = QT2VCG_Y(this, event);
            activePathForViewport(gPickingViewport).AddNewPath();
            updateGL();
            return;
        }
        vcg::Trackball &tb = activeTrackball(this, event->x());
        tb.MouseDown(QT2VCG_X(this, event),
                     QT2VCG_Y(this, event),
                     QT2VCG(event->button(), event->modifiers()));
        updateGL();
    }
}

void ReconfigGarmentWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (event->buttons()) {
        if (gUserIsPicking)
        {
            gPickX = QT2VCG_X(this, event);
            gPickY = QT2VCG_Y(this, event);
        }
        else
        {
            vcg::Trackball &tb = activeTrackball(this, event->x());
            tb.MouseMove(QT2VCG_X(this, event), QT2VCG_Y(this, event));
        }
        updateGL();
    }
    TwMouseMotion(QTLogicalToDevice(this, event->x()),
                  QTLogicalToDevice(this, event->y()));
}

void ReconfigGarmentWidget::mouseReleaseEvent(QMouseEvent *event)
{
    TwMouseReleaseQt(this, event);
    if (gUserIsPicking)
    {
        gUserIsPicking = false;
        int viewportIndex = gPickingViewport;
        gPickingViewport = -1;
        if (viewportIndex == 0 || viewportIndex == 1)
        {
            PathGL<TraceMesh> &path = activePathForViewport(viewportIndex);
            if (!path.PickedPoints.empty() && path.PickedPoints.back().empty())
                path.PickedPoints.pop_back();
            else
                RunBatchForViewport(viewportIndex);
        }
    }
    else
    {
        vcg::Trackball &tb = activeTrackball(this, event->x());
        tb.MouseUp(QT2VCG_X(this, event),
                   QT2VCG_Y(this, event),
                   QT2VCG(event->button(), event->modifiers()));
    }
    updateGL();
}

void ReconfigGarmentWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
    if (event->buttons())
    {
        gPickX = QT2VCG_X(this, event);
        gPickY = QT2VCG_Y(this, event);
        if (event->button() == Qt::LeftButton)
        {
            gHasDoubleClick = true;
            gDoubleClickViewport = viewportIndexFromX(this, event->x());
        }
        updateGL();
    }
}

void ReconfigGarmentWidget::wheelEvent(QWheelEvent *event)
{
    const int WHEEL_STEP = 120;
    QPoint p = event->pos();
    vcg::Trackball &tb = activeTrackball(this, p.x());
    tb.MouseWheel(event->delta() / float(WHEEL_STEP),
                  QTWheel2VCG(event->modifiers()));
    updateGL();
}

