#include "manual_border_seam.h"

#include <vcg/complex/algorithms/geodesic.h>
#include <vcg/complex/algorithms/update/flag.h>
#include <vcg/complex/algorithms/update/selection.h>
#include <vcg/space/segment3.h>
#include <vcg/space/distance3.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <set>

using MeshType = GarmentSession::MeshType;
using CoordType = GarmentSession::CoordType;
using ScalarType = GarmentSession::ScalarType;
using VertexType = MeshType::VertexType;
using FaceType = MeshType::FaceType;

static bool IsPickableBorderEdge(const MeshType &mesh, size_t faceIndex, size_t edgeIndex)
{
    const auto &f = mesh.face[faceIndex];
    if (f.IsD()) return false;
    return vcg::face::IsBorder(f, edgeIndex) || f.IsFaceEdgeS(edgeIndex);
}

BorderVertexPick SnapPickToBorderVertex(
    MeshType &mesh,
    const CoordType &pickPos,
    ScalarType maxSnapDistance)
{
    BorderVertexPick result;
    if (mesh.face.empty() || mesh.vert.empty())
        return result;

    ScalarType minDist = std::numeric_limits<ScalarType>::max();
    size_t bestFace = 0;
    size_t bestEdge = 0;
    CoordType bestOnEdge = pickPos;

    for (size_t fi = 0; fi < mesh.face.size(); ++fi) {
        for (size_t ej = 0; ej < 3; ++ej) {
            if (!IsPickableBorderEdge(mesh, fi, ej))
                continue;

            const CoordType p0 = mesh.face[fi].P0(ej);
            const CoordType p1 = mesh.face[fi].P1(ej);
            vcg::Segment3<ScalarType> seg(p0, p1);
            CoordType closest;
            ScalarType dist = 0;
            vcg::SegmentPointDistance(seg, pickPos, closest, dist);
            if (dist >= minDist)
                continue;

            minDist = dist;
            bestFace = fi;
            bestEdge = ej;
            bestOnEdge = closest;
        }
    }

    if (minDist > maxSnapDistance)
        return result;

    const VertexType *v0 = mesh.face[bestFace].V0(bestEdge);
    const VertexType *v1 = mesh.face[bestFace].V1(bestEdge);
    const size_t indexV0 = vcg::tri::Index(mesh, v0);
    const size_t indexV1 = vcg::tri::Index(mesh, v1);
    const ScalarType d0 = (mesh.vert[indexV0].P() - bestOnEdge).Norm();
    const ScalarType d1 = (mesh.vert[indexV1].P() - bestOnEdge).Norm();

    result.vertexIndex = (d0 <= d1) ? indexV0 : indexV1;
    result.position = mesh.vert[result.vertexIndex].P();
    result.ok = true;
    return result;
}

bool ComputeGeodesicVertexPath(
    MeshType &mesh,
    size_t vertexIndex0,
    size_t vertexIndex1,
    std::vector<size_t> &outVertexPath)
{
    outVertexPath.clear();
    if (vertexIndex0 >= mesh.vert.size() || vertexIndex1 >= mesh.vert.size())
        return false;
    if (vertexIndex0 == vertexIndex1) {
        outVertexPath.push_back(vertexIndex0);
        return true;
    }

    VertexType *v0 = &mesh.vert[vertexIndex0];
    VertexType *v1 = &mesh.vert[vertexIndex1];

    MeshType::template PerVertexAttributeHandle<VertexType *> father;
    father = vcg::tri::Allocator<MeshType>::template GetPerVertexAttribute<VertexType *>(mesh, "father");

    vcg::tri::UnMarkAll(mesh);
    std::vector<VertexType *> seedVec;
    seedVec.push_back(v0);
    vcg::tri::Mark(mesh, v0);

    vcg::tri::EuclideanDistance<MeshType> edgeDist;
    const ScalarType maxDistanceThr = mesh.bbox.Diag() * ScalarType(4);

    vcg::tri::Geodesic<MeshType>::PerVertexDijkstraCompute(
        mesh, seedVec, edgeDist, maxDistanceThr,
        nullptr, nullptr, &father, false, v1);

    if (father[v1] == nullptr) {
        std::cout << "Geodesic path failed between vertices "
                  << vertexIndex0 << " and " << vertexIndex1 << std::endl;
        return false;
    }

    VertexType *currV = v1;
    do {
        outVertexPath.push_back(vcg::tri::Index(mesh, currV));
        currV = father[currV];
    } while (currV != v0);

    outVertexPath.push_back(vertexIndex0);
    std::reverse(outVertexPath.begin(), outVertexPath.end());

    vcg::tri::UnMarkAll(mesh);
    return outVertexPath.size() >= 2;
}

static bool GeodesicPathBetweenVertices(
    MeshType &mesh,
    VertexType *v0,
    VertexType *v1,
    std::vector<size_t> &path)
{
    path.clear();
    if (v0 == nullptr || v1 == nullptr || v0 == v1)
        return false;

    MeshType::template PerVertexAttributeHandle<VertexType *> father;
    father = vcg::tri::Allocator<MeshType>::template GetPerVertexAttribute<VertexType *>(
        mesh, "father");

    vcg::tri::UnMarkAll(mesh);
    std::vector<VertexType *> seedVec;
    seedVec.push_back(v0);
    vcg::tri::Mark(mesh, v0);

    vcg::tri::EuclideanDistance<MeshType> edgeDist;
    const ScalarType maxDistanceThr = mesh.bbox.Diag() * ScalarType(4);
    vcg::tri::Geodesic<MeshType>::PerVertexDijkstraCompute(
        mesh, seedVec, edgeDist, maxDistanceThr,
        nullptr, nullptr, &father, false, v1);

    if (father[v1] == nullptr) {
        vcg::tri::UnMarkAll(mesh);
        return false;
    }

    VertexType *currV = v1;
    do {
        path.push_back(vcg::tri::Index(mesh, currV));
        currV = father[currV];
    } while (currV != v0);

    path.push_back(vcg::tri::Index(mesh, v0));
    std::reverse(path.begin(), path.end());
    vcg::tri::UnMarkAll(mesh);
    return path.size() >= 2;
}

void CompleteVertexPath(MeshType &mesh, std::vector<size_t> &vertexPath)
{
    if (vertexPath.size() < 2)
        return;

    std::set<std::pair<size_t, size_t>> edgeMap;
    for (size_t fi = 0; fi < mesh.face.size(); ++fi) {
        for (size_t ej = 0; ej < 3; ++ej) {
            const size_t indexV0 = vcg::tri::Index(mesh, mesh.face[fi].V0(ej));
            const size_t indexV1 = vcg::tri::Index(mesh, mesh.face[fi].V1(ej));
            edgeMap.insert({std::min(indexV0, indexV1), std::max(indexV0, indexV1)});
        }
    }

    std::vector<size_t> newSeq;
    for (size_t j = 0; j + 1 < vertexPath.size(); ++j) {
        newSeq.push_back(vertexPath[j]);
        const size_t indexV0 = vertexPath[j];
        const size_t indexV1 = vertexPath[j + 1];
        const std::pair<size_t, size_t> key(
            std::min(indexV0, indexV1), std::max(indexV0, indexV1));

        if (edgeMap.count(key) > 0)
            continue;

        std::vector<size_t> bridge;
        if (!GeodesicPathBetweenVertices(
                mesh, &mesh.vert[indexV0], &mesh.vert[indexV1], bridge))
            continue;

        for (size_t k = 1; k + 1 < bridge.size(); ++k)
            newSeq.push_back(bridge[k]);
    }
    newSeq.push_back(vertexPath.back());
    vertexPath = std::move(newSeq);
}

static void SmoothVertOnPath(
    MeshType &mesh,
    const std::set<std::pair<VertexType *, VertexType *>> &vertexPathSet,
    const std::vector<bool> &onPath,
    bool onlyLine,
    ScalarType damp)
{
    std::vector<CoordType> newPos(mesh.vert.size(), CoordType(0, 0, 0));
    std::vector<size_t> sum(mesh.vert.size(), 0);

    for (size_t fi = 0; fi < mesh.face.size(); ++fi) {
        for (size_t ej = 0; ej < 3; ++ej) {
            VertexType *v0 = mesh.face[fi].V0(ej);
            VertexType *v1 = mesh.face[fi].V1(ej);
            const size_t indexV0 = vcg::tri::Index(mesh, v0);
            const size_t indexV1 = vcg::tri::Index(mesh, v1);
            const std::pair<VertexType *, VertexType *> key(std::min(v0, v1), std::max(v0, v1));
            if (vertexPathSet.count(key) == 0 && onlyLine)
                continue;

            newPos[indexV0] += v1->P();
            newPos[indexV1] += v0->P();
            sum[indexV0]++;
            sum[indexV1]++;
        }
    }

    for (size_t i = 0; i < mesh.vert.size(); ++i) {
        if (sum[i] == 0)
            continue;
        newPos[i] /= sum[i];
        if (onlyLine && !onPath[i])
            continue;
        if (onlyLine && sum[i] == 2)
            continue;
        if (!onlyLine && onPath[i])
            continue;
        if (mesh.vert[i].IsB())
            continue;
        if (!mesh.vert[i].IsS())
            continue;

        mesh.vert[i].P() = mesh.vert[i].P() * damp + newPos[i] * (ScalarType(1) - damp);
    }
}

static void ReprojectPathVertices(MeshType &mesh, vcg::GridStaticPtr<FaceType, ScalarType> &gridF)
{
    for (size_t i = 0; i < mesh.vert.size(); ++i) {
        if (!mesh.vert[i].IsS())
            continue;
        if (mesh.vert[i].IsB())
            continue;

        ScalarType minD = 0;
        CoordType closestPt;
        const ScalarType maxD = mesh.bbox.Diag();
        vcg::tri::GetClosestFaceBase(mesh, gridF, mesh.vert[i].P(), maxD, minD, closestPt);
        mesh.vert[i].P() = closestPt;
    }
}

void RefineAndSmoothVertexPath(MeshType &mesh, std::vector<size_t> &vertexPath, size_t smoothSteps)
{
    if (vertexPath.size() < 2)
        return;

    CompleteVertexPath(mesh, vertexPath);

    std::set<std::pair<VertexType *, VertexType *>> vertexPathSet;
    vcg::tri::UpdateSelection<MeshType>::Clear(mesh);
    std::vector<bool> onPath(mesh.vert.size(), false);

    for (size_t j = 0; j + 1 < vertexPath.size(); ++j) {
        VertexType *v0 = &mesh.vert[vertexPath[j]];
        VertexType *v1 = &mesh.vert[vertexPath[j + 1]];
        vertexPathSet.insert({std::min(v0, v1), std::max(v0, v1)});
        const size_t indexV0 = vcg::tri::Index(mesh, v0);
        const size_t indexV1 = vcg::tri::Index(mesh, v1);
        onPath[indexV0] = true;
        onPath[indexV1] = true;
        mesh.vert[indexV0].SetS();
        mesh.vert[indexV1].SetS();
    }

    vcg::GridStaticPtr<FaceType, ScalarType> gridF;
    gridF.Set(mesh.face.begin(), mesh.face.end());

    const int dilateStep = 1;
    for (int i = 0; i < dilateStep; ++i) {
        vcg::tri::UpdateSelection<MeshType>::FaceFromVertexLoose(mesh);
        vcg::tri::UpdateSelection<MeshType>::VertexFromFaceLoose(mesh);
    }

    for (size_t s = 0; s < smoothSteps; ++s) {
        SmoothVertOnPath(mesh, vertexPathSet, onPath, true, ScalarType(0.5));
        SmoothVertOnPath(mesh, vertexPathSet, onPath, false, ScalarType(0.5));
        ReprojectPathVertices(mesh, gridF);
    }

    vcg::tri::UpdateSelection<MeshType>::Clear(mesh);
}

void ResnapVertexPathFromSmoothedPositions(MeshType &mesh, std::vector<size_t> &vertexPath)
{
    if (vertexPath.empty())
        return;

    vcg::GridStaticPtr<VertexType, ScalarType> grid;
    grid.Set(mesh.vert.begin(), mesh.vert.end());
    const ScalarType maxD = mesh.bbox.Diag();

    std::vector<size_t> newPath;
    newPath.reserve(vertexPath.size());
    for (size_t vi : vertexPath) {
        if (vi >= mesh.vert.size())
            continue;
        ScalarType minD = 0;
        VertexType *v = vcg::tri::GetClosestVertex(
            mesh, grid, mesh.vert[vi].P(), maxD, minD);
        if (v == nullptr)
            continue;
        const size_t indexV = vcg::tri::Index(mesh, v);
        if (!newPath.empty() && newPath.back() == indexV)
            continue;
        newPath.push_back(indexV);
    }

    if (newPath.size() >= 2)
        vertexPath = std::move(newPath);
}

bool ApplyVertexPathAsCuttingSeam(
    MeshType &mesh,
    const std::vector<size_t> &vertexPath)
{
    if (vertexPath.size() < 2)
        return false;

    std::set<std::pair<size_t, size_t>> addedEdges;
    size_t markedCount = 0;

    for (size_t i = 0; i + 1 < vertexPath.size(); ++i) {
        const size_t v0 = vertexPath[i];
        const size_t v1 = vertexPath[i + 1];
        if (v0 == v1)
            continue;

        int indexF = -1;
        int indexE = -1;
        mesh.WichFaceEdge(v0, v1, indexF, indexE);
        if (indexF < 0 || indexE < 0)
            continue;

        const std::pair<size_t, size_t> key(
            std::min(v0, v1), std::max(v0, v1));
        if (!addedEdges.insert(key).second)
            continue;

        mesh.face[indexF].SetFaceEdgeS(indexE);
        mesh.SharpFeatures.push_back(std::pair<size_t, size_t>(
            static_cast<size_t>(indexF), static_cast<size_t>(indexE)));

        if (!vcg::face::IsBorder(mesh.face[indexF], indexE)) {
            MeshType::FaceType *opp = mesh.face[indexF].FFp(indexE);
            if (opp != nullptr) {
                const int oppE = mesh.face[indexF].FFi(indexE);
                opp->SetFaceEdgeS(oppE);
            }
        }
        ++markedCount;
    }

    mesh.UpdateSharpFeaturesFromSelection();
    mesh.InitRPos();
    return markedCount > 0;
}

std::vector<CoordType> VertexPathToPolyline(
    const MeshType &mesh,
    const std::vector<size_t> &vertexPath)
{
    std::vector<CoordType> polyline;
    polyline.reserve(vertexPath.size());
    for (size_t vi : vertexPath) {
        if (vi < mesh.vert.size())
            polyline.push_back(mesh.vert[vi].P());
    }
    return polyline;
}

BorderVertexPick SnapBorderPickOnMesh(
    GarmentSession &session,
    const CoordType &pickPos)
{
    const ScalarType maxSnap = session.deformed_mesh.bbox.Diag() / ScalarType(15);
    return SnapPickToBorderVertex(session.deformed_mesh, pickPos, maxSnap);
}

