#pragma once
#include <tracing/patch_tracer.h>
#include "patchgraph.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <set>
#include <unordered_set>
template<typename MeshType>
class Segment;


template<typename MeshType>
void samplePatchids(PatchGraph<MeshType> & patchGraph, std::vector<std::vector<int>> & seg2paids){
    // To-Do:
    // Add every segments made of two patch as input.
    for (auto& patchEdge:patchGraph.patchEdges)
    {
        std::vector<int> tmp;
        tmp.push_back(patchEdge.first); tmp.push_back(patchEdge.second);
        seg2paids.push_back(tmp);
    }

    // for (size_t i = 0; i<patchGraph.patches.size(); i++)
    // {
    //     std::vector<int> tmp;
    //     tmp.push_back(i); 
    //     seg2paids.push_back(tmp);
    // }

}

template<typename MeshType>
void sampleSeamedges(PatchGraph<MeshType> & patchGraph,
                     std::vector<std::vector<typename PatchGraph<MeshType>::SeamEdge>> & seg2sedges,
                     size_t minPatchPerSegment = 3,
                     size_t maxPatchPerSegment = 3){
    // Enumerate connected patch subgraphs on seam connectivity.
    // Each sampled connected patch-set is converted to a seam-edge set
    // by collecting all seam edges whose two endpoints are inside the set.
    seg2sedges.clear();
    if (patchGraph.num_patch() == 0 || patchGraph.seamEdges.empty()) return;
    if (minPatchPerSegment < 2) minPatchPerSegment = 2;
    if (maxPatchPerSegment < minPatchPerSegment) return;
    maxPatchPerSegment = std::min(maxPatchPerSegment, patchGraph.num_patch());

    std::vector<std::vector<int>> patchAdj(patchGraph.num_patch());
    for (const auto &sedge : patchGraph.seamEdges)
    {
        patchAdj[sedge.paid0].push_back(sedge.paid1);
        patchAdj[sedge.paid1].push_back(sedge.paid0);
    }

    std::set<std::vector<int>> visitedPatchSets;
    std::vector<int> currentSet;
    std::function<void()> dfsConnectedPatchSets;
    dfsConnectedPatchSets = [&](){
        std::vector<int> sortedSet = currentSet;
        std::sort(sortedSet.begin(), sortedSet.end());

        if (sortedSet.size() >= minPatchPerSegment && sortedSet.size() <= maxPatchPerSegment)
        {
            if (visitedPatchSets.insert(sortedSet).second)
            {
                std::unordered_set<int> patchSet(sortedSet.begin(), sortedSet.end());
                std::vector<typename PatchGraph<MeshType>::SeamEdge> mergedSedges;
                for (const auto &sedge : patchGraph.seamEdges)
                {
                    if (patchSet.count(sedge.paid0) && patchSet.count(sedge.paid1))
                    {
                        mergedSedges.push_back(sedge);
                    }
                }
                if (!mergedSedges.empty()) seg2sedges.push_back(mergedSedges);
            }
        }

        if (sortedSet.size() >= maxPatchPerSegment) return;

        std::set<int> candidates;
        for (int paid : sortedSet)
        {
            for (int nPaid : patchAdj[paid])
            {
                if (std::find(sortedSet.begin(), sortedSet.end(), nPaid) == sortedSet.end())
                {
                    candidates.insert(nPaid);
                }
            }
        }

        for (int nPaid : candidates)
        {
            currentSet.push_back(nPaid);
            dfsConnectedPatchSets();
            currentSet.pop_back();
        }
    };

    for (size_t seedPid = 0; seedPid < patchGraph.num_patch(); ++seedPid)
    {
        currentSet.clear();
        currentSet.push_back(seedPid);
        dfsConnectedPatchSets();
    }
}

template<typename MeshType>
void createSegments(PatchGraph<MeshType> & patchGraph, std::vector<Segment<MeshType>*> & segmentps){
    typedef typename MeshType::ScalarType ScalarType;
    segmentps.clear();

    // Sample connected patchids for each segment;
    std::vector<std::vector<int>> seg2paids;
    std::vector<MeshType*> segmentMeshps;
    samplePatchids(patchGraph, seg2paids);
    ScalarType vertArea=0;
    for (auto& paids: seg2paids)
    {
        Segment<MeshType>* segmentp = new Segment<MeshType>(patchGraph);
        segmentps.push_back(segmentp);
        segmentp->build(paids);
        
        ScalarType area = 0;

        vertArea += vcg::tri::UV_Utils<MeshType>::PerVertUVArea(segmentp->mesh);
        segmentMeshps.push_back(&(segmentp->mesh));
    }

    // Rearrange the UV coordinates:
    ScalarType interDist=math::Sqrt(vertArea/segmentps.size())*0.03;
    PatchManager<MeshType>::ArrangeUVPatches(segmentMeshps, interDist,false);


}

template<typename MeshType>
typename MeshType::ScalarType evaluateSegmentDistortion(Segment<MeshType> & segment)
{
    typedef typename MeshType::FaceType FaceType;
    typedef typename MeshType::ScalarType ScalarType;
    typedef typename MeshType::CoordType CoordType;
    typedef typename vcg::Point2<ScalarType> Point2DType;
    const ScalarType eps = std::numeric_limits<ScalarType>::epsilon();

    if (segment.mesh.FN() == 0) return std::numeric_limits<ScalarType>::max();

    ScalarType totalArea3D = 0;
    ScalarType weightedDist = 0;
    for (size_t fi = 0; fi < segment.mesh.face.size(); ++fi)
    {
        FaceType &f = segment.mesh.face[fi];
        const CoordType e01 = f.P(1) - f.P(0);
        const CoordType e02 = f.P(2) - f.P(0);
        const ScalarType area3D = (e01 ^ e02).Norm() * ScalarType(0.5);

        const auto uv0 = f.V(0)->T().P();
        const auto uv1 = f.V(1)->T().P();
        const auto uv2 = f.V(2)->T().P();
        const Point2DType uvE01 = uv1 - uv0;
        const Point2DType uvE02 = uv2 - uv0;
        const ScalarType doubleAreaUV = uvE01.X() * uvE02.Y() - uvE01.Y() * uvE02.X();
        const ScalarType areaUV = std::abs(doubleAreaUV) * ScalarType(0.5);

        const ScalarType safeArea3D = std::max(area3D, eps);
        const ScalarType safeAreaUV = std::max(areaUV, eps);
        const ScalarType scale = safeAreaUV / safeArea3D;
        const ScalarType localDist = std::abs(std::log(scale));

        weightedDist += localDist * safeArea3D;
        totalArea3D += safeArea3D;
    }

    return weightedDist / std::max(totalArea3D, eps);
}

template<typename MeshType>
void buildManualSegmentCandidates(
    PatchGraph<MeshType> & patchGraph,
    const std::vector<std::vector<int>> & manualPatchSets,
    std::vector<Segment<MeshType>*> & manualSegmentps)
{
    manualSegmentps.clear();
    for (const auto &rawPaids : manualPatchSets)
    {
        std::vector<int> paids;
        for (int paid : rawPaids)
        {
            if (paid < 0 || static_cast<size_t>(paid) >= patchGraph.num_patch())
                continue;
            if (std::find(paids.begin(), paids.end(), paid) == paids.end())
                paids.push_back(paid);
        }
        if (paids.empty())
            continue;

        Segment<MeshType> *segmentp = new Segment<MeshType>(patchGraph);
        segmentp->build(paids);
        manualSegmentps.push_back(segmentp);
    }
}

template<typename MeshType>
void createSegmentsfromSids(PatchGraph<MeshType> & patchGraph,
                            std::vector<Segment<MeshType>*> & segmentps,
                            size_t minPatchPerSegment = 2,
                            size_t maxPatchPerSegment = 2,
                            bool includeSinglePatchSegments = true,
                            size_t topKLowestDistortion = 500,
                            const std::vector<std::vector<int>> *manualPatchSets = nullptr){
    typedef typename MeshType::ScalarType ScalarType;
    segmentps.clear();

    // Sample connected patchids for each segment;
    std::vector<std::vector<typename PatchGraph<MeshType>::SeamEdge>>  seg2sedges;
    std::vector<MeshType*> segmentMeshps;
    sampleSeamedges(patchGraph, seg2sedges, minPatchPerSegment, maxPatchPerSegment);
    struct SegmentCandidate{
        Segment<MeshType>* segp;
        ScalarType distortion;
    };
    std::vector<SegmentCandidate> candidates;
    candidates.reserve(seg2sedges.size() + patchGraph.num_patch());

    for (auto& sedges: seg2sedges)
    {
        Segment<MeshType>* segmentp = new Segment<MeshType>(patchGraph);
        segmentp->build_from_sedges(sedges);
        candidates.push_back({segmentp, evaluateSegmentDistortion(*segmentp)});
    }

    std::vector<int> paids;
    if (includeSinglePatchSegments)
    {
        for (size_t i = 0; i < patchGraph.num_patch(); i++)
        {
            paids.clear();
            paids.push_back(i);
            Segment<MeshType>* segmentp = new Segment<MeshType>(patchGraph);
            segmentp->build(paids);
            candidates.push_back({segmentp, evaluateSegmentDistortion(*segmentp)});
        }
    }

    std::vector<Segment<MeshType>*> manualSegmentps;
    if (manualPatchSets != nullptr && !manualPatchSets->empty())
        buildManualSegmentCandidates(patchGraph, *manualPatchSets, manualSegmentps);

    const size_t manualCount = manualSegmentps.size();
    const size_t autoKeepCount = (topKLowestDistortion > manualCount)
        ? topKLowestDistortion - manualCount
        : 0;

    if (candidates.empty() && manualCount == 0)
        return;

    if (!candidates.empty() && autoKeepCount > 0)
    {
        const size_t keepCount = std::min(autoKeepCount, candidates.size());
        std::partial_sort(
            candidates.begin(),
            candidates.begin() + keepCount,
            candidates.end(),
            [](const SegmentCandidate &a, const SegmentCandidate &b){
                return a.distortion < b.distortion;
            }
        );

        segmentps.reserve(manualCount + keepCount);
        segmentMeshps.reserve(manualCount + keepCount);
        ScalarType vertArea = 0;
        for (auto *segp : manualSegmentps)
        {
            segmentps.push_back(segp);
            segmentMeshps.push_back(&(segp->mesh));
            vertArea += vcg::tri::UV_Utils<MeshType>::PerVertUVArea(segp->mesh);
        }
        for (size_t i = 0; i < keepCount; ++i)
        {
            segmentps.push_back(candidates[i].segp);
            segmentMeshps.push_back(&(candidates[i].segp->mesh));
            vertArea += vcg::tri::UV_Utils<MeshType>::PerVertUVArea(candidates[i].segp->mesh);
        }
        for (size_t i = keepCount; i < candidates.size(); ++i)
            delete candidates[i].segp;
    }
    else
    {
        for (size_t i = 0; i < candidates.size(); ++i)
            delete candidates[i].segp;

        segmentps.reserve(manualCount);
        segmentMeshps.reserve(manualCount);
        ScalarType vertArea = 0;
        for (auto *segp : manualSegmentps)
        {
            segmentps.push_back(segp);
            segmentMeshps.push_back(&(segp->mesh));
            vertArea += vcg::tri::UV_Utils<MeshType>::PerVertUVArea(segp->mesh);
        }
    }

    // Rearrange the UV coordinates:
    if (segmentps.empty()) return;
    ScalarType vertArea=0;
    for (auto *segp : segmentps)
        vertArea += vcg::tri::UV_Utils<MeshType>::PerVertUVArea(segp->mesh);
    ScalarType interDist=math::Sqrt(vertArea/segmentps.size())*0.03;
    std::vector<vcg::Similarity2<ScalarType> > trVec; //Transformation info.
    PatchManager<MeshType>::ArrangeUVPatches(segmentMeshps, trVec, interDist,false);
    for (size_t i = 0; i < segmentps.size(); i++)
    {
        segmentps[i]->load_trVec(trVec[i]);
    }
}




template<typename MeshType>
class Segment{
    typedef typename MeshType::VertexType	 VertexType;
    typedef typename MeshType::EdgeType		 EdgeType;
    typedef typename MeshType::FaceType		FaceType;
    typedef typename MeshType::CoordType    CoordType;
    typedef typename MeshType::ScalarType ScalarType;
    typedef typename vcg::Point2<ScalarType> Point2DType;
    typedef typename PatchGraph<MeshType>::Seam Seam;
    typedef typename PatchGraph<MeshType>::SeamEdge SeamEdge;
    typedef vcg::face::Pos<FaceType> PosType;


    struct Line2D {
        ScalarType a;
        ScalarType b; 
        ScalarType c; 

        void approximate_seam(std::vector<Point2DType>& points2D){
            const int n = (int)points2D.size();
            assert(n >= 2);

            ScalarType sumX = 0, sumY = 0;
            ScalarType sumXX = 0, sumXY = 0, sumYY = 0;

            for (auto& p : points2D) {
                ScalarType x = p.X();
                ScalarType y = p.Y();
                sumX  += x;
                sumY  += y;
                sumXX += x * x;
                sumXY += x * y;
                sumYY += y * y;
            }

            ScalarType meanX = sumX / n;
            ScalarType meanY = sumY / n;

            // covariance terms
            ScalarType Sxx = 0, Sxy = 0, Syy = 0;
            for (auto& p : points2D) {
                ScalarType dx = p.X() - meanX;
                ScalarType dy = p.Y() - meanY;
                Sxx += dx * dx;
                Sxy += dx * dy;
                Syy += dy * dy;
            }

            ScalarType trace = Sxx + Syy;
            ScalarType det = Sxx * Syy - Sxy * Sxy;
            ScalarType eigen = trace / 2 - std::sqrt(std::max((trace * trace) / 4 - det, ScalarType(0)));

            // Eigenvector (a, b)
            if (std::abs(Sxy) > 1e-12) {
                a = eigen - Syy;
                b = Sxy;
            } else {
                a = 1;
                b = 0;
            }

            // Normalize so that a² + b² = 1
            ScalarType L = std::sqrt(a * a + b * b);
            a /= L;
            b /= L;
            c = -(a * meanX + b * meanY);
        }

        Point2DType intersect(const Line2D& L) const 
        {
            ScalarType D = -a * L.b + L.a * b;
            assert(std::abs(D) > 1e-12); // no parallel lines

            ScalarType Dx = c * L.b - L.c * b;
            ScalarType Dy = a * L.c - L.a * c;

            ScalarType x = Dx / D;
            ScalarType y = Dy / D;

            return Point2DType(x, y);
        }

        bool isAlmostParallel(const Line2D& L, ScalarType eps = 1e-6) const
        {
            // Normal vectors (a, b) and (L.a, L.b)
            ScalarType dot = a * L.a + b * L.b;
            ScalarType norm1 = std::sqrt(a * a + b * b);
            ScalarType norm2 = std::sqrt(L.a * L.a + L.b * L.b);

            if (norm1 < eps || norm2 < eps)
                return false;

            ScalarType cosTheta = dot / (norm1 * norm2);

            // For parallel lines, |cosθ| ≈ 1
            return std::abs(std::abs(cosTheta) - 1.0) < eps;
        }

        Line2D(){
            a=0, b=0, c=0;
        }

        Line2D(std::vector<Point2DType>& points2D){
            approximate_seam(points2D);
        }
    };

    struct ApproxPolygon{

        std::vector<Point2DType> polyVs;
        std::vector<Point2DType> mergedPolyVs;

        ApproxPolygon(){}

        bool build(std::vector<Line2D>& lines){
            // Based on the line information compute the joints.
            int numVert = lines.size();
            polyVs.reserve(numVert);
            for (int i = 0; i < numVert; ++i) {
                Line2D& L1 = lines[i];
                Line2D& L2 = lines[(i + 1) % numVert];  // wrap-around
                polyVs.push_back(L1.intersect(L2));
            }
            return true;
        }

        bool build(std::vector<Point2DType>& points){
            polyVs = points;


            return true;
        }

        bool build(std::vector<std::vector<Point2DType>>& edgePoints){
            int numEdge = edgePoints.size();
            polyVs.reserve(numEdge);
            Point2DType last_p = edgePoints[0][0];
            for (int i = 0; i < numEdge; ++i) {
                assert(edgePoints[i].size()==2);
                Point2DType p0 = edgePoints[i][0]; 
                Point2DType p1 = edgePoints[i][1];

                if (p0==last_p) {polyVs.push_back(p0); last_p=p1; continue;}
                if (p1==last_p) {polyVs.push_back(p1); last_p=p0;}
            }
        }

        bool build_mergedPolygon(){
        }

        bool exportToTxt(const std::string& filename) const
        {
            std::ofstream fout(filename);
            if (!fout.is_open()) return false;

            // Write number of vertices (optional but useful)
            fout << polyVs.size() << "\n";

            for (const auto& p : polyVs) {
                fout << p.X() << " " << p.Y() << "\n";
            }

            return true;
        }

    };

    public:
    PatchGraph<MeshType> & patchGraph;
    MeshType mesh;
    std::vector<ScalarType> orgvid2ang2D;
    std::vector<int> paids;
    std::map<size_t,size_t> org2newvid;
    std::vector<size_t> new2orgvid;
    std::map<size_t,size_t> org2newfid;
    std::vector<size_t> new2orgfid;

    std::vector<Seam> seams; // seams on original mesh.
    std::vector<int> seamOrder;
    ApproxPolygon polygon;

    // Parameterization related members:
    vcg::Similarity2<ScalarType> trvec;
    void load_trVec(vcg::Similarity2<ScalarType> _trvec){
        trvec = _trvec;
    }
    Point2DType revertUVvert(Point2DType p){
        // std::cout<<trvec.tra.X()<<trvec.tra.Y()<<std::endl;

        p -= trvec.tra;
        p /= trvec.sca;
        p.Rotate(-trvec.rotRad);
        return p;
    }

    Segment(PatchGraph<MeshType> &_pg):patchGraph(_pg){};

    VertexType* get_vRef(size_t fid, size_t eid, bool end=false){
        // Load vertRef on the new mesh. from original fid and vid. (Assume the topology of mesh triangles unchanged.)
        size_t new_fid = org2newfid[fid];
        if (end)
        {
            return mesh.face[new_fid].V1(eid);
        }
        else{
            return mesh.face[new_fid].V0(eid);
        }
    }

    size_t get_vid(size_t fid, size_t eid, bool end=false){
        // Load the new vid on the mesh; from old fid, eid.
        return vcg::tri::Index(mesh, get_vRef(fid, eid, end));
    }

    void flatten_segment(){
        // std::cout<< (mesh.face[0].V(0)->T().P() == patchGraph.mesh.face[new2orgfid[0]].V(0)->T().P())<<std::endl;
        vcg::tri::OptimizeUV_ARAP(mesh,100,0,true);
        // std::cout<<"Check flatten_segment"<<std::endl;
        // std::cout<< (mesh.face[0].V(0)->T().P() == patchGraph.mesh.face[new2orgfid[0]].V(0)->T().P())<<std::endl;
    }

    void build_patchmesh(){

        // Gather org face ids.
        std::vector<size_t> orgfids;
        for (auto paid: paids)
        {
            for (auto fid: patchGraph.paid2fids[paid]) orgfids.push_back(fid);
        }

        // Initialize data.
        mesh.Clear();
        mesh.face.reserve(orgfids.size());
        mesh.vert.reserve(orgfids.size());

        org2newvid.clear();
        new2orgvid.clear();
        
        org2newfid.clear();
        new2orgfid.clear();

        // Build mapping between face and face ids.
        MeshType & totalMesh = patchGraph.mesh;

        std::vector<std::vector<size_t>> newfid2vids;
        newfid2vids.resize(orgfids.size(),std::vector<size_t>(3,-1));
        for (size_t i=0;i<orgfids.size();i++)
        {
            size_t IndexF=orgfids[i];
            //std::vector<int> FaceV(3,-1);
            for (int j=0;j<totalMesh.face[IndexF].VN();j++)
            {
                VertexType *v=totalMesh.face[IndexF].V(j);
                size_t IndexV=vcg::tri::Index(totalMesh,v);
                //allocate in case it is not already there
                if (org2newvid.count(IndexV)==0)
                {
                    new2orgvid.push_back(IndexV);
                    org2newvid[IndexV]=new2orgvid.size()-1;
                    newfid2vids[i][j]=(new2orgvid.size()-1);
                }else
                {
                    newfid2vids[i][j]=org2newvid[IndexV];
                }
            }
            assert(newfid2vids[i][0]>=0);
            assert(newfid2vids[i][0]<(int)new2orgvid.size());
            assert(newfid2vids[i][1]>=0);
            assert(newfid2vids[i][1]<(int)new2orgvid.size());
            assert(newfid2vids[i][2]>=0);
            assert(newfid2vids[i][2]<(int)new2orgvid.size());
        }


        //Allocate vertices
        vcg::tri::Allocator<MeshType>::AddVertices(mesh, new2orgvid.size());
        assert(mesh.vert.size()==new2orgvid.size());

        //Copy values from vertices
        for (size_t i=0;i<mesh.vert.size();i++)
        {
            size_t IndexV=new2orgvid[i];
            VertexType *OrigV=&totalMesh.vert[IndexV];
            mesh.vert[i].ImportData(*OrigV);
        }

        //then add faces
        vcg::tri::Allocator<MeshType>::AddFaces(mesh,newfid2vids.size());
        assert(mesh.face.size()==newfid2vids.size());
        assert(mesh.face.size()==orgfids.size());

        //add vertices and copy values from original ace
        for (size_t i=0;i<mesh.face.size();i++)
        {
            VertexType *v0=&mesh.vert[newfid2vids[i][0]];
            VertexType *v1=&mesh.vert[newfid2vids[i][1]];
            VertexType *v2=&mesh.vert[newfid2vids[i][2]];

            mesh.face[i].V(0)=v0;
            mesh.face[i].V(1)=v1;
            mesh.face[i].V(2)=v2;

            FaceType *f=&totalMesh.face[orgfids[i]];
            mesh.face[i].ImportData(*f);

            new2orgfid.push_back(orgfids[i]);
            org2newfid[orgfids[i]] = i;
            // for (size_t k=0;k<3;k++)
            // {
            //     mesh.face[i].WT(k).P()=mesh.face[i].V(k)->T().P();
            // }

        }
        mesh.UpdateAttributes();

    }

    void build_patchmesh(std::vector<SeamEdge> & mergingSedges){
        // intput: 
        // sedges: the seams that needed to be merged. ( the seams should otherwise be boarder or cut)

        // 1. Find seams needed to be cut at the garment.
        assert(paids.size()!=0);
        std::map<int, std::vector<size_t>> paid2ignoredsid;
        std::vector<SeamEdge> cuttingSedges;

        // Traverse each seam edge that is inside and on the segment, it should be either merging edge or cutting edge.
        for (auto & seamEdge : patchGraph.seamEdges)
        {
            // Filter seams that are in or on the segments.
            if (std::find(paids.begin(), paids.end(), seamEdge.paid0) != paids.end() &&
            std::find(paids.begin(), paids.end(), seamEdge.paid1) != paids.end()){
                // Exclude the merging seams.
                if (std::find(mergingSedges.begin(), mergingSedges.end(), seamEdge)== mergingSedges.end()) cuttingSedges.push_back(seamEdge);
            }
        }
        // 2. Copy verts & faces from patches. (same as build_patchmesh())
        build_patchmesh();
        // vcg::tri::io::ExporterOBJ<MeshType>::Save(mesh, "./debugmesh_before_update.obj",vcg::tri::io::Mask::IOM_ALL);
        // 3. traverse each seams tha neeed to be cut.
        //    3.1 create new vertices; copy data.
        //    3.2 record id mapping.
        //    3.2 traverse each vertex edge, reassign the mesh topology.
        for (auto & seamEdge : cuttingSedges){
            auto & seam0 = patchGraph.patches[seamEdge.paid0].seams[seamEdge.sid0];
            auto & seam1 = patchGraph.patches[seamEdge.paid1].seams[seamEdge.sid1];

            assert(seam0.num_edges() == seam1.num_edges());
            //Create new vertices.
            size_t num_verts = mesh.VN();
            size_t num_new_verts = seam0.num_verts();
            vcg::tri::Allocator<MeshType>::AddVertices(mesh, num_new_verts);

            // Init startPos as the starting point of traverse.
            auto & seam = seam0;
            PosType startPos(&mesh.face[org2newfid[seam.fids.back()]], seam.eids.back());

            startPos.FlipV();
            do{
                startPos.FlipE();
                startPos.FlipF();
            } while (!startPos.IsEdgeS() && !startPos.IsBorder());

            // Start traversing
            std::vector<size_t> tmp_fids;
            std::vector<size_t> tmp_eids;
            std::vector<VertexType*> tmp_vertRefs;
            int i = 0;
            while (i!=num_new_verts)
            {
                auto newendVertRef = &(mesh.vert[i+num_verts]);
                do{
                    startPos.FlipE();
                    tmp_fids.push_back(vcg::tri::Index(mesh, startPos.F()));
                    tmp_eids.push_back(startPos.E());
                    tmp_vertRefs.push_back(newendVertRef);
                    newendVertRef->ImportData(*(startPos.F()->V1(startPos.E())));
                    startPos.FlipF();
                } while (!startPos.IsEdgeS()&& !startPos.IsBorder());
                i++;
                startPos.FlipF();
                startPos.FlipV();
            }

            for (size_t i = 0; i < tmp_fids.size(); i++)
            {
                mesh.face[tmp_fids[i]].V1(tmp_eids[i]) = tmp_vertRefs[i];
            }

            mesh.UpdateAttributes();
        }

        // vcg::tri::Clean<MeshType>::RemoveDuplicateVertex(mesh);
        // vcg::tri::Clean<MeshType>::RemoveUnreferencedVertex(mesh);
        // vcg::tri::Allocator<MeshType>::CompactEveryVector(mesh);
    }

    void load_seamTextCoords(int sidx, std::vector<Point2DType>& points2D){
        points2D.clear();
        Seam& seam = seams[sidx];

        // Linear approximate approach:
        // for (size_t i = 0; i < seam.fids.size(); i++)
        // {
        //     VertexType *v=patchGraph.mesh.face[seam.fids[i]].V(seam.eids[i]);
        //     size_t IndexV=vcg::tri::Index(patchGraph.mesh,v);
        //     points2D.push_back(mesh.vert[org2newvid[IndexV]].T().P());
        // }

        // Simple endpoints connecting approach:
        // std::array<size_t, 2> endvids;
        
        VertexType* startVert = mesh.face[org2newfid[seam.fids.front()]].V0(seam.eids.front());
        VertexType* endVert = mesh.face[org2newfid[seam.fids.back()]].V1(seam.eids.back());
        

        // seam.load_endvIds(endvids);
        points2D.push_back(startVert->T().P());
        points2D.push_back(endVert->T().P());
    }

    void compute_vertex2Dangle(){
        //To-Do:: based on VertexClassifier::GetVertexAngle 
    }

    void build_approxPolygon(){
        if (seamOrder.size() != seams.size())
            seamOrder.clear();
        if (seamOrder.empty() && !seams.empty())
            build_seamOrder();

        // Total 2D points:
        std::vector<Point2DType> totalPoints;

        // Approximate each seam:
        std::vector<Line2D> lines;
        lines.resize(seamOrder.size());
        std::vector<std::vector<Point2DType>> edgePoints;
        for (size_t i = 0; i < seamOrder.size(); i++)
        {
            std::vector<Point2DType> points2D;
            load_seamTextCoords(seamOrder[i], points2D);
            edgePoints.push_back(points2D);
            lines[i].approximate_seam(points2D);
        }
        // Build Polygon
        polygon.build(edgePoints);
        // polygon.build(lines);
        // polygon.build(totalPoints);
    }

    void build_seamOrder() {
        // Map each original face id to the seams that touch it

        std::vector<std::pair<size_t, size_t>> seamEndpoints;
        for (auto & seam: seams)
        {
            size_t v0id = vcg::tri::Index(mesh, mesh.face[org2newfid[seam.fids.front()]].V0(seam.eids.front()));
            size_t v1id = vcg::tri::Index(mesh, mesh.face[org2newfid[seam.fids.back()]].V1(seam.eids.back()));
            seamEndpoints.push_back({v0id, v1id});
        }

        const size_t n = seamEndpoints.size();
        assert(n > 0);

        // Build adjacency: vertex -> edges touching it
        std::unordered_map<size_t, std::vector<size_t>> vert2edges;
        vert2edges.reserve(n * 2);

        for (size_t i = 0; i < n; ++i) {
            auto [a, b] = seamEndpoints[i];
            vert2edges[a].push_back(i);
            vert2edges[b].push_back(i);
        }

        // Result list of edge indices
        seamOrder.clear();
        seamOrder.reserve(n);

        // Start with the first edge
        size_t currEdge = 0;
        seamOrder.push_back(currEdge);

        // Determine walking direction
        auto [u, v] = seamEndpoints[currEdge];
        size_t currV = v; // start going from u -> v

        std::vector<char> visited(n, false);
        visited[currEdge] = true;

        for (size_t step = 1; step < n; ++step) {
            const auto& inc = vert2edges[currV];

            // currV must connect to exactly 2 edges in a simple loop
            assert(inc.size() == 2);

            size_t nextEdge =
                (inc[0] == currEdge ? inc[1] : inc[0]);

            assert(!visited[nextEdge]);
            visited[nextEdge] = true;

            seamOrder.push_back(nextEdge);

            // move to next vertex
            auto [a2, b2] = seamEndpoints[nextEdge];
            currV = (a2 == currV ? b2 : a2);

            currEdge = nextEdge;
        }
        
    }

    void ensureSeamOrderForExport() {
        if (seams.empty()) {
            seamOrder.clear();
            return;
        }
        if (seamOrder.size() != seams.size())
            build_seamOrder();
    }

    void build_seams(){
        seams.clear();
        seamOrder.clear();
        std::map<int, std::vector<size_t>> paid2ignoredsid;

        // Traverse each seam edge that both paid is contained in paids, record it as merged seam id.
        for (auto& seamEdge:patchGraph.seamEdges)
        {
            if (std::find(paids.begin(), paids.end(), seamEdge.paid0) != paids.end() &&
            std::find(paids.begin(), paids.end(), seamEdge.paid1) != paids.end())
            {
                paid2ignoredsid[seamEdge.paid0].push_back(seamEdge.sid0);
                paid2ignoredsid[seamEdge.paid1].push_back(seamEdge.sid1);
            }
        }
        
        // For each patch traverse each seam, ignore merged seam.
        for (auto paid:paids)
        {
            auto& patch = patchGraph.patches[paid];
            for (size_t sid = 0; sid < patch.num_seams(); sid++)
            {
                auto& ignoredsids = paid2ignoredsid[paid];
                if (std::find(ignoredsids.begin(), ignoredsids.end(), sid) != ignoredsids.end()) continue;
                seams.push_back(patch.get_seam(sid));
            }
        }
    }

    void build_seams(std::vector<SeamEdge> & sedges){
        seams.clear();
        seamOrder.clear();
        std::map<int, std::vector<size_t>> paid2ignoredsid;

        // Traverse each seam edge that both paid is contained in paids, record it as merged seam id.
        for (auto& seamEdge:patchGraph.seamEdges)
        {
            if (std::find(sedges.begin(), sedges.end(), seamEdge) != sedges.end()) 
            {
                paid2ignoredsid[seamEdge.paid0].push_back(seamEdge.sid0);
                paid2ignoredsid[seamEdge.paid1].push_back(seamEdge.sid1);
            }
        }
        
        // For each patch traverse each seam, ignore merged seam.
        for (auto paid:paids)
        {
            auto& patch = patchGraph.patches[paid];
            for (size_t sid = 0; sid < patch.num_seams(); sid++)
            {
                auto& ignoredsids = paid2ignoredsid[paid];
                if (std::find(ignoredsids.begin(), ignoredsids.end(), sid) != ignoredsids.end()) continue;
                seams.push_back(patch.get_seam(sid));
            }
        }
    }

    void build(std::vector<int> & _paids){
        assert(_paids.size()>0);
        paids.clear();
        paids = _paids;

        build_patchmesh();
        build_seams();
        flatten_segment();

    }

    void build_from_sedges(std::vector<SeamEdge> & sedges){
        // Build segment by merging patches from given seams.

        // Init paids:
        assert(sedges.size()>0);
        std::set<int> tmp_paids;
        for (auto& sedge : sedges)
        {
            tmp_paids.insert(sedge.paid0);
            tmp_paids.insert(sedge.paid1);
        }
        paids.clear();
        for (auto &paid : tmp_paids) {paids.push_back(paid);} 
        build_patchmesh(sedges);
        // vcg::tri::io::ExporterOBJ<MeshType>::Save(mesh, "./debugmesh.obj",vcg::tri::io::Mask::IOM_ALL);
        build_seams(sedges);
        flatten_segment();
    }

    void showPos(PosType & pos){
        size_t v0id = vcg::tri::Index(mesh, pos.F()->V0(pos.E()));
        size_t v1id = vcg::tri::Index(mesh, pos.F()->V1(pos.E()));
        std::cout<<"("<<v0id<<", "<<v1id<<", "<<vcg::tri::Index(mesh, pos.V())<<")"<<" - ";
    }

    void showFace(FaceType & face){
        size_t v0id = vcg::tri::Index(mesh, face.V(0));
        size_t v1id = vcg::tri::Index(mesh, face.V(1));
        size_t v2id = vcg::tri::Index(mesh, face.V(2));
        std::cout<<"v0id, v1id, v2id: "<<v0id<<", "<<v1id<<", "<<v2id<<std::endl;
    }

    void checkFlipE(PosType & pos){
        auto f = pos.F();
        auto z = pos.E();
        auto v = pos.V();
        size_t v0id = vcg::tri::Index(mesh, v);
        size_t v1id = vcg::tri::Index(mesh, f->V(f->Next(z)));
        std::cout<<"CheckFlipE: v0id, v1id: "<<v0id<<", "<<v1id<<std::endl;
        // f->V(f->Next(z))==v;
    }

    void traverse_seam(Seam & seam){
        // Traverse all the triangles touching the seam. (contain one of the vertex on the seam)

        // Init startPos as the starting point of traverse.
        PosType startPos(&mesh.face[org2newfid[seam.fids.back()]], seam.eids.back());
        std::cout<<"Start start Pos: "; showPos(startPos); 
        // std::cout<<"Step0"<<std::endl;
        do{
            startPos.FlipV();
            startPos.FlipE();
            if (!startPos.IsEdgeS() && !startPos.IsBorder())
            {
                startPos.FlipF();
                startPos.FlipV();
            }
        } while (!startPos.IsEdgeS() && !startPos.IsBorder());

        // std::cout<<"Step1"<<std::endl;
         // Init endPos as the end point of traverse.
        PosType endPos(&mesh.face[org2newfid[seam.fids.front()]], seam.eids.front());
        do{
            endPos.FlipE();
            endPos.FlipV();
            if (!endPos.IsEdgeS() && !endPos.IsBorder())
            {
                endPos.FlipF();
                endPos.FlipV();
            }
        } while (!endPos.IsEdgeS() && !endPos.IsBorder());

        // std::cout<<"Step2"<<std::endl;
        // Start traversing      
        std::vector<size_t> passed_fids; passed_fids.push_back(vcg::tri::Index(mesh, startPos.F()));
        std::vector<size_t> passed_vids; passed_vids.push_back(vcg::tri::Index(mesh, startPos.F()->V0(startPos.E())));
        std::vector<bool> passed_vids_IsB; passed_vids_IsB.push_back(startPos.F()->V0(startPos.E())->IsB());
        std::vector<bool> pos_IsB; passed_vids_IsB.push_back(startPos.IsEdgeS());
        std::vector<bool> pos_IsS; passed_vids_IsB.push_back(startPos.IsBorder());
        int i = 0;
        int loop_count = 0;
        std::cout<<"Start Pos: "; showPos(startPos); 
        std::cout<<"End Pos: "; showPos(endPos); std::cout<<std::endl;
        // showFace(*startPos.F());
        do
        {
            i++; loop_count++;
            // std::cout<<"Step2.0"<<std::endl;
            do{
                loop_count++; if (loop_count>=200) break;
                
                startPos.FlipE();
                startPos.FlipV();
                // std::cout<<"FlipE "; 
                showPos(startPos);
                pos_IsS.push_back(startPos.IsEdgeS());
                pos_IsB.push_back(startPos.IsBorder());
                // showFace(*startPos.F());
                // checkFlipE(startPos);
                if (!startPos.IsEdgeS() && !startPos.IsBorder())
                {
                    startPos.FlipF();
                    startPos.FlipV();
                    // std::cout<<"FlipF ";
                    showPos(startPos);
                    // showFace(*startPos.F());
                    // checkFlipE(startPos);
                    passed_fids.push_back(vcg::tri::Index(mesh, startPos.F()));
                    pos_IsS.push_back(startPos.IsEdgeS());
                    pos_IsB.push_back(startPos.IsBorder());
                }
            } while (!startPos.IsEdgeS() && !startPos.IsBorder());
            if (loop_count>=200) break;
            // if (vcg::tri::Index(mesh, startPos.F()->V0(startPos.E()))==1) {assert(false); std::cout<<"1????";}
            // std::cout<<vcg::tri::Index(mesh, startPos.F()->V0(startPos.E()))<<"-"<<std::endl;
            passed_vids.push_back(vcg::tri::Index(mesh, startPos.F()->V0(startPos.E())));
            passed_vids_IsB.push_back(startPos.F()->V0(startPos.E())->IsB());
        } while (startPos!=endPos);
        std::cout<<"end"<<": Looped "<<loop_count<<" Times."<<std::endl;

        // Show results:
        // std::cout<<"----------------------------------"<<std::endl;
        std::cout<<"passed vids: "; for (auto vid : passed_vids) std::cout<<vid<<"-"; std::cout<<"end"<<std::endl;
        std::cout<<"Passed vids IsB: "; for (auto isb : passed_vids_IsB) std::cout<<isb<<"-"; std::cout<<"end"<<std::endl;
        std::cout<<"pos isB: "; for (auto ib : pos_IsB) std::cout<<ib<<"-"; std::cout<<"end"<<std::endl;
        std::cout<<"pos isS: "; for (auto is : pos_IsS) std::cout<<is<<"-"; std::cout<<"end"<<std::endl;
        // std::cout<<"----------------------------------"<<std::endl<<std::endl;
        
    }

    static void debugPrint(typename Segment<MeshType>::Line2D&line){
        std::cout<<"Line:"<<std::endl;
        std::cout<<"a: "<<line.a<<", b: "<<line.b<<", c: "<<line.c<<std::endl;
    }

    static void debugPrint(typename Segment<MeshType>::Point2DType&point){
        std::cout<<"point 2D:"<<std::endl;
        std::cout<<"u: "<<point.X()<<", v: "<<point.Y()<<std::endl;
    }

    static void debugPrint(std::vector<typename Segment<MeshType>::Point2DType>&points){
        std::cout<<"points 2D:"<<std::endl;
        for (auto&point:points)
        {
            std::cout<<"u: "<<point.X()<<", v: "<<point.Y()<<std::endl;
        }
    }    
};
