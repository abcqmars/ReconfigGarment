#pragma once
#include <tracing/patch_tracer.h>

inline void groupEdges2Seams(const std::vector<std::pair<size_t, size_t>>& edges, std::vector<std::vector<size_t>>& groups)
{
    // Build adjacency list (vertex -> connected vertices)
    groups.clear();
    std::unordered_map<size_t, std::vector<size_t>> adj;
    for (auto [u, v] : edges) {
        adj[u].push_back(v);
        adj[v].push_back(u);
    }

    std::unordered_set<size_t> visited_nodes;
    std::vector<bool> edge_used(edges.size(), false);

    for (size_t i = 0; i < edges.size(); ++i) {
        auto [u, v] = edges[i];
        // Skip if both endpoints already belong to visited components
        if (visited_nodes.count(u) && visited_nodes.count(v))
            continue;

        std::vector<size_t> component_nodes;
        std::stack<size_t> stack;
        stack.push(u);
        visited_nodes.insert(u);

        // DFS to find all connected nodes
        while (!stack.empty()) {
            size_t cur = stack.top();
            stack.pop();
            component_nodes.push_back(cur);

            for (auto nei : adj[cur]) {
                if (!visited_nodes.count(nei)) {
                    visited_nodes.insert(nei);
                    stack.push(nei);
                }
            }
        }

        // Collect edges that connect nodes in this component
        std::unordered_set<size_t> node_set(component_nodes.begin(), component_nodes.end());
        std::vector<size_t> comp_edge_indices;

        for (size_t j = 0; j < edges.size(); ++j) {
            if (edge_used[j]) continue;
            auto [a, b] = edges[j];
            if (node_set.count(a) && node_set.count(b)) {
                comp_edge_indices.push_back(j);
                edge_used[j] = true;
            }
        }

        if (!comp_edge_indices.empty())
            groups.push_back(std::move(comp_edge_indices));
    }
}

template<typename MeshType>
class PatchGraph{
    public:
    typedef typename MeshType::VertexType	 VertexType;
    typedef typename MeshType::EdgeType		 EdgeType;
    typedef typename MeshType::FaceType		FaceType;
    typedef typename MeshType::CoordType    CoordType;
    typedef typename MeshType::ScalarType ScalarType;
    typedef typename vcg::Point2<ScalarType> Point2DType;

    struct Seam {
        MeshType& mesh;
        bool is_boarder;
        std::vector<size_t> fids;
        std::vector<size_t> eids;
        std::vector<size_t> vids;
        std::vector<size_t> _order;

        size_t num_edges(){
            return fids.size();
        }

        size_t num_verts(){
            // To-Improve: the seam may form a loop.
            return num_edges() + 1;
        }

        bool load_2dcoord(std::vector<Point2DType>& points2D){
            points2D.clear();
            assert(fids.size()==eids.size());
            for (size_t i=0; i<fids.size(); i++)
            {
                points2D.push_back(mesh.face[fids[i]].V(eids[i])->T().P());
            }
        }
        
        bool load_adjfids(std::vector<size_t>& adjfids){
            adjfids.clear();
            for (size_t i = 0; i < fids.size(); i++)
            {
                adjfids.push_back(vcg::tri::Index(mesh, mesh.face[fids[i]].FFp(eids[i])));
            }
        }

        void load_endvIds(std::array<size_t, 2>& endvids){
            std::map<size_t, int> vertCount;
            for (size_t i = 0; i < fids.size(); i++)
            {
                size_t vid0 = vcg::tri::Index(mesh, mesh.face[fids[i]].V0(eids[i]));
                size_t vid1 = vcg::tri::Index(mesh, mesh.face[fids[i]].V1(eids[i]));
                if (!vertCount.count(vid0))
                {
                    vertCount[vid0] = 1;
                }
                else{
                    vertCount[vid0] += 1;
                }

                if (!vertCount.count(vid1))
                {
                    vertCount[vid1] = 1;
                }
                else{
                    vertCount[vid1] += 1;
                }
            }
            
            int endCout = 0;
            for (auto it = vertCount.begin(); it!=vertCount.end(); it++)
            {
                if (it->second==1)
                {
                    if (endCout<=1) endvids[endCout] = it->first;
                    endCout ++;
                }
            }
        }

        size_t contain(std::vector<size_t>& in_fids){
            std::unordered_set<size_t> fidset(fids.begin(), fids.end());
            return std::count_if(in_fids.begin(), in_fids.end(), [&](size_t x) {
                return fidset.count(x);
            });
        }

        bool isBoarder(){
            return is_boarder;
        }

        bool build(std::vector<size_t> & _fids, std::vector<size_t>& _eids){
            if (!(_fids.size() == _eids.size())) return false;
            if (_fids.size() == 0) return false;

            fids = _fids;
            eids = _eids;
            size_t num_boarderf = 0;
            for (size_t i = 0; i < fids.size(); i++)
            {
                if (mesh.face[fids[i]].IsB(eids[i])) num_boarderf++;
            }
            if (num_boarderf==0) {
                is_boarder=false;
            }
            else{
                if (!(num_boarderf == fids.size())){return false;}
                is_boarder=true;
            }
            bool succeed = build_order(); assert(succeed);
            reorder();
            return true;
        }

        bool build_order(){
            std::vector<std::pair<size_t,size_t>> edges;
            assert(fids.size()==eids.size());
            for (size_t i = 0; i < fids.size(); i++)
            {
                size_t fid = fids[i];
                size_t eid = eids[i];
                size_t vid0 = vcg::tri::Index(mesh, mesh.face[fid].V0(eid));
                size_t vid1 = vcg::tri::Index(mesh, mesh.face[fid].V1(eid));
                edges.push_back({vid0, vid1});
            }
            
            const int N = edges.size();
            if (N == 0) return {};

            // Compute in-degree and out-degree
            std::unordered_map<size_t, int> indeg, outdeg;
            for (int i = 0; i < N; ++i) {
                auto [a, b] = edges[i];
                outdeg[a]++;
                indeg[b]++;
                // ensure keys exist for isolated nodes
                if (!indeg.count(a)) indeg[a] = 0;
                if (!outdeg.count(b)) outdeg[b] = 0;
            }

            // Find start vertex: indegree 0, outdegree 1
            size_t start = 0;
            bool found = false;
            for (auto& kv : outdeg) {
                size_t node = kv.first;
                if (indeg[node] == 0 && outdeg[node] == 1) {
                    start = node;
                    found = true;
                    break;
                }
            }

            if (!found) {
                std::cerr << "Error: no start vertex, graph may not be a single chain.\n";
                return {};
            }

            // Build mapping: from vertex -> outgoing edge index
            // (should be unique since it's a chain)
            std::unordered_map<size_t, int> nextEdge;
            for (int i = 0; i < N; ++i) {
                auto [a, b] = edges[i];
                nextEdge[a] = i; // record edge index
            }

            // Traverse chain
            _order.reserve(N);

            size_t curr = start;

            while (nextEdge.count(curr)) {
                int ei = nextEdge[curr];
                _order.push_back(ei);

                curr = edges[ei].second; // follow chain
            }

            return true;
        }

        void reorder(){
            vids.clear();
            assert(_order.size()==fids.size());
            std::vector<size_t> ordered_fids;
            std::vector<size_t> ordered_eids;
            for (size_t idx: _order)
            {
                ordered_fids.push_back(fids[idx]);
                ordered_eids.push_back(eids[idx]);
                vids.push_back(vcg::tri::Index(mesh, mesh.face[fids[idx]].V0(eids[idx])));
            }
            vids.push_back(vcg::tri::Index(mesh, mesh.face[fids[_order.back()]].V1(eids[_order.back()])));
            fids = ordered_fids;
            eids = ordered_eids;

            _order.clear();
            for (size_t i = 0; i < num_edges(); i++) _order.push_back(i);
        }

        void show_seam(){
            std::cout<<"show seam: ";
            for (auto idx : _order)
            {
                size_t fid = fids[idx];
                size_t eid = eids[idx];
                size_t v0id = vcg::tri::Index(mesh, mesh.face[fid].V0(eid));
                size_t v1id = vcg::tri::Index(mesh, mesh.face[fid].V1(eid));
                std::cout<<"(v0id, v1id)"<<"("<<v0id<<", "<<v1id<<")"<<" - ";
            }
            std::cout<<std::endl;
            
        }

        Seam(MeshType& _mesh):mesh(_mesh){};
    };

    struct Patch
    {
        MeshType& mesh;
        std::vector<size_t> fids;
        std::vector<Seam> seams;
        std::vector<int> seamOrder;

        CoordType patch_center(){
            CoordType totalCoord(0, 0, 0);
            for(auto fid: fids){
                for (size_t i = 0; i < 3; i++)
                {
                    totalCoord += mesh.face[fid].P(i)/3.;
                }
            }
            return totalCoord/fids.size();
        }

        Seam& get_seam(size_t sid){
            assert(sid< num_seams());
            return seams[sid];
        }

        size_t get_sid(std::vector<size_t>& _fids){
            // To-Do: Need to improve
            size_t i=0;
            for (auto&seam: seams)
            {

                size_t num = seam.contain(_fids);
                if (num>0 && (num==_fids.size() || num==fids.size())) return i;
                i++;
            }
            assert(false);
            return 0;
        }

        size_t num_seams(){
            return seams.size();
        }

        bool build_seams(std::vector<size_t>& fids, std::vector<size_t>& feids){
            assert(fids.size()==feids.size());
            if (fids.size()==0) return true;

            // Initialize edges:
            std::vector<std::pair<size_t, size_t>> edges;
            int i = 0;
            for (auto fid:fids)
            {
                size_t vid0 = vcg::tri::Index(mesh, mesh.face[fid].V0(feids[i]));
                size_t vid1 = vcg::tri::Index(mesh, mesh.face[fid].V1(feids[i]));
                edges.push_back(std::pair<size_t, size_t>(vid0, vid1));
                i++;
            }
            
            std::vector<std::vector<size_t>> sid2beindices;
            groupEdges2Seams(edges, sid2beindices);
            assert(sid2beindices.size()>0);

            for (size_t sid = 0; sid < sid2beindices.size(); sid++)
            {
                Seam seam(mesh);
                std::vector<size_t> subfids; std::vector<size_t> subfeids;
                for (auto j: sid2beindices[sid])
                {
                    subfids.push_back(fids[j]); subfeids.push_back(feids[j]);
                }
                
                bool succeed = seam.build(subfids, subfeids);
                assert(succeed);
                seams.push_back(seam);
            }
        }

        bool build(std::vector<size_t> & _fids, std::vector<int> & fid2paid){
            // Intput: fids inside a single patch.
            fids.clear();
            fids = _fids;
            // 1. classify fids into two types: seams fids; boarder fids;
            std::vector<std::vector<size_t>> sid2fid; std::vector<std::vector<size_t>> sid2feid;
            
            //Imtermediate Data
            // for seam fids:
            std::map<size_t, std::vector<size_t>> paid2fids;
            std::map<size_t, std::vector<size_t>> paid2feids;
            // for boarder fids:
            std::vector<size_t> bfids;
            std::vector<size_t> bfeids;

            // find boarder seams and joint seams.
            for (size_t fid: _fids)
            {
                size_t paid = fid2paid[fid];
                for (size_t j = 0; j < 3; j++)
                {
                    if (mesh.face[fid].IsB(j)){
                        bfids.push_back(fid);
                        bfeids.push_back(j);
                    }
                    else{
                        size_t nextfid=vcg::tri::Index(mesh, mesh.face[fid].FFp(j));
                        size_t nextpaid = fid2paid[nextfid];
                        if (nextpaid!=paid)
                        {
                            paid2fids[nextpaid].push_back(fid);
                            paid2feids[nextpaid].push_back(j);
                        }
                    }
                }
            }

            // 2. create seams for seam fids
            for (auto & [key, value]: paid2fids){
                bool succeed = build_seams(paid2fids[key], paid2feids[key]);
                assert(succeed);
            }

            // 3. create seams for boarder seams.
            build_seams(bfids, bfeids);

            return true;
        }

        bool get_neibor_sids(Seam & seam, std::vector<int> & sids){
            sids.clear();
            for (size_t sid = 0; sid < seams.size(); sid++)
            {
                std::unordered_set<size_t> inner_vids;
                Seam& inner_seam = seams[sid];

                for (size_t i = 0; i < inner_seam.fids.size(); i++)
                {
                    size_t oidx = inner_seam._order[i];
                    size_t vid0 = vcg::tri::Index(mesh, mesh.face[inner_seam.fids[oidx]].V0(inner_seam.eids[oidx]));
                    inner_vids.insert(vid0);
                }

                size_t oidx = inner_seam._order[inner_seam.fids.size()-1];
                size_t vid1 = vcg::tri::Index(mesh, mesh.face[inner_seam.fids[oidx]].V1(inner_seam.eids[oidx]));
                inner_vids.insert(vid1);
                
                std::unordered_set<size_t> outer_vids;
                for (size_t i = 0; i < seam.fids.size(); i++)
                {
                    size_t oidx = seam._order[i];
                    size_t vid0 = vcg::tri::Index(mesh, mesh.face[seam.fids[oidx]].V0(seam.eids[oidx]));
                    outer_vids.insert(vid0);
                }
                oidx = seam._order[seam.fids.size()-1];
                vid1 = vcg::tri::Index(mesh, mesh.face[seam.fids[oidx]].V1(seam.eids[oidx]));
                outer_vids.insert(vid1);

                // check A ⊆ B
                bool aInB = true;
                for (auto& x : inner_vids)
                    if (!outer_vids.count(x))
                        aInB = false;

                // check B ⊆ A
                bool bInA = true;
                for (auto& x : outer_vids)
                    if (!inner_vids.count(x))
                        bInA = false;

                if (bInA || aInB) sids.push_back(sid);
            }
            return !sids.empty();
        }

        void check_seams(){
            std::cout<<"-------------------"<<std::endl;
            for (size_t sid = 0; sid < seams.size(); sid++)
            {
                std::cout<<"seam Id: "<<sid<<std::endl;
                Seam& seam = seams[sid];
                std::cout<<"fids size: "<<seam.fids.size()<<", "<<"eids size: "<<seam.eids.size();
                for (size_t i = 0; i < seam.fids.size(); i++)
                {
                    std::cout<<"fid: "<<seam.fids[i]<<", "<<"eid: "<<seam.eids[i]<<" ";
                    size_t vid0 = vcg::tri::Index(mesh, mesh.face[seam.fids[i]].V0(seam.eids[i]));
                    size_t vid1 = vcg::tri::Index(mesh, mesh.face[seam.fids[i]].V1(seam.eids[i]));
                    std::cout<<"vid0: "<<vid0<<", "<<"vid1: "<<vid1 <<std::endl;
                }
            }
            std::cout<<"-------------------"<<std::endl;
        }

        Patch(MeshType& _mesh):mesh(_mesh){};
    };

    struct SeamEdge{
        int paid0;
        size_t sid0;
        int paid1;
        size_t sid1;

        SeamEdge (){}

        SeamEdge (int _paid0, size_t _sid0, int _paid1, size_t _sid1){

            if (_paid0 <= _paid1) {
                paid0 = _paid0; sid0 = _sid0;
                paid1 = _paid1; sid1 = _sid1;
            } else {
                paid0 = _paid1; sid0 = _sid1;
                paid1 = _paid0; sid1 = _sid0;
            }
        }

        bool operator==(const SeamEdge& other) const {
            return paid0 == other.paid0 && sid0 == other.sid0 &&
                paid1 == other.paid1 && sid1 == other.sid1;
        }

    };
    
    MeshType& mesh;
    std::vector<std::vector<size_t>> paid2fids;
    std::vector<int> fid2paid;
    std::vector<Patch> patches;
    std::vector<SeamEdge> seamEdges;
    std::vector<std::pair<int, int>> patchEdges;

    PatchGraph(){}

    PatchGraph(MeshType& m, std::vector<std::vector<size_t>> & partitions, std::vector<int> & totalPartitions):mesh(m), paid2fids(partitions), fid2paid(totalPartitions){
        assert(paid2fids.size()>0);
        assert(fid2paid.size()==m.FN());
        for (auto& fids: paid2fids)
        {
            assert(fids.size()>0);
        }
    }

    /* Primitive Member Function*/
    size_t add_Edge(size_t pid1, size_t psid1, size_t pid2, size_t psid2){
        // To-Do
        assert(pid1<patches.size());
        assert(pid2<patches.size());
        assert(psid1<patches[pid1].num_seams());
        assert(psid2<patches[pid2].num_seams());

        SeamEdge newEdge;
        if (pid1<pid2)
        {
            newEdge =  SeamEdge(pid1, psid1, pid2, psid2);
        }
        else{
            newEdge =  SeamEdge(pid2, psid2, pid1, psid1);
        }

        for (const auto& e : seamEdges) {
            if (e == newEdge) return seamEdges.size(); // already exists, do nothing
        }
        seamEdges.push_back(newEdge);
        return seamEdges.size();
    }

    size_t add_patchEdge(int paid0, int paid1){
        assert(paid0<patches.size());
        assert(paid1<patches.size());
        assert(paid0!=paid1);
        std::pair<int, int> candiEdge(std::min(paid0, paid1), std::max(paid0, paid1));
        if (patchEdges.empty()) patchEdges.push_back(candiEdge);
        for (auto& patchEdge:patchEdges)
        {
            if (candiEdge.first==patchEdge.first && candiEdge.second==patchEdge.second) return patchEdges.size();
        }
        patchEdges.push_back(candiEdge);
        return patchEdges.size();
    }

    std::vector<size_t> get_paid(std::vector<size_t>&fids){
        std::vector<size_t> paids;
        for (auto fid: fids)
        {
            paids.push_back(fid2paid[fid]);
        }
        return paids;
    }

    size_t num_patch(){
        return paid2fids.size();
    }

    /* Initialization Member Function*/
    bool InitPatches(){
        for (size_t i = 0; i < num_patch(); i++)
        {
            Patch patch(mesh);
            bool succeed = patch.build(paid2fids[i], fid2paid);
            if (!succeed) return false;
            patches.push_back(patch);
        }
        return true;
    }

    bool InitConnections(){
        assert(patches.size()==num_patch());
        for (size_t pid = 0; pid < num_patch(); pid++)
        {
            for (size_t sid = 0; sid < patches[pid].num_seams(); sid++)
            {

                Seam & seam = patches[pid].get_seam(sid);
                if (seam.isBoarder()) continue;
                std::vector<size_t> adjfids;
                seam.load_adjfids(adjfids);
                std::vector<size_t> adjpids = get_paid(adjfids);
                for (auto adjpid: adjpids)
                {
                    std::vector<int> adjsids;
                    patches[adjpid].get_neibor_sids(seam, adjsids);
                    for (auto adjsid: adjsids)
                    {
                        add_Edge(pid, sid, adjpid, adjsid);
                    }
                    add_patchEdge(pid, adjpid);
                }
            }
        }
    }

    /* Geometric Related Functions*/
    std::vector<CoordType> get_patchcenters(){
        std::vector<CoordType> pacenters;
        for (auto& pa: patches){
            pacenters.push_back(pa.patch_center());
        }
        return pacenters;
    }

};