/****************************************************************************
**
** Copyright (C) 2011 Nokia Corporation and/or its subsidiary(-ies).
** All rights reserved.
** Contact: Nokia Corporation (qt-info@nokia.com)
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of Nokia Corporation and its Subsidiary(-ies) nor
**     the names of its contributors may be used to endorse or promote
**     products derived from this software without specific prior written
**     permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
** $QT_END_LICENSE$
**
****************************************************************************/

#include <GL/glew.h>
#include <QMouseEvent>

#include <math.h>
#include <tracing/GL_mesh_drawing.h>
#include <tracing/mesh_type.h>
#include "myglwidget.h"
#include <wrap/qt/trackball.h>
#include <wrap/qt/anttweakbarMapper.h>
#include <wrap/gl/gl_field.h>
#include <wrap/gl/gl_geometry.h>
#include <qtimer.h>
#include <qfont.h>
#include <QGLWidget>
#include <QGuiApplication>
#include <QOpenGLTexture>

#include "trace_path_GL.h"
#include "wrap/qt/Outline2ToQImage.h"
#include <svg_exporter.h>
#include "parafashion.h"
#include "garment_session.h"
#include "manual_border_seam.h"
#include <wrap/gl/pick.h>
#include <wrap/gl/picking.h>
#include <vcg/complex/algorithms/closest.h>
#include <vcg/space/index/grid_static_ptr.h>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <memory>
#include <set>

#include <vcg/complex/algorithms/stat.h>
#include <vcg/complex/allocate.h>
#include <vcg/complex/algorithms/polygonal_algorithms.h>
//#include "parafashion_interface.h"

std::string pathRef="";
std::string pathDef="";
std::string pathFrames="";

vcg::Trackball track;//the active manipulator

bool drawfield=false;

// Single garment session backing the original parafashion UI.
GarmentSession gSession;
// Pointer to the garment session currently targeted by processing operations.
GarmentSession* gActiveSession = &gSession;

typename TraceMesh::CoordType &CenterDef = gSession.centerDef;
typename TraceMesh::CoordType &CenterRef = gSession.centerRef;
TraceMesh &deformed_mesh   = gSession.deformed_mesh;
TraceMesh &reference_mesh  = gSession.reference_mesh;
TraceMesh &half_def_mesh   = gSession.half_def_mesh;

TwBar *barFashion;

vcg::GlTrimesh<TraceMesh> glWrap;

vcg::GLField<TraceMesh> glField;

typedef typename TraceMesh::ScalarType ScalarType;
typedef typename TraceMesh::CoordType CoordType;
typedef typename TraceMesh::FaceType TraceFaceType;
typedef typename vcg::face::Pos<TraceFaceType> TracePosType;
typedef typename TraceMesh::VertexType	 VertexType;

int Iterations;
ScalarType EdgeStep;
bool drawRefMesh=true;
bool drawDefMesh=true;
bool drawSymmetryPlane=false;
bool parametrized=false;
bool colored_distortion=false;
bool draw3D=true;
bool textured=false;
bool drawParam=false;
bool matchcurvature=false;
bool &hasFrames = gSession.hasFrames;
bool drawConstraints=true;
bool drawPatchGraph=false;
bool drawSegmentUV = false;
bool drawApproximatePolygon = false;
bool drawArrangedApproxPolygonUV = false;

// Maximum number of mapping pairs to visualize (-1 = all)
int gMaxMappingPairs = -1;


int xMouse,yMouse;

int oldFrame=-1;
int selectedF=-1;

//typedef FieldSmoother<TraceMesh> FieldSmootherType;
//FieldSmootherType::SmoothParam FieldParam;
vcg::GLW::DrawMode drawmode=vcg::GLW::DMSmooth;     /// the current drawmode

bool do_rotate=false;
bool do_anim=false;
int curr_frame=0;

QTimer *timerRot;
QTimer *timerAnim;

bool HasTxt=false;

PathGL<TraceMesh> GPath; // MYNOTE: maintain pathes as vector of coordinates; reseponsible for draw path in GL.

//#define MAX_DIST 0.05
vcg::Similarityf trackView;

AnimationManager<TraceMesh> &AManager = *gSession.AManager;
Parafashion<TraceMesh>      &PFashion = *gSession.PFashion;

// GarmentSession implementation lives here to keep heavy symbols
// (from parafashion/trace_path) in a single translation unit.

GarmentSession::GarmentSession()
{
    AManager = new AnimationManager<MeshType>(deformed_mesh);
    PFashion = new Parafashion<MeshType>(deformed_mesh, reference_mesh, *AManager);
}

bool GarmentSession::loadMeshes(const std::string &defPath,
                                const std::string &refPath,
                                const std::string &framesPath)
{
    // Load deformed mesh.
    bool loaded = deformed_mesh.LoadMesh(defPath.c_str());
    if (!loaded) {
        std::cout << "Error loading deformed mesh: " << defPath << std::endl;
        return false;
    }
    deformed_mesh.UpdateAttributes();

    // Load reference mesh.
    std::string refPathResolved = refPath.empty() ? defPath : refPath;
    loaded = reference_mesh.LoadMesh(refPathResolved.c_str());
    if (!loaded) {
        std::cout << "Error loading reference mesh: " << refPathResolved << std::endl;
        return false;
    }
    reference_mesh.UpdateAttributes();

    // Center both meshes around the origin, storing original centers.
    CenterRef = reference_mesh.MoveCenterOnZero();
    CenterDef = deformed_mesh.MoveCenterOnZero();

    // Default base colors, consistent with the original viewer.
    vcg::tri::UpdateColor<MeshType>::PerFaceConstant(
        deformed_mesh, vcg::Color4b(220, 220, 220, 255));
    vcg::tri::UpdateColor<MeshType>::PerFaceConstant(
        reference_mesh, vcg::Color4b(255, 185, 15, 255));

    // Optional animation frames.
    hasFrames = false;
    if (!framesPath.empty()) {
        AManager->Init();
        std::cout << "Loading Frames from: " << framesPath << std::endl;
        hasFrames = AManager->LoadPosFrames(framesPath.c_str());
        if (!hasFrames) {
            std::cout << "Error loading frames" << std::endl;
            return false;
        }
        std::cout << "Frames loaded successfully" << std::endl;
    }

    // Initialize the parafashion pipeline for this garment.
    PFashion->Init();

    // Update bounding boxes after potential changes.
    vcg::tri::UpdateBounding<MeshType>::Box(deformed_mesh);
    vcg::tri::UpdateBounding<MeshType>::Box(reference_mesh);

    //
    meshPath = defPath;

    return true;
}

void GarmentSession::initMesh(){
    PFashion->Init();
}

//std::vector<std::vector<TracePosType > > TestPosSeq;

QImage SVGTxt;
GLuint layoutTxtIdx=0;
bool HasLayoutTxt=false;
bool has_to_update_layout=false;

#define COLOR_LIMITS 1//1.2

typedef typename vcg::Point2<ScalarType> UVCoordType;
std::vector<std::vector<UVCoordType > > UVPolyL;
std::vector<vcg::Color4b> Color;
std::vector<UVCoordType> Dots;
vcg::Color4b ColorDots;

QGLWidget *my_window;

void GLDrawPoints(const std::vector<CoordType> &DrawPos,
                  const ScalarType &GLSize,const vcg::Color4b &Col)
{
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glDisable(GL_LIGHTING);
    glDepthRange(0,0.9995);
    glPointSize(GLSize);

    for (size_t i=0;i<DrawPos.size();i++)
    {

        vcg::glColor(Col);
        glBegin(GL_POINTS);
        vcg::glVertex(DrawPos[i]);
        glEnd();
    }
    glPopAttrib();
}

void GLDrawPatchGraph(const std::vector<CoordType> &pcenters, 
                      const std::vector<std::pair<int, int>>& pEdges,
                    double nodeScale = 1.)
{

    // Scale the graph a bit.
    std::vector<CoordType> pcenters_scaled;
    for (size_t i = 0; i < pcenters.size(); i++)
    {
        pcenters_scaled.push_back(pcenters[i] * 1.2);
    }
    
    // Save current OpenGL attributes
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    // Draw edges (as red lines)
    glDisable(GL_LIGHTING);

    glLineWidth(5.);
    vcg::glColor(vcg::Color4b(255, 0, 0, 255));  // solid red

    glBegin(GL_LINES);
    for (const auto& e : pEdges) {
        int i0 = e.first;
        int i1 = e.second;
        if (i0 >= 0 && i0 < (int)pcenters_scaled.size() &&
            i1 >= 0 && i1 < (int)pcenters_scaled.size()) {
            vcg::glVertex(pcenters_scaled[i0]);
            vcg::glVertex(pcenters_scaled[i1]);
        }
    }
    glEnd();

    // Draw nodes (as small spheres)
    glEnable(GL_LIGHTING);
    vcg::glColor(vcg::Color4b(0, 255, 0, 255));  // green for nodes

    for (const auto& c : pcenters_scaled) {
        glPushMatrix();
        glTranslatef(c.X(), c.Y(), c.Z());
        vcg::glScale(1/nodeScale);
        glutSolidSphere(0.02, 24, 24);
        glPopMatrix();
    }

    // Restore previous OpenGL state
    glPopAttrib();
}

//void GlDrawPosSeq()
//{
//    glPushAttrib(GL_ALL_ATTRIB_BITS);
//    glDisable(GL_LIGHTING);
//    glDepthRange(0,0.9995);
//    glLineWidth(20);
//    glPointSize(40);

//    for (size_t i=0;i<TestPosSeq.size();i++)
//    {
//        vcg::Color4b col;
//        col=vcg::Color4b::Scatter(TestPosSeq.size(),i);
//        vcg::glColor(col);
//        glBegin(GL_LINES);
//        for (size_t j=0;j<TestPosSeq[i].size();j++)
//        {
//           CoordType P0=TestPosSeq[i][j].V()->P();
//           CoordType P1=TestPosSeq[i][j].VFlip()->P();
//           vcg::glVertex(P0);
//           vcg::glVertex(P1);
//        }
//        glEnd();
//    }

//    vcg::Color4b col=vcg::Color4b::Red;
//    vcg::glColor(col);
//    glBegin(GL_POINTS);
//    for (size_t i=0;i<deformed_mesh.vert.size();i++)
//    {
//        if (!deformed_mesh.vert[i].IsS())continue;
//        CoordType P=deformed_mesh.vert[i].P();
//        vcg::glVertex(P);
//    }
//    glEnd();

//    glPopAttrib();
//}

void UpdateSelectedFrameIfneeded()
{
    if (AManager.NumFrames()==0)return;
    if (oldFrame==selectedF)return;
    selectedF=selectedF % AManager.NumFrames();

    if (selectedF==0)
        deformed_mesh.RestoreRPos();
    else
        AManager.UpdateToFrame(selectedF,false,false);

    oldFrame=selectedF;

    my_window->update();
}

template <class ScalarType>
void GlDrawPlane(const vcg::Plane3<ScalarType> &Pl,
                 const ScalarType &size)
{
    typedef typename vcg::Point3<ScalarType> CoordType;
    CoordType p[4];
    p[0]=CoordType(-size,-size,0);
    p[1]=CoordType(size,-size,0);
    p[2]=CoordType(size,size,0);
    p[3]=CoordType(-size,size,0);
    CoordType N0=CoordType(0,0,1);
    CoordType N1=Pl.Direction();

    ///then get rotation matrix
    vcg::Matrix33<ScalarType> RotPl=vcg::RotationMatrix(N0,N1);
    ///then rotate-translate the points to math the right direction
    for (int i=0;i<4;i++)
    {
        p[i]=RotPl*p[i];
        CoordType Transl(0,0,0);
        Transl+=Pl.Direction()*Pl.Offset();
        p[i]+=Transl;
        //p[i]+=Center;
    }

    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glDisable(GL_LIGHTING);
    vcg::glColor(vcg::Color4b(255,0,0,255));
    glLineWidth(3);
    glBegin(GL_LINE_LOOP);
    for (int i=0;i<4;i++)
        vcg::glVertex(p[i]);
    glEnd();

    glDisable(GL_CULL_FACE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    glEnable(GL_BLEND);                //activate blending mode
    //glBlendFunc(GL_SRC_ALPHA,GL_ONE);  //define blending factors
    glBlendFunc(GL_SRC_ALPHA, GL_SRC_ALPHA);
    vcg::glColor(vcg::Color4b(255,0,0,100));

    glBegin(GL_QUADS);
    for (int i=0;i<4;i++)
        vcg::glVertex(p[i]);
    glEnd();

    glPopAttrib();
}

bool gGarmentColorGreyMode = false;

void ColorSessionNeutralGrey(GarmentSession &s)
{
    vcg::tri::UpdateColor<GarmentSession::MeshType>::PerFaceConstant(
        s.deformed_mesh, vcg::Color4b(220, 220, 220, 255));
}

void BuildPaidToMappedSegment(GarmentSession &s,
                              const std::vector<std::pair<int, int>> &pairs,
                              bool isGarmentA,
                              std::vector<int> &paid2seg)
{
    paid2seg.clear();
    if (!s.PFashion || !s.PFashion->pagraph || pairs.empty())
        return;

    auto &pg = *s.PFashion->pagraph;
    auto &segmentps = s.PFashion->segmentps;
    paid2seg.assign(pg.num_patch(), -1);

    for (const auto &pr : pairs) {
        const int seg = isGarmentA ? pr.first : pr.second;
        if (seg < 0 || static_cast<size_t>(seg) >= segmentps.size())
            continue;
        for (int paid : segmentps[seg]->paids) {
            if (paid >= 0 && static_cast<size_t>(paid) < paid2seg.size())
                paid2seg[paid] = seg;
        }
    }
}

void GLDrawSegmentEdges(GarmentSession &session,
                        vcg::Color4b col,
                        float lineWidth,
                        const std::vector<std::pair<int, int>> *mappingPairs,
                        bool isGarmentA)
{
    if (!session.PFashion || !session.PFashion->pagraph)
        return;

    auto &pg = *session.PFashion->pagraph;
    auto &segmentps = session.PFashion->segmentps;
    TraceMesh &mesh = session.deformed_mesh;

    if (pg.fid2paid.size() != mesh.face.size())
        return;

    std::vector<int> paid2seg;
    if (mappingPairs != nullptr && !mappingPairs->empty()) {
        BuildPaidToMappedSegment(session, *mappingPairs, isGarmentA, paid2seg);
    } else {
        paid2seg.assign(pg.num_patch(), -1);
        for (size_t si = 0; si < segmentps.size(); ++si) {
            for (int paid : segmentps[si]->paids) {
                if (paid >= 0 && static_cast<size_t>(paid) < paid2seg.size())
                    paid2seg[paid] = static_cast<int>(si);
            }
        }
    }

    auto segOfFace = [&](size_t fi) -> int {
        if (mesh.face[fi].IsD())
            return -1;
        const int paid = pg.fid2paid[fi];
        if (paid < 0 || static_cast<size_t>(paid) >= paid2seg.size())
            return -1;
        return paid2seg[paid];
    };

    using CoordType = TraceMesh::CoordType;
    using FaceType = TraceMesh::FaceType;

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glDisable(GL_LIGHTING);
    glDepthRange(0, 0.9999);
    glLineWidth(lineWidth);
    vcg::glColor(col);
    glBegin(GL_LINES);

    for (size_t fi = 0; fi < mesh.face.size(); ++fi) {
        if (mesh.face[fi].IsD())
            continue;
        const int segI = segOfFace(fi);
        for (int j = 0; j < 3; ++j) {
            bool drawEdge = false;
            if (vcg::face::IsBorder(mesh.face[fi], j)) {
                drawEdge = true;
            } else {
                FaceType *adj = mesh.face[fi].FFp(j);
                if (adj == nullptr) {
                    drawEdge = true;
                } else {
                    const ptrdiff_t fj = adj - &mesh.face[0];
                    if (fj < 0 || static_cast<size_t>(fj) >= mesh.face.size())
                        drawEdge = true;
                    else
                        drawEdge = (segI != segOfFace(static_cast<size_t>(fj)));
                }
            }
            // Do not use IsFaceEdgeS here: traced/manual seams can lie inside a segment.
            if (!drawEdge)
                continue;
            const CoordType pos0 = mesh.face[fi].P0(j);
            const CoordType pos1 = mesh.face[fi].P1(j);
            vcg::glVertex(pos0);
            vcg::glVertex(pos1);
        }
    }
    glEnd();
    glPopAttrib();
}

void GLDrawPatchEdges(vcg::Color4b col=vcg::Color4b(0,0,0,255),
                      ScalarType GLSize=5)
{
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glDisable(GL_LIGHTING);
    glDepthRange(0,0.9999);
    glLineWidth(GLSize);
    vcg::glColor(col);
    glBegin(GL_LINES);
    for (size_t i=0;i<deformed_mesh.face.size();i++)
    {
        //size_t IndexP0=deformed_mesh.face[i].Q();
        for (size_t j=0;j<3;j++)
        {
            bool drawEdge=false;
            drawEdge|=vcg::face::IsBorder(deformed_mesh.face[i],j);
            drawEdge|=deformed_mesh.face[i].IsFaceEdgeS(j);
            //            size_t IndexP1=deformed_mesh.face[i].FFp(j)->Q();
            //            drawEdge|=(IndexP1!=IndexP0);
            if (!drawEdge)continue;
            CoordType Pos0=deformed_mesh.face[i].P0(j);
            CoordType Pos1=deformed_mesh.face[i].P1(j);
            vcg::glVertex(Pos0);
            vcg::glVertex(Pos1);
        }
    }
    glEnd();
    glPopAttrib();
}

void MyGLWidget::GLDrawLegenda()
{
    vcg::Color4b col0=vcg::Color4b::ColorRamp(-COLOR_LIMITS,COLOR_LIMITS,-1);
    vcg::Color4b colM=vcg::Color4b::ColorRamp(-COLOR_LIMITS,COLOR_LIMITS,0);
    vcg::Color4b col1=vcg::Color4b::ColorRamp(-COLOR_LIMITS,COLOR_LIMITS,1);

    ScalarType sizeX=0.5;
    ScalarType sizeY=0.05;
    glPushMatrix();
    vcg::glTranslate(CoordType(0,-1,0));
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glDisable(GL_LIGHTING);

    //renderText(floor(-2*sizeX), floor(-2*sizeY), "<5% Compression", QFont("Arial", 16, QFont::Bold,false) );

    glBegin(GL_QUADS);
    //glColor(vcg::Color4b(0,0,255,255));
    glColor(col1);
    glVertex(CoordType(-sizeX,-sizeY,0));
    //glColor(vcg::Color4b(0,255,0,255));
    glColor(colM);
    glVertex(CoordType(0,-sizeY,0));
    glVertex(CoordType(0,sizeY,0));
    //glColor(vcg::Color4b(0,0,255,255));
    glColor(col1);
    glVertex(CoordType(-sizeX,sizeY,0));

    //glColor(vcg::Color4b(0,255,0,255));
    glColor(colM);
    glVertex(CoordType(0,-sizeY,0));
    //glColor(vcg::Color4b(255,0,0,255));
    glColor(col0);
    glVertex(CoordType(sizeX,-sizeY,0));
    glVertex(CoordType(sizeX,sizeY,0));
    //glColor(vcg::Color4b(0,255,0,255));
    glColor(colM);
    glVertex(CoordType(0,sizeY,0));
    glEnd();

    glPopAttrib();
    glPopMatrix();

    vcg::glColor(vcg::Color4b(0,0,0,255));

    std::string ComprString=std::to_string(((int)fabs(PFashion.max_compression*100)));
    ComprString+="% Compression";
    int Width=width();
    int Height=height();
    double posX=(double)Width*0.25;
    double posY=(double)Height*0.9;
    renderText(floor(posX), floor(posY), ComprString.c_str(), QFont("Arial", 16, QFont::Bold,false) );

    std::string TensionString=std::to_string(((int)fabs(PFashion.max_compression*100)));
    TensionString+="% Tension";
    posX=(double)Width*0.65;
    posY=(double)Height*0.9;
    renderText(floor(posX), floor(posY), TensionString.c_str(), QFont("Arial", 16, QFont::Bold,false) );
}

void DoColorByDistortion()
{
    if (!parametrized)return;
    //save old partitioning first
    std::vector<ScalarType> FaceQ;
    for (size_t i=0;i<deformed_mesh.face.size();i++)
        FaceQ.push_back(deformed_mesh.face[i].Q());

    if(PFashion.UVMode==PMCloth)
         Parametrizer<TraceMesh>::SetQasClothDistorsion(deformed_mesh);
    if(PFashion.UVMode==PMArap)
        vcg::tri::Distortion<TraceMesh,true>::SetQasDistorsion(deformed_mesh,vcg::tri::Distortion<TraceMesh,true>::ARAPDist);
    if(PFashion.UVMode==PMConformal)
        vcg::tri::Distortion<TraceMesh,true>::SetQasDistorsion(deformed_mesh,vcg::tri::Distortion<TraceMesh,true>::EdgeDist);

    //copy per vertex
    std::pair<ScalarType,ScalarType> test=vcg::tri::Stat<TraceMesh>::ComputePerFaceQualityMinMax(deformed_mesh);
    std::cout<<"MinV:"<<test.first<<std::endl;
    std::cout<<"MaxV:"<<test.second<<std::endl;
    vcg::tri::UpdateQuality<TraceMesh>::VertexFromFace(deformed_mesh);

    //    //clamp
    //    for (size_t i=0;i<deformed_mesh.vert.size();i++)
    //    {
    //        deformed_mesh.vert[i].Q()=std::min(deformed_mesh.vert[i].Q(),PFashion.max_tension);
    //        deformed_mesh.vert[i].Q()=std::max(deformed_mesh.vert[i].Q(),PFashion.max_);
    //    }

    colored_distortion=true;

    for (size_t i=0;i<deformed_mesh.vert.size();i++)
    {
        ScalarType Val=deformed_mesh.vert[i].Q();
        if (deformed_mesh.vert[i].Q()<=PFashion.max_compression)
        {
            deformed_mesh.vert[i].C()=vcg::Color4b::Blue;
            continue;
        }
        if (deformed_mesh.vert[i].Q()>=PFashion.max_tension)
        {
            deformed_mesh.vert[i].C()=vcg::Color4b::Red;
            continue;
        }
        deformed_mesh.vert[i].C()=vcg::Color4b::ColorRamp(PFashion.max_compression*COLOR_LIMITS,
                                                         PFashion.max_tension*COLOR_LIMITS,
                                                         Val);
    }

    //vcg::tri::UpdateColor<TraceMesh>::PerVertexQualityRamp(deformed_mesh);

    //vcg::tri::UpdateColor<TraceMesh>::PerVertexQualityRamp(deformed_mesh,MAX_DIST,-MAX_DIST);

    //restore old quality
    for (size_t i=0;i<deformed_mesh.face.size();i++)
    {
        deformed_mesh.face[i].Q()=FaceQ[i];
    }
}

void DoColorByPatch()
{
    //SelectPatchBorderEdges(deformed_mesh);
    MakePartitionOnQConsistent(deformed_mesh);
    deformed_mesh.ScatterColorByQualityFace();
}

void TW_CALL ColorByNone(void *)
{
    vcg::tri::UpdateColor<TraceMesh>::PerFaceConstant(deformed_mesh,vcg::Color4b(255,255,255,255));
}

void TW_CALL ColorByArapDist(void *)
{
    DoColorByDistortion();
}

void TW_CALL ColorByPatch(void *)
{
    DoColorByPatch();
    colored_distortion=false;
}

void TW_CALL InitByCurvature(void *)
{
//    DoColorByPatch();
//    colored_distortion=false;
}

void DoSymmetrize()
{
    PFashion.MakeMeshSymmetric(GPath.PickedPoints);
    //vcg::tri::io::ExporterPLY<TraceMesh>::Save(deformed_mesh,"symmetrized.ply");
}

void TW_CALL SymmetrizeDeformed(void *)
{
    DoSymmetrize();
    if (hasFrames)
    {
        AManager.UpdateProjectionBasis();
        //AManager.UpdateRestInfo();
        //AManager.UpdateCurvatureAndStretch();
    }
}

void DoParametrize()
{

    PFashion.DoParametrize();
    PFashion.GetUVSeamsPolylines(UVPolyL,Color,Dots,ColorDots);
}

void TW_CALL Parametrize(void *)
{
    DoParametrize();
    drawParam=true;
}

void DoTracePath()
{
    PFashion.TracePatch();
    if (hasFrames)
    {
        AManager.UpdateProjectionBasis();
        //AManager.UpdateRestInfo();
    }
    //TestPosSeq.clear();
}

void TW_CALL TracePath(void *)
{
    DoTracePath();
    //TestPosSeq.clear();
}

void DoGenerateSVG(std::string ProjM)
{
    //THEN SAVE THE PATCH DATA
    std::string pathPartitions=ProjM;
    pathPartitions=ProjM+"_patch.txt";
    FILE *F=fopen(pathPartitions.c_str(),"wt");
    assert(F!=NULL);
    fprintf(F,"%d\n",(int)deformed_mesh.face.size());
    for (size_t i=0;i<deformed_mesh.face.size();i++)
        fprintf(F,"%d\n",(int)deformed_mesh.face[i].Q());
    fclose(F);

    //SAVE THE SVG
    std::string pathPatch=ProjM;
    pathPatch=ProjM+"_patch.svg";
    std::string pathPatchPNG=ProjM;
    pathPatchPNG=ProjM+"_patch.png";
    float scale=1000;
    SvgExporter<TraceMesh>::ExportUVPolyline(deformed_mesh,
                                             pathPatch.c_str(),
                                             pathPatchPNG.c_str(),
                                             SVGTxt,
                                             scale);//,4,
                                             //PFashion.param_boundary*scale,
                                             //PFashion.param_boundary*scale*0.75);
    //prepare the texure
    HasLayoutTxt=true;
    has_to_update_layout=true;
}

void TW_CALL GenerateSVG(void *)
{
    std::string ProjM=pathDef;
    size_t indexExt=ProjM.find_last_of(".");
    ProjM=ProjM.substr(0,indexExt);
    DoGenerateSVG(ProjM);
}

void TW_CALL SaveDebugPatches(void *)
{
    PFashion.SaveDebugPatches(pathDef);
}

void TW_CALL SaveData(void *)
{
    PFashion.SaveDebugPatches(pathDef);

    //get the path
    std::string ProjM=pathDef;
    size_t indexExt=ProjM.find_last_of(".");
    ProjM=ProjM.substr(0,indexExt);
    std::string saveMeshName=ProjM+std::string("_patch.obj");

    //SAVE THE MESH
    TraceMesh saveM;
    vcg::tri::Append<TraceMesh,TraceMesh>::Mesh(saveM,deformed_mesh);


    vcg::tri::Clean<TraceMesh>::RemoveDuplicateVertex(saveM);

    for (size_t i=0;i<saveM.vert.size();i++)
        saveM.vert[i].P()+=CenterDef;

    saveM.UpdateAttributes();
    vcg::tri::io::ExporterOBJ<TraceMesh>::Save(saveM,saveMeshName.c_str(),
                                               vcg::tri::io::Mask::IOM_WEDGTEXCOORD|
                                               vcg::tri::io::Mask::IOM_FACECOLOR);

    //THEN SAVE THE PATCH DATA
    std::string pathPartitions=ProjM;
    pathPartitions=ProjM+"_patch.txt";
    FILE *F=fopen(pathPartitions.c_str(),"wt");
    assert(F!=NULL);
    fprintf(F,"%d\n",(int)deformed_mesh.face.size());
    for (size_t i=0;i<deformed_mesh.face.size();i++)
        fprintf(F,"%d\n",(int)deformed_mesh.face[i].Q());
    fclose(F);

    //    //SAVE THE SVG
    //    std::string pathPatch=ProjM;
    //    pathPatch=ProjM+"_patch.svg";
    //    std::string pathPatchPNG=ProjM;
    //    pathPatchPNG=ProjM+"_patch.png";
    //    float scale=1000;
    //    SvgExporter<TraceMesh>::ExportUVPolyline(deformed_mesh,
    //                                             pathPatch.c_str(),
    //                                             pathPatchPNG.c_str(),
    //                                             SVGTxt,
    //                                             scale,4,
    //                                             PFashion.param_boundary*scale,
    //                                             PFashion.param_boundary*scale*0.75);
    //    //prepare the texure
    //    HasLayoutTxt=true;
    //    has_to_update_layout=true;

    DoGenerateSVG(ProjM);

    //THEN save the mesh in UV
    std::string pathUV=ProjM;
    pathUV=ProjM+"_UV.txt";
    F=fopen(pathUV.c_str(),"wt");
    assert(F!=NULL);
    fprintf(F,"%d\n",(int)deformed_mesh.face.size());
    for (size_t i=0;i<deformed_mesh.face.size();i++)
    {
        vcg::Point2<ScalarType> T0=deformed_mesh.face[i].WT(0).P();
        vcg::Point2<ScalarType> T1=deformed_mesh.face[i].WT(1).P();
        vcg::Point2<ScalarType> T2=deformed_mesh.face[i].WT(2).P();
        fprintf(F,"%f,%f;%f,%f;%f,%f\n",
                T0.X(),T0.Y(),
                T1.X(),T1.Y(),
                T2.X(),T2.Y());
    }
    fclose(F);

}

void ExportPolygonsToStream(GarmentSession &s, std::ostream &fout)
{
    if (!s.PFashion || !s.PFashion->pagraph) return;
    auto &PF = *s.PFashion;
    fout << PF.pagraph->num_patch() << "\n";
    fout << PF.segmentps.size() << "\n";
    for (const auto segp : PF.segmentps) {
        for (const auto& paid : segp->paids)
            fout << paid << " ";
        fout << "\n";
        fout << segp->seams.size() << "\n";
        size_t start_idx = 0;
        for (auto i : segp->seamOrder) {
            auto& seam = segp->seams[i];
            fout << start_idx << " " << start_idx + seam.num_verts() - 1 << "\n";
            start_idx += seam.num_verts() - 1;
        }
        const size_t firstSeamIdx = segp->seamOrder.empty() ? 0 : segp->seamOrder.front();
        auto& seam0 = segp->seams[firstSeamIdx];
        VertexType* last_end_vertRef =
            segp->mesh.face[segp->org2newfid[seam0.fids.front()]].V0(seam0.eids.front());
        size_t last_end_vid = vcg::tri::Index(segp->mesh, last_end_vertRef);

        // Use reverted UVs (original garment UV space) for export, same as ExportPolygons.
        {
            vcg::Point2<ScalarType> uv = segp->revertUVvert(last_end_vertRef->T().P());
            fout << uv.X() << " " << uv.Y() << "\n";
        }

        for (auto i : segp->seamOrder) {
            auto& seam = segp->seams[i];
            size_t start_vid = segp->get_vid(seam.fids.front(), seam.eids.front());
            size_t end_vid   = segp->get_vid(seam.fids.back(),  seam.eids.back(),  true);

            if (last_end_vid == start_vid) {
                for (size_t k = 1; k < seam.fids.size(); k++) {
                    auto vertRef = segp->get_vRef(seam.fids[k], seam.eids[k]);
                    vcg::Point2<ScalarType> uv = segp->revertUVvert(vertRef->T().P());
                    fout << uv.X() << " " << uv.Y() << "\n";
                }
                auto endvRef = segp->get_vRef(seam.fids.back(), seam.eids.back(), true);
                vcg::Point2<ScalarType> uv = segp->revertUVvert(endvRef->T().P());
                fout << uv.X() << " " << uv.Y() << "\n";
                last_end_vid = end_vid;
            } else {
                for (size_t k = 1; k < seam.fids.size(); k++) {
                    size_t idx = seam.fids.size() - 1 - k;
                    auto vertRef = segp->get_vRef(seam.fids[idx], seam.eids[idx], true);
                    vcg::Point2<ScalarType> uv = segp->revertUVvert(vertRef->T().P());
                    fout << uv.X() << " " << uv.Y() << "\n";
                }
                auto endvRef = segp->get_vRef(seam.fids.front(), seam.eids.front());
                vcg::Point2<ScalarType> uv = segp->revertUVvert(endvRef->T().P());
                fout << uv.X() << " " << uv.Y() << "\n";
                last_end_vid = start_vid;
            }
        }
    }
}

void ExportPolygonsForSession(GarmentSession &s, const std::string &basePath)
{
    std::string saveMeshName = basePath + "_polygons.txt";
    std::ofstream fout(saveMeshName);
    if (!fout.is_open()) {
        std::cout << "ExportPolygonsForSession: could not open " << saveMeshName << std::endl;
        return;
    }
    ExportPolygonsToStream(s, fout);
    std::cout << "Exported polygons to " << saveMeshName << std::endl;
}

void ApplySegmentMappingColorsToMesh(GarmentSession &s,
                                     TraceMesh &mesh,
                                     const std::vector<std::pair<int,int>> &pairs,
                                     bool isGarmentA)
{
    if (!s.PFashion || !s.PFashion->pagraph || pairs.empty())
        return;
    auto &pg = *s.PFashion->pagraph;
    const size_t nf = mesh.face.size();
    if (pg.num_patch() == 0 || nf == 0)
        return;

    if (pg.fid2paid.size() != nf)
        return;

    std::vector<int> paid2seg;
    BuildPaidToMappedSegment(s, pairs, isGarmentA, paid2seg);

    const vcg::Color4b gray(220, 220, 220, 255);

    for (size_t f = 0; f < nf; f++) {
        const int paid = pg.fid2paid[f];
        int seg = (paid >= 0 && (size_t)paid < paid2seg.size()) ? paid2seg[paid] : -1;
        int pairIdx = -1;
        for (size_t k = 0; k < pairs.size(); k++) {
            const int segInGarment = isGarmentA ? pairs[k].first : pairs[k].second;
            if (segInGarment == seg) {
                pairIdx = static_cast<int>(k);
                break;
            }
        }
        if (pairIdx >= 0 &&
            (gMaxMappingPairs < 0 || pairIdx < gMaxMappingPairs))
            mesh.face[f].C() = ColorForPair(pairIdx);
        else
            mesh.face[f].C() = gray;
    }
}

void ApplySegmentMappingColors(GarmentSession &s,
                               const std::vector<std::pair<int,int>> &pairs,
                               bool isGarmentA)
{
    ApplySegmentMappingColorsToMesh(s, s.deformed_mesh, pairs, isGarmentA);
}

TraceMesh::ScalarType SegmentApproxPolygonCenterX(GarmentSession &s, int segIndex)
{
    if (!s.PFashion || !s.PFashion->pagraph)
        return TraceMesh::ScalarType(0);

    auto &segmentps = s.PFashion->segmentps;
    if (segIndex < 0 || static_cast<size_t>(segIndex) >= segmentps.size())
        return TraceMesh::ScalarType(0);

    auto &poly = segmentps[segIndex]->polygon.polyVs;
    if (poly.empty())
        return TraceMesh::ScalarType(0);

    TraceMesh::ScalarType sumx = 0;
    for (const auto &p : poly)
        sumx += segmentps[segIndex]->revertUVvert(p).X();
    return sumx / TraceMesh::ScalarType(poly.size());
}

int SessionNumSegments(GarmentSession &s)
{
    if (!s.PFashion || !s.PFashion->pagraph)
        return 0;
    return static_cast<int>(s.PFashion->segmentps.size());
}

int SessionNumPatches(GarmentSession &s)
{
    if (!s.PFashion || !s.PFashion->pagraph)
        return 0;
    return static_cast<int>(s.PFashion->pagraph->num_patch());
}

bool SessionFacePatchId(GarmentSession &s, size_t faceIndex, int &patchIdOut)
{
    patchIdOut = -1;
    if (!s.PFashion || !s.PFashion->pagraph)
        return false;
    auto &pg = *s.PFashion->pagraph;
    if (faceIndex >= pg.fid2paid.size())
        return false;
    patchIdOut = pg.fid2paid[faceIndex];
    return (patchIdOut >= 0);
}

bool SessionSegmentPaids(GarmentSession &s, int segIndex, std::vector<int> &outPaids)
{
    outPaids.clear();
    if (!s.PFashion)
        return false;
    auto &segmentps = s.PFashion->segmentps;
    if (segIndex < 0 || static_cast<size_t>(segIndex) >= segmentps.size())
        return false;
    outPaids = segmentps[segIndex]->paids;
    return !outPaids.empty();
}

bool SessionSegmentFacePatchId(GarmentSession &s, int segIndex, size_t segFaceIndex, int &patchIdOut)
{
    patchIdOut = -1;
    if (!s.PFashion || !s.PFashion->pagraph)
        return false;
    auto &segmentps = s.PFashion->segmentps;
    if (segIndex < 0 || static_cast<size_t>(segIndex) >= segmentps.size())
        return false;
    auto *segp = segmentps[segIndex];
    if (segFaceIndex >= segp->mesh.face.size() || segFaceIndex >= segp->new2orgfid.size())
        return false;
    const size_t orgFid = segp->new2orgfid[segFaceIndex];
    auto &pg = *s.PFashion->pagraph;
    if (orgFid >= pg.fid2paid.size())
        return false;
    patchIdOut = pg.fid2paid[orgFid];
    return (patchIdOut >= 0);
}

int SessionMappingPairIndexForSegment(GarmentSession &s,
                                      const std::vector<std::pair<int, int>> &pairs,
                                      int segIndex,
                                      bool isGarmentA)
{
    (void)s;
    if (segIndex < 0 || pairs.empty())
        return -1;
    for (size_t k = 0; k < pairs.size(); ++k) {
        if (gMaxMappingPairs >= 0 && static_cast<int>(k) >= gMaxMappingPairs)
            break;
        const int segInGarment = isGarmentA ? pairs[k].first : pairs[k].second;
        if (segInGarment == segIndex)
            return static_cast<int>(k);
    }
    return -1;
}

TraceMesh* SessionSegmentMesh(GarmentSession &s, int segIndex)
{
    if (!s.PFashion || !s.PFashion->pagraph)
        return nullptr;
    auto &segmentps = s.PFashion->segmentps;
    if (segIndex < 0 || static_cast<size_t>(segIndex) >= segmentps.size())
        return nullptr;
    return &segmentps[segIndex]->mesh;
}

void SessionRevertSegmentUVToGarment(GarmentSession &s, int segIndex, TraceMesh &mesh)
{
    if (!s.PFashion || !s.PFashion->pagraph)
        return;
    auto &segmentps = s.PFashion->segmentps;
    if (segIndex < 0 || static_cast<size_t>(segIndex) >= segmentps.size())
        return;

    using ScalarType = TraceMesh::ScalarType;
    auto *segp = segmentps[segIndex];

    // Update per-vertex UVs: take current arranged UVs in mesh, map back to garment UVs.
    for (size_t i = 0; i < mesh.vert.size(); ++i) {
        // mesh.vert[i].T().P() holds the arranged UV; apply segment's inverse Similarity2.
        vcg::Point2<ScalarType> uvArr = mesh.vert[i].T().P();
        vcg::Point2<ScalarType> uvGarment = segp->revertUVvert(uvArr);
        mesh.vert[i].T().P() = uvGarment;
    }

    // Propagate vertex UVs to per-wedge UVs so that WT().P() matches T().P().
    for (size_t fi = 0; fi < mesh.face.size(); ++fi) {
        auto &f = mesh.face[fi];
        if (f.IsD()) continue;
        for (int j = 0; j < 3; ++j) {
            auto *v = f.V(j);
            if (!v) continue;
            f.WT(j).P() = v->T().P();
        }
    }
}

bool SessionApproxPolygonVerts(GarmentSession &s,
                               int segIndex,
                               std::vector<vcg::Point2<TraceMesh::ScalarType>> &outVerts)
{
    outVerts.clear();
    if (!s.PFashion)
        return false;
    auto &segmentps = s.PFashion->segmentps;
    if (segIndex < 0 || static_cast<size_t>(segIndex) >= segmentps.size())
        return false;
    const auto &polyVs = segmentps[segIndex]->polygon.polyVs;
    if (polyVs.size() < 3)
        return false;
    outVerts = polyVs;
    return true;
}

bool SessionApproxPolygonGarmentUV(GarmentSession &s,
                                   int segIndex,
                                   std::vector<vcg::Point2<TraceMesh::ScalarType>> &outUV)
{
    if (!SessionApproxPolygonVerts(s, segIndex, outUV))
        return false;
    auto *segp = s.PFashion->segmentps[segIndex];
    for (auto &p : outUV)
        p = segp->revertUVvert(p);
    return outUV.size() >= 3;
}

void TransformApproxPolygonWithPlacement(
    const std::vector<vcg::Point2<TraceMesh::ScalarType>> &canonical,
    const ApproxPolygonPlacement &placement,
    std::vector<vcg::Point2<TraceMesh::ScalarType>> &outGarmentUV)
{
    using ScalarType = TraceMesh::ScalarType;
    outGarmentUV = canonical;
    const int numVerts = static_cast<int>(outGarmentUV.size());
    if (numVerts < 3)
        return;

    if (placement.flip) {
        ScalarType cx = ScalarType(0);
        for (int i = 0; i < numVerts; ++i)
            cx += outGarmentUV[static_cast<size_t>(i)].X();
        cx /= ScalarType(numVerts);
        for (int i = 0; i < numVerts; ++i) {
            auto &p = outGarmentUV[static_cast<size_t>(i)];
            p.X() = ScalarType(2) * cx - p.X();
        }
    }

    const ScalarType c = static_cast<ScalarType>(std::cos(placement.rot));
    const ScalarType s = static_cast<ScalarType>(std::sin(placement.rot));
    const ScalarType tx = static_cast<ScalarType>(placement.tx);
    const ScalarType ty = static_cast<ScalarType>(placement.ty);

    for (int i = 0; i < numVerts; ++i) {
        auto &p = outGarmentUV[static_cast<size_t>(i)];
        const ScalarType x = p.X();
        const ScalarType y = p.Y();
        p.X() = c * x - s * y + tx;
        p.Y() = s * x + c * y + ty;
    }
}

void ScalePolygonVertsInPlace(std::vector<vcg::Point2<TraceMesh::ScalarType>> &verts,
                              TraceMesh::ScalarType linearScale)
{
    for (auto &p : verts) {
        p.X() *= linearScale;
        p.Y() *= linearScale;
    }
}

std::string MappingColorToFileStem(const vcg::Color4b &color, int pairIndex)
{
    char buf[64];
    std::snprintf(buf,
                  sizeof(buf),
                  "pair%03d_R%03u_G%03u_B%03u",
                  pairIndex,
                  static_cast<unsigned>(color[0]),
                  static_cast<unsigned>(color[1]),
                  static_cast<unsigned>(color[2]));
    return std::string(buf);
}

double SessionGarmentMeshArea(GarmentSession &s)
{
    return vcg::tri::Stat<TraceMesh>::ComputeMeshArea(s.deformed_mesh);
}

bool WriteApproxPolygonTxt(const std::string &path,
                           const std::vector<vcg::Point2<TraceMesh::ScalarType>> &verts)
{
    if (verts.size() < 3)
        return false;

    std::ofstream fout(path);
    if (!fout.is_open())
        return false;

    fout << verts.size() << "\n";
    for (const auto &p : verts)
        fout << p.X() << " " << p.Y() << "\n";
    return true;
}

namespace {

bool BuildTraceMeshFromPolygon2D(TraceMesh &mesh,
                                 const std::vector<vcg::Point2<TraceMesh::ScalarType>> &poly,
                                 const vcg::Color4b &faceColor)
{
    mesh.Clear();
    const size_t n = poly.size();
    if (n < 3)
        return false;

    using ScalarType = TraceMesh::ScalarType;
    using CoordType = TraceMesh::CoordType;

    vcg::tri::Allocator<TraceMesh>::AddVertices(mesh, n);
    for (size_t i = 0; i < n; ++i) {
        mesh.vert[i].P() = CoordType(poly[i].X(), poly[i].Y(), ScalarType(0));
        mesh.vert[i].T().P() = poly[i];
    }

    const size_t numFaces = n - 2;
    vcg::tri::Allocator<TraceMesh>::AddFaces(mesh, numFaces);
    for (size_t fi = 0; fi < numFaces; ++fi) {
        auto &f = mesh.face[fi];
        f.V(0) = &mesh.vert[0];
        f.V(1) = &mesh.vert[fi + 1];
        f.V(2) = &mesh.vert[fi + 2];
        f.C() = faceColor;
        for (int j = 0; j < 3; ++j)
            f.WT(j).P() = f.V(j)->T().P();
    }
    mesh.UpdateAttributes();
    return true;
}

} // namespace

void ArrangeTraceMeshesUVLayout(std::vector<TraceMesh *> &meshes)
{
    if (meshes.empty())
        return;

    using ScalarType = TraceMesh::ScalarType;
    ScalarType vertArea = ScalarType(0);
    for (TraceMesh *mp : meshes) {
        if (mp != nullptr)
            vertArea += vcg::tri::UV_Utils<TraceMesh>::PerVertUVArea(*mp);
    }

    const ScalarType interDist =
        math::Sqrt(vertArea / ScalarType(meshes.size())) * ScalarType(0.03);
    PatchManager<TraceMesh>::ArrangeUVPatches(meshes, interDist, false);
    for (TraceMesh *mp : meshes) {
        if (mp != nullptr)
            MeshDrawing<TraceMesh>::SyncVertexUVToWedgeUV(*mp);
    }
}

void GLDrawArrangedPolygonPatches(const std::vector<Polygon2DDrawItem> &items)
{
    if (items.empty())
        return;

    std::vector<std::unique_ptr<TraceMesh>> meshes;
    meshes.reserve(items.size());
    std::vector<TraceMesh *> meshps;
    meshps.reserve(items.size());

    for (const auto &item : items) {
        auto m = std::make_unique<TraceMesh>();
        if (!BuildTraceMeshFromPolygon2D(*m, item.verts, item.color))
            continue;
        meshps.push_back(m.get());
        meshes.push_back(std::move(m));
    }
    if (meshps.empty())
        return;

    ArrangeTraceMeshesUVLayout(meshps);
    MeshDrawing<TraceMesh>::GLDrawArrangedUVPatches(meshps);
}

void DrawArrangedApproxPolygonsFromSession(
    GarmentSession &s,
    const std::vector<std::pair<int, int>> *mappingPairs,
    bool isGarmentA)
{
    if (!s.PFashion)
        return;

    const vcg::Color4b gray(220, 220, 220, 255);
    std::vector<Polygon2DDrawItem> items;
    auto &segmentps = s.PFashion->segmentps;
    items.reserve(segmentps.size());

    for (size_t si = 0; si < segmentps.size(); ++si) {
        const auto &polyVs = segmentps[si]->polygon.polyVs;
        if (polyVs.size() < 3)
            continue;

        int pairIdx = -1;
        if (mappingPairs != nullptr) {
            for (size_t k = 0; k < mappingPairs->size(); ++k) {
                const int segInGarment =
                    isGarmentA ? (*mappingPairs)[k].first : (*mappingPairs)[k].second;
                if (segInGarment == static_cast<int>(si)) {
                    pairIdx = static_cast<int>(k);
                    break;
                }
            }
        } else {
            pairIdx = static_cast<int>(si);
        }

        Polygon2DDrawItem item;
        item.verts = polyVs;
        if (pairIdx >= 0 &&
            (gMaxMappingPairs < 0 || pairIdx < gMaxMappingPairs))
            item.color = ColorForPair(pairIdx);
        else
            item.color = gray;
        items.push_back(std::move(item));
    }

    GLDrawArrangedPolygonPatches(items);
}


// Check that, for a given garment session, all patches are covered by
// segments that appear in at least one mapping pair.
bool CheckPatchCoverageForSession(GarmentSession &s,
                                  const std::vector<std::pair<int,int>> &pairs,
                                  bool useFirstIndex)
{
    if (!s.PFashion || !s.PFashion->pagraph) return false;
    auto &pg = *s.PFashion->pagraph;
    auto &segmentps = s.PFashion->segmentps;

    const size_t numPatch = pg.num_patch();
    std::vector<bool> covered(numPatch, false);

    for (const auto &pr : pairs) {
        int seg = useFirstIndex ? pr.first : pr.second;
        if (seg < 0 || static_cast<size_t>(seg) >= segmentps.size())
            continue;
        const auto &paids = segmentps[seg]->paids;
        for (int paid : paids) {
            if (paid >= 0 && static_cast<size_t>(paid) < numPatch)
                covered[paid] = true;
        }
    }

    size_t missing = 0;
    for (size_t p = 0; p < numPatch; ++p)
        if (!covered[p]) ++missing;

    if (missing > 0) {
        std::cout << "[SegmentMapping] Warning: " << missing
                  << " patches are not covered by any mapped segment"
                  << (useFirstIndex ? " (left garment)." : " (right garment).")
                  << std::endl;
        return false;
    }
    return true;
}

void ColorSessionByPatch(GarmentSession &s)
{
    if (!s.PFashion) return;
    MakePartitionOnQConsistent(s.deformed_mesh);
    s.deformed_mesh.ScatterColorByQualityFace();
}

static void ClearPatchGraphArtifacts(Parafashion<TraceMesh> &pf)
{
    if (pf.pagraph != nullptr) {
        delete pf.pagraph;
        pf.pagraph = nullptr;
    }
    for (auto *seg : pf.segmentps)
        delete seg;
    pf.segmentps.clear();
}

void RebuildGarmentSegments(GarmentSession &session)
{
    if (!session.PFashion || !session.PFashion->pagraph)
        return;

    Parafashion<TraceMesh> &pf = *session.PFashion;
    for (auto *seg : pf.segmentps)
        delete seg;
    pf.segmentps.clear();

    const std::vector<std::vector<int>> *manualSets = session.manualSegmentPatchSets.empty()
        ? nullptr
        : &session.manualSegmentPatchSets;
    pf.BuildSegments(manualSets);
    ColorSessionManualSegmentPicks(session);
    std::cout << "Rebuilt segments: " << pf.segmentps.size()
              << " (manual candidates: " << session.manualSegmentPatchSets.size() << ")"
              << std::endl;
}

void RebuildGarmentPatchesFromSeams(GarmentSession &session)
{
    if (!session.PFashion)
        return;

    Parafashion<TraceMesh> &pf = *session.PFashion;
    ClearPatchGraphArtifacts(pf);
    pf.BuildPatchGraph();
    RebuildGarmentSegments(session);
}

int PickPatchIdAtScreen(GarmentSession &session, int screenX, int screenY)
{
    if (!session.PFashion || !session.PFashion->pagraph)
        return -1;

    TraceMesh &mesh = session.deformed_mesh;
    mesh.UpdateAttributes();
    auto &pg = *session.PFashion->pagraph;
    if (pg.fid2paid.size() != mesh.face.size())
        return -1;

    // Same depth unproject as border-seam picking (works in both dual viewports).
    TraceMesh::CoordType pickPos;
    if (!vcg::Pick(screenX, screenY, pickPos))
        return -1;

    typedef TraceMesh::FaceType FaceType;
    typedef FaceType *FacePointer;

    vcg::GridStaticPtr<FaceType, TraceMesh::ScalarType> faceGrid;
    faceGrid.Set(mesh.face.begin(), mesh.face.end());

    TraceMesh::ScalarType minDist = 0;
    TraceMesh::CoordType closestPt;
    const TraceMesh::ScalarType maxDist = mesh.bbox.Diag() * TraceMesh::ScalarType(0.25);
    FacePointer pickedFace = vcg::tri::GetClosestFaceBase(
        mesh, faceGrid, pickPos, maxDist, minDist, closestPt);
    if (pickedFace == nullptr)
        return -1;

    const size_t fid = vcg::tri::Index(mesh, pickedFace);
    if (fid >= pg.fid2paid.size())
        return -1;

    const int paid = pg.fid2paid[fid];
    if (paid < 0 || static_cast<size_t>(paid) >= pg.num_patch())
        return -1;
    return paid;
}

bool TogglePatchInManualSegmentPick(GarmentSession &session, int patchId)
{
    if (patchId < 0)
        return false;

    auto &picks = session.manualSegmentPickInProgress;
    auto it = std::find(picks.begin(), picks.end(), patchId);
    if (it != picks.end())
        picks.erase(it);
    else
        picks.push_back(patchId);

    ColorSessionManualSegmentPicks(session);
    return true;
}

bool CommitManualSegmentCandidate(GarmentSession &session)
{
    if (session.manualSegmentPickInProgress.empty())
        return false;

    std::vector<int> paids = session.manualSegmentPickInProgress;
    std::sort(paids.begin(), paids.end());
    paids.erase(std::unique(paids.begin(), paids.end()), paids.end());
    session.manualSegmentPatchSets.push_back(paids);
    session.manualSegmentPickInProgress.clear();
    RebuildGarmentSegments(session);
    return true;
}

void ClearManualSegmentPickInProgress(GarmentSession &session)
{
    session.manualSegmentPickInProgress.clear();
    ColorSessionManualSegmentPicks(session);
}

void ClearManualSegmentCandidates(GarmentSession &session)
{
    session.manualSegmentPatchSets.clear();
    session.manualSegmentPickInProgress.clear();
    RebuildGarmentSegments(session);
}

void GetGarmentSessionTensionParams(const GarmentSession &session,
                                    double &maxCompression,
                                    double &maxTension)
{
    maxCompression = -0.05;
    maxTension = 0.03;
    if (!session.PFashion)
        return;
    maxCompression = session.PFashion->max_compression;
    maxTension = session.PFashion->max_tension;
}

void SetGarmentSessionTensionParams(GarmentSession &session,
                                    double maxCompression,
                                    double maxTension)
{
    if (!session.PFashion)
        return;
    session.PFashion->max_compression = maxCompression;
    session.PFashion->max_tension = maxTension;
}

void ColorSessionManualSegmentPicks(GarmentSession &session)
{
    if (!session.PFashion || !session.PFashion->pagraph)
        return;

    TraceMesh &mesh = session.deformed_mesh;
    auto &pg = *session.PFashion->pagraph;
    const size_t np = pg.num_patch();
    if (np == 0 || pg.fid2paid.size() != mesh.face.size())
        return;

    std::vector<vcg::Color4b> patchColors(np);
    for (size_t i = 0; i < np; ++i)
        patchColors[i] = vcg::Color4b::Scatter((int)i, 0, 255);

    std::set<int> inProgress(session.manualSegmentPickInProgress.begin(),
                             session.manualSegmentPickInProgress.end());
    std::set<int> committed;
    for (const auto &paids : session.manualSegmentPatchSets)
        for (int paid : paids)
            committed.insert(paid);

    const vcg::Color4b neutralGrey(220, 220, 220, 255);

    for (size_t fi = 0; fi < mesh.face.size(); ++fi)
    {
        if (mesh.face[fi].IsD())
            continue;
        const int paid = pg.fid2paid[fi];
        if (paid < 0 || static_cast<size_t>(paid) >= np)
            continue;
        if (inProgress.count(paid))
            mesh.face[fi].C() = vcg::Color4b(255, 220, 0, 255);
        else if (committed.count(paid))
            mesh.face[fi].C() = vcg::Color4b(255, 80, 80, 255);
        else
            mesh.face[fi].C() = gGarmentColorGreyMode ? neutralGrey : patchColors[paid];
    }
}

bool AddManualBorderSeamToSession(GarmentSession &session,
                                  size_t borderVertex0,
                                  size_t borderVertex1)
{
    if (!session.PFashion)
        return false;

    TraceMesh &mesh = session.deformed_mesh;
    mesh.UpdateAttributes();

    std::vector<size_t> vertexPath;
    if (!ComputeGeodesicVertexPath(mesh, borderVertex0, borderVertex1, vertexPath))
        return false;

    // PathUI pipeline: complete + smooth, then use smoothed geometry for final seams.
    RefineAndSmoothVertexPath(mesh, vertexPath);
    ResnapVertexPathFromSmoothedPositions(mesh, vertexPath);
    CompleteVertexPath(mesh, vertexPath);

    session.manualBorderSeamPolylines.push_back(VertexPathToPolyline(mesh, vertexPath));
    mesh.UpdateAttributes();

    if (!ApplyVertexPathAsCuttingSeam(mesh, vertexPath))
        return false;

    session.manualSeamVertexPaths.push_back(vertexPath);
    RebuildGarmentPatchesFromSeams(session);
    return true;
}

void MergeManualSeamsIntoMesh(GarmentSession &session)
{
    if (!session.PFashion || session.manualSeamVertexPaths.empty())
        return;

    TraceMesh &mesh = session.deformed_mesh;
    mesh.UpdateAttributes();
    for (const std::vector<size_t> &vertexPath : session.manualSeamVertexPaths)
        ApplyVertexPathAsCuttingSeam(mesh, vertexPath);

    session.manualBorderSeamPolylines.clear();
    for (const std::vector<size_t> &vertexPath : session.manualSeamVertexPaths)
        session.manualBorderSeamPolylines.push_back(
            VertexPathToPolyline(mesh, vertexPath));

    RebuildGarmentPatchesFromSeams(session);
}

void TW_CALL ExportPolygons(void *){
    std::string ProjM=gActiveSession->meshPath;
    size_t indexExt=ProjM.find_last_of(".");
    ProjM=ProjM.substr(0,indexExt);
    std::string saveMeshName=ProjM+std::string("_polygons.txt");

    std::cout<<saveMeshName<<std::endl;
    std::ofstream fout(saveMeshName);
    if (!fout.is_open()) assert(false);


    // num patch -> num_segments -> [[paids] -> num_seams -> [(start_vidx, end_vidx)]] * nums_segment
    fout << ((gActiveSession->PFashion)->pagraph)->num_patch() << "\n";
    fout << (gActiveSession->PFashion)->segmentps.size() << "\n";

    for (const auto segp : (gActiveSession->PFashion)->segmentps) {
        //Org implementation.
        // for (const auto&paid : segp->paids)
        // {
        //     fout << paid<<" ";
        // }
        // fout<<"\n";
        
        // std::cout<< "Polygon: "<< segp->polygon.polyVs.size() << "\n";
        // for (const auto& p : segp->polygon.polyVs) {
        //     std::cout << p.X() << " " << p.Y() << "\n";
        // }

        for (const auto&paid : segp->paids)
        {
            fout << paid<<" ";
        }
        fout<<"\n";

        // Num seams:
        fout << segp->seams.size() << "\n";
        size_t start_idx=0;
        for (auto i: segp->seamOrder)
        {
            auto& seam = segp->seams[i];
            // Vert Index: (start end)
            fout << start_idx <<" "<< start_idx + seam.num_verts() - 1 << "\n";
            start_idx += seam.num_verts() - 1;
        }

        // Verts:
        const size_t firstSeamIdx = segp->seamOrder.empty() ? 0 : segp->seamOrder.front();
        auto& seam0 = segp->seams[firstSeamIdx];
        VertexType* last_end_vertRef = segp->mesh.face[segp->org2newfid[seam0.fids.front()]].V0(seam0.eids.front());
        size_t last_end_vid = vcg::tri::Index(segp->mesh, last_end_vertRef);
        // fout << last_end_vertRef->T().P().X() << " " << last_end_vertRef->T().P().Y() << "\n";
        vcg::Point2<ScalarType> _uvVert = segp->revertUVvert(last_end_vertRef->T().P());
        fout << _uvVert.X() << " " << _uvVert.Y() << "\n";        
        for (auto i: segp->seamOrder)
        {
            auto& seam = segp->seams[i];
            size_t start_vid = segp->get_vid(seam.fids.front(), seam.eids.front());
            size_t end_vid = segp->get_vid(seam.fids.back(), seam.eids.back(), true);

            // Debug
            VertexType* startVert = segp->get_vRef(seam.fids.front(), seam.eids.front());
            VertexType* endVert = segp->get_vRef(seam.fids.back(), seam.eids.back(), true);
            // std::cout<<"Seam id: "<<i<<std::endl;
            // std::cout<<"start v: "<< startVert->T().P().X() << " " << startVert->T().P().Y() << "\n";
            // std::cout<<"end v: "<< endVert->T().P().X() << " " << endVert->T().P().Y() << "\n";
            // // seam.load_endvIds(endvids);
            // points2D.push_back(startVert->T().P());
            // points2D.push_back(endVert->T().P());

            if (last_end_vid==start_vid)
            {
                for (size_t i = 1; i < seam.fids.size(); i++)
                {
                    auto vertRef = segp->get_vRef(seam.fids[i], seam.eids[i]);
                    vcg::Point2<ScalarType> _uvVert = segp->revertUVvert(vertRef->T().P());
                    fout << _uvVert.X() << " " << _uvVert.Y() << "\n";
                }
                auto endvRef = segp->get_vRef(seam.fids.back(), seam.eids.back(), true);
                vcg::Point2<ScalarType> _uvVert = segp->revertUVvert(endvRef->T().P());
                fout << _uvVert.X() << " " << _uvVert.Y() << "\n";
                last_end_vid = end_vid;
            }
            else{
                for (size_t i = 1; i < seam.fids.size(); i++)
                {
                    size_t idx = seam.fids.size() - 1 - i;
                    auto vertRef = segp->get_vRef(seam.fids[idx], seam.eids[idx], true);
                    vcg::Point2<ScalarType> _uvVert = segp->revertUVvert(vertRef->T().P());
                    fout << _uvVert.X() << " " << _uvVert.Y() << "\n";
                }
                auto endvRef = segp->get_vRef(seam.fids.front(), seam.eids.front());
                vcg::Point2<ScalarType> _uvVert = segp->revertUVvert(endvRef->T().P());
                fout << _uvVert.X() << " " << _uvVert.Y() << "\n";
                last_end_vid = start_vid;
            }
        }
    }
}

void DoSmoothField()
{
    PFashion.ComputeField();
    if (hasFrames)
    {
        AManager.UpdateProjectionBasis();
        //AManager.UpdateRestInfo();
        //AManager.UpdateCurvatureAndStretch();
    }
}

void TW_CALL RemoveAlongSymmetryLine(void *)
{
    //    for (size_t i=0;i<deformed_mesh.face.size();i++)
    //        for (size_t j=0;j<3;j++)
    //        {
    //            if (!vcg::face::IsBorder(deformed_mesh.face[i],j))continue;
    //            deformed_mesh.face[i].SetFaceEdgeS(j);
    //        }
    //    vcg::tri::Clean<TraceMesh>::RemoveDuplicateVertex(deformed_mesh);
    //    deformed_mesh.UpdateAttributes();
    //    RetrievePosSeqFromSelEdges(deformed_mesh,TestPosSeq);
    //vcg::tri::io::ExporterPLY<TraceMesh>::Save(deformed_mesh,"test_mesh.ply");

    PFashion.RemoveOnSymmetryPathIfPossible();
    DoParametrize();
    DoColorByPatch();

}


void TW_CALL SmoothField(void *)
{
    DoSmoothField();
    drawfield=true;
}

void UpdateBaseColorMesh()
{
    if (colored_distortion)
        DoColorByDistortion();
    else
        DoColorByPatch();
}

// Forward declarations for per-active-garment AntTweakBar callbacks
void TW_CALL GetFieldModeCB(void *value, void *clientData);
void TW_CALL SetFieldModeCB(const void *value, void *clientData);
void TW_CALL GetMatchValenceCB(void *value, void *clientData);
void TW_CALL SetMatchValenceCB(const void *value, void *clientData);
void TW_CALL GetPatchModeCB(void *value, void *clientData);
void TW_CALL SetPatchModeCB(const void *value, void *clientData);
void TW_CALL GetMaxCornersCB(void *value, void *clientData);
void TW_CALL SetMaxCornersCB(const void *value, void *clientData);
void TW_CALL GetSelfGlueCB(void *value, void *clientData);
void TW_CALL SetSelfGlueCB(const void *value, void *clientData);
void TW_CALL GetDartsCB(void *value, void *clientData);
void TW_CALL SetDartsCB(const void *value, void *clientData);
void TW_CALL GetDartIntervalsCB(void *value, void *clientData);
void TW_CALL SetDartIntervalsCB(const void *value, void *clientData);
void TW_CALL GetParamBoundaryCB(void *value, void *clientData);
void TW_CALL SetParamBoundaryCB(const void *value, void *clientData);
void TW_CALL GetUVModeCB(void *value, void *clientData);
void TW_CALL SetUVModeCB(const void *value, void *clientData);
void TW_CALL GetDartContCB(void *value, void *clientData);
void TW_CALL SetDartContCB(const void *value, void *clientData);
void TW_CALL GetSeamsContCB(void *value, void *clientData);
void TW_CALL SetSeamsContCB(const void *value, void *clientData);
void TW_CALL GetRemoveSymCB(void *value, void *clientData);
void TW_CALL SetRemoveSymCB(const void *value, void *clientData);
void TW_CALL GetCheckStressCB(void *value, void *clientData);
void TW_CALL SetCheckStressCB(const void *value, void *clientData);
void TW_CALL GetRemeshOnTestCB(void *value, void *clientData);
void TW_CALL SetRemeshOnTestCB(const void *value, void *clientData);
void TW_CALL GetMaxComprCB(void *value, void *clientData);
void TW_CALL SetMaxComprCB(const void *value, void *clientData);
void TW_CALL GetMaxTensCB(void *value, void *clientData);
void TW_CALL SetMaxTensCB(const void *value, void *clientData);
void TW_CALL GetSampleRateCB(void *value, void *clientData);
void TW_CALL SetSampleRateCB(const void *value, void *clientData);
void TW_CALL GetPrioModeCB(void *value, void *clientData);
void TW_CALL SetPrioModeCB(const void *value, void *clientData);
void TW_CALL GetCheckUVCB(void *value, void *clientData);
void TW_CALL SetCheckUVCB(const void *value, void *clientData);
void TW_CALL GetSmoothRemCB(void *value, void *clientData);
void TW_CALL SetSmoothRemCB(const void *value, void *clientData);
void TW_CALL GetCheckTCB(void *value, void *clientData);
void TW_CALL SetCheckTCB(const void *value, void *clientData);
void TW_CALL GetFinalRemCB(void *value, void *clientData);
void TW_CALL SetFinalRemCB(const void *value, void *clientData);

void DoBatchProcessWithPickedPoints(
    GarmentSession &s,
    const std::vector<std::vector<GarmentSession::CoordType>> &pickedPoints)
{

    if (!s.PFashion || !s.AManager) return;

    s.constraintPickedPoints = pickedPoints;
    s.manualSegmentPatchSets.clear();
    s.manualSegmentPickInProgress.clear();

    std::vector<bool> Soft(pickedPoints.size(),true);
    s.PFashion->BatchProcess(pickedPoints, *s.AManager);//,Soft);

    MergeManualSeamsIntoMesh(s);

    if (s.hasFrames)
    {
        s.AManager->UpdateProjectionBasis();
        //s.AManager->UpdateRestInfo();
        //s.AManager->UpdateCurvatureAndStretch();
    }
    drawDefMesh=true;
    drawRefMesh=false;
    drawParam=false;
    parametrized=true;

    if (gGarmentColorGreyMode)
        ColorSessionNeutralGrey(s);
    else
        ColorSessionByPatch(s);

    if (gActiveSession == &s)
        s.PFashion->GetUVSeamsPolylines(UVPolyL, Color, Dots, ColorDots);

    //TestPosSeq.clear();
}

void DoBatchProcess(GarmentSession &s)
{
    const auto &pts = !s.constraintPickedPoints.empty()
                          ? s.constraintPickedPoints
                          : GPath.PickedPoints;
    DoBatchProcessWithPickedPoints(s, pts);
}

void TW_CALL BatchProcess(void *)
{
    if (!gActiveSession) return;
    gActiveSession->constraintPickedPoints = GPath.PickedPoints;
    DoBatchProcess(*gActiveSession);
}

void TW_CALL TestCol(void *)
{
    PFashion.ColorByConvexity();
}



void SetFieldBarSizePosition(QWidget *w)
{
    int params[2];
    params[0] = QTDeviceWidth(w) / 5.0; // AntTweakBar Menu size here
    params[1] = QTDeviceHeight(w)*0.8;
    TwSetParam(barFashion, NULL, "size", TW_PARAM_INT32, 2, params);
    params[0] = QTLogicalToDevice(w, 10);
    params[1] = 30;//QTDeviceHeight(w) - params[1] - QTLogicalToDevice(w, 10);
    TwSetParam(barFashion, NULL, "position", TW_PARAM_INT32, 2, params);
}

enum FieldAnimMode{FANone,FACurvature,FAStretchCompress};
FieldAnimMode FAnimMode=FANone;

void InitBar(QWidget *w) // AntTweakBar menu
{
    if (barFashion == nullptr)
        barFashion = TwNewBar("ReconfigGarment Menu");

    SetFieldBarSizePosition(w);

    if (barFashion == nullptr)
        return;

    // Register controls once; resizeGL calls InitBar repeatedly.
    static bool barControlsInitialized = false;
    if (barControlsInitialized)
        return;
    barControlsInitialized = true;

    TwEnumVal drawmodes[4] = { {vcg::GLW::DMSmooth, "Smooth"},
                               {vcg::GLW::DMPoints, "Per Points"},
                               {vcg::GLW::DMFlatWire, "FlatWire"},
                               {vcg::GLW::DMFlat, "Flat"}};
    // Create a type for the enum shapeEV
    TwType drawMode = TwDefineEnum("DrawMode", drawmodes, 4);
    TwAddVarRW(barFashion, "Draw Mode", drawMode, &drawmode, " keyIncr='<' keyDecr='>' help='Change draw mode.' ");

    TwAddVarRW(barFashion,"draw3D",TW_TYPE_BOOLCPP, &draw3D," label='Draw 3D Mesh'");
    TwAddVarRW(barFashion,"draw patch graph",TW_TYPE_BOOLCPP, &drawPatchGraph," label='Draw Patch Graph'");
    TwAddVarRW(barFashion,"drawUV",TW_TYPE_BOOLCPP, &drawParam," label='Draw UV Mesh'");
    TwAddVarRW(barFashion,"drawSegment",TW_TYPE_BOOLCPP, &drawSegmentUV," label='Draw Flattened Segment'");
    TwAddVarRW(barFashion,"drawApproximatePolygon",TW_TYPE_BOOLCPP, &drawApproximatePolygon," label='Draw Polygon'");
    TwAddVarRW(barFashion,"drawArrangedApproxPolygonUV",TW_TYPE_BOOLCPP, &drawArrangedApproxPolygonUV,
               " label='Draw Approx Polygon UV' help='Arrange approx polygons on a 2D plane (non-overlapping).' ");

    TwAddVarRW(barFashion,"textured",TW_TYPE_BOOLCPP, &textured," label='Textured'");

    TwAddVarRW(barFashion,"doRotate",TW_TYPE_BOOLCPP, &do_rotate," label='Rotate'");

    TwEnumVal field_anim_mode[3] = { {FANone, "Field Anim None"},
                                     {FACurvature, "Field Anim Curv"},
                                     {FAStretchCompress, "Field Anim Stretch/Compress"}
                                   };
    TwType fieldAnimMode = TwDefineEnum("FieldAnimMode", field_anim_mode, 3);
    TwAddVarRW(barFashion, "Field Anim Mode", fieldAnimMode, &FAnimMode, " keyIncr='<' keyDecr='>' help='Change Field Anim mode.' ");

    TwAddVarRW(barFashion,"doAnim",TW_TYPE_BOOLCPP, &do_anim," label='Animate'");


    TwAddVarRW(barFashion,"currF",TW_TYPE_INT32, &selectedF," label='CurrentFrame'");



    TwAddButton(barFashion,"ColorDist",ColorByArapDist,0,"label='Color Distortion'");
    TwAddButton(barFashion,"ColorPatch",ColorByPatch,0,"label='Color Patch'");
    TwAddButton(barFashion,"ColorNone",ColorByNone,0,"label='Color None'");


    TwAddVarRW(barFashion,"drawDef",TW_TYPE_BOOLCPP, &drawDefMesh," label='Draw Deformed'");
    TwAddVarRW(barFashion,"drawRef",TW_TYPE_BOOLCPP, &drawRefMesh," label='Draw Reference'");
    TwAddVarRW(barFashion,"drawField",TW_TYPE_BOOLCPP, &drawfield," label='Draw Field'");
    TwAddVarRW(barFashion,"drawPlane",TW_TYPE_BOOLCPP, &drawSymmetryPlane," label='Draw Symm Plane'");
    TwAddVarRW(barFashion,"drawConstr",TW_TYPE_BOOLCPP, &drawConstraints," label='Draw Constraints'");


    TwAddSeparator(barFashion,NULL,NULL);

    TwAddButton(barFashion,"SymmetrizeDef",SymmetrizeDeformed,0,"label='Symmetrize Deformed'");

    TwEnumVal fieldmodes[4] = { {FMCurvatureOnly, "Curvature Only"},
                                {FMBoundary, "Boundary Only"},
                                {FMCurvature, "Curvature"},
                                {FMCurvatureFrames, "Curvature Frames"}
                              };

    TwType fieldMode = TwDefineEnum("FieldMode", fieldmodes, 4);
    TwAddVarCB(barFashion, "Field Mode", fieldMode, SetFieldModeCB, GetFieldModeCB, nullptr, " keyIncr='<' keyDecr='>' help='Change field mode.' ");

    TwAddButton(barFashion,"ComputeField",SmoothField,0,"label='Compute Field'");
    TwAddVarCB(barFashion,"matchCurv",TW_TYPE_BOOLCPP,
               SetMatchValenceCB,GetMatchValenceCB,nullptr," label='Match Valence'");

    TwEnumVal patchmode[3] = { {PMMinTJuncions, "Min T-Junctions"},
                               {PMAvgTJuncions, "Avg T-Junctions"},
                               {PMAllTJuncions, "All T-Junctions"}
                             };
    TwType patchMode = TwDefineEnum("PatchMode", patchmode, 3);
    TwAddVarCB(barFashion, "Patch Mode", patchMode, SetPatchModeCB, GetPatchModeCB, nullptr, " keyIncr='<' keyDecr='>' help='Change patch mode.' ");


    TwAddVarCB(barFashion,"MaxCorners",TW_TYPE_INT32,SetMaxCornersCB,GetMaxCornersCB,nullptr," label='Max Corners'");
    TwAddVarCB(barFashion,"SelfGlue",TW_TYPE_BOOLCPP,SetSelfGlueCB,GetSelfGlueCB,nullptr," label='Allow SelfGlue'");
    TwAddVarCB(barFashion,"Darts",TW_TYPE_BOOLCPP,SetDartsCB,GetDartsCB,nullptr," label='Allow Darts'");
    TwAddVarCB(barFashion,"DartInt",TW_TYPE_INT32,SetDartIntervalsCB,GetDartIntervalsCB,nullptr," label='Dart Interval'");


    TwAddButton(barFashion,"TracePaths",TracePath,0,"label='Trace Paths'");
    TwAddVarCB(barFashion,"ParamBound",TW_TYPE_DOUBLE,
               SetParamBoundaryCB,GetParamBoundaryCB,nullptr," label='Boundary Tolerance'");

    TwEnumVal parammode[3] = { {PMConformal, "Conformal"},
                               {PMArap, "Arap"},
                               {PMCloth, "Cloth"}
                             };
    TwType paramMode = TwDefineEnum("ParamMode", parammode, 3);
    TwAddVarCB(barFashion, "UV Mode", paramMode, SetUVModeCB, GetUVModeCB, nullptr, " keyIncr='<' keyDecr='>' help='Change param mode.' ");

    TwAddVarCB(barFashion,"Dart Cont",TW_TYPE_BOOLCPP,SetDartContCB,GetDartContCB,nullptr," label='Dart Continuity'");
    TwAddVarCB(barFashion,"Seams Cont",TW_TYPE_BOOLCPP,SetSeamsContCB,GetSeamsContCB,nullptr," label='Seams Continuity'");
    TwAddButton(barFashion,"Parametrize",Parametrize,0,"label='Parametrize Deformed'");

    TwAddSeparator(barFashion,NULL,NULL);
    TwAddVarCB(barFashion,"RemoveSym",TW_TYPE_BOOLCPP,SetRemoveSymCB,GetRemoveSymCB,nullptr," label='Rem Symmetry'");
    TwAddVarCB(barFashion,"checkStress",TW_TYPE_BOOLCPP,SetCheckStressCB,GetCheckStressCB,nullptr," label='Check Stress'");
    TwAddVarCB(barFashion,"remesh",TW_TYPE_BOOLCPP,SetRemeshOnTestCB,GetRemeshOnTestCB,nullptr," label='Simplify to Test'");

    TwAddVarCB(barFashion,"MaxCompr",TW_TYPE_DOUBLE,SetMaxComprCB,GetMaxComprCB,nullptr," label='Max Compression'");
    TwAddVarCB(barFashion,"MaxTens",TW_TYPE_DOUBLE,SetMaxTensCB,GetMaxTensCB,nullptr," label='Max Tension'");


    TwAddVarCB(barFashion,"SamplRate",TW_TYPE_DOUBLE,SetSampleRateCB,GetSampleRateCB,nullptr," label='Sample Rate'");

    TwEnumVal priomode[3] = { {PrioModBlend, "Blend"},
                               {PrioModeLoop, "Loop"},
                               {PrioModBorder, "Border"}
                             };
    TwType prioMode = TwDefineEnum("PrioMode", priomode, 3);
    TwAddVarCB(barFashion, "Priority Mode", prioMode, SetPrioModeCB, GetPrioModeCB, nullptr, " keyIncr='<' keyDecr='>' help='Change priority mode.' ");
    TwAddVarCB(barFashion,"CheckUV",TW_TYPE_BOOLCPP,SetCheckUVCB,GetCheckUVCB,nullptr," label='Check UV Inters'");
    TwAddVarCB(barFashion,"SmoothRem",TW_TYPE_BOOLCPP,SetSmoothRemCB,GetSmoothRemCB,nullptr," label='Smooth Before Remove'");
    TwAddVarCB(barFashion,"CheckT",TW_TYPE_BOOLCPP,SetCheckTCB,GetCheckTCB,nullptr," label='Check T Junction'");
    TwAddVarCB(barFashion,"FinalRem",TW_TYPE_BOOLCPP,SetFinalRemCB,GetFinalRemCB,nullptr," label='Final Removal'");
#ifdef MULTI_FRAME
    TwAddVarRW(barFashion,"UseFr",TW_TYPE_BOOLCPP,&PFashion.useFrames," label='Use Frames'");
#endif

    TwAddButton(barFashion,"BatchProcess",BatchProcess,0,"label='Batch Process'");
    //TwAddButton(barFashion,"BatchProcess 2",BatchProcess2,0,"label=' 2'");
    TwAddButton(barFashion,"RemoveAlongSym",RemoveAlongSymmetryLine,0,"label='Remove Along Symmetry'");
    TwAddButton(barFashion,"GenerateSVG",GenerateSVG,0,"label='Generate SVG'");


    TwAddButton(barFashion,"SaveData",SaveData,0,"label='Save Data'");
    TwAddButton(barFashion,"SaveDebug",SaveDebugPatches,0,"label='Save Debug Patches'");
    TwAddButton(barFashion,"ExportPolygons",ExportPolygons,0,"label='Export Polygons In txt'");

    TwAddButton(barFashion,"Test Color",TestCol,0,"label='Test Colorization'");


}

ScalarType angleR=0;
ScalarType alpha=0.4;

void MyGLWidget::UpdateAnim()
{
    if (!hasFrames)return;

    curr_frame=(curr_frame+1)%AManager.NumFrames();

    if (FAnimMode==FANone)
    {
        AManager.UpdateToFrame(curr_frame,false,false);
        UpdateBaseColorMesh();
    }
    if (FAnimMode==FACurvature)
    {
        AManager.UpdateToFrame(curr_frame,true,false);
        AManager.ColorByAnisotropy();
    }
    if (FAnimMode==FAStretchCompress)
    {
        AManager.UpdateToFrame(curr_frame,false,true);
        AManager.ColorByStretch();
    }

    update();
}

void MyGLWidget::UpdateRot()
{
    angleR+=alpha;
    update();
}

QImage txt_image;
//GLuint texture_dress;

//QOpenGLTexture * texture = nullptr;

MyGLWidget::MyGLWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
    trackView.SetIdentity();

    timerRot= new QTimer(this);
    connect(timerRot, SIGNAL(timeout()), this, SLOT(UpdateRot()));

    timerAnim=new QTimer(this);
    connect(timerAnim, SIGNAL(timeout()), this, SLOT(UpdateAnim()));


    // Load meshes and initialize the per-garment processing session.
    if (!gSession.loadMeshes(pathDef, pathRef, pathFrames))
    {
        std::cout<<"Error initializing garment session"<<std::endl;
        exit(0);
    }

    // In the single-garment app, the active session is always gSession.
    gActiveSession = &gSession;

    //see if there is the texture
    std::string pathTxt=pathDef;
    pathTxt.erase(pathTxt.find_last_of("/"));
    pathTxt.append("/txt.png");
    //    texture = LoadDDS(textureFile);
    //	Q_ASSERT(texture != nullptr);
    txt_image=QImage(pathTxt.c_str());
    HasTxt=(!txt_image.isNull());
    if (HasTxt)
        std::cout<<"Texture Loaded "<<std::endl;

    hasDoubleClick=false;
}


GLuint texture_dress;

void MyGLWidget::initializeGL ()
{
    //initialize Glew
    glewInit();
    //CaptInt.GLInit( MyGLWidget::width(),MyGLWidget::height());
    glClearColor(0, 0, 0, 0);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);

    if (HasTxt)
    {
        //        glGenTextures(1, &texture_dress);
        //        glBindTexture(GL_TEXTURE_2D, texture_dress);

        //        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, txt_image.width(), txt_image.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, txt_image.constBits());
        //            // glGenerateMipmap(GL_TEXTURE_2D);

        //        glActiveTexture(GL_TEXTURE0);
        //        //glBindTexture(GL_TEXTURE_2D, texture_dress);

        glGenTextures( 1, & texture_dress );
        glEnable(GL_TEXTURE_2D);
        glBindTexture( GL_TEXTURE_2D, texture_dress );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        //load texture at level i
        //txt_image.
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, txt_image.width(), txt_image.height(), 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, txt_image.constBits());
        glDisable(GL_TEXTURE_2D);
        glBindTexture( GL_TEXTURE_2D, 0 );
    }
}

// Helpers to access the active garment's Parafashion instance (single or dual app).
static Parafashion<TraceMesh>* ActivePFashion()
{
    return (gActiveSession ? gActiveSession->PFashion : nullptr);
}

// --- AntTweakBar callbacks that are per-active garment (work in single + dual) ---

void TW_CALL GetFieldModeCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<int*>(value) = pf ? pf->FMode : 0;
}

void TW_CALL SetFieldModeCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->FMode = static_cast<FieldMode>(*static_cast<const int*>(value));
}

void TW_CALL GetMatchValenceCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<int*>(value) = (pf && pf->match_valence) ? 1 : 0;
}

void TW_CALL SetMatchValenceCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->match_valence = (*static_cast<const int*>(value) != 0);
}

void TW_CALL GetPatchModeCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<int*>(value) = pf ? pf->PMode : 0;
}

void TW_CALL SetPatchModeCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->PMode = static_cast<PatchMode>(*static_cast<const int*>(value));
}

void TW_CALL GetMaxCornersCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<int*>(value) = pf ? pf->max_corners : 0;
}

void TW_CALL SetMaxCornersCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->max_corners = *static_cast<const int*>(value);
}

void TW_CALL GetSelfGlueCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<int*>(value) = (pf && pf->allow_self_glue) ? 1 : 0;
}

void TW_CALL SetSelfGlueCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->allow_self_glue = (*static_cast<const int*>(value) != 0);
}

void TW_CALL GetDartsCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<int*>(value) = (pf && pf->use_darts) ? 1 : 0;
}

void TW_CALL SetDartsCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->use_darts = (*static_cast<const int*>(value) != 0);
}

void TW_CALL GetDartIntervalsCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<int*>(value) = pf ? pf->dart_intervals : 0;
}

void TW_CALL SetDartIntervalsCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->dart_intervals = *static_cast<const int*>(value);
}

void TW_CALL GetParamBoundaryCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<double*>(value) = pf ? pf->param_boundary : 0.0;
}

void TW_CALL SetParamBoundaryCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->param_boundary = *static_cast<const double*>(value);
}

void TW_CALL GetUVModeCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<int*>(value) = pf ? pf->UVMode : 0;
}

void TW_CALL SetUVModeCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->UVMode = static_cast<ParamMode>(*static_cast<const int*>(value));
}

void TW_CALL GetDartContCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<int*>(value) = (pf && pf->continuity_darts) ? 1 : 0;
}

void TW_CALL SetDartContCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->continuity_darts = (*static_cast<const int*>(value) != 0);
}

void TW_CALL GetSeamsContCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<int*>(value) = (pf && pf->continuity_seams) ? 1 : 0;
}

void TW_CALL SetSeamsContCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->continuity_seams = (*static_cast<const int*>(value) != 0);
}

void TW_CALL GetRemoveSymCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<int*>(value) = (pf && pf->remove_along_symmetry) ? 1 : 0;
}

void TW_CALL SetRemoveSymCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->remove_along_symmetry = (*static_cast<const int*>(value) != 0);
}

void TW_CALL GetCheckStressCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<int*>(value) = (pf && pf->check_stress) ? 1 : 0;
}

void TW_CALL SetCheckStressCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->check_stress = (*static_cast<const int*>(value) != 0);
}

void TW_CALL GetRemeshOnTestCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<int*>(value) = (pf && pf->remesh_on_test) ? 1 : 0;
}

void TW_CALL SetRemeshOnTestCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->remesh_on_test = (*static_cast<const int*>(value) != 0);
}

void TW_CALL GetMaxComprCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<double*>(value) = pf ? pf->max_compression : 0.0;
}

void TW_CALL SetMaxComprCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->max_compression = *static_cast<const double*>(value);
}

void TW_CALL GetMaxTensCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<double*>(value) = pf ? pf->max_tension : 0.0;
}

void TW_CALL SetMaxTensCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->max_tension = *static_cast<const double*>(value);
}

void TW_CALL GetSampleRateCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<double*>(value) = pf ? pf->sample_rate : 0.0;
}

void TW_CALL SetSampleRateCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->sample_rate = *static_cast<const double*>(value);
}

void TW_CALL GetPrioModeCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<int*>(value) = pf ? pf->PrioMode : 0;
}

void TW_CALL SetPrioModeCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->PrioMode = static_cast<PriorityMode>(*static_cast<const int*>(value));
}

void TW_CALL GetCheckUVCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<int*>(value) = (pf && pf->CheckUVIntersection) ? 1 : 0;
}

void TW_CALL SetCheckUVCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->CheckUVIntersection = (*static_cast<const int*>(value) != 0);
}

void TW_CALL GetSmoothRemCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<int*>(value) = (pf && pf->SmoothBeforeRemove) ? 1 : 0;
}

void TW_CALL SetSmoothRemCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->SmoothBeforeRemove = (*static_cast<const int*>(value) != 0);
}

void TW_CALL GetCheckTCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<int*>(value) = (pf && pf->check_T_junction) ? 1 : 0;
}

void TW_CALL SetCheckTCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->check_T_junction = (*static_cast<const int*>(value) != 0);
}

void TW_CALL GetFinalRemCB(void *value, void *)
{
    auto *pf = ActivePFashion();
    *static_cast<int*>(value) = (pf && pf->final_removal) ? 1 : 0;
}

void TW_CALL SetFinalRemCB(const void *value, void *)
{
    auto *pf = ActivePFashion();
    if (!pf) return;
    pf->final_removal = (*static_cast<const int*>(value) != 0);
}


void MyGLWidget::resizeGL (int w, int h)
{
    glViewport (0, 0, (GLsizei) w, (GLsizei) h);
    TwWindowSize(w, h);
    InitBar(this);
    initializeGL();
}


static void GLDrawSVGLayout(size_t sizeU,
                            size_t sizeV,
                            GLuint TxtIndex)
{
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glPushMatrix();

    ScalarType MaxSize=std::max(sizeU,sizeV);
    ScalarType DimX=sizeU/MaxSize;
    ScalarType DimY=sizeV/MaxSize;

    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);

    if (TxtIndex>=0)
    {
        glActiveTexture(GL_TEXTURE0);
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, TxtIndex);
        //std::cout<<"USE TXT"<<std::endl;
    }
    vcg::glColor(vcg::Color4b(255,255,255,255));
    glBegin(GL_QUADS);
    vcg::glTexCoord(vcg::Point2<ScalarType>(0,0));
    vcg::glVertex(CoordType(0,0,0));
    vcg::glTexCoord(vcg::Point2<ScalarType>(1,0));
    vcg::glVertex(CoordType(DimX,0,0));
    vcg::glTexCoord(vcg::Point2<ScalarType>(1,1));
    vcg::glVertex(CoordType(DimX,DimY,0));
    vcg::glTexCoord(vcg::Point2<ScalarType>(0,1));
    vcg::glVertex(CoordType(0,DimY,0));
    glEnd();

    glPopMatrix();
    glPopAttrib();
}

void MyGLWidget::paintGL ()
{
    UpdateSelectedFrameIfneeded();
    if (has_to_update_layout)
    {
        glGenTextures( 1, & layoutTxtIdx );
        glEnable(GL_TEXTURE_2D);
        glBindTexture( GL_TEXTURE_2D, layoutTxtIdx );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        //load texture at level i
        //txt_image.

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, SVGTxt.width(), SVGTxt.height(), 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, SVGTxt.constBits());
        glDisable(GL_TEXTURE_2D);
        glBindTexture( GL_TEXTURE_2D, 0 );
        has_to_update_layout=false;
        //std::cout<<"LOADED TXT"<<std::endl;
    }

    if (do_rotate)
        timerRot->start(0);
    else
        timerRot->stop();

    if (do_anim)
        timerAnim->start(0);
    else
        timerAnim->stop();

    glClearColor(255,255,255,255);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40, MyGLWidget::width()/(float)MyGLWidget::height(), 0.1, 100);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0,0,3.5f,   0,0,0,   0,1,0);
    track.center=vcg::Point3f(0, 0, 0);
    track.radius= 1;
    track.GetView();

    glPushMatrix();

    bool draw_uv_and_3D=(draw3D && drawParam && parametrized);

    if (colored_distortion)
        GLDrawLegenda();

    if ((parametrized)&&(drawParam))
    {
        if (draw_uv_and_3D)
        {
            glPushMatrix();
            glTranslate(CoordType(1.4, 0.6,0)); // TODO find formula to fit exactly in corner
            //vcg::Box2<ScalarType> uv_box=vcg::tri::UV_Utils<TraceMesh>::PerWedgeUVBox(deformed_mesh);
            //glTranslate(CoordType(uv_box.DimX()/3,0,0));
            vcg::glScale(0.5);
        }
        glPushAttrib(GL_ALL_ATTRIB_BITS);
        glDepthRange(0.0001,1);
        if (drawArrangedApproxPolygonUV)
        {
            DrawArrangedApproxPolygonsFromSession(gSession, nullptr, true);
        }
        else if (drawSegmentUV)
        {   
            std::vector<TraceMesh*> meshps;
            for (auto& seg: PFashion.segmentps)
            {
                meshps.push_back(&(seg->mesh));
            }
            MeshDrawing<TraceMesh>::GLDrawUV(meshps,textured,colored_distortion);
        }
        else{
            MeshDrawing<TraceMesh>::GLDrawUV(deformed_mesh,textured,colored_distortion);
        }
        //deformed_mesh.GLDrawUV(textured,colored_distortion);
        glEnable( GL_LINE_SMOOTH );
        glHint( GL_LINE_SMOOTH_HINT, GL_NICEST );
        glDepthRange(0,0.999);
        if (!drawArrangedApproxPolygonUV && drawSegmentUV)
        {
            std::vector<TraceMesh*> meshps;
            for (auto& segp: PFashion.segmentps)
            {
                meshps.push_back(&(segp->mesh));
            }


            if (drawApproximatePolygon)
            {
                std::vector<std::vector<UVCoordType>> polygonsVerts;
                for (auto& segp: PFashion.segmentps)
                {
                    polygonsVerts.push_back((segp->polygon).polyVs);
                }
                MeshDrawing<TraceMesh>::GLDrawUVPolygon(meshps, polygonsVerts);
            }
            else{
                MeshDrawing<TraceMesh>::GLDrawEdgeUV(meshps);
            }
        }
        else if (!drawArrangedApproxPolygonUV){
            MeshDrawing<TraceMesh>::GLDrawEdgeUV(deformed_mesh);
        }
//        glDepthRange(0,0.99);
//        MeshDrawing<TraceMesh>::GLDrawUVPolylines(deformed_mesh,UVPolyL,Color,Dots,ColorDots);
        //deformed_mesh.GLDrawEdgeUV();
        if (draw_uv_and_3D)
        {
            glPopMatrix();
            //            glMatrixMode(GL_MODELVIEW);
            //            glLoadIdentity();
            if (HasLayoutTxt)
            {
                glPushMatrix();
                glTranslate(CoordType(0.8,-1,0));
                //vcg::glScale(0.5);
                GLDrawSVGLayout(SVGTxt.width(),SVGTxt.height(),layoutTxtIdx);
                glPopMatrix();
            }
        }
        //        if (draw_uv_and_3D)
        //        {
        //            glPopMatrix();
        //        }
        glPopAttrib();
    }


    if (draw_uv_and_3D)
    {
        glPushMatrix();
        glTranslate(CoordType(0,0,0)); // Offset for main 3D mesh when UV is also displayed here
        //vcg::glScale(2.0);
    }

    track.Apply();
    glPushMatrix();

    glDisable(GL_CULL_FACE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

    glEnable( GL_POLYGON_SMOOTH );
    glHint( GL_POLYGON_SMOOTH_HINT, GL_NICEST );

    glRotated(angleR,0,1,0);
    vcg::glScale(2.0f/reference_mesh.bbox.Diag());
    glTranslate(-reference_mesh.bbox.Center());
    //glTranslate(CoordType(-reference_mesh.bbox.Diag()/2,0,0));

//    if (PFashion.FMode==FMCurvatureOnly)//&&(drawfield))
//    {
//        std::pair<ScalarType,ScalarType> MinMaxQ;
//        MinMaxQ=vcg::tri::Stat<TraceMesh>::ComputePerFaceQualityMinMax(deformed_mesh);
//        vcg::GLField<TraceMesh>::GLDrawFaceField(deformed_mesh,false,false,0.007,
//                                                 MinMaxQ.second,0,false);
//    }

    if (PFashion.pagraph!=nullptr && drawPatchGraph)
    {
        GLDrawPatchGraph(PFashion.pagraph->get_patchcenters(), PFashion.pagraph->patchEdges, 2.0/reference_mesh.bbox.Diag());
    }

    if (do_anim && hasFrames)
    {
        if (FAnimMode==FACurvature)
            vcg::GLField<TraceMesh>::GLDrawFaceField(deformed_mesh,false,false,0.007,
                                                     AManager.MaxAnisotropy(),0,false);

        if (FAnimMode==FAStretchCompress)
            vcg::GLField<TraceMesh>::GLDrawFaceField(deformed_mesh,false,false,0.007,
                                                     AManager.MaxStretchCompress(),
                                                     -AManager.MaxStretchCompress(),true);
    }

    if ((drawDefMesh)&&(draw3D))
    {

        glPushAttrib(GL_ALL_ATTRIB_BITS);
        //glPushMatrix();

        if ((HasTxt)&&(textured))
        {
            glActiveTexture(GL_TEXTURE0);
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, texture_dress);
        }
        else
            glDisable(GL_TEXTURE_2D);

        vcg::GLW::ColorMode CM=vcg::GLW::CMPerFace;
        if (colored_distortion)
            CM=vcg::GLW::CMPerVert;

        glWrap.m=&deformed_mesh;
        if ((textured)&&(HasTxt))
            glWrap.Draw(drawmode,CM,vcg::GLW::TMPerWedge);
        else
            glWrap.Draw(drawmode,CM,vcg::GLW::TMNone);

        glDisable(GL_TEXTURE_2D);

        //half_def_mesh.GLDrawSharpEdges(vcg::Color4b(255,0,0,255),10);
        if (drawConstraints)
            MeshDrawing<TraceMesh>::GLDrawSharpEdges(deformed_mesh,vcg::Color4b(255,0,0,255),10);
        //deformed_mesh.GLDrawSharpEdges(vcg::Color4b(255,0,0,255),10);

        GLDrawPatchEdges();

        glPopAttrib();
    }

    if ((drawRefMesh)&&(draw3D))
    {
        glWrap.m=&reference_mesh;
        glWrap.Draw(drawmode,vcg::GLW::CMPerFace,vcg::GLW::TMNone);
    }

    if ((drawfield)&&(draw3D))
    {
        vcg::GLField<TraceMesh>::GLDrawFaceField(deformed_mesh,false,false,0.007);
        vcg::GLField<TraceMesh>::GLDrawSingularity(deformed_mesh);
    }

    if ((drawSymmetryPlane)&&(draw3D))
    {
        GlDrawPlane(Symmetrizer<TraceMesh>::SymmPlane(),deformed_mesh.bbox.Diag()/3.8);//,deformed_mesh.bbox.Center());
    }

    if  (user_is_picking)
    {
        GPath.GLAddPoint(vcg::Point2i(PickX,PickY));
        GPath.GlDrawLastPath();

        //GPath.GlDrawLastPath();
        //TPath.GLAddPoint(vcg::Point2i(PickX,PickY));
        //TPath.GlDrawLastPath();
    }
    if (hasDoubleClick)
    {
        std::cout<<"test"<<std::endl;
        bool has_removed=GPath.GLRemovePathFromPoint(vcg::Point2i(PickX,PickY),
                                                     half_def_mesh.bbox.Diag()/10);

        if (has_removed)
        {
            std::cout<<"removed"<<std::endl;
            if (gActiveSession)
                DoBatchProcess(*gActiveSession);
        }

        hasDoubleClick=false;
    }
    //TPath.GLDrawSnapped();
    //TPath.GlDrawPath();

    //GlDrawPosSeq();

    if (draw_uv_and_3D)
    {
        glPopMatrix();
    }

    glPopMatrix();
    glPopMatrix();
    //    glPopMatrix();

    //    if (draw_uv_and_3D)
    //    {
    //        glPopMatrix();
    //    }

    TwDraw();

    //GLenum GlErr=glGetError();

    switch(glGetError()) {

    case GL_INVALID_ENUM :
        std::cout<<"Invalid Enum"<<std::endl;
        assert(0);
        break;
    case GL_INVALID_VALUE :
        std::cout<<"Invalid Value"<<std::endl;
        assert(0);
        break;
    case GL_INVALID_OPERATION :
        std::cout<<"Invalid Operation"<<std::endl;
        assert(0);
        break;
    case GL_INVALID_FRAMEBUFFER_OPERATION :
        std::cout<<"Invalid FB Operation"<<std::endl;
        assert(0);
        break;
    case GL_OUT_OF_MEMORY :
        std::cout<<"Out of Memory"<<std::endl;
        assert(0);
        break;
    case GL_STACK_UNDERFLOW :
        std::cout<<"Stuck Underflow"<<std::endl;
        assert(0);
        break;
    case GL_STACK_OVERFLOW :
        std::cout<<"Stuck Overflow"<<std::endl;
        assert(0);
        break;
        //default :
    }

    //assert(glGetError()==GL_NO_ERROR);
}


void MyGLWidget::keyReleaseEvent (QKeyEvent * e)
{
    e->ignore ();
    if (e->key () == Qt::Key_Control)  track.ButtonUp (QT2VCG (Qt::NoButton, Qt::ControlModifier));
    if (e->key () == Qt::Key_Shift)  track.ButtonUp (QT2VCG (Qt::NoButton, Qt::ShiftModifier));
    if (e->key () == Qt::Key_Alt) track.ButtonUp (QT2VCG (Qt::NoButton, Qt::AltModifier));
    if (e->key () == Qt::Key_Space)
    {
        spacebar_being_pressed = false;
        return; // This gets called often when spacebar is pressed... don't updateGL
    }
    updateGL ();
}


void MyGLWidget::keyPressEvent (QKeyEvent * e)
{
    e->ignore ();
    if (e->key () == Qt::Key_Control) track.ButtonDown (QT2VCG (Qt::NoButton, Qt::ControlModifier));
    if (e->key () == Qt::Key_Shift)  track.ButtonDown (QT2VCG (Qt::NoButton, Qt::ShiftModifier));
    if (e->key () == Qt::Key_Alt)  track.ButtonDown (QT2VCG (Qt::NoButton, Qt::AltModifier));
    if (e->key () == Qt::Key_Space) {
        spacebar_being_pressed = true;
    }

    if (e->key () == Qt::Key_Q)
    {
        trackView=track.track;
    }
    if (e->key () == Qt::Key_W)
    {
        track.track=trackView;
        //    reloadView=true;
        my_window->update();
    }
    TwKeyPressQt(e);
    updateGL ();
}

void MyGLWidget::mousePressEvent (QMouseEvent * e)
{
    if(!TwMousePressQt(this,e))
    {
        e->accept ();
        setFocus ();
        track.MouseDown(QT2VCG_X(this, e), QT2VCG_Y(this, e), QT2VCG (e->button (), e->modifiers ()));

        //if(e->button() == Qt::RightButton)
        //        if (QGuiApplication::keyboardModifiers().testFlag(Qt::ShiftModifier))
        //        {
        if (spacebar_being_pressed)
        {
            user_is_picking = true;
            xMouse=QT2VCG_X(this, e);
            yMouse=QT2VCG_Y(this, e);
            PickX=xMouse;
            PickY=yMouse;
            GPath.AddNewPath();
            //pointToPick=Point2i(xMouse,yMouse);
        }
    }
    updateGL ();
}

void MyGLWidget::mouseMoveEvent (QMouseEvent * e)
{
    if (e->buttons ()) {
        if (user_is_picking)
        {
            xMouse=QT2VCG_X(this, e);
            yMouse=QT2VCG_Y(this, e);
            PickX=xMouse;
            PickY=yMouse;
        }
        else
            track.MouseMove(QT2VCG_X(this, e), QT2VCG_Y(this, e));

        updateGL ();
    }
    TwMouseMotion(QTLogicalToDevice(this, e->x()), QTLogicalToDevice(this, e->y()));
}

void MyGLWidget::mouseDoubleClickEvent (QMouseEvent * e)
{
    if (e->buttons ())
    {
        xMouse=QT2VCG_X(this, e);
        yMouse=QT2VCG_Y(this, e);
        if(e->button() == Qt::LeftButton)
        {
            hasDoubleClick=true;
            PickX=xMouse;
            PickY=yMouse;
        }
        updateGL ();
    }
    updateGL();
}


void MyGLWidget::mouseReleaseEvent (QMouseEvent * e)
{
    track.MouseUp(QT2VCG_X(this, e), QT2VCG_Y(this, e), QT2VCG(e->button (), e->modifiers ()));
    TwMouseReleaseQt(this,e);
    if (user_is_picking)
    {
        user_is_picking = false;
        //TPath.AddSharpConstraints(GPath.PickedPoints);
        //TPath.AddSharpConstraints();
        if (GPath.PickedPoints.back().size() < 1){
            GPath.PickedPoints.pop_back();
            return;
        }
        if (gActiveSession)
            DoBatchProcess(*gActiveSession);
    }
    updateGL ();
}

void MyGLWidget::wheelEvent (QWheelEvent * e)
{
    const int WHEEL_STEP = 120;
    track.MouseWheel (e->delta () / float (WHEEL_STEP), QTWheel2VCG (e->modifiers ()));
    updateGL ();
}
