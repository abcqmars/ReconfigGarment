#include <GL/glew.h>
#include <QMouseEvent>
#include <QWheelEvent>

#include <wrap/gui/trackball.h>
#include <wrap/qt/trackball.h>

#include <tracing/GL_mesh_drawing.h>
#include <tracing/mesh_type.h>

#include "dualglwidget.h"
#include <vcg/complex/algorithms/update/color.h>
#include <vcg/complex/algorithms/stat.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/position.h>

// Global mesh paths; set by the reconfigGarment entry point.
// The actual definitions now live in reconfigGarment_widget.cpp.
extern std::string pathMeshA;
extern std::string pathMeshB;

namespace {

using TraceMesh = ::TraceMesh;

TraceMesh garmentA;
TraceMesh garmentB;

// One trackball per viewport (left/right)
vcg::Trackball trackLeft;
vcg::Trackball trackRight;

vcg::GlTrimesh<TraceMesh> glWrap;

vcg::Similarityf trackView; // kept for potential future view save/restore

vcg::GLW::DrawMode drawmode = vcg::GLW::DMSmooth;

float globalScale = 1.0f;

inline vcg::Trackball &activeTrackball(int mouseX, int widgetWidth)
{
    if (mouseX < widgetWidth / 2)
        return trackLeft;
    return trackRight;
}

} // namespace

DualGLWidget::DualGLWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
    trackView.SetIdentity();

    bool loaded = garmentA.LoadMesh(pathMeshA.c_str());
    if (!loaded) {
        std::cout << "Error loading first mesh: " << pathMeshA << std::endl;
        std::exit(1);
    }
    garmentA.UpdateAttributes();

    loaded = garmentB.LoadMesh(pathMeshB.c_str());
    if (!loaded) {
        std::cout << "Error loading second mesh: " << pathMeshB << std::endl;
        std::exit(1);
    }
    garmentB.UpdateAttributes();

    // Normalize surface area so both garments have the same total area.
    // We scale garmentB to match garmentA's area.
    double areaA = vcg::tri::Stat<TraceMesh>::ComputeMeshArea(garmentA);
    double areaB = vcg::tri::Stat<TraceMesh>::ComputeMeshArea(garmentB);
    if (areaA > 0.0 && areaB > 0.0) {
        double scale = std::sqrt(areaA / areaB);
        vcg::tri::UpdatePosition<TraceMesh>::Scale(garmentB, (float)scale);
        garmentB.bbox.SetNull();
        vcg::tri::UpdateBounding<TraceMesh>::Box(garmentB);
    }

    // Neutral base colors for the two garments.
    vcg::tri::UpdateColor<TraceMesh>::PerFaceConstant(garmentA, vcg::Color4b(200, 200, 255, 255));
    vcg::tri::UpdateColor<TraceMesh>::PerFaceConstant(garmentB, vcg::Color4b(255, 200, 200, 255));

    // Compute a global scale factor so both garments fit nicely in view
    // after the area normalization.
    float diagA = garmentA.bbox.Diag();
    float diagB = garmentB.bbox.Diag();
    float diag = std::max(diagA, diagB);
    if (diag > 0.0f) {
        globalScale = 2.0f / diag;
    } else {
        globalScale = 1.0f;
    }
}

void DualGLWidget::initializeGL()
{
    glewInit();

    glClearColor(1.f, 1.f, 1.f, 1.f);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
}

void DualGLWidget::resizeGL(int w, int h)
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
}

void DualGLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    const int w = std::max(1, QTDeviceWidth(this));
    const int h = std::max(1, QTDeviceHeight(this));
    const int halfW = std::max(1, w / 2);

    glEnable(GL_POLYGON_SMOOTH);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

    // --- Left viewport: garment A ---
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
        vcg::Point3f centerA = garmentA.bbox.Center();
        glScalef(globalScale, globalScale, globalScale);
        glTranslatef(-centerA.X(), -centerA.Y(), -centerA.Z());
        glWrap.m = &garmentA;
        glWrap.Draw(drawmode, vcg::GLW::CMPerFace, vcg::GLW::TMNone);
        glPopMatrix();
    }
    glPopMatrix();

    // --- Right viewport: garment B ---
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
        vcg::Point3f centerB = garmentB.bbox.Center();
        glScalef(globalScale, globalScale, globalScale);
        glTranslatef(-centerB.X(), -centerB.Y(), -centerB.Z());
        glWrap.m = &garmentB;
        glWrap.Draw(drawmode, vcg::GLW::CMPerFace, vcg::GLW::TMNone);
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

    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    glColor3f(0.f, 0.f, 0.f);
    glLineWidth(2.f);

    glBegin(GL_LINES);
    glVertex2f(halfW + 0.5f, 0.0f);
    glVertex2f(halfW + 0.5f, (float)h);
    glEnd();
}

void DualGLWidget::mousePressEvent(QMouseEvent *event)
{
    event->accept();
    setFocus();
    vcg::Trackball &tb = activeTrackball(event->x(), width());
    tb.MouseDown(QT2VCG_X(this, event),
                 QT2VCG_Y(this, event),
                 QT2VCG(event->button(), event->modifiers()));
    updateGL();
}

void DualGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (event->buttons()) {
        vcg::Trackball &tb = activeTrackball(event->x(), width());
        tb.MouseMove(QT2VCG_X(this, event), QT2VCG_Y(this, event));
        updateGL();
    }
}

void DualGLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    vcg::Trackball &tb = activeTrackball(event->x(), width());
    tb.MouseUp(QT2VCG_X(this, event),
               QT2VCG_Y(this, event),
               QT2VCG(event->button(), event->modifiers()));
    updateGL();
}

void DualGLWidget::wheelEvent(QWheelEvent *event)
{
    const int WHEEL_STEP = 120;
    // Use the mouse cursor X at the time of the wheel event
    QPoint p = event->pos();
    vcg::Trackball &tb = activeTrackball(p.x(), width());
    tb.MouseWheel(event->delta() / float(WHEEL_STEP),
                  QTWheel2VCG(event->modifiers()));
    updateGL();
}

