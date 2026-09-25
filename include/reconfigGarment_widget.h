#ifndef RECONFIG_GARMENT_WIDGET_H
#define RECONFIG_GARMENT_WIDGET_H

#include <QGLWidget>
#include <string>

// If set before constructing ReconfigGarmentWidget, load this experiment on startup.
extern std::string gPendingExperimentJsonPath;

class ReconfigGarmentWidget : public QGLWidget
{
public:
    explicit ReconfigGarmentWidget(QWidget *parent = nullptr);
    ~ReconfigGarmentWidget() override = default;

    QSize sizeHint() const override { return QSize(1400, 1200); }

    // Reload meshes and re-run batch + manual edits (name under exp/ or folder path).
    bool loadExperiment(const std::string &experimentNameOrPath);

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;

    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;

    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
};

#endif // RECONFIG_GARMENT_WIDGET_H

