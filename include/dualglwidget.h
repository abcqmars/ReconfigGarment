#ifndef DUALGLWIDGET_H
#define DUALGLWIDGET_H

#include <QGLWidget>

class DualGLWidget : public QGLWidget
{
public:
    explicit DualGLWidget(QWidget *parent = nullptr);
    ~DualGLWidget() override = default;

    QSize sizeHint() const override { return QSize(1400, 1200); }

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;

    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
};

#endif // DUALGLWIDGET_H

