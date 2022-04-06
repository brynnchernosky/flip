#ifndef VIEW_H
#define VIEW_H

#include "arap.h"
#include "graphics/camera.h"
#include "graphics/Shader.h"

#include <QGLWidget>
#include <QElapsedTimer>
#include <QTimer>
#include <memory>

/**
 * This is similar to your "CS1971FrontEnd" class. Here you will receive all of the input events
 * to forward to your game.
 *
 * @brief The View class
 */
class View : public QGLWidget
{
    Q_OBJECT

  public:

    View(QWidget *parent);
    ~View();

  private:

    static const int FRAMES_TO_AVERAGE = 30;

    // ================== GL STUFF

    void initializeGL();
    void paintGL();
    void resizeGL(int w, int h);

    // ================== EVENT HANDLERS

    Eigen::Vector3f transformToWorldRay(int x, int y);

    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void keyRepeatEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);

    ARAP m_arap;
    Camera m_camera;
    Shader *m_defaultShader;
    Shader *m_pointShader;

    // Timing
    QWidget *m_window;
    QElapsedTimer m_time;
    QTimer m_timer;

    GLuint m_surfaceFbo;
    GLuint depthBuffer;
    GLuint frameTexture;

    // Movement
    int m_forward;
    int m_sideways;
    int m_vertical;
    Eigen::Vector3f m_move;
    float m_vSize;

    // Mouse handler stuff
    int m_lastX;
    int m_lastY;
    bool m_capture;
    int m_lastSelected = -1;

  private slots:

    void tick();
};

#endif // VIEW_H

