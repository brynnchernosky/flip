#include "view.h"
#include "viewformat.h"

#include <QApplication>
#include <QKeyEvent>
#include <Eigen/StdVector>
#include <iostream>

using namespace std;
using namespace Eigen;

#define SPEED 1
#define ROTATE_SPEED 0.005

// ================== CONSTRUCTOR

View::View(QWidget *parent) :
  QGLWidget(ViewFormat(), parent),
  m_window(parent->parentWidget()),
  // Timing
  m_time(),
  m_timer(),
  // Movement
  m_forward(),
  m_sideways(),
  m_vertical(),
  // Mouse handler stuff
  m_lastX(),
  m_lastY(),
  m_capture(false)
{
  // View needs all mouse move events, not just mouse drag events
  setMouseTracking(true);

  // Hide the cursor since this is a fullscreen app
  QApplication::setOverrideCursor(Qt::ArrowCursor);

  // View needs keyboard focus
  setFocusPolicy(Qt::StrongFocus);

  // The game loop is implemented using a timer
  connect(&m_timer, SIGNAL(timeout()), this, SLOT(tick()));
}

// ================== DESTRUCTOR

View::~View()
{
  delete m_defaultShader;
  delete m_pointShader;
}

// ================== GL STUFF

void View::initializeGL()
{
  glewExperimental = GL_TRUE;

  if (glewInit() != GLEW_OK) {
    cerr << "glew initialization failed" << endl;
  }
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);

  // Alice blue
  glClearColor(240.0 / 255.0f, 248.0f / 255.0f, 255.0f / 255.0f, 1);

  // Create shaders
  m_defaultShader = new Shader(":/shaders/shader.vert", ":/shaders/shader.frag");
  m_pointShader   = new Shader(":/shaders/anchorPoint.vert", ":/shaders/anchorPoint.geom", ":/shaders/anchorPoint.frag");

  Vector3f min, max, center, range, position;
  m_arap.init(min, max);
  center = (min + max) / 2.;
  range = max - min;

  m_vSize = range.maxCoeff() * 7.5f;

  float yLength = std::max(range[0] / static_cast<float>(width()) * height(), range[1]);
  float fovY = 120.;
  float epsilon = 0.1;
  float zLength = yLength / 2. / tanf(fovY / 2.) * (1. + epsilon);
  Vector3f near = center - range;
  Vector3f far = center + range;
  position = center - Vector3f::UnitZ() * (zLength + range[2]);

  m_move = range * 0.8;

  m_camera.lookAt(position, center, Vector3f::UnitY());
  m_camera.setPerspective(120, width() / static_cast<float>(height()), near[2], far[2]);

  m_time.start();
  m_timer.start(1000 / 60);
}

void View::paintGL()
{
  glEnable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  Matrix4f model = Matrix4f::Identity();
  Matrix4f p = m_camera.getProjection();
  Matrix4f v = m_camera.getView();
  Matrix4f mvp = p * v;

  m_defaultShader->bind();
  m_defaultShader->setUniform("m", model);
  m_defaultShader->setUniform("vp", mvp);
  m_arap.draw(m_defaultShader, GL_TRIANGLES);
  m_defaultShader->unbind();

  m_pointShader->bind();
  m_pointShader->setUniform("m", model);
  m_pointShader->setUniform("vp", mvp);
  m_pointShader->setUniform("vSize", m_vSize);
  m_pointShader->setUniform("width", width());
  m_pointShader->setUniform("height", height());
  m_arap.draw(m_pointShader, GL_POINTS);
  m_pointShader->unbind();
}

void View::resizeGL(int w, int h)
{
  glViewport(0, 0, w, h);
  m_camera.setAspect(static_cast<float>(w) / h);
}

// ================== EVENT HANDLERS

Vector3f View::transformToWorldRay(int x, int y)
{
  Vector4f clipCoords = Vector4f(
                          (float(x) / width()) * 2.f - 1.f,
                          1.f - (float(y) / height()) * 2.f, -1.f, 1.f
                          );

  Vector4f transformed_coords = m_camera.getProjection().inverse() * clipCoords;
  transformed_coords = Vector4f(transformed_coords.x(), transformed_coords.y(), -1.f, 0.f);
  transformed_coords = m_camera.getView().inverse() * transformed_coords;

  return Vector3f(transformed_coords.x(), transformed_coords.y(), transformed_coords.z()).normalized();
}

void View::mousePressEvent(QMouseEvent *event)
{
  // Get current mouse coordinates
  const int currX = event->x();
  const int currY = event->y();

  // Get closest vertex to ray
  const Vector3f ray = transformToWorldRay(currX, currY);
  int closest_vertex = m_arap.getClosestVertex(m_camera.getPosition(), ray);

  // Switch on button
  switch (event->button()) {
    case Qt::MouseButton::RightButton: {
      // Return if there was no closest vertex
      if (closest_vertex == -1) {
        cout << "What are you right-clicking on, dude...?" << endl;
        break;
      }
      // Anchor/un-anchor the vertex
      m_arap.select(m_pointShader, closest_vertex);
      break;
    }
    case Qt::MouseButton::LeftButton: {
      // Capture
      m_capture = true;
      // Select this vertex
      m_lastSelected = closest_vertex;
      break;
    }
  }

  // Set last mouse coordinates
  m_lastX = currX;
  m_lastY = currY;
}

void View::mouseMoveEvent(QMouseEvent *event)
{
  // Return if the left mouse button is not currently held down
  if (!m_capture) {
    return;
  }

  // Get current mouse coordinates
  const int currX = event->x();
  const int currY = event->y();

  // Find ray
  const Vector3f ray = transformToWorldRay(event->x(), event->y());
  Vector3f pos;

  // If the selected point is an anchor point
  if (m_lastSelected != -1 && m_arap.getAnchorPos(m_lastSelected, pos, ray, m_camera.getPosition())) {
    // Move it
    m_arap.move(m_lastSelected, pos);
  } else {
    // Rotate the camera
    const int deltaX = currX - m_lastX;
    const int deltaY = currY - m_lastY;
    if (deltaX != 0 || deltaY != 0) {
      m_camera.rotate(-deltaX * ROTATE_SPEED, deltaY * ROTATE_SPEED);
    }
  }

  // Set last mouse coordinates
  m_lastX = currX;
  m_lastY = currY;
}

void View::mouseReleaseEvent(QMouseEvent *)
{
  m_capture = false;
  m_lastSelected = -1;
}

void View::wheelEvent(QWheelEvent *event)
{
  float zoom = 1 - event->delta() * 0.1f / 120;
  m_camera.zoom(zoom);
}

void View::keyPressEvent(QKeyEvent *event)
{
  // Don't remove this -- helper code for key repeat events
  if (event->isAutoRepeat()) {
    keyRepeatEvent(event);
    return;
  }

  // Switch on key
  switch (event->key()) {
    case Qt::Key_W:
      m_forward  += SPEED; break;
    case Qt::Key_S:
      m_forward  -= SPEED; break;
    case Qt::Key_A:
      m_sideways -= SPEED; break;
    case Qt::Key_D:
      m_sideways += SPEED; break;
    case Qt::Key_F:
      m_vertical -= SPEED; break;
    case Qt::Key_R:
      m_vertical += SPEED; break;
    case Qt::Key_C:
      m_camera.toggleOrbit(); break;
    case Qt::Key_T:
      m_arap.toggleWire(); break;
    case Qt::Key_Escape:
      QApplication::quit();
  }
}

void View::keyRepeatEvent(QKeyEvent *) {}

void View::keyReleaseEvent(QKeyEvent *event)
{
  // Don't remove this -- helper code for key repeat events
  if (event->isAutoRepeat()) {
    return;
  }

  // Switch on key
  switch(event->key()) {
    case Qt::Key_W:
      m_forward  -= SPEED; break;
    case Qt::Key_S:
      m_forward  += SPEED; break;
    case Qt::Key_A:
      m_sideways += SPEED; break;
    case Qt::Key_D:
      m_sideways -= SPEED; break;
    case Qt::Key_F:
      m_vertical += SPEED; break;
    case Qt::Key_R:
      m_vertical -= SPEED; break;
  }
}

// ================== UPDATE

void View::tick()
{
  float deltaSeconds = m_time.restart() * 0.001;

  // Reposition camera
  auto look = m_camera.getLook();
  look.y() = 0;
  look.normalize();
  Vector3f perp(-look.z(), 0, look.x());
  Vector3f moveVec = m_forward * look + m_sideways * perp + m_vertical * Vector3f::UnitY();
  moveVec = moveVec.cwiseProduct(m_move);
  moveVec *= deltaSeconds;
  m_camera.move(moveVec);

  // Flag this view for repainting (Qt will call paintGL() soon after)
  update();
}
