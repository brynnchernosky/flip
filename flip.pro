QT += core gui opengl

TARGET = flip
TEMPLATE = app

QMAKE_CXXFLAGS += -mstackrealign
QMAKE_CXXFLAGS += -I "../../eigen-git-mirror"
CONFIG += c++17

unix:!macx {
    LIBS += -lGLU
}
win32 {
    DEFINES += GLEW_STATIC
    LIBS += -lopengl32 -lglu32
}

SOURCES += \
    libs/glew-1.10.0/src/glew.c \
    src/view.cpp \
    src/flip.cpp \
    src/main.cpp \
    src/mainwindow.cpp \
    src/viewformat.cpp \
    src/graphics/shape.cpp \
    src/reconstruction.cpp \
    src/macGrid/MacGrid.cpp \
    src/graphics/MeshLoader.cpp \
    src/graphics/Shader.cpp \
    src/graphics/GraphicsDebug.cpp \
    src/graphics/camera.cpp

HEADERS += \
    src/flip.h \
    src/Debug.h \
    src/macGrid/Cell.h \
    src/macGrid/HashMap.h \
    src/macGrid/Particle.h \
    src/macGrid/MacGrid.h \
    src/macGrid/Material.h \
    src/graphics/shape.h \
    src/graphics/MeshLoader.h \
    src/reconstruction.h \ 
    src/mainwindow.h \
    src/view.h \
    src/viewformat.h \
    src/graphics/Shader.h \
    src/graphics/ShaderAttribLocations.h \
    src/graphics/GraphicsDebug.h \
    src/graphics/camera.h \
    ui_mainwindow.h

FORMS += src/mainwindow.ui

RESOURCES += \
    res/shaders/shaders.qrc

HOME_DIR = $$(HOME)

QMAKE_CXXFLAGS = -I "$${HOME_DIR}/eigen-git-mirror"
INCLUDEPATH    += "$${HOME_DIR}/eigen-git-mirror"
DEPENDPATH     += "$${HOME_DIR}/eigen-git-mirror"

INCLUDEPATH += src libs glm libs/glew-1.10.0/include
DEPENDPATH += src libs glm libs/glew-1.10.0/include

DISTFILES += \
    res/shaders/shader.vert \
    res/shaders/shader.frag \
    src/config.ini


