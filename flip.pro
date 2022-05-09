QT += core gui opengl concurrent

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
    src/testing.cpp \
    src/flip.cpp \
    src/main.cpp \
    src/reconstruction.cpp \
    src/graphics/shape.cpp \
    src/macGrid/MacGrid.cpp \
    src/graphics/MeshLoader.cpp

HEADERS += \
    src/flip.h \
    src/Debug.h \
    src/reconstruction.h \
    src/macGrid/Cell.h \
    src/macGrid/HashMap.h \
    src/macGrid/Particle.h \
    src/macGrid/MacGrid.h \
    src/macGrid/Material.h \
    src/graphics/shape.h \
    src/graphics/MeshLoader.h \
    src/testing.h

HOME_DIR = $$(HOME)

QMAKE_CXXFLAGS = -I "$${HOME_DIR}/eigen-git-mirror"
INCLUDEPATH    += "$${HOME_DIR}/eigen-git-mirror"
DEPENDPATH     += "$${HOME_DIR}/eigen-git-mirror"

INCLUDEPATH += src libs glm libs/glew-1.10.0/include
DEPENDPATH += src libs glm libs/glew-1.10.0/include
