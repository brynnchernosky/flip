QT += core gui opengl

TARGET = flip

QMAKE_CXXFLAGS += -mstackrealign
CONFIG += c++17

SOURCES += \
    src/flip.cpp \
    src/main.cpp \
    src/macGrid/MacGrid.cpp \
    src/graphics/shape.cpp \
    src/graphics/MeshLoader.cpp

HEADERS += \
    src/flip.h \
    src/macGrid/Cell.h \
    src/macGrid/Face.h \
    src/macGrid/MacGrid.h \
    src/graphics/shape.h \
    src/graphics/MeshLoader.h

HOME_DIR = $$(HOME)

QMAKE_CXXFLAGS = -I "$${HOME_DIR}/eigen-git-mirror"
INCLUDEPATH    += "$${HOME_DIR}/eigen-git-mirror"
DEPENDPATH     += "$${HOME_DIR}/eigen-git-mirror"

