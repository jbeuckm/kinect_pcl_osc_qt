#-------------------------------------------------
#
# Project created by QtCreator 2014-02-23T12:36:17
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = kpoAppCurses
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    kpoAppCurses.cpp

HEADERS += \
    kpoAppCurses.h


LIBS += -L../KPO_Base


INCLUDEPATH += /usr/include/vtk-5.8 /usr/include/pcl-1.7 /usr/include/eigen3 /usr/include/flann


INCLUDEPATH += $$PWD/../../../../usr/include
DEPENDPATH += $$PWD/../../../../usr/include
