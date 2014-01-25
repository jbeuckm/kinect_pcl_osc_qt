#-------------------------------------------------
#
# Project created by QtCreator 2014-01-24T22:22:39
#
#-------------------------------------------------

QT       += core gui

TARGET = KinectPclOsc
TEMPLATE = app


SOURCES += main.cpp\
        workflowui.cpp \
        vtkpointcloudviewer.cpp

HEADERS  += workflowui.h \
            vtkpointcloudviewer.h

FORMS    += workflowui.ui

OTHER_FILES += \
    CMakeLists.txt

LIBS += -L/usr/lib/vtk-5.8
LIBS += -lQTVK

LIBS += -L/usr/lib/pcl-1.7
LIBS += pcl_visualization

INCLUDEPATH += /usr/include/vtk-5.8 /usr/include/pcl-1.7 /usr/include/eigen3 /usr/include/flann
INCLUDEPATH += /usr/include/c++/4.3


