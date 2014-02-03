#-------------------------------------------------
#
# Project created by QtCreator 2014-01-26T19:09:10
#
#-------------------------------------------------

QT       += core gui

TARGET = KinectPclOsc
TEMPLATE = app


SOURCES += \
    kpoPclFunctions.cpp \
    kpoApp.cpp \
    kpoOscSender.cpp \
    kpoObjectDescription.cpp

HEADERS += \
    openni_passthrough_qt.h \
    kpoPclFunctions.h \
    kpoApp.h \
    kpoOscSender.h \
    kpoObjectDescription.h

FORMS += \
    kpoApp.ui


LIBS += -L/usr/lib/vtk-5.8

LIBS += -L/usr/lib/pcl-1.7

INCLUDEPATH += /usr/include/vtk-5.8 /usr/include/pcl-1.7 /usr/include/eigen3 /usr/include/flann


INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -loscpack
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -loscpack
else:symbian: LIBS += -loscpack
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -loscpack

INCLUDEPATH += $$PWD/../../../usr/include/oscpack
DEPENDPATH += $$PWD/../../../usr/include/oscpack

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lvtkRendering
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lvtkRendering
else:symbian: LIBS += -lvtkRendering
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lvtkRendering


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lQVTK
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lQVTK
else:symbian: LIBS += -lQVTK
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lQVTK


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lboost_system
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lboost_system
else:symbian: LIBS += -lboost_system
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lboost_system


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lpcl_visualization
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lpcl_visualization
else:symbian: LIBS += -lpcl_visualization
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lpcl_visualization


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lvtkWidgets
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lvtkWidgets
else:symbian: LIBS += -lvtkWidgets
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lvtkWidgets


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lvtkCommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lvtkCommon
else:symbian: LIBS += -lvtkCommon
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lvtkCommon


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lvtkFiltering
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lvtkFiltering
else:symbian: LIBS += -lvtkFiltering
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lvtkFiltering



win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lpcl_common
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lpcl_common
else:symbian: LIBS += -lpcl_common
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lpcl_common


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lOpenNI
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lOpenNI
else:symbian: LIBS += -lOpenNI
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lOpenNI

INCLUDEPATH += /usr/include/openni/


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lpcl_io
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lpcl_io
else:symbian: LIBS += -lpcl_io
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lpcl_io

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lpcl_octree
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lpcl_octree
else:symbian: LIBS += -lpcl_octree
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lpcl_octree


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lpcl_io_ply
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lpcl_io_ply
else:symbian: LIBS += -lpcl_io_ply
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lpcl_io_ply



win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/lib/release/ -lboost_thread
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/lib/debug/ -lboost_thread
else:symbian: LIBS += -lboost_thread
else:unix: LIBS += -L$$PWD/../../../../usr/lib/ -lboost_thread


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/lib/release/ -lpcl_filters
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/lib/debug/ -lpcl_filters
else:symbian: LIBS += -lpcl_filters
else:unix: LIBS += -L$$PWD/../../../../usr/lib/ -lpcl_filters


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/lib/release/ -lpcl_search
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/lib/debug/ -lpcl_search
else:symbian: LIBS += -lpcl_search
else:unix: LIBS += -L$$PWD/../../../../usr/lib/ -lpcl_search


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/lib/release/ -lpcl_features
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/lib/debug/ -lpcl_features
else:symbian: LIBS += -lpcl_features
else:unix: LIBS += -L$$PWD/../../../../usr/lib/ -lpcl_features


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/lib/release/ -lpcl_surface
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/lib/debug/ -lpcl_surface
else:symbian: LIBS += -lpcl_surface
else:unix: LIBS += -L$$PWD/../../../../usr/lib/ -lpcl_surface


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/lib/release/ -lpcl_keypoints
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/lib/debug/ -lpcl_keypoints
else:symbian: LIBS += -lpcl_keypoints
else:unix: LIBS += -L$$PWD/../../../../usr/lib/ -lpcl_keypoints


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/lib/release/ -lpcl_recognition
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/lib/debug/ -lpcl_recognition
else:symbian: LIBS += -lpcl_recognition
else:unix: LIBS += -L$$PWD/../../../../usr/lib/ -lpcl_recognition


