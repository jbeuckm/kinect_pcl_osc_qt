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
    oscsender.cpp

HEADERS  += workflowui.h \
    oscsender.h

FORMS    += workflowui.ui

OTHER_FILES += \
    CMakeLists.txt

LIBS += -L/usr/lib/vtk-5.8
#LIBS += -lvtkRendering -l libQVTKWidget

LIBS += -L/usr/lib/pcl-1.7
#LIBS += pcl_visualization

INCLUDEPATH += /usr/include/vtk-5.8 /usr/include/pcl-1.7 /usr/include/eigen3 /usr/include/flann
#INCLUDEPATH += /usr/include/c++/4.3



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

INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lQVTK
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lQVTK
else:symbian: LIBS += -lQVTK
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lQVTK

INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lboost_system
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lboost_system
else:symbian: LIBS += -lboost_system
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lboost_system

INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lpcl_visualization
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lpcl_visualization
else:symbian: LIBS += -lpcl_visualization
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lpcl_visualization

INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lvtkWidgets
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lvtkWidgets
else:symbian: LIBS += -lvtkWidgets
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lvtkWidgets

INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lvtkCommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lvtkCommon
else:symbian: LIBS += -lvtkCommon
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lvtkCommon

INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lvtkFiltering
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lvtkFiltering
else:symbian: LIBS += -lvtkFiltering
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lvtkFiltering

INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lpcl_common
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lpcl_common
else:symbian: LIBS += -lpcl_common
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lpcl_common

INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include
