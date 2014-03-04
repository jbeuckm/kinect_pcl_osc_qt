/********************************************************************************
** Form generated from reading UI file 'kpoAppGui.ui'
**
** Created: Mon Mar 3 14:50:48 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_KPOAPPGUI_H
#define UI_KPOAPPGUI_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_KinectPclOsc
{
public:
    QWidget *centralwidget;
    QVTKWidget *qvtk_widget;
    QCheckBox *pauseCheckBox;
    QPushButton *saveCloudButton;
    QLineEdit *objectNameTextInput;
    QLabel *label_3;
    QCheckBox *matchModelsCheckbox;
    QPushButton *loadRawCloudButton;
    QCheckBox *processSceneCheckBox;
    QLineEdit *ipTextInput;
    QLabel *label_4;
    QLabel *label_5;
    QLineEdit *portTextInput;
    QPushButton *setOscTargetButton;
    QLabel *label_6;
    QSlider *depthThresholdSlider;
    QLabel *label;
    QLineEdit *downsamplingRadiusEdit;
    QSlider *downsamplingRadiusSlider;
    QLineEdit *modelsFolderEdit;
    QPushButton *browseForModelsButton;
    QSlider *depthImageThresholdSlider;
    QWidget *blobs;

    void setupUi(QMainWindow *KinectPclOsc)
    {
        if (KinectPclOsc->objectName().isEmpty())
            KinectPclOsc->setObjectName(QString::fromUtf8("KinectPclOsc"));
        KinectPclOsc->resize(985, 729);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(1);
        sizePolicy.setVerticalStretch(1);
        sizePolicy.setHeightForWidth(KinectPclOsc->sizePolicy().hasHeightForWidth());
        KinectPclOsc->setSizePolicy(sizePolicy);
        KinectPclOsc->setIconSize(QSize(22, 22));
        centralwidget = new QWidget(KinectPclOsc);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        centralwidget->setEnabled(true);
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(255);
        sizePolicy1.setVerticalStretch(255);
        sizePolicy1.setHeightForWidth(centralwidget->sizePolicy().hasHeightForWidth());
        centralwidget->setSizePolicy(sizePolicy1);
        qvtk_widget = new QVTKWidget(centralwidget);
        qvtk_widget->setObjectName(QString::fromUtf8("qvtk_widget"));
        qvtk_widget->setGeometry(QRect(470, 10, 461, 361));
        pauseCheckBox = new QCheckBox(centralwidget);
        pauseCheckBox->setObjectName(QString::fromUtf8("pauseCheckBox"));
        pauseCheckBox->setGeometry(QRect(10, 50, 131, 22));
        saveCloudButton = new QPushButton(centralwidget);
        saveCloudButton->setObjectName(QString::fromUtf8("saveCloudButton"));
        saveCloudButton->setGeometry(QRect(10, 220, 121, 27));
        objectNameTextInput = new QLineEdit(centralwidget);
        objectNameTextInput->setObjectName(QString::fromUtf8("objectNameTextInput"));
        objectNameTextInput->setGeometry(QRect(110, 180, 151, 27));
        label_3 = new QLabel(centralwidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(10, 180, 91, 17));
        matchModelsCheckbox = new QCheckBox(centralwidget);
        matchModelsCheckbox->setObjectName(QString::fromUtf8("matchModelsCheckbox"));
        matchModelsCheckbox->setGeometry(QRect(10, 260, 131, 22));
        loadRawCloudButton = new QPushButton(centralwidget);
        loadRawCloudButton->setObjectName(QString::fromUtf8("loadRawCloudButton"));
        loadRawCloudButton->setGeometry(QRect(150, 220, 131, 27));
        processSceneCheckBox = new QCheckBox(centralwidget);
        processSceneCheckBox->setObjectName(QString::fromUtf8("processSceneCheckBox"));
        processSceneCheckBox->setGeometry(QRect(10, 80, 141, 22));
        processSceneCheckBox->setChecked(false);
        ipTextInput = new QLineEdit(centralwidget);
        ipTextInput->setObjectName(QString::fromUtf8("ipTextInput"));
        ipTextInput->setGeometry(QRect(700, 650, 131, 27));
        label_4 = new QLabel(centralwidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(670, 650, 21, 17));
        label_5 = new QLabel(centralwidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(840, 650, 31, 20));
        portTextInput = new QLineEdit(centralwidget);
        portTextInput->setObjectName(QString::fromUtf8("portTextInput"));
        portTextInput->setGeometry(QRect(880, 650, 51, 27));
        setOscTargetButton = new QPushButton(centralwidget);
        setOscTargetButton->setObjectName(QString::fromUtf8("setOscTargetButton"));
        setOscTargetButton->setGeometry(QRect(797, 690, 131, 27));
        label_6 = new QLabel(centralwidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(10, 290, 101, 17));
        depthThresholdSlider = new QSlider(centralwidget);
        depthThresholdSlider->setObjectName(QString::fromUtf8("depthThresholdSlider"));
        depthThresholdSlider->setGeometry(QRect(940, 10, 29, 701));
        depthThresholdSlider->setMinimum(250);
        depthThresholdSlider->setMaximum(2500);
        depthThresholdSlider->setPageStep(1);
        depthThresholdSlider->setOrientation(Qt::Vertical);
        depthThresholdSlider->setInvertedAppearance(true);
        depthThresholdSlider->setInvertedControls(true);
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(480, 400, 161, 17));
        downsamplingRadiusEdit = new QLineEdit(centralwidget);
        downsamplingRadiusEdit->setObjectName(QString::fromUtf8("downsamplingRadiusEdit"));
        downsamplingRadiusEdit->setGeometry(QRect(640, 400, 113, 27));
        downsamplingRadiusSlider = new QSlider(centralwidget);
        downsamplingRadiusSlider->setObjectName(QString::fromUtf8("downsamplingRadiusSlider"));
        downsamplingRadiusSlider->setGeometry(QRect(480, 440, 441, 29));
        downsamplingRadiusSlider->setMinimum(1);
        downsamplingRadiusSlider->setMaximum(100);
        downsamplingRadiusSlider->setOrientation(Qt::Horizontal);
        modelsFolderEdit = new QLineEdit(centralwidget);
        modelsFolderEdit->setObjectName(QString::fromUtf8("modelsFolderEdit"));
        modelsFolderEdit->setEnabled(false);
        modelsFolderEdit->setGeometry(QRect(10, 310, 221, 27));
        browseForModelsButton = new QPushButton(centralwidget);
        browseForModelsButton->setObjectName(QString::fromUtf8("browseForModelsButton"));
        browseForModelsButton->setGeometry(QRect(240, 310, 98, 27));
        depthImageThresholdSlider = new QSlider(centralwidget);
        depthImageThresholdSlider->setObjectName(QString::fromUtf8("depthImageThresholdSlider"));
        depthImageThresholdSlider->setGeometry(QRect(430, 350, 29, 331));
        depthImageThresholdSlider->setMaximum(255);
        depthImageThresholdSlider->setValue(128);
        depthImageThresholdSlider->setOrientation(Qt::Vertical);
        blobs = new QWidget(centralwidget);
        blobs->setObjectName(QString::fromUtf8("blobs"));
        blobs->setGeometry(QRect(10, 360, 411, 321));
        KinectPclOsc->setCentralWidget(centralwidget);

        retranslateUi(KinectPclOsc);

        QMetaObject::connectSlotsByName(KinectPclOsc);
    } // setupUi

    void retranslateUi(QMainWindow *KinectPclOsc)
    {
        KinectPclOsc->setWindowTitle(QString());
        pauseCheckBox->setText(QApplication::translate("KinectPclOsc", "Pause Grabber", 0, QApplication::UnicodeUTF8));
        saveCloudButton->setText(QApplication::translate("KinectPclOsc", "Save Pointcloud", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("KinectPclOsc", "Object Name", 0, QApplication::UnicodeUTF8));
        matchModelsCheckbox->setText(QApplication::translate("KinectPclOsc", "Match Models", 0, QApplication::UnicodeUTF8));
        loadRawCloudButton->setText(QApplication::translate("KinectPclOsc", "Load Raw Cloud", 0, QApplication::UnicodeUTF8));
        processSceneCheckBox->setText(QApplication::translate("KinectPclOsc", "Process Scene", 0, QApplication::UnicodeUTF8));
        ipTextInput->setText(QApplication::translate("KinectPclOsc", "192.168.0.4", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("KinectPclOsc", "IP", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("KinectPclOsc", "port", 0, QApplication::UnicodeUTF8));
        portTextInput->setText(QApplication::translate("KinectPclOsc", "12345", 0, QApplication::UnicodeUTF8));
        setOscTargetButton->setText(QApplication::translate("KinectPclOsc", "Set OSC Target", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("KinectPclOsc", "Models Folder", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("KinectPclOsc", "Downsampling Radius", 0, QApplication::UnicodeUTF8));
        browseForModelsButton->setText(QApplication::translate("KinectPclOsc", "Browse", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class KinectPclOsc: public Ui_KinectPclOsc {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_KPOAPPGUI_H
