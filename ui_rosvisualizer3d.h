/********************************************************************************
** Form generated from reading UI file 'rosvisualizer3d.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ROSVISUALIZER3D_H
#define UI_ROSVISUALIZER3D_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ROSVisualizer3D
{
public:
    QLabel *label;

    void setupUi(QWidget *ROSVisualizer3D)
    {
        if (ROSVisualizer3D->objectName().isEmpty())
            ROSVisualizer3D->setObjectName(QString::fromUtf8("ROSVisualizer3D"));
        ROSVisualizer3D->resize(400, 300);
        label = new QLabel(ROSVisualizer3D);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(80, 130, 69, 19));

        retranslateUi(ROSVisualizer3D);

        QMetaObject::connectSlotsByName(ROSVisualizer3D);
    } // setupUi

    void retranslateUi(QWidget *ROSVisualizer3D)
    {
        ROSVisualizer3D->setWindowTitle(QCoreApplication::translate("ROSVisualizer3D", "Form", nullptr));
        label->setText(QCoreApplication::translate("ROSVisualizer3D", "\346\265\213\350\257\225", nullptr));
    } // retranslateUi

};

namespace Ui {
    class ROSVisualizer3D: public Ui_ROSVisualizer3D {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ROSVISUALIZER3D_H
