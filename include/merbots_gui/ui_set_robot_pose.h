/********************************************************************************
** Form generated from reading UI file 'set_robot_pose.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SET_ROBOT_POSE_H
#define UI_SET_ROBOT_POSE_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_set_robot_pose
{
public:
    QWidget *widget;
    QHBoxLayout *horizontalLayout;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QPushButton *pushButton_3;

    void setupUi(QDialog *SetRobotPoseDlg)
    {
        if (SetRobotPoseDlg->objectName().isEmpty())
            SetRobotPoseDlg->setObjectName(QString::fromUtf8("SetRobotPoseDlg"));
        SetRobotPoseDlg->resize(792, 406);
        widget = new QWidget(SetRobotPoseDlg);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setGeometry(QRect(110, 220, 511, 61));
        horizontalLayout = new QHBoxLayout(widget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        pushButton = new QPushButton(widget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        horizontalLayout->addWidget(pushButton);

        pushButton_2 = new QPushButton(widget);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));

        horizontalLayout->addWidget(pushButton_2);

        pushButton_3 = new QPushButton(widget);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));

        horizontalLayout->addWidget(pushButton_3);


        retranslateUi(SetRobotPoseDlg);

        QMetaObject::connectSlotsByName(SetRobotPoseDlg);
    } // setupUi

    void retranslateUi(QDialog *SetRobotPoseDlg)
    {
        SetRobotPoseDlg->setWindowTitle(QApplication::translate("SetRobotPoseDlg", "SetRobotPoseDlg", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("SetRobotPoseDlg", "PushButton", 0, QApplication::UnicodeUTF8));
        pushButton_2->setText(QApplication::translate("SetRobotPoseDlg", "PushButton", 0, QApplication::UnicodeUTF8));
        pushButton_3->setText(QApplication::translate("SetRobotPoseDlg", "PushButton", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SetRobotPoseDlg: public Ui_set_robot_pose {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SET_ROBOT_POSE_H
