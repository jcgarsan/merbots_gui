/**
 * Copyright (c) 2014 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 *
 * Author:
 *     Juan Carlos García
 *
 * Date February 2017
 **/


#include "../include/merbots_gui/set_robot_pose.h"
#include "ui_set_robot_pose.h"

#include <ros/package.h>

SetRobotPoseDlg::SetRobotPoseDlg(ros::NodeHandle *nodeHdl, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SetRobotPoseDlg),
    _nh(nodeHdl)
{
    ui->setupUi(this);
   	QObject::connect(ui->getMarkerPoseButton, SIGNAL(clicked()), this, SLOT(testButtonClicked()));
   	QObject::connect(ui->cancelButton, SIGNAL(clicked()), this, SLOT(close()));

	sub_tf	= _nh->subscribe<auv_msgs::NavSts>("/tf", 1, &SetRobotPoseDlg::get500MarkerPose, this); 
	//sub_ry	=  nh->subscribe<auv_msgs::NavSts>(ui.g500TopicOdometry->text().toUtf8().constData(), 1, &MainWindow::g500OdometryCallback, this); 

}

SetRobotPoseDlg::~SetRobotPoseDlg()
{
    delete ui;
}

void SetRobotPoseDlg::testButtonClicked()
{
	qDebug() << "Getting the Marker Pose...";
}

void SetRobotPoseDlg::get500MarkerPose(const auv_msgs::NavSts::ConstPtr& g500OdometryInfo)
{
	ui->xValue->setText(QString::number(g500OdometryInfo->position.north));
	ui->yValue->setText(QString::number(g500OdometryInfo->position.north));
	ui->zValue->setText(QString::number(g500OdometryInfo->position.north));
	ui->rollValue->setText(QString::number(g500OdometryInfo->position.north));
	ui->pitchValue->setText(QString::number(g500OdometryInfo->position.north));
	ui->yawValue->setText(QString::number(g500OdometryInfo->position.north));
}