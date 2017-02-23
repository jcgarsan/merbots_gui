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
   	QObject::connect(ui->getMarkerPoseButton, SIGNAL(clicked()), this, SLOT(getMarkerPoseButtonClicked()));
   	QObject::connect(ui->acceptButton, SIGNAL(clicked()), this, SLOT(acceptButtonClicked()));
   	QObject::connect(ui->cancelButton, SIGNAL(clicked()), this, SLOT(close()));

   	//ToDo: this subscriber must be modified using the real topic name and type
	sub_tf	= _nh->subscribe<auv_msgs::NavSts>("/tf", 1, &SetRobotPoseDlg::get500MarkerPose, this); 

	getMarkerData = false;

}

SetRobotPoseDlg::~SetRobotPoseDlg()
{
    delete ui;
}


void SetRobotPoseDlg::getMarkerPoseButtonClicked()
{
	qDebug() << "Getting the Marker Pose...";
	getMarkerData = true;
}


//ToDo: change topic type
void SetRobotPoseDlg::get500MarkerPose(const auv_msgs::NavSts::ConstPtr& g500OdometryInfo)
{
	if (getMarkerData)
	{
		ui->xValue->setText(QString::number(g500OdometryInfo->position.north));
		ui->yValue->setText(QString::number(g500OdometryInfo->position.north));
		ui->zValue->setText(QString::number(g500OdometryInfo->position.north));
		ui->rollValue->setText(QString::number(g500OdometryInfo->position.north));
		ui->pitchValue->setText(QString::number(g500OdometryInfo->position.north));
		ui->yawValue->setText(QString::number(g500OdometryInfo->position.north));
		getMarkerData = false;
	}
}


void SetRobotPoseDlg::acceptButtonClicked()
{
	xValueDlg = ui->xValue->text().toDouble();
	yValueDlg = ui->yValue->text().toDouble();
	zValueDlg = ui->zValue->text().toDouble();
	rollValueDlg  = ui->rollValue->text().toDouble();
	pitchValueDlg = ui->pitchValue->text().toDouble();
	yawValueDlg	  = ui->yawValue->text().toDouble();

	Q_EMIT newRobotPose(xValueDlg, yValueDlg, zValueDlg, rollValueDlg, pitchValueDlg, yawValueDlg);
}


