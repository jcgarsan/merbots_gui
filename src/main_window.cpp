/**
 * Copyright (c) 2014 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 *
 * This file has been adapted from ROS command catkin_create_qt_pkg
 *
 * Author:
 *     Juan Carlos García
 *
 * Date February 2017
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include "../include/merbots_gui/main_window.hpp"
#include "../include/merbots_gui/set_robot_pose.h"

#include <QtGui>
#include <QMessageBox>
#include <QtWebKit/QWebView>
#include <QLineEdit>
#include <QString>
#include <QtCore>
#include <QThread>
#include <QTimer>
#include <QTableWidget>
#include <QSlider>

#include <ros/package.h>


#define DebugTOI	false


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace enc = sensor_msgs::image_encodings;

namespace merbots_gui {

using namespace std;
using namespace Qt;


/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	ui.mainTabs->setCurrentIndex(0);
	ui.graspSpecTab->setCurrentIndex(0);


	//Init section
	ros::init(argc,argv,"merbots_gui");
	ros::start(); 	//explicitly needed since our nodehandle is going out of scope
	if (!ros::master::check())
	{
		qDebug()<<"No ROS master found";
		showNoMasterMessage();
	}

	//Odometry tables init
	for (int i=0; i<2; i++)
	{
		for (int j=0; j<6; j++)
			{
				ui.g500OdometryTable->setItem(i, j, new QTableWidgetItem("0.0"));
				ui.sparusOdometryTable->setItem(i, j, new QTableWidgetItem("0.0"));
			}
	}

	//Service tables init
	for (int i=0; i<3; i++)
	{
		ui.g500ServiceStatus->setItem(0, i, new QTableWidgetItem("0.0"));
		ui.sparusServiceStatus->setItem(0, i, new QTableWidgetItem("0.0"));
	}

	//Arm limit table init
	for (int i=0; i<5; i++)
		ui.armJointValues->setItem(i, 0, new QTableWidgetItem("0.0"));
	
	ui.armJointValues->setItem(0, 1, new QTableWidgetItem("-1.571"));
	ui.armJointValues->setItem(1, 1, new QTableWidgetItem("0.000"));
	ui.armJointValues->setItem(2, 1, new QTableWidgetItem("0.000"));
	ui.armJointValues->setItem(3, 1, new QTableWidgetItem("0.000"));
	ui.armJointValues->setItem(4, 1, new QTableWidgetItem("-0.058"));
	ui.armJointValues->setItem(0, 2, new QTableWidgetItem("0.549"));
	ui.armJointValues->setItem(1, 2, new QTableWidgetItem("1.587"));
	ui.armJointValues->setItem(2, 2, new QTableWidgetItem("2.153"));
	ui.armJointValues->setItem(3, 2, new QTableWidgetItem("0.000"));
	ui.armJointValues->setItem(4, 2, new QTableWidgetItem("1.338"));




    //Main App connections
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	//Menu objects connections
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    //TAB1: Image stream connections
    QObject::connect(ui.g500LoadStreamButton, SIGNAL(clicked()), this, SLOT(g500LoadStream()));
    QObject::connect(ui.g500LoadStreamButton2, SIGNAL(clicked()), this, SLOT(g500LoadStream2()));
    QObject::connect(ui.g500StopStreamButton, SIGNAL(clicked()), this, SLOT(g500StopStream()));
    QObject::connect(ui.g500StopStreamButton2, SIGNAL(clicked()), this, SLOT(g500StopStream2()));
    QObject::connect(ui.g500StreamIP, SIGNAL(returnPressed()), this, SLOT(g500LoadStream()));
    QObject::connect(ui.g500StreamTopic, SIGNAL(returnPressed()), this, SLOT(g500LoadStream()));
    QObject::connect(ui.g500StreamType, SIGNAL(currentIndexChanged(int)), this, SLOT(g500LoadStream()));
    QObject::connect(ui.g500TopicsButton, SIGNAL(clicked()), this, SLOT(g500TopicsButtonClicked()));
    QObject::connect(ui.g500GoToSurfaceButton, SIGNAL(clicked()), this, SLOT(g500GoToSurface()));

    QObject::connect(ui.sparusLoadStreamButton, SIGNAL(clicked()), this, SLOT(sparusLoadStream()));
    QObject::connect(ui.sparusStopStreamButton, SIGNAL(clicked()), this, SLOT(sparusStopStream()));
    QObject::connect(ui.sparusStreamIP, SIGNAL(returnPressed()), this, SLOT(sparusLoadStream()));
    QObject::connect(ui.sparusStreamTopic, SIGNAL(returnPressed()), this, SLOT(sparusLoadStream()));
    QObject::connect(ui.sparusStreamType, SIGNAL(currentIndexChanged(int)), this, SLOT(sparusLoadStream()));
    QObject::connect(ui.sparusTopicsButton, SIGNAL(clicked()), this, SLOT(sparusTopicsButtonClicked()));

    QObject::connect(ui.vsPublishButton, SIGNAL(clicked()),this, SLOT(vsPublishButtonClicked()));
	QObject::connect(ui.vsCancelButton, SIGNAL(clicked()),this, SLOT(vsCancelButtonClicked()));
    QObject::connect(ui.vsTopicsButton, SIGNAL(clicked()), this, SLOT(vsTopicsButtonClicked()));

	QObject::connect(ui.g500MoveRobotButton, SIGNAL(clicked()), this, SLOT(testButton()));
    
    QObject::connect(ui.getGraspingPoseButton, SIGNAL(clicked()), this, SLOT(getInitGraspPose()));
    QObject::connect(ui.graspSpecTab, SIGNAL(currentChanged(int)), this, SLOT(setSpecificationMode(int)));
    QObject::connect(ui.interactiveSpecSlider1, SIGNAL(sliderMoved(int)), this, SLOT(updateInteractiveSpecParams()));
    QObject::connect(ui.interactiveSpecSlider2, SIGNAL(sliderMoved(int)), this, SLOT(updateInteractiveSpecParams()));
    QObject::connect(ui.interactiveSpecSlider3, SIGNAL(sliderMoved(int)), this, SLOT(updateInteractiveSpecParams()));
    QObject::connect(ui.interactiveSpecSlider4, SIGNAL(sliderMoved(int)), this, SLOT(updateInteractiveSpecParams()));
    QObject::connect(ui.interactiveSpecSlider5, SIGNAL(sliderMoved(int)), this, SLOT(updateInteractiveSpecParams()));
    QObject::connect(ui.interactiveSpecSlider6, SIGNAL(sliderMoved(int)), this, SLOT(updateInteractiveSpecParams()));    
    QObject::connect(ui.interactiveSpecSlider1, SIGNAL(sliderReleased()), this, SLOT(updateAndResetInteractiveSpecParams()));
    QObject::connect(ui.interactiveSpecSlider2, SIGNAL(sliderReleased()), this, SLOT(updateAndResetInteractiveSpecParams()));
    QObject::connect(ui.interactiveSpecSlider3, SIGNAL(sliderReleased()), this, SLOT(updateAndResetInteractiveSpecParams()));
    QObject::connect(ui.interactiveSpecSlider4, SIGNAL(sliderReleased()), this, SLOT(updateAndResetInteractiveSpecParams()));
    QObject::connect(ui.interactiveSpecSlider5, SIGNAL(sliderReleased()), this, SLOT(updateAndResetInteractiveSpecParams()));
    QObject::connect(ui.interactiveSpecSlider6, SIGNAL(sliderReleased()), this, SLOT(updateAndResetInteractiveSpecParams()));
    QObject::connect(ui.guidedSpecSlider1, SIGNAL(sliderMoved(int)), this, SLOT(updateGuidedSpecParams()));
    QObject::connect(ui.guidedSpecSlider2, SIGNAL(sliderMoved(int)), this, SLOT(updateGuidedSpecParams()));
    QObject::connect(ui.guidedSpecSlider3, SIGNAL(sliderMoved(int)), this, SLOT(updateGuidedSpecParams()));


    ui.interactiveSpecSlider1->setMaximum(100);
    ui.interactiveSpecSlider2->setMaximum(100);
    ui.interactiveSpecSlider3->setMaximum(100);
    ui.interactiveSpecSlider4->setMaximum(180);
    ui.interactiveSpecSlider5->setMaximum(180);
    ui.interactiveSpecSlider6->setMaximum(180);
    ui.guidedSpecSlider1->setMaximum(100);
    ui.guidedSpecSlider2->setMaximum(180);
    ui.guidedSpecSlider3->setMaximum(100);

    ui.interactiveSpecSlider1->setMinimum(-100);
    ui.interactiveSpecSlider2->setMinimum(-100);
    ui.interactiveSpecSlider3->setMinimum(-100);
    ui.interactiveSpecSlider4->setMinimum(-180);
    ui.interactiveSpecSlider5->setMinimum(-180);
    ui.interactiveSpecSlider6->setMinimum(-180);
    ui.guidedSpecSlider1->setMinimum(-100);
    ui.guidedSpecSlider2->setMinimum(-180);
    ui.guidedSpecSlider3->setMinimum(-100);

    //@TODO Teleoperation mode switch; goto dredge position and execute grasp buttons...    
    QObject::connect(ui.executeGraspingButton, SIGNAL(clicked()), this, SLOT(executeGrasping()));
    QObject::connect(ui.executeDredgingButton, SIGNAL(clicked()), this, SLOT(executeDredging()));
    QObject::connect(ui.addWaypointButton, SIGNAL(clicked()), this, SLOT(addWaypoint()));
    QObject::connect(ui.clearWaypointsButton, SIGNAL(clicked()), this, SLOT(clearWaypoints()));
    QObject::connect(ui.removeLastWaypointButton, SIGNAL(clicked()), this, SLOT(removeLastWaypoint()));

	//Connecting ROS callbacks
	nh = new ros::NodeHandle();
	image_transport::ImageTransport it(*nh);
	sub_g500Odometry		= nh->subscribe<auv_msgs::NavSts>(ui.g500TopicOdometry->text().toUtf8().constData(), 1, &MainWindow::g500OdometryCallback, this); 
	sub_g500Battery			= nh->subscribe<cola2_msgs::BatteryLevel>(ui.g500TopicBatteryLevel->text().toUtf8().constData(), 1, &MainWindow::g500BatteryCallback, this); 
	sub_g500Runningtime		= nh->subscribe<cola2_msgs::TotalTime>(ui.g500TopicRunningTime->text().toUtf8().constData(), 1, &MainWindow::g500RunningTimeCallback, this); 
	sub_g500Diagnostics		= nh->subscribe<diagnostic_msgs::DiagnosticArray>(ui.g500TopicDiagnostics->text().toUtf8().constData(), 1, &MainWindow::g500DiagnosticsCallback, this); 

	sub_sparusOdometry		= nh->subscribe<auv_msgs::NavSts>(ui.sparusTopicOdometry->text().toUtf8().constData(), 1, &MainWindow::sparusOdometryCallback, this); 
	sub_sparusBattery		= nh->subscribe<cola2_msgs::BatteryLevel>(ui.sparusTopicBatteryLevel->text().toUtf8().constData(), 1, &MainWindow::sparusBatteryCallback, this); 
	sub_sparusRunningtime	= nh->subscribe<cola2_msgs::TotalTime>(ui.sparusTopicRunningTime->text().toUtf8().constData(), 1, &MainWindow::sparusRunningTimeCallback, this); 
	sub_sparusDiagnostics	= nh->subscribe<diagnostic_msgs::DiagnosticArray>(ui.sparusTopicDiagnostics->text().toUtf8().constData(), 1, &MainWindow::sparusDiagnosticsCallback, this); 

	srv_g500GoTo 	= nh->serviceClient<cola2_msgs::Goto>(ui.g500TopicGoToService->text().toUtf8().constData());

	sub_imageTopic	= nh->subscribe<sensor_msgs::Image>(ui.vsCameraInput->text().toUtf8().constData(), 1, &MainWindow::imageCallback, this); 
	pub_target		= it.advertise(ui.vsCroppedImage->text().toUtf8().constData(), 1);

	sub_spec_params	= nh->subscribe<std_msgs::Float32MultiArray>("/specification_params_to_gui", 1, &MainWindow::specParamsCallback, this);
	pub_spec_params	= nh->advertise<std_msgs::Float32MultiArray>("/specification_params_to_uwsim", 1);
	pub_spec_action	= nh->advertise<std_msgs::String>("/specification_status", 1);
  pub_dredg_action	= nh->advertise<std_msgs::String>("/dredging_status", 1);


    //Timer to ensure the ROS communications
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(processSpinOnce()));
    connect(timer, SIGNAL(timeout()), this, SLOT(publishCroppedImage()));
    timer->start();

	//sub_joystick		= nh->subscribe<sensor_msgs::Joy>("/joystick_out", 1, &MainWindow::joystickCallback, this); 
	

    //VisualServoing user interaction init
    ui.vsCameraInputViewer->setPixmap(pixmapTopic);
    ui.vsCameraInputViewer->installEventFilter(this);
    roiStarted = false;
    x0 = 0; y0 = 0;
    x1 = 1; y1 = 1;

    activeCurrentVS = false;

}


void MainWindow::testButton()
{
	qDebug() << "testButton";
	SetRobotPoseDlg *dlg = new SetRobotPoseDlg(nh, this);
	dlg->show();
}

MainWindow::~MainWindow() { }



/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage()
{
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}


void MainWindow::processSpinOnce()
{
	//We need to exectue the SpinOnce with the timer
	ros::spinOnce();
}


void MainWindow::g500TopicsButtonClicked()
{
	qDebug()<<"g500TopicsButton clicked: reconnecting all the G500 topics...";
	sub_g500Odometry.shutdown();
	sub_g500Battery.shutdown();
	sub_g500Runningtime.shutdown();
	sub_g500Diagnostics.shutdown();
	qDebug()<<"g500 topics have been shutdown";
	sub_g500Odometry	= nh->subscribe<auv_msgs::NavSts>(ui.g500TopicOdometry->text().toUtf8().constData(), 1, &MainWindow::g500OdometryCallback, this); 
	sub_g500Battery		= nh->subscribe<cola2_msgs::BatteryLevel>(ui.g500TopicBatteryLevel->text().toUtf8().constData(), 1, &MainWindow::g500BatteryCallback, this); 
	sub_g500Runningtime	= nh->subscribe<cola2_msgs::TotalTime>(ui.g500TopicRunningTime->text().toUtf8().constData(), 1, &MainWindow::g500RunningTimeCallback, this); 
	sub_g500Diagnostics	= nh->subscribe<diagnostic_msgs::DiagnosticArray>(ui.g500TopicDiagnostics->text().toUtf8().constData(), 1, &MainWindow::g500DiagnosticsCallback, this); 
	qDebug()<<"g500 topics have been reconnected";
}


void MainWindow::sparusTopicsButtonClicked()
{
	qDebug()<<"sparusTopicsButton clicked: reconnecting all the G500 topics";
	sub_sparusOdometry.shutdown();
	sub_sparusBattery.shutdown();
	sub_sparusRunningtime.shutdown();
	sub_sparusDiagnostics.shutdown();
	qDebug()<<"sparus topics have been shutdown";
	sub_sparusOdometry		= nh->subscribe<auv_msgs::NavSts>(ui.sparusTopicOdometry->text().toUtf8().constData(), 1, &MainWindow::sparusOdometryCallback, this); 
	sub_sparusBattery		= nh->subscribe<cola2_msgs::BatteryLevel>(ui.sparusTopicBatteryLevel->text().toUtf8().constData(), 1, &MainWindow::sparusBatteryCallback, this); 
	sub_sparusRunningtime	= nh->subscribe<cola2_msgs::TotalTime>(ui.sparusTopicRunningTime->text().toUtf8().constData(), 1, &MainWindow::sparusRunningTimeCallback, this); 
	sub_sparusDiagnostics	= nh->subscribe<diagnostic_msgs::DiagnosticArray>(ui.sparusTopicDiagnostics->text().toUtf8().constData(), 1, &MainWindow::sparusDiagnosticsCallback, this); 
	qDebug()<<"sparus topics have been reconnected";
}


void MainWindow::g500LoadStream()
{
//    QString text = ui.g500StreamIP->text() + ":8080/stream?topic=" \
     				+ ui.g500StreamTopic->text() + "&type=" + ui.g500StreamType->currentText();
    QString text = ui.g500StreamIP->text() + ":8080/stream?topic=" \
     				+ ui.g500StreamTopic->text();
    qDebug() << "New G500 stream: " <<  text.toUtf8().constData();
    //ui.g500StreamView->load(QUrl("http://www.google.com"));
    ui.g500StreamView->load(QUrl("http://localhost:8080/stream?topic=/uwsim/camera1"));
    //ui.g500StreamView->load(text);
}


void MainWindow::g500LoadStream2()
{
    QString text = ui.g500StreamIP->text() + ":8080/stream?topic=" \
     				+ ui.g500StreamTopic->text() + "&type=" + ui.g500StreamType->currentText();
    qDebug() << "New G500 stream2: " <<  text.toUtf8().constData();
    ui.g500StreamView2->load(QUrl("http://www.google.com"));
    //ui.g500StreamView->load(text);
}


void MainWindow::sparusLoadStream()
{
    QString text = ui.sparusStreamIP->text() + ":8080/stream?topic=" \
     				+ ui.sparusStreamTopic->text() + "&type=" + ui.sparusStreamType->currentText();
    qDebug() << "New SPARUS stream: " <<  text.toUtf8().constData();
    ui.sparusStreamView->load(QUrl("http://www.google.com"));
    //ui.sparusStreamView->load(text);
}


void MainWindow::g500StopStream()
{
	qDebug() << "G500 stream stopped";
	ui.g500StreamView->stop();
}


void MainWindow::g500StopStream2()
{
	qDebug() << "G500 stream stopped";
	ui.g500StreamView2->stop();
}


void MainWindow::sparusStopStream()
{
	qDebug() << "SPARUS stream stopped";
	ui.sparusStreamView->stop();
}


void MainWindow::g500GoToSurface()
{
	qDebug() << "Sending the G500 to the surface...";
	cola2_msgs::Goto srv;
    srv.request.position.x		= 0;
    srv.request.position.y		= 0;
    srv.request.position.z		= 0;
    srv.request.blocking 		= false;
    srv.request.keep_position	= false;
    srv.request.position_tolerance.x = 0.4;
    srv.request.position_tolerance.y = 0.4;
    srv.request.position_tolerance.z = 0.4;
    srv.request.altitude_mode	= false;
    srv.request.priority		= 10;
    srv.request.reference 		= srv.request.REFERENCE_NED;

    if(srv_g500GoTo.call(srv))
    {
        qDebug() << "Service call success. Result: {}", srv.response.success ? "success" : "failed";
    }
    else
    {
        qDebug() << "Service call failed";
    }
}

void MainWindow::getInitGraspPose(){
  std_msgs::String msg;
  msg.data = "init";
  pub_spec_action.publish(msg);
  ros::spinOnce();
}

void MainWindow::setSpecificationMode( int tab ){

  std_msgs::String msg;
  if( tab == 0 ){
    msg.data = "guided";
  }else{
    msg.data = "interactive";
  }
  pub_spec_action.publish(msg);
  ros::spinOnce();
}


void MainWindow::updateInteractiveSpecParams(){
  std_msgs::Float32MultiArray msg;
  //Scale params. From int (deg fractions) to rad.
  msg.data.push_back(ui.interactiveSpecSlider1->value()/300.0);
  msg.data.push_back(ui.interactiveSpecSlider2->value()/300.0);
  msg.data.push_back(ui.interactiveSpecSlider3->value()/300.0);
  msg.data.push_back(ui.interactiveSpecSlider4->value()*3.14/180/4);
  msg.data.push_back(ui.interactiveSpecSlider5->value()*3.14/180/4);
  msg.data.push_back(ui.interactiveSpecSlider6->value()*3.14/180/4);
  msg.data.push_back(ui.gripperOpeningSlider->value());
  pub_spec_params.publish(msg);
  ros::spinOnce();
}

void MainWindow::executeGrasping(){
  std_msgs::String msg;
  msg.data = "execute";
  pub_spec_action.publish(msg);
  ros::spinOnce();
}

void MainWindow::executeDredging(){
  std_msgs::String msg;
  msg.data = "execute";
  pub_spec_action.publish(msg);
  ros::spinOnce();
}

void MainWindow::addWaypoint(){
  std_msgs::String msg;
  msg.data = "add";
  pub_dredg_action.publish(msg);
  ros::spinOnce();
}

void MainWindow::clearWaypoints(){
  std_msgs::String msg;
  msg.data = "clear";
  pub_dredg_action.publish(msg);
  ros::spinOnce();
}

void MainWindow::removeLastWaypoint(){
  std_msgs::String msg;
  msg.data = "delete";
  pub_dredg_action.publish(msg);
  ros::spinOnce();
}

void MainWindow::vsPublishButtonClicked()
{
	QPixmap tmpPixmap = croppedPixmapTopic.copy();
	QImage imageToSend = tmpPixmap.toImage().convertToFormat(QImage::Format_RGB888);

	//Converting the QImage to cv:Mat format
	cv::Mat cvImage;
	cvImage = cv::Mat(imageToSend.height(),
	                    imageToSend.width(),
	                    CV_8UC3,
	                    imageToSend.bits(),
	                    imageToSend.bytesPerLine());

	// Converting the image to sensor_msgs::ImagePtr bgr8
	cropeedImageMsg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cvImage).toImageMsg();
	activeCurrentVS = true;
}


void MainWindow::vsCancelButtonClicked()
{
	//Flag to disable publishing the cropped image
	activeCurrentVS = false;
}

void MainWindow::vsTopicsButtonClicked()
{
	qDebug()<<"vsTopicsButton clicked: reconnecting all the VisualServoing topics";
	sub_imageTopic.shutdown();
	pub_target.shutdown();
	qDebug()<<"VisualServoing topics have been shutdown";
	image_transport::ImageTransport it(*nh);
	sub_imageTopic  = nh->subscribe<sensor_msgs::Image>(ui.vsCameraInput->text().toUtf8().constData(), 1, &MainWindow::imageCallback, this); 
	pub_target		= it.advertise(ui.vsCroppedImage->text().toUtf8().constData(), 1);
	qDebug()<<"VisualServoing topics have been reconnected";	
}


void MainWindow::publishCroppedImage()
{
	if (activeCurrentVS)
	{	// Publishing the new target
		pub_target.publish(cropeedImageMsg);
	}
}

void MainWindow::updateAndResetInteractiveSpecParams(){
  std_msgs::Float32MultiArray msg;
  //Scale params. From int (deg fractions) to rad.
  msg.data.push_back(ui.interactiveSpecSlider1->value()/300.0);
  msg.data.push_back(ui.interactiveSpecSlider2->value()/300.0);
  msg.data.push_back(ui.interactiveSpecSlider3->value()/300.0);
  msg.data.push_back(ui.interactiveSpecSlider4->value()*3.14/180/4);
  msg.data.push_back(ui.interactiveSpecSlider5->value()*3.14/180/4);
  msg.data.push_back(ui.interactiveSpecSlider6->value()*3.14/180/4);
  pub_spec_params.publish(msg);

  std_msgs::String str_msg;
  str_msg.data = "markerReset";
  pub_spec_action.publish(str_msg);

  ros::spinOnce();

  ui.interactiveSpecSlider1->setValue(0);
  ui.interactiveSpecSlider2->setValue(0);
  ui.interactiveSpecSlider3->setValue(0);
  ui.interactiveSpecSlider4->setValue(0);
  ui.interactiveSpecSlider5->setValue(0);
  ui.interactiveSpecSlider6->setValue(0);


}

void MainWindow::updateGuidedSpecParams(){
  std_msgs::Float32MultiArray msg;
  msg.data.push_back(ui.guidedSpecSlider1->value());
  msg.data.push_back(ui.guidedSpecSlider2->value());
  msg.data.push_back(ui.guidedSpecSlider3->value());
  msg.data.push_back(ui.gripperOpeningSlider->value());
  pub_spec_params.publish(msg);
  ros::spinOnce();
}


/*****************************************************************************
** Implemenation [Callbacks]
*****************************************************************************/

void MainWindow::g500OdometryCallback(const auv_msgs::NavSts::ConstPtr& g500OdometryInfo)
{
	ui.g500OdometryTable->item(0, 0)->setText(QString::number(g500OdometryInfo->position.north));
	ui.g500OdometryTable->item(0, 1)->setText(QString::number(g500OdometryInfo->position.east));
	ui.g500OdometryTable->item(0, 2)->setText(QString::number(g500OdometryInfo->position.depth));
	ui.g500OdometryTable->item(0, 3)->setText(QString::number(g500OdometryInfo->orientation.roll));
	ui.g500OdometryTable->item(0, 4)->setText(QString::number(g500OdometryInfo->orientation.pitch));
	ui.g500OdometryTable->item(0, 5)->setText(QString::number(g500OdometryInfo->orientation.yaw));
}


void MainWindow::g500BatteryCallback(const cola2_msgs::BatteryLevel::ConstPtr& g500BatteryInfo)
{
	//QString labelText = "G500 BatteryLevel: " + QString::number(g500BatteryInfo->charge);
	//qDebug() << g500BatteryInfo->charge;
	ui.g500ServiceStatus->item(0, 0)->setText(QString::number(g500BatteryInfo->charge));
	//ui.g500BatteryLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
	//ui.g500BatteryLabel->setText(labelText);
}


void MainWindow::g500RunningTimeCallback(const cola2_msgs::TotalTime::ConstPtr& g500RunningTimeInfo)
{
	//QString labelText = "G500 RunningTime: " + QString::number(g500RunningTimeInfo->total_time);
	//qDebug() << g500RunningTimeInfo->total_time;
	ui.g500ServiceStatus->item(0, 1)->setText(QString::number(g500RunningTimeInfo->total_time));
}


void MainWindow::g500DiagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& g500DiagnosticsInfo)
{
	//stuff
}


void MainWindow::sparusOdometryCallback(const auv_msgs::NavSts::ConstPtr& sparusOdometryInfo)
{
	ui.sparusOdometryTable->item(0, 0)->setText(QString::number(sparusOdometryInfo->position.north));
	ui.sparusOdometryTable->item(0, 1)->setText(QString::number(sparusOdometryInfo->position.east));
	ui.sparusOdometryTable->item(0, 2)->setText(QString::number(sparusOdometryInfo->position.depth));
	ui.sparusOdometryTable->item(0, 3)->setText(QString::number(sparusOdometryInfo->orientation.roll));
	ui.sparusOdometryTable->item(0, 4)->setText(QString::number(sparusOdometryInfo->orientation.pitch));
	ui.sparusOdometryTable->item(0, 5)->setText(QString::number(sparusOdometryInfo->orientation.yaw));
}


void MainWindow::sparusBatteryCallback(const cola2_msgs::BatteryLevel::ConstPtr& sparusBatteryInfo)
{
	//QString labelText = "SPARUS BatteryLevel: " + QString::number(sparusBatteryInfo->charge);
	ui.sparusServiceStatus->item(0, 0)->setText(QString::number(sparusBatteryInfo->charge));
	//ui.sparusBatteryLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
	//ui.sparusBatteryLabel->setText(labelText);
}


void MainWindow::sparusRunningTimeCallback(const cola2_msgs::TotalTime::ConstPtr& sparusRunningTimeInfo)
{
	//QString labelText = "SPARUS RunningTime: " + QString::number(sparusRunningTimeInfo->total_time);
	ui.sparusServiceStatus->item(0, 1)->setText(QString::number(sparusRunningTimeInfo->total_time));
	//ui.sparusTimeLabel->setText(labelText);
}


void MainWindow::sparusDiagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& sparusDiagnosticsInfo)
{
	//stuff
}

void MainWindow::specParamsCallback(const std_msgs::Float32MultiArrayConstPtr& specificationParams){
  //Used to receive defaults.
  if(specificationParams->data.size()==4){
    ui.guidedSpecSlider1->setValue( specificationParams->data[0] );
    ui.guidedSpecSlider2->setValue( specificationParams->data[1] );
    ui.guidedSpecSlider3->setValue( specificationParams->data[2] );
    ui.gripperOpeningSlider->setValue( specificationParams->data[3] );
  }
}

void MainWindow::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	QImage dest (msg->data.data(), msg->width, msg->height, QImage::Format_RGB888);
	imageTopic = dest.copy();
	pixmapTopic = QPixmap::fromImage(imageTopic);
	width = pixmapTopic.width();
	height = pixmapTopic.height();
	ui.vsCameraInputViewer->setPixmap(pixmapTopic);
	drawCurrentROI();
}


/*****************************************************************************
** ToI selection
*****************************************************************************/
void MainWindow::mouseReleaseEvent(QMouseEvent * _event)
{
    endROI(x1, y1);
    if ((ui.mainTabs->currentIndex() == 2) and DebugTOI)
	    qDebug() << "Mouse release";
    QMainWindow::mouseReleaseEvent(_event);
}


bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    if(obj == ui.vsCameraInputViewer)
    {
        QEvent::Type etype = event->type();
        QPoint position;
        QString sposition;
        QMouseEvent * _event;
        if(etype == QEvent::MouseMove)
        {
            _event = static_cast<QMouseEvent*>(event);
            position = _event->pos();
            sposition = QString("(x: %0 ; y: %1 )").arg(QString::number(position.x()), QString::number(position.y()));
            notifyPoint1(position.x(), position.y());
			if ((ui.mainTabs->currentIndex() == 2) and DebugTOI)
        	    qDebug() << sposition << ": Mouse MOVE event";
        }
        else if (etype == QEvent::MouseButtonPress)
        {
            _event = static_cast<QMouseEvent*>(event);
            position = _event->pos();

            sposition = QString("(x: %0 ; y: %1 )").arg(QString::number(position.x()), QString::number(position.y()));
			startROI(position.x(), position.y());
			if ((ui.mainTabs->currentIndex() == 2) and DebugTOI)
            	qDebug() << sposition << ": Mouse button PRESSED";
        }
        else if (etype == QEvent::MouseButtonRelease)
        {
            _event = static_cast<QMouseEvent*>(event);
            position = _event->pos();

            sposition = QString("(x: %0 ; y: %1 )").arg(QString::number(position.x()), QString::number(position.y()));
			endROI(position.x(), position.y());
			if ((ui.mainTabs->currentIndex() == 2) and DebugTOI)
        	    qDebug() << sposition << ": Mouse button RELEASED";
        }
        else if(etype == QEvent::HoverMove)
        {
			if ((ui.mainTabs->currentIndex() == 2) and DebugTOI)
    	        qDebug() << sposition << "Mouse hover move event";
        }
        else if(etype == QEvent::MouseTrackingChange)
        {
			if ((ui.mainTabs->currentIndex() == 2) and DebugTOI)
	            qDebug() << sposition << "Mouse tracking change";
        }
    }

    return QMainWindow::eventFilter(obj, event);
}


bool MainWindow::validPoint0(int x, int y)
{

    return pointIn(x, y);
}


bool MainWindow::validPoint1(int x, int y)
{

    return pointIn(x,y) && x0 < x &&  y0 < y;
}


bool MainWindow::pointIn(int x, int y)
{

    return x >= 0 && y >= 0 && x < width && y < height;
}


void MainWindow::startROI(int _x0, int _y0)
{
    if(validPoint0(_x0, _y0))
    {
        x0 = _x0;
        y0 = _y0;
        roiStarted = true;
        QString sposition = QString("(x: %0 ; y: %1 )").arg(QString::number(x0), QString::number(y0));
        if ((ui.mainTabs->currentIndex() == 2) and DebugTOI)
	        qDebug() << sposition << ": START ROI";
    }
}


void MainWindow::endROI(int _x1, int _y1)
{
    if(validPoint1(_x1, _y1))
    {
        updatePoint1(_x1, _y1);
        roiStarted = false;
        QString sposition = QString("(x: %0 ; y: %1 )").arg(QString::number(x1), QString::number(y1));
        if ((ui.mainTabs->currentIndex() == 2) and DebugTOI)
	        qDebug() << sposition << ": END ROI";
    }
}


void MainWindow::notifyPoint1(int x, int y)
{
    if(validPoint1(x, y))
    {
        x1 = x;
    	y1 = y;
        QString sposition = QString("(x: %0 ; y: %1 )").arg(QString::number(x1), QString::number(y1));
		if ((ui.mainTabs->currentIndex() == 2) and DebugTOI)
        	qDebug() << sposition << ": updating ROI";
    }
}


void MainWindow::updatePoint1(int x, int y)
{
    x1 = x;
    y1 = y;
    drawCurrentROI();
}


void MainWindow::drawCurrentROI()
{
    QString sposition = QString("(x0: %0 ; y0: %1 ; x1: %2 ; y1: %3)").arg(
                QString::number(x0), QString::number(y0),QString::number(x1), QString::number(y1) );
    if ((ui.mainTabs->currentIndex() == 2) and DebugTOI)
	    qDebug() << sposition << ": Drawing Rectangle";

    painter.begin(&pixmapTopic);
    painter.setBrush(Qt::NoBrush);
    QPen pen(Qt::red, 3, Qt::DashDotLine, Qt::RoundCap, Qt::RoundJoin);
    painter.setPen(pen);
    painter.drawRect(x0, y0, x1-x0, y1-y0);
    ui.vsCameraInputViewer->setPixmap(pixmapTopic);
    painter.end();
    showCropROI();
}


void MainWindow::showCropROI() 
{
	QRect rect(x0+3, y0+3, x1-x0-6, y1-y0-6);
	croppedPixmapTopic = pixmapTopic.copy(rect);
	ui.vsTarget->setPixmap(croppedPixmapTopic);
}


/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::about(this, tr("About ..."),tr("<h2>MERBOTS GUI</h2> \
    											<p>Copyright IRSLab - Universitat Jaume I (<a href='mailto:irslab@uji.es'>irslab@uji.es</a>)</p> \
    											<p>This GUI will control the most relevant options to carry out the mission.</p>"));
}



/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace merbots_gui

