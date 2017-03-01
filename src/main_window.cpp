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
#include <merbots_ibvs/Rotate.h>
#include <math.h>

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
#include <sensor_msgs/image_encodings.h>


#define DEBUG	false


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

    activateVS         = true;
    g500CameraEnable   = false;
    g500CameraEnable2  = false;
    sparusCameraEnable = false;

	//Init section
	ros::init(argc,argv,"merbots_gui");
	ros::start(); 	//explicitly needed since our nodehandle is going out of scope
	if (!ros::master::check())
	{
		qDebug()<<"No ROS master found";
		showNoMasterMessage();
	}

	//Odometry tables init
	for (int i=0; i<4; i++)
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
    QObject::connect(ui.g500TopicsButton, SIGNAL(clicked()), this, SLOT(g500TopicsButtonClicked()));
    QObject::connect(ui.g500GoToPositionButton, SIGNAL(clicked()), this, SLOT(g500GoToPositionButtonClicked()));
    QObject::connect(ui.g500GoToSurfaceButton, SIGNAL(clicked()), this, SLOT(g500GoToSurface()));

    QObject::connect(ui.sparusLoadStreamButton, SIGNAL(clicked()), this, SLOT(sparusLoadStream()));
    QObject::connect(ui.sparusStopStreamButton, SIGNAL(clicked()), this, SLOT(sparusStopStream()));
    QObject::connect(ui.sparusTopicsButton, SIGNAL(clicked()), this, SLOT(sparusTopicsButtonClicked()));

    QObject::connect(ui.vsPublishButton, SIGNAL(clicked()),this, SLOT(vsPublishButtonClicked()));
    QObject::connect(ui.vsStartCameraButton, SIGNAL(clicked()), this, SLOT(vsStartCameraButtonClicked()));
	QObject::connect(ui.vsCancelButton, SIGNAL(clicked()),this, SLOT(vsCancelButtonClicked()));
    QObject::connect(ui.vsTopicsButton, SIGNAL(clicked()), this, SLOT(vsTopicsButtonClicked()));
    QObject::connect(ui.vsRotationButton, SIGNAL(clicked()), this, SLOT(vsRotationButtonClicked()));
    
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
    QObject::connect(ui.armTopicButton, SIGNAL(clicked()), this, SLOT(armTopicButtonClicked()));

	//Connecting ROS callbacks
	nh = new ros::NodeHandle();
    image_transport::ImageTransport it(*nh);
    sub_g500Odometry      = nh->subscribe<auv_msgs::NavSts>(ui.g500TopicOdometry->text().toUtf8().constData(), 1, &MainWindow::g500OdometryCallback, this); 
    sub_g500MergedBodyVel = nh->subscribe<auv_msgs::BodyVelocityReq>(ui.g500TopicMergedBodyVel->text().toUtf8().constData(), 1, &MainWindow::g500MergedBodyVelCallback, this); 
    sub_g500MergedWorld   = nh->subscribe<auv_msgs::WorldWaypointReq>(ui.g500TopicMergedWorld->text().toUtf8().constData(), 1, &MainWindow::g500MergedWorldCallback, this); 
	sub_g500Battery		  = nh->subscribe<cola2_msgs::BatteryLevel>(ui.g500TopicBatteryLevel->text().toUtf8().constData(), 1, &MainWindow::g500BatteryCallback, this); 
	sub_g500Runningtime	  = nh->subscribe<cola2_msgs::TotalTime>(ui.g500TopicRunningTime->text().toUtf8().constData(), 1, &MainWindow::g500RunningTimeCallback, this); 
	sub_g500Diagnostics	  = nh->subscribe<diagnostic_msgs::DiagnosticArray>(ui.g500TopicDiagnostics->text().toUtf8().constData(), 1, &MainWindow::g500DiagnosticsCallback, this); 

	sub_sparusOdometry	  = nh->subscribe<auv_msgs::NavSts>(ui.sparusTopicOdometry->text().toUtf8().constData(), 1, &MainWindow::sparusOdometryCallback, this); 
	sub_sparusBattery	  = nh->subscribe<cola2_msgs::BatteryLevel>(ui.sparusTopicBatteryLevel->text().toUtf8().constData(), 1, &MainWindow::sparusBatteryCallback, this); 
	sub_sparusRunningtime = nh->subscribe<cola2_msgs::TotalTime>(ui.sparusTopicRunningTime->text().toUtf8().constData(), 1, &MainWindow::sparusRunningTimeCallback, this); 
	sub_sparusDiagnostics = nh->subscribe<diagnostic_msgs::DiagnosticArray>(ui.sparusTopicDiagnostics->text().toUtf8().constData(), 1, &MainWindow::sparusDiagnosticsCallback, this); 

    srv_g500GoTo    = nh->serviceClient<cola2_msgs::Goto>(ui.g500TopicGoToService->text().toUtf8().constData());
    srv_vsRotation  = nh->serviceClient<merbots_ibvs::Rotate>(ui.vsRotationService->text().toUtf8().constData());
    srv_vsCancel    = nh->serviceClient<std_srvs::Empty>(ui.vsCancelService->text().toUtf8().constData());

    sub_g500Camera   = it.subscribe(ui.g500CameraTopic->text().toUtf8().constData(), 1, &MainWindow::g500CameraCallback, this);
    sub_sparusCamera = it.subscribe(ui.sparusCameraTopic->text().toUtf8().constData(), 1, &MainWindow::sparusCameraCallback, this);
    sub_imageTopic   = it.subscribe(ui.vsCameraInput->text().toUtf8().constData(), 1, &MainWindow::vsInputImageCallback, this);
	sub_resultTopic  = it.subscribe(ui.vsResult->text().toUtf8().constData(), 1, &MainWindow::vsResultImageCallback, this);
	pub_target		 = it.advertise(ui.vsCroppedImage->text().toUtf8().constData(), 1);

	sub_arm_state	 = nh->subscribe<sensor_msgs::JointState>(ui.armTopic->text().toUtf8().constData(), 1, &MainWindow::armStateCallback, this);
	sub_spec_params	 = nh->subscribe<std_msgs::Float32MultiArray>("/specification_params_to_gui", 1, &MainWindow::specParamsCallback, this);
	pub_spec_params	 = nh->advertise<std_msgs::Float32MultiArray>("/specification_params_to_uwsim", 1);
	pub_spec_action	 = nh->advertise<std_msgs::String>("/specification_status", 1);
    pub_dredg_action = nh->advertise<std_msgs::String>("/dredging_status", 1);


    //Timer to ensure the ROS communications
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(processSpinOnce()));
    timer->start();


    //VisualServoing user interaction init
    ui.vsCameraInputViewer->setPixmap(pixmapTopic);
    ui.vsCameraInputViewer->installEventFilter(this);
    x0 = 0; y0 = 0;
    x1 = 1; y1 = 1;


	dlg = new SetRobotPoseDlg(nh, this);
    QObject::connect(dlg, SIGNAL(newRobotPose(double, double, double, double, double, double)),
    				 this, SLOT(setRobotPosition(double, double, double, double, double, double)));

    //Parameters to set content label to fill the label size
    ui.g500StreamView->setScaledContents(true);
    ui.g500StreamView2->setScaledContents(true);
    ui.sparusStreamView->setScaledContents(true);
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
    sub_g500MergedBodyVel.shutdown();
    sub_g500MergedWorld.shutdown();
	sub_g500Battery.shutdown();
	sub_g500Runningtime.shutdown();
	sub_g500Diagnostics.shutdown();
	srv_g500GoTo.shutdown();
    sub_g500Camera.shutdown();
	qDebug()<<"g500 topics have been shutdown";
	sub_g500Odometry	  = nh->subscribe<auv_msgs::NavSts>(ui.g500TopicOdometry->text().toUtf8().constData(), 1, &MainWindow::g500OdometryCallback, this); 
    sub_g500MergedBodyVel = nh->subscribe<auv_msgs::BodyVelocityReq>(ui.g500TopicMergedBodyVel->text().toUtf8().constData(), 1, &MainWindow::g500MergedBodyVelCallback, this); 
    sub_g500MergedWorld   = nh->subscribe<auv_msgs::WorldWaypointReq>(ui.g500TopicMergedWorld->text().toUtf8().constData(), 1, &MainWindow::g500MergedWorldCallback, this); 
	sub_g500Battery		  = nh->subscribe<cola2_msgs::BatteryLevel>(ui.g500TopicBatteryLevel->text().toUtf8().constData(), 1, &MainWindow::g500BatteryCallback, this); 
	sub_g500Runningtime	  = nh->subscribe<cola2_msgs::TotalTime>(ui.g500TopicRunningTime->text().toUtf8().constData(), 1, &MainWindow::g500RunningTimeCallback, this); 
	sub_g500Diagnostics	  = nh->subscribe<diagnostic_msgs::DiagnosticArray>(ui.g500TopicDiagnostics->text().toUtf8().constData(), 1, &MainWindow::g500DiagnosticsCallback, this); 
	srv_g500GoTo 		  = nh->serviceClient<cola2_msgs::Goto>(ui.g500TopicGoToService->text().toUtf8().constData());
    image_transport::ImageTransport it(*nh);
    sub_g500Camera   = it.subscribe(ui.g500CameraTopic->text().toUtf8().constData(), 1, &MainWindow::g500CameraCallback, this);
	qDebug()<<"g500 topics have been reconnected";
}


void MainWindow::sparusTopicsButtonClicked()
{
	qDebug()<<"sparusTopicsButton clicked: reconnecting all the G500 topics";
	sub_sparusOdometry.shutdown();
	sub_sparusBattery.shutdown();
	sub_sparusRunningtime.shutdown();
	sub_sparusDiagnostics.shutdown();
    sub_sparusCamera.shutdown();
	qDebug()<<"sparus topics have been shutdown";
	sub_sparusOdometry		= nh->subscribe<auv_msgs::NavSts>(ui.sparusTopicOdometry->text().toUtf8().constData(), 1, &MainWindow::sparusOdometryCallback, this); 
	sub_sparusBattery		= nh->subscribe<cola2_msgs::BatteryLevel>(ui.sparusTopicBatteryLevel->text().toUtf8().constData(), 1, &MainWindow::sparusBatteryCallback, this); 
	sub_sparusRunningtime	= nh->subscribe<cola2_msgs::TotalTime>(ui.sparusTopicRunningTime->text().toUtf8().constData(), 1, &MainWindow::sparusRunningTimeCallback, this); 
	sub_sparusDiagnostics	= nh->subscribe<diagnostic_msgs::DiagnosticArray>(ui.sparusTopicDiagnostics->text().toUtf8().constData(), 1, &MainWindow::sparusDiagnosticsCallback, this); 
    image_transport::ImageTransport it(*nh);
    sub_sparusCamera = it.subscribe(ui.sparusCameraTopic->text().toUtf8().constData(), 1, &MainWindow::sparusCameraCallback, this);
	qDebug()<<"sparus topics have been reconnected";
}


void MainWindow::armTopicButtonClicked()
{
    qDebug()<<"armTopicsButton clicked: reconnecting all the G500 topics";
    sub_arm_state.shutdown();
    qDebug()<<"arm topic has been shutdown";
    sub_arm_state	= nh->subscribe<sensor_msgs::JointState>(ui.armTopic->text().toUtf8().constData(), 1, &MainWindow::armStateCallback, this);
    qDebug()<<"arm topic has been reconnected";
}


void MainWindow::g500LoadStream()
{

    g500CameraEnable = true;
}


void MainWindow::g500LoadStream2()
{

    g500CameraEnable2 = true;
}


void MainWindow::sparusLoadStream()
{

    sparusCameraEnable = true;
}


void MainWindow::g500StopStream()
{
    g500CameraEnable = false;
    ui.g500StreamView->clear();
    ui.g500StreamView->setText("G500 camera input");
}


void MainWindow::g500StopStream2()
{
    g500CameraEnable2 = false;
    ui.g500StreamView2->clear();
    ui.g500StreamView2->setText("G500 camera input");
}


void MainWindow::sparusStopStream()
{
    sparusCameraEnable = false;
    ui.sparusStreamView->clear();
    ui.sparusStreamView->setText("SPARUS camera input");
}


void MainWindow::g500GoToPositionButtonClicked()
{
    //This shows a QDialog to enter manually the robot desired position or get it from UWSim (ToDo)
	dlg->show();
}


void MainWindow::setRobotPosition(double xValueSrv, double yValueSrv, double zValueSrv, double rollValueSrv, double pitchValueSrv, double yawValueSrv)
{
    qDebug() << "Sending the G500 to the new position...";
    qDebug() << xValueSrv << " " << yValueSrv << " " << zValueSrv << " " << rollValueSrv << " " << pitchValueSrv << " " << yawValueSrv;

    cola2_msgs::Goto srv;
    srv.request.position.x      = xValueSrv;
    srv.request.position.y      = yValueSrv;
    srv.request.position.z      = zValueSrv;
    srv.request.disable_axis.x  = false;
    srv.request.disable_axis.y  = true;
    srv.request.disable_axis.z  = false;
    srv.request.disable_axis.roll  = true;
    srv.request.disable_axis.pitch = true;
    srv.request.disable_axis.yaw   = false;
    srv.request.blocking        = false;
    srv.request.keep_position   = false;
    srv.request.position_tolerance.x = 0.4;
    srv.request.position_tolerance.y = 0.4;
    srv.request.position_tolerance.z = 0.4;
    srv.request.altitude_mode   = false;
    srv.request.priority        = 10;
    srv.request.reference       = srv.request.REFERENCE_NED;

    if(srv_g500GoTo.call(srv))
    {
        qDebug() << "Service call success. Result: {}", srv.response.success ? "success" : "failed";
    }
    else
    {
        qDebug() << "Service call failed.";
        QMessageBox msgBox;
        msgBox.setText("Service call failed.");
        msgBox.exec();
    }
}

void MainWindow::g500GoToSurface()
{
	qDebug() << "Sending the G500 to the surface...";
	cola2_msgs::Goto srv;
    srv.request.position.z      = 0;
    srv.request.disable_axis.x  = true;
    srv.request.disable_axis.y  = true;
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
        qDebug() << "Service call failed.";
        QMessageBox msgBox;
        msgBox.setText("Service call failed.");
        msgBox.exec();
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
  msg.data = "send_first";
  pub_dredg_action.publish(msg);
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
  msg.data = "remove_last";
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
	croppedImageMsg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cvImage).toImageMsg();
    qDebug() << "Publishing ToI...";
	pub_target.publish(croppedImageMsg);
    activateVS = true;
}


void MainWindow::vsStartCameraButtonClicked()
{
    //Re-activate VS
    activateVS = true;
}


void MainWindow::vsCancelButtonClicked()
{
    std_srvs::Empty srv;

    if (srv_vsCancel.call(srv))
        qDebug() << "Current VS cancelled.";
    else
    {
        qDebug() << "Service call failed.";
        QMessageBox msgBox;
        msgBox.setText("Service call failed.");
        msgBox.exec();
    }
}


void MainWindow::vsTopicsButtonClicked()
{
	qDebug()<<"vsTopicsButton clicked: reconnecting all the VisualServoing topics";
    sub_imageTopic.shutdown();
	sub_resultTopic.shutdown();
	pub_target.shutdown();
	qDebug()<<"VisualServoing topics have been shutdown";
	image_transport::ImageTransport it(*nh);
    sub_imageTopic  = it.subscribe(ui.vsCameraInput->text().toUtf8().constData(), 1, &MainWindow::vsInputImageCallback, this);
    sub_resultTopic = it.subscribe(ui.vsResult->text().toUtf8().constData(), 1, &MainWindow::vsResultImageCallback, this);
	pub_target		= it.advertise(ui.vsCroppedImage->text().toUtf8().constData(), 1);
	qDebug()<<"VisualServoing topics have been reconnected";
}


void MainWindow::vsRotationButtonClicked()
{
    qDebug() << "VS rotation: " << ui.vsRotationDataSpinBox->value();
    double spinBoxData = ui.vsRotationDataSpinBox->value();

    QMessageBox msgBox;
    merbots_ibvs::Rotate srv;
    if (spinBoxData != 0.0)
    {
        srv.request.data = spinBoxData / 180.0 * 3.141592;
        if (srv_vsRotation.call(srv))
            qDebug() << "Service merbots_ibvs::Rotate called";
        else
        {
            qDebug() << "Service call failed.";
            msgBox.setText("Service call failed.");
            msgBox.exec();
        }
    }
    else
    {
        msgBox.setText("Rotation angle must be different to 0.");
        msgBox.exec();
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

double MainWindow::rad2grad(double rads)
{
    //Converts radians to grads
    return (rads*180/M_PI);
}

void MainWindow::g500OdometryCallback(const auv_msgs::NavSts::ConstPtr& g500OdometryInfo)
{
	ui.g500OdometryTable->item(0, 0)->setText(QString::number(g500OdometryInfo->position.north));
	ui.g500OdometryTable->item(0, 1)->setText(QString::number(g500OdometryInfo->position.east));
	ui.g500OdometryTable->item(0, 2)->setText(QString::number(g500OdometryInfo->position.depth));
	ui.g500OdometryTable->item(0, 3)->setText(QString::number(rad2grad(g500OdometryInfo->orientation.roll)));
	ui.g500OdometryTable->item(0, 4)->setText(QString::number(rad2grad(g500OdometryInfo->orientation.pitch)));
	ui.g500OdometryTable->item(0, 5)->setText(QString::number(rad2grad(g500OdometryInfo->orientation.yaw)));

    ui.g500OdometryTable->item(2, 0)->setText(QString::number(g500OdometryInfo->body_velocity.x));
    ui.g500OdometryTable->item(2, 1)->setText(QString::number(g500OdometryInfo->body_velocity.y));
    ui.g500OdometryTable->item(2, 2)->setText(QString::number(g500OdometryInfo->body_velocity.z));
    ui.g500OdometryTable->item(2, 3)->setText(QString::number(rad2grad(g500OdometryInfo->orientation_rate.roll)));
    ui.g500OdometryTable->item(2, 4)->setText(QString::number(rad2grad(g500OdometryInfo->orientation_rate.pitch)));
    ui.g500OdometryTable->item(2, 5)->setText(QString::number(rad2grad(g500OdometryInfo->orientation_rate.yaw)));
}


void MainWindow::g500MergedWorldCallback(const auv_msgs::WorldWaypointReq::ConstPtr& g500MergedWorldInfo)
{
    ui.g500OdometryTable->item(1, 0)->setText(QString::number(g500MergedWorldInfo->position.north));
    ui.g500OdometryTable->item(1, 1)->setText(QString::number(g500MergedWorldInfo->position.east));
    ui.g500OdometryTable->item(1, 2)->setText(QString::number(g500MergedWorldInfo->position.depth));
    ui.g500OdometryTable->item(1, 3)->setText(QString::number(rad2grad(g500MergedWorldInfo->orientation.roll)));
    ui.g500OdometryTable->item(1, 4)->setText(QString::number(rad2grad(g500MergedWorldInfo->orientation.pitch)));
    ui.g500OdometryTable->item(1, 5)->setText(QString::number(rad2grad(g500MergedWorldInfo->orientation.yaw)));

    if (g500MergedWorldInfo->disable_axis.x)
//        ui.g500OdometryTable->item(3, 0)->setBackground(Qt::lightGray);
        ui.g500OdometryTable->item(3, 0)->setForeground(Qt::gray);
    else
        ui.g500OdometryTable->item(3, 0)->setForeground(Qt::darkGray);

    if (g500MergedWorldInfo->disable_axis.y)
        ui.g500OdometryTable->item(3, 1)->setForeground(Qt::gray);
    else
        ui.g500OdometryTable->item(3, 1)->setForeground(Qt::darkGray);

    if (g500MergedWorldInfo->disable_axis.z)
        ui.g500OdometryTable->item(3, 2)->setForeground(Qt::gray);
    else
        ui.g500OdometryTable->item(3, 2)->setForeground(Qt::darkGray);

    if (g500MergedWorldInfo->disable_axis.roll)
        ui.g500OdometryTable->item(3, 3)->setForeground(Qt::gray);
    else
        ui.g500OdometryTable->item(3, 3)->setForeground(Qt::darkGray);

    if (g500MergedWorldInfo->disable_axis.pitch)
        ui.g500OdometryTable->item(3, 4)->setForeground(Qt::gray);
    else
        ui.g500OdometryTable->item(3, 4)->setForeground(Qt::darkGray);

    if (g500MergedWorldInfo->disable_axis.yaw)
        ui.g500OdometryTable->item(3, 5)->setForeground(Qt::gray);
    else
        ui.g500OdometryTable->item(3, 5)->setForeground(Qt::darkGray);
}


void MainWindow::g500MergedBodyVelCallback(const auv_msgs::BodyVelocityReq::ConstPtr& g500MergedBodyVelInfo)
{
    ui.g500OdometryTable->item(3, 0)->setText(QString::number(g500MergedBodyVelInfo->twist.linear.x));
    ui.g500OdometryTable->item(3, 1)->setText(QString::number(g500MergedBodyVelInfo->twist.linear.y));
    ui.g500OdometryTable->item(3, 2)->setText(QString::number(g500MergedBodyVelInfo->twist.linear.z));
    ui.g500OdometryTable->item(3, 3)->setText(QString::number(g500MergedBodyVelInfo->twist.angular.x));
    ui.g500OdometryTable->item(3, 4)->setText(QString::number(g500MergedBodyVelInfo->twist.angular.y));
    ui.g500OdometryTable->item(3, 5)->setText(QString::number(g500MergedBodyVelInfo->twist.angular.z));

    if (g500MergedBodyVelInfo->disable_axis.x)
//        ui.g500OdometryTable->item(3, 0)->setBackground(Qt::lightGray);
        ui.g500OdometryTable->item(3, 0)->setForeground(Qt::gray);
    else
        ui.g500OdometryTable->item(3, 0)->setForeground(Qt::darkGray);

    if (g500MergedBodyVelInfo->disable_axis.y)
        ui.g500OdometryTable->item(3, 1)->setForeground(Qt::gray);
    else
        ui.g500OdometryTable->item(3, 1)->setForeground(Qt::darkGray);

    if (g500MergedBodyVelInfo->disable_axis.z)
        ui.g500OdometryTable->item(3, 2)->setForeground(Qt::gray);
    else
        ui.g500OdometryTable->item(3, 2)->setForeground(Qt::darkGray);

    if (g500MergedBodyVelInfo->disable_axis.roll)
        ui.g500OdometryTable->item(3, 3)->setForeground(Qt::gray);
    else
        ui.g500OdometryTable->item(3, 3)->setForeground(Qt::darkGray);

    if (g500MergedBodyVelInfo->disable_axis.pitch)
        ui.g500OdometryTable->item(3, 4)->setForeground(Qt::gray);
    else
        ui.g500OdometryTable->item(3, 4)->setForeground(Qt::darkGray);

    if (g500MergedBodyVelInfo->disable_axis.yaw)
        ui.g500OdometryTable->item(3, 5)->setForeground(Qt::gray);
    else
        ui.g500OdometryTable->item(3, 5)->setForeground(Qt::darkGray);
}


void MainWindow::g500BatteryCallback(const cola2_msgs::BatteryLevel::ConstPtr& g500BatteryInfo)
{
	ui.g500ServiceStatus->item(0, 0)->setText(QString::number(g500BatteryInfo->charge));
	//ui.g500BatteryLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
	//ui.g500BatteryLabel->setText(labelText);
}


void MainWindow::g500RunningTimeCallback(const cola2_msgs::TotalTime::ConstPtr& g500RunningTimeInfo)
{

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
	ui.sparusServiceStatus->item(0, 0)->setText(QString::number(sparusBatteryInfo->charge));
	//ui.sparusBatteryLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
	//ui.sparusBatteryLabel->setText(labelText);
}


void MainWindow::sparusRunningTimeCallback(const cola2_msgs::TotalTime::ConstPtr& sparusRunningTimeInfo)
{
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


void MainWindow::g500CameraCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    QImage dest (msg->data.data(), msg->width, msg->height, QImage::Format_RGB888);

    QString msgEncoding = QString::fromUtf8(msg->encoding.c_str());
    if (msgEncoding.compare(QString::fromStdString(sensor_msgs::image_encodings::RGB8)) != 0)
        dest = dest.rgbSwapped();

    g500Image = dest.copy();
    g500Pixmap = QPixmap::fromImage(g500Image);
    if (g500CameraEnable)
        ui.g500StreamView->setPixmap(g500Pixmap);
    if (g500CameraEnable2)
        ui.g500StreamView2->setPixmap(g500Pixmap);
}


void MainWindow::sparusCameraCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    QImage dest (msg->data.data(), msg->width, msg->height, QImage::Format_RGB888);

    QString msgEncoding = QString::fromUtf8(msg->encoding.c_str());
    if (msgEncoding.compare(QString::fromStdString(sensor_msgs::image_encodings::RGB8)) != 0)
        dest = dest.rgbSwapped();

    sparusImage = dest.copy();
    sparusPixmap = QPixmap::fromImage(sparusImage);
    if (sparusCameraEnable)
        ui.sparusStreamView->setPixmap(sparusPixmap);
}


void MainWindow::vsInputImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	QImage dest (msg->data.data(), msg->width, msg->height, QImage::Format_RGB888);

	if (activateVS)
	{	
        QString msgEncoding = QString::fromUtf8(msg->encoding.c_str());
        if (msgEncoding.compare(QString::fromStdString(sensor_msgs::image_encodings::RGB8)) != 0)
            dest = dest.rgbSwapped();

        dest = dest.scaled(400, 400, Qt::KeepAspectRatio);
        imageTopic = dest.copy();
		pixmapTopic = QPixmap::fromImage(imageTopic);
		width = pixmapTopic.width();
		height = pixmapTopic.height();
		ui.vsCameraInputViewer->setPixmap(pixmapTopic);
		drawCurrentROI();
	}
}

void MainWindow::vsResultImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    QImage dest (msg->data.data(), msg->width, msg->height, QImage::Format_RGB888);

    //qDebug() << "msg->encoding:" << msg->encoding.c_str() << ".";
    QString msgEncoding = QString::fromUtf8(msg->encoding.c_str());
    if (msgEncoding.compare(QString::fromStdString(sensor_msgs::image_encodings::RGB8)) != 0)
        dest = dest.rgbSwapped();

    resultPixmapTopic = QPixmap::fromImage(dest);
    width = resultPixmapTopic.width();
    height = resultPixmapTopic.height();
    ui.vsResultViewer->setPixmap(resultPixmapTopic);
    drawCurrentROI();
}


void MainWindow::armStateCallback(const sensor_msgs::JointState::ConstPtr& armStateMsg)
{
  ui.armJointValues->item(0, 0)->setText(QString::number(armStateMsg->position[0]));
  ui.armJointValues->item(1, 0)->setText(QString::number(armStateMsg->position[1]));
  ui.armJointValues->item(2, 0)->setText(QString::number(armStateMsg->position[2]));
  ui.armJointValues->item(3, 0)->setText(QString::number(armStateMsg->position[3]));
  ui.armJointValues->item(4, 0)->setText(QString::number(armStateMsg->position[4]));
}


/*****************************************************************************
** ToI selection
*****************************************************************************/
void MainWindow::mouseReleaseEvent(QMouseEvent * _event)
{
    endROI(x1, y1);
    if ((ui.mainTabs->currentIndex() == 2) and DEBUG)
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
            notifyPoint(position.x(), position.y());
            drawCurrentROI();
			if ((ui.mainTabs->currentIndex() == 2) and DEBUG)
        	    qDebug() << sposition << ": Mouse MOVE event";
        }
        else if (etype == QEvent::MouseButtonPress)
        {
            _event = static_cast<QMouseEvent*>(event);
            position = _event->pos();

            sposition = QString("(x: %0 ; y: %1 )").arg(QString::number(position.x()), QString::number(position.y()));
			startROI(position.x(), position.y());
			activateVS = false;
			if ((ui.mainTabs->currentIndex() == 2) and DEBUG)
            	qDebug() << sposition << ": Mouse button PRESSED";
        }
        else if (etype == QEvent::MouseButtonRelease)
        {
            _event = static_cast<QMouseEvent*>(event);
            position = _event->pos();

            sposition = QString("(x: %0 ; y: %1 )").arg(QString::number(position.x()), QString::number(position.y()));
			endROI(position.x(), position.y());
			if ((ui.mainTabs->currentIndex() == 2) and DEBUG)
        	    qDebug() << sposition << ": Mouse button RELEASED";
        }
        else if(etype == QEvent::HoverMove)
        {
			if ((ui.mainTabs->currentIndex() == 2) and DEBUG)
    	        qDebug() << sposition << "Mouse hover move event";
        }
        else if(etype == QEvent::MouseTrackingChange)
        {
			if ((ui.mainTabs->currentIndex() == 2) and DEBUG)
	            qDebug() << sposition << "Mouse tracking change";
        }
    }

    return QMainWindow::eventFilter(obj, event);
}


bool MainWindow::validPoint(int x, int y)
{

    return pointIn(x, y);
}


bool MainWindow::pointIn(int x, int y)
{

    return x >= 0 && y >= 0 && x < width && y < height;
}


void MainWindow::startROI(int _x0, int _y0)
{
    if(validPoint(_x0, _y0))
    {
        x0 = _x0;
        y0 = _y0;
        QString sposition = QString("(x: %0 ; y: %1 )").arg(QString::number(x0), QString::number(y0));
        if ((ui.mainTabs->currentIndex() == 2) and DEBUG)
	        qDebug() << sposition << ": START ROI";
    }
}


void MainWindow::endROI(int _x1, int _y1)
{
    if(validPoint(_x1, _y1))
    {
        updatePoint(_x1, _y1);
        QString sposition = QString("(x: %0 ; y: %1 )").arg(QString::number(x1), QString::number(y1));
        if ((ui.mainTabs->currentIndex() == 2) and DEBUG)
	        qDebug() << sposition << ": END ROI";
    }
}


void MainWindow::notifyPoint(int x, int y)
{
    if(validPoint(x, y))
    {
        x1 = x;
    	y1 = y;
        QString sposition = QString("(x: %0 ; y: %1 )").arg(QString::number(x1), QString::number(y1));
		if ((ui.mainTabs->currentIndex() == 2) and DEBUG)
        	qDebug() << sposition << ": updating ROI";
    }
}


void MainWindow::updatePoint(int x, int y)
{
    x1 = x;
    y1 = y;
    drawCurrentROI();
}


void MainWindow::drawCurrentROI()
{
    QString sposition = QString("(x0: %0 ; y0: %1 ; x1: %2 ; y1: %3)").arg(
                QString::number(x0), QString::number(y0),QString::number(x1), QString::number(y1) );
    if ((ui.mainTabs->currentIndex() == 2) and DEBUG)
	    qDebug() << sposition << ": Drawing Rectangle";
	if (!activateVS)
	{
        QPixmap tmpPixmap = pixmapTopic.copy(); 
        painter.begin(&tmpPixmap);
	    painter.setBrush(Qt::NoBrush);
	    QPen pen(Qt::red, 3, Qt::DashDotLine, Qt::RoundCap, Qt::RoundJoin);
	    painter.setPen(pen);
	    painter.drawRect(x0, y0, x1-x0, y1-y0);
        ui.vsCameraInputViewer->setPixmap(tmpPixmap);
	    painter.end();
	    showCropROI();
	}
}


void MainWindow::showCropROI() 
{
    QRect rect(min(x0,x1)+3, min(y0,y1)+3, max(x1-x0,x0-x1)-6, max(y1-y0,y0-y1)-6);
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

