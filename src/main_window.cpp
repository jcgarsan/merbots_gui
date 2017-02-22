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


/*****************************************************************************
** Namespaces
*****************************************************************************/
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
		for (int j=0; j<6; j++)
		{
			//ui.g500OdometryTable->setItem(i, j, new QTableWidgetItem("0.0"));
			//ui.sparusOdometryTable->setItem(i, j, new QTableWidgetItem("0.0"));
		}


    //Main App connections
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	//Menu objects connections
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    //TAB1: Image stream connections
    QObject::connect(ui.g500LoadStreamButton, SIGNAL(clicked()), this, SLOT(g500LoadStream()));
    QObject::connect(ui.g500StopStreamButton, SIGNAL(clicked()), this, SLOT(g500StopStream()));
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

	//Connecting ROS callbacks
	nh = new ros::NodeHandle();
	sub_g500Odometry		= nh->subscribe<auv_msgs::NavSts>(ui.g500TopicOdometry->text().toUtf8().constData(), 1, &MainWindow::g500OdometryCallback, this); 
	sub_g500Battery			= nh->subscribe<cola2_msgs::BatteryLevel>(ui.g500TopicBatteryLevel->text().toUtf8().constData(), 1, &MainWindow::g500BatteryCallback, this); 
	sub_g500Runningtime		= nh->subscribe<cola2_msgs::TotalTime>(ui.g500TopicRunningTime->text().toUtf8().constData(), 1, &MainWindow::g500RunningTimeCallback, this); 
	sub_g500Diagnostics		= nh->subscribe<diagnostic_msgs::DiagnosticArray>(ui.g500TopicDiagnostics->text().toUtf8().constData(), 1, &MainWindow::g500DiagnosticsCallback, this); 

	sub_sparusOdometry		= nh->subscribe<auv_msgs::NavSts>(ui.sparusTopicOdometry->text().toUtf8().constData(), 1, &MainWindow::sparusOdometryCallback, this); 
	sub_sparusBattery		= nh->subscribe<cola2_msgs::BatteryLevel>(ui.sparusTopicBatteryLevel->text().toUtf8().constData(), 1, &MainWindow::sparusBatteryCallback, this); 
	sub_sparusRunningtime	= nh->subscribe<cola2_msgs::TotalTime>(ui.sparusTopicRunningTime->text().toUtf8().constData(), 1, &MainWindow::sparusRunningTimeCallback, this); 
	sub_sparusDiagnostics	= nh->subscribe<diagnostic_msgs::DiagnosticArray>(ui.sparusTopicDiagnostics->text().toUtf8().constData(), 1, &MainWindow::sparusDiagnosticsCallback, this); 

	srv_g500GoTo 			= nh->serviceClient<cola2_msgs::Goto>("/cola2_control/enable_goto");

  sub_spec_params = nh->subscribe<std_msgs::Float32MultiArray>("/specification_params_to_gui", 1, &MainWindow::specParamsCallback, this);
  pub_spec_params = nh->advertise<std_msgs::Float32MultiArray>("/specification_params_to_uwsim", 1);
  pub_spec_action = nh->advertise<std_msgs::String>("/specification_status", 1);

    //Timer to ensure the ROS communications
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(processSpinOnce()));
    timer->start();

	//sub_joystick		= nh->subscribe<sensor_msgs::Joy>("/joystick_out", 1, &MainWindow::joystickCallback, this); 
	
}





/*void MainWindow::joystickCallback(const sensor_msgs::Joy::ConstPtr& joystick)
{
	ros::Time currentPressUserControl = ros::Time::now();
	ros::Duration difTimeUserControl = currentPressUserControl - lastPressUserControl;
	if ((difTimeUserControl.toSec() > 0.5) and (joystick->buttons[0] == 1))
	{
		qDebug()<<"Inside if. ijoy= ";
		lastPressUserControl = currentPressUserControl;
	
		QString labelText = "G500 BatteryLevel: " + QString::number(joystick->axes[1]);
		ui.g500BatteryLabel->setStyleSheet("QLabel { background-color : red; color : blue; }");
		ui.g500BatteryLabel->setText(labelText);
	}
	if (ijoy>3)
		sub_g500Odometry.shutdown();

	for (int i=0; i<4; i++)
		ui.g500OdometryTable->item(0, i)->setText(QString::number(joystick->axes[i]));
	for (int i=0; i<4; i++)
		ui.g500OdometryTable->item(1, i)->setText(QString::number(joystick->buttons[i])); 
}*/



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
	ros::spinOnce();
}


void MainWindow::g500TopicsButtonClicked()
{
	qDebug()<<"g500TopicsButton clicked: reconnecting all the G500 topics...";
	sub_g500Odometry.shutdown();
	sub_g500Battery.shutdown();
	sub_g500Runningtime.shutdown();
	sub_g500Diagnostics.shutdown();
	qDebug()<<"g500Topics has been shutdown";
	sub_g500Odometry	= nh->subscribe<auv_msgs::NavSts>(ui.g500TopicOdometry->text().toUtf8().constData(), 1, &MainWindow::g500OdometryCallback, this); 
	sub_g500Battery		= nh->subscribe<cola2_msgs::BatteryLevel>(ui.g500TopicBatteryLevel->text().toUtf8().constData(), 1, &MainWindow::g500BatteryCallback, this); 
	sub_g500Runningtime	= nh->subscribe<cola2_msgs::TotalTime>(ui.g500TopicRunningTime->text().toUtf8().constData(), 1, &MainWindow::g500RunningTimeCallback, this); 
	sub_g500Diagnostics	= nh->subscribe<diagnostic_msgs::DiagnosticArray>(ui.g500TopicDiagnostics->text().toUtf8().constData(), 1, &MainWindow::g500DiagnosticsCallback, this); 
	qDebug()<<"g500Topics has been reconnected";
}


void MainWindow::sparusTopicsButtonClicked()
{
	qDebug()<<"g500TopicsButton clicked: reconnecting all the G500 topics";
	sub_sparusOdometry.shutdown();
	sub_sparusBattery.shutdown();
	sub_sparusRunningtime.shutdown();
	sub_sparusDiagnostics.shutdown();
	qDebug()<<"g500Topics has been shutdown";
	sub_sparusOdometry		= nh->subscribe<auv_msgs::NavSts>(ui.sparusTopicOdometry->text().toUtf8().constData(), 1, &MainWindow::sparusOdometryCallback, this); 
	sub_sparusBattery		= nh->subscribe<cola2_msgs::BatteryLevel>(ui.sparusTopicBatteryLevel->text().toUtf8().constData(), 1, &MainWindow::sparusBatteryCallback, this); 
	sub_sparusRunningtime	= nh->subscribe<cola2_msgs::TotalTime>(ui.sparusTopicRunningTime->text().toUtf8().constData(), 1, &MainWindow::sparusRunningTimeCallback, this); 
	sub_sparusDiagnostics	= nh->subscribe<diagnostic_msgs::DiagnosticArray>(ui.sparusTopicDiagnostics->text().toUtf8().constData(), 1, &MainWindow::sparusDiagnosticsCallback, this); 
	qDebug()<<"g500Topics has been reconnected";
}


void MainWindow::g500LoadStream()
{
    QString text = ui.g500StreamIP->text() + ":8080/stream?topic=" \
     				+ ui.g500StreamTopic->text() + "&type=" + ui.g500StreamType->currentText();
    qDebug() << "New G500 stream: " <<  text.toUtf8().constData();
    ui.g500StreamView->load(QUrl("http://www.google.com"));
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
    srv.request.blocking 		= true;
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
//	ui.g500OdometryTable->item(0, i)->setText(QString::number(g500OdometryInfo->));
}


void MainWindow::g500BatteryCallback(const cola2_msgs::BatteryLevel::ConstPtr& g500BatteryInfo)
{
	QString labelText = "G500 BatteryLevel: " + QString::number(g500BatteryInfo->charge);
	//ui.g500BatteryLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
	//ui.g500BatteryLabel->setText(labelText);
}


void MainWindow::g500RunningTimeCallback(const cola2_msgs::TotalTime::ConstPtr& g500RunningTimeInfo)
{
	QString labelText = "G500 RunningTime: " + QString::number(g500RunningTimeInfo->total_time);
	//ui.g500TimeLabel->setText(labelText);
}


void MainWindow::g500DiagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& g500DiagnosticsInfo)
{
	//stuff
}


void MainWindow::sparusOdometryCallback(const auv_msgs::NavSts::ConstPtr& sparusOdometryInfo)
{
//	ui.sparusOdometryTable->item(row, col)->setText(data);
}


void MainWindow::sparusBatteryCallback(const cola2_msgs::BatteryLevel::ConstPtr& sparusBatteryInfo)
{
	QString labelText = "SPARUS BatteryLevel: " + QString::number(sparusBatteryInfo->charge);
	//ui.sparusBatteryLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
	//ui.sparusBatteryLabel->setText(labelText);
}


void MainWindow::sparusRunningTimeCallback(const cola2_msgs::TotalTime::ConstPtr& sparusRunningTimeInfo)
{
	QString labelText = "SPARUS RunningTime: " + QString::number(sparusRunningTimeInfo->total_time);
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

