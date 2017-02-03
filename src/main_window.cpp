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
			ui.g500OdometryTable->setItem(i, j, new QTableWidgetItem("0.0"));
			ui.sparusOdometryTable->setItem(i, j, new QTableWidgetItem("0.0"));
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

    QObject::connect(ui.sparusLoadStreamButton, SIGNAL(clicked()), this, SLOT(sparusLoadStream()));
    QObject::connect(ui.sparusStopStreamButton, SIGNAL(clicked()), this, SLOT(sparusStopStream()));
    QObject::connect(ui.sparusStreamIP, SIGNAL(returnPressed()), this, SLOT(sparusLoadStream()));
    QObject::connect(ui.sparusStreamTopic, SIGNAL(returnPressed()), this, SLOT(sparusLoadStream()));
    QObject::connect(ui.sparusStreamType, SIGNAL(currentIndexChanged(int)), this, SLOT(sparusLoadStream()));
    QObject::connect(ui.sparusTopicsButton, SIGNAL(clicked()), this, SLOT(sparusTopicsButtonClicked()));
    


	//Connecting ROS callbacks
	nh = new ros::NodeHandle();
	sub_g500Odometry	= nh->subscribe<auv_msgs::NavSts>(ui.g500TopicOdometry->text().toUtf8().constData(), 1, &MainWindow::g500OdometryCallback, this); 
	sub_g500Battery		= nh->subscribe<cola2_msgs::BatteryLevel>(ui.g500TopicBatteryLevel->text().toUtf8().constData(), 1, &MainWindow::g500BatteryCallback, this); 
	sub_g500Runningtime	= nh->subscribe<cola2_msgs::TotalTime>(ui.g500TopicRunningTime->text().toUtf8().constData(), 1, &MainWindow::g500RunningTimeCallback, this); 
	sub_g500Diagnostics	= nh->subscribe<diagnostic_msgs::DiagnosticArray>(ui.g500TopicDiagnostics->text().toUtf8().constData(), 1, &MainWindow::g500DiagnosticsCallback, this); 

	sub_sparusOdometry		= nh->subscribe<auv_msgs::NavSts>(ui.sparusTopicOdometry->text().toUtf8().constData(), 1, &MainWindow::sparusOdometryCallback, this); 
	sub_sparusBattery		= nh->subscribe<cola2_msgs::BatteryLevel>(ui.sparusTopicBatteryLevel->text().toUtf8().constData(), 1, &MainWindow::sparusBatteryCallback, this); 
	sub_sparusRunningtime	= nh->subscribe<cola2_msgs::TotalTime>(ui.sparusTopicRunningTime->text().toUtf8().constData(), 1, &MainWindow::sparusRunningTimeCallback, this); 
	sub_sparusDiagnostics	= nh->subscribe<diagnostic_msgs::DiagnosticArray>(ui.sparusTopicDiagnostics->text().toUtf8().constData(), 1, &MainWindow::sparusDiagnosticsCallback, this); 


    //Timer to ensure the ROS communications
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(processSpinOnce()));
    timer->start();

	
}




/*void MainWindow::joystickCallback(const sensor_msgs::Joy::ConstPtr& joystick)
{
	ros::Time currentPressUserControl = ros::Time::now();
	ros::Duration difTimeUserControl = currentPressUserControl - lastPressUserControl;
	if ((difTimeUserControl.toSec() > 0.5) and (joystick->buttons[0] == 1))
	{
		qDebug()<<"Inside if. ijoy= " << ijoy;
		lastPressUserControl = currentPressUserControl;
		ijoy++;
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
	qDebug()<<"g500TopicsButton clicked: reconnecting all the G500 topics";
	sub_g500Odometry.shutdown();
	sub_g500Battery.shutdown();
	sub_g500Runningtime.shutdown();
	sub_g500Diagnostics.shutdown();
	sub_g500Odometry	= nh->subscribe<auv_msgs::NavSts>(ui.g500TopicOdometry->text().toUtf8().constData(), 1, &MainWindow::g500OdometryCallback, this); 
	sub_g500Battery		= nh->subscribe<cola2_msgs::BatteryLevel>(ui.g500TopicBatteryLevel->text().toUtf8().constData(), 1, &MainWindow::g500BatteryCallback, this); 
	sub_g500Runningtime	= nh->subscribe<cola2_msgs::TotalTime>(ui.g500TopicRunningTime->text().toUtf8().constData(), 1, &MainWindow::g500RunningTimeCallback, this); 
	sub_g500Diagnostics	= nh->subscribe<diagnostic_msgs::DiagnosticArray>(ui.g500TopicDiagnostics->text().toUtf8().constData(), 1, &MainWindow::g500DiagnosticsCallback, this); 
}


void MainWindow::sparusTopicsButtonClicked()
{
	qDebug()<<"g500TopicsButton clicked: reconnecting all the G500 topics";
	sub_sparusOdometry.shutdown();
	sub_sparusBattery.shutdown();
	sub_sparusRunningtime.shutdown();
	sub_sparusDiagnostics.shutdown();
	sub_sparusOdometry		= nh->subscribe<auv_msgs::NavSts>(ui.sparusTopicOdometry->text().toUtf8().constData(), 1, &MainWindow::sparusOdometryCallback, this); 
	sub_sparusBattery		= nh->subscribe<cola2_msgs::BatteryLevel>(ui.sparusTopicBatteryLevel->text().toUtf8().constData(), 1, &MainWindow::sparusBatteryCallback, this); 
	sub_sparusRunningtime	= nh->subscribe<cola2_msgs::TotalTime>(ui.sparusTopicRunningTime->text().toUtf8().constData(), 1, &MainWindow::sparusRunningTimeCallback, this); 
	sub_sparusDiagnostics	= nh->subscribe<diagnostic_msgs::DiagnosticArray>(ui.sparusTopicDiagnostics->text().toUtf8().constData(), 1, &MainWindow::sparusDiagnosticsCallback, this); 
}


void MainWindow::g500LoadStream()
{
    QString text = "http://" + ui.g500StreamIP->text() + ":8080/stream?topic=" \
     				+ ui.g500StreamTopic->text() + "&type=" + ui.g500StreamType->currentText();
    qDebug() << "New G500 stream: " <<  text.toUtf8().constData();
    ui.g500StreamView->load(QUrl("http://www.google.com"));
    //ui.g500StreamView->load(text);
}



void MainWindow::sparusLoadStream()
{
    QString text = "http://" + ui.sparusStreamIP->text() + ":8080/stream?topic=" \
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
	ui.g500BatteryLabel->setText(labelText);
}


void MainWindow::g500RunningTimeCallback(const cola2_msgs::TotalTime::ConstPtr& g500RunningTimeInfo)
{
	QString labelText = "G500 RunningTime: " + QString::number(g500RunningTimeInfo->total_time);
	ui.g500TimeLabel->setText(labelText);
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
	ui.sparusBatteryLabel->setText(labelText);
}

void MainWindow::sparusRunningTimeCallback(const cola2_msgs::TotalTime::ConstPtr& sparusRunningTimeInfo)
{
	QString labelText = "SPARUS RunningTime: " + QString::number(sparusRunningTimeInfo->total_time);
	ui.sparusTimeLabel->setText(labelText);
}


void MainWindow::sparusDiagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& sparusDiagnosticsInfo)
{
	//stuff
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

