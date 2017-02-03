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
	ui.mainTabs->setCurrentIndex(4); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).


	//Init section
	ros::init(argc,argv,"merbots_gui");
	ros::start(); 	//explicitly needed since our nodehandle is going out of scope
	
	if (!ros::master::check())
	{
		qDebug()<<"No ROS master";
		showNoMasterMessage();
	}

	for (int i=0; i<2; i++) //Odometry tables init
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
    QObject::connect(ui.sparusLoadStreamButton, SIGNAL(clicked()), this, SLOT(sparusLoadStream()));
    QObject::connect(ui.sparusStopStreamButton, SIGNAL(clicked()), this, SLOT(sparusStopStream()));
    QObject::connect(ui.sparusStreamIP, SIGNAL(returnPressed()), this, SLOT(sparusLoadStream()));
    QObject::connect(ui.sparusStreamTopic, SIGNAL(returnPressed()), this, SLOT(sparusLoadStream()));
    QObject::connect(ui.sparusStreamType, SIGNAL(currentIndexChanged(int)), this, SLOT(sparusLoadStream()));
    

    QObject::connect(ui.g500TopicsButton, SIGNAL(clicked()), this, SLOT(g500TopicsButtonClicked()));

	//ros::NodeHandle *nh;
	nh = new ros::NodeHandle();
    sub_g500Odometry = nh->subscribe<sensor_msgs::Joy>(ui.g500TopicOdometry->text().toUtf8().constData(), 1, &MainWindow::joystickCallback, this); 
	pub_armControlRequest = nh->advertise<std_msgs::Bool>("armControlRequest", 1);
	armControlRequest.data = 1;

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(processSpinOnce()));
    timer->start();

    ijoy = 0;
	
}



void MainWindow::g500TopicsButtonClicked()
{
	qDebug()<<"g500TopicsButton";
	pub_armControlRequest.publish(armControlRequest);
    sub_g500Odometry = nh->subscribe<sensor_msgs::Joy>(ui.g500TopicOdometry->text().toUtf8().constData(), 1, &MainWindow::joystickCallback, this); 
    ijoy = 0;
}

void MainWindow::joystickCallback(const sensor_msgs::Joy::ConstPtr& joystick)
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
	ros::spinOnce();
}


/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/
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


void MainWindow::g500OdometryTableUpdate()
{
//	ui.g500OdometryTable->item(row, col)->setText(data);
}


void MainWindow::sparusOdometryTableUpdate()
{
//	ui.sparusOdometryTable->item(row, col)->setText(data);
}


/*****************************************************************************
** Implemenation [Callbacks]
*****************************************************************************/


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

