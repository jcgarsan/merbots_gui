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

#ifndef merbots_gui_MAIN_WINDOW_H
#define merbots_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include "ui_main_window.h"
#include "qnode.hpp"

#include <QtGui/QMainWindow>
#include <QtWebKit/QWebView>

#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

/*****************************************************************************
** Namespace
*****************************************************************************/
namespace merbots_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	int 				ijoy;
	ros::Time 			lastPressUserControl;
	std_msgs::Bool		armControlRequest;

	ros::NodeHandle		*nh;
	ros::Subscriber		sub_g500Odometry;
	ros::Publisher		pub_armControlRequest;

	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void processSpinOnce();
	
    /******************************************
    ** Manual connections
    *******************************************/
    void g500LoadStream();
    void g500StopStream();
    void sparusLoadStream();
    void sparusStopStream();


	/******************************************
	** Implemenation [Callbacks]
	*******************************************/
//	void g500OdometryTableUpdate(int row, int col, QString data);
	void g500OdometryTableUpdate();
	void sparusOdometryTableUpdate();
	void g500TopicsButtonClicked();

	void joystickCallback(const sensor_msgs::Joy::ConstPtr& joystick);


private Q_SLOTS:


private:
	Ui::MainWindowDesign	ui;
	QNode 					qnode;



};

}  // namespace merbots_gui

#endif // merbots_gui_MAIN_WINDOW_H
