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
#include <QtOpenGL>
#include <QWidget>
#include <QImage>
#include <QMouseEvent>
#include <QPainter>
#include <QTcpSocket>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <auv_msgs/NavSts.h>
#include <cola2_msgs/BatteryLevel.h>
#include <cola2_msgs/TotalTime.h>
#include <cola2_msgs/Goto.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Joy.h>
#include <compressed_image_transport/compression_common.h>


#ifndef Q_MOC_RUN
#include <image_transport/image_transport.h>
#endif
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

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
	ros::NodeHandle		*nh;
	ros::Subscriber		sub_g500Odometry, sub_g500Battery, sub_g500Runningtime, sub_g500Diagnostics;
	ros::Subscriber		sub_sparusOdometry, sub_sparusBattery, sub_sparusRunningtime, sub_sparusDiagnostics;
	ros::Subscriber		sub_imageTopic;
	ros::Subscriber		sub_spec_params;
	ros::Publisher		pub_spec_action, pub_spec_params;
	ros::ServiceClient 	srv_g500GoTo;

	
	image_transport::Publisher 	pub_target;
	sensor_msgs::ImagePtr 		cropeedImageMsg;

	QTcpSocket *tcpSocket;

	bool activeCurrentVS;



	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();


Q_SIGNALS:

public Q_SLOTS:
	void on_actionAbout_triggered();
	void processSpinOnce();

	void g500TopicsButtonClicked();
    void g500LoadStream();
    void g500LoadStream2();
    void g500StopStream();
    void g500StopStream2();
    void g500GoToSurface();
//    void g500MoveRobotButtonClicked();

	void sparusTopicsButtonClicked();
    void sparusLoadStream();
    void sparusStopStream();

    void getInitGraspPose();
    void setSpecificationMode(int);
    void updateInteractiveSpecParams();
    void updateAndResetInteractiveSpecParams();
    void updateGuidedSpecParams();

    void executeGrasping();
    void executeDredging();
    void addWaypoint();
    void clearWaypoints();
    void removeLastWaypoint();

    void vsPublishButtonClicked();
    void vsCancelButtonClicked();
    void vsTopicsButtonClicked();
    void publishCroppedImage();

    void testButton();
	void tcpDataReceive();

	/******************************************
	** Implemenation [Callbacks]
	*******************************************/
	void g500OdometryCallback(const auv_msgs::NavSts::ConstPtr& g500OdometryInfo);
	void g500BatteryCallback(const cola2_msgs::BatteryLevel::ConstPtr& g500BatteryInfo);
	void g500RunningTimeCallback(const cola2_msgs::TotalTime::ConstPtr& g500RunningTimeInfo);
	void g500DiagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& g500DiagnosticsInfo);

	void sparusOdometryCallback(const auv_msgs::NavSts::ConstPtr& sparusOdometryInfo);
	void sparusBatteryCallback(const cola2_msgs::BatteryLevel::ConstPtr& sparusBatteryInfo);
	void sparusRunningTimeCallback(const cola2_msgs::TotalTime::ConstPtr& sparusRunningTimeInfo);
	void sparusDiagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& sparusDiagnosticsInfo);

	void specParamsCallback(const std_msgs::Float32MultiArrayConstPtr& specificationParams);
	void imageCallback(const sensor_msgs::Image::ConstPtr& msg);


private Q_SLOTS:

protected:
    bool eventFilter(QObject *, QEvent *);
    void updateROI(int x0, int y0, int x1, int y1);
    void mouseReleaseEvent(QMouseEvent *);

private:
	Ui::MainWindowDesign	ui;
	QNode 					qnode;

	QImage		imageTopic;
    QPainter	painter;
    QPixmap		pixmapTopic, croppedPixmapTopic;

    int width, height;
    int x0,y0,x1,y1;
    bool roiStarted;

    void startROI(int x0, int y0);
    void endROI(int x1, int y1);
    void drawCurrentROI();
    void updatePoint1(int x, int y);
    bool validPoint0(int x, int y);
    bool validPoint1(int x, int y);
    void notifyPoint1(int x, int y);
    bool pointIn(int x, int y);
    void showCropROI();

};

}  // namespace merbots_gui

#endif // merbots_gui_MAIN_WINDOW_H
