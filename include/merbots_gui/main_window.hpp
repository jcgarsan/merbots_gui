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
#include "set_robot_pose.h"

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
#include <std_srvs/Empty.h>
#include <auv_msgs/NavSts.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/WorldWaypointReq.h>
#include <cola2_msgs/BatteryLevel.h>
#include <cola2_msgs/TotalTime.h>
#include <cola2_msgs/Goto.h>
#include <cola2_msgs/Setpoints.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
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
	ros::Subscriber		sub_g500Odometry, sub_g500MergedBodyVel, sub_g500MergedWorld, sub_g500Battery, sub_g500Runningtime, sub_g500Diagnostics;
	ros::Subscriber		sub_sparusOdometry, sub_sparusBattery, sub_sparusRunningtime, sub_sparusDiagnostics;
	ros::Subscriber		sub_arm_state;
	ros::Subscriber		sub_spec_params;
	ros::Publisher		pub_spec_action, pub_spec_params;
	ros::Publisher 		pub_dredg_action, pub_dredging;
    ros::ServiceClient  srv_g500GoTo;
	ros::ServiceClient 	srv_vsRotation, srv_vsCancel;

	image_transport::Subscriber sub_imageTopic, sub_resultTopic;
	image_transport::Subscriber	sub_g500Camera, sub_sparusCamera;
	image_transport::Publisher 	pub_target;

	sensor_msgs::ImagePtr 		croppedImageMsg;

	bool activateVS;
    bool g500CameraEnable, g500CameraEnable2, sparusCameraEnable;
    int  g500DiagnosticsErrorLevel, sparusDiagnosticsErrorLevel;
    
    QString g500DiagnosticsErrorName, sparusDiagnosticsErrorName;

	SetRobotPoseDlg *dlg;

	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    double rad2grad(double rads);

    //Arm data
	int axisindex[6];
	int AxisDir[5];
	int buttonindex[5], lastValue_[5];
	double error_;
	double scale_;
	double current;
	ros::Publisher vel_pub_;
	ros::Subscriber joy_sub_;
	ros::Subscriber arm_sub_, arm_angle_sub_;
	bool slewLocked_, jawRotateLocked_, jawOpenLocked_, fixSlew_, park_;
	double angles_[5], desired_[5];
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void armCallback(const sensor_msgs::JointState::ConstPtr& mes);
	void armAngleCallback(const sensor_msgs::JointState::ConstPtr& mes);
	sensor_msgs::JointState js;


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
    void g500GoToPositionButtonClicked();
    void setRobotPosition(double xValueSrv, double yValueSrv, double zValueSrv, 
                        double rollValueSrv, double pitchValueSrv, double yawValueSrv);

	void sparusTopicsButtonClicked();
    void sparusLoadStream();
    void sparusStopStream();

    void getInitGraspPose();
    void getInitGraspPoseRansac();
    void setSpecificationMode(int);
    void updateInteractiveSpecParams();
    void updateAndResetInteractiveSpecParams();
    void updateGuidedSpecParams();
    void updateGuidedSpecParamsSpin1();
    void updateGuidedSpecParamsSpin2();
    void updateGuidedSpecParamsSpin3();
    void publishSliders();
	void armPublisher();
    void dredgingPublisher();

    void executeGrasping();
    void executeDredging();
    void addWaypoint();
    void clearWaypoints();
    void removeLastWaypoint();
    void armTopicButtonClicked();

    void vsPublishButtonClicked();
    void vsStartCameraButtonClicked();
    void vsCancelButtonClicked();
    void vsTopicsButtonClicked();
    void vsRotationButtonClicked();

	/******************************************
	** Implemenation [Callbacks]
	*******************************************/
	void g500OdometryCallback(const auv_msgs::NavSts::ConstPtr& g500OdometryInfo);
    void g500MergedBodyVelCallback(const auv_msgs::BodyVelocityReq::ConstPtr& g500MergedBodyVelInfo);
    void g500MergedWorldCallback(const auv_msgs::WorldWaypointReq::ConstPtr& g500MergedWorldInfo);
	void g500BatteryCallback(const cola2_msgs::BatteryLevel::ConstPtr& g500BatteryInfo);
	void g500RunningTimeCallback(const cola2_msgs::TotalTime::ConstPtr& g500RunningTimeInfo);
	void g500DiagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& g500DiagnosticsInfo);
    void g500CameraCallback(const sensor_msgs::Image::ConstPtr& msg);

	void sparusOdometryCallback(const auv_msgs::NavSts::ConstPtr& sparusOdometryInfo);
    void sparusMergedBodyVelCallback(const auv_msgs::BodyVelocityReq::ConstPtr& sparusMergedBodyVelInfo);
    void sparusMergedWorldCallback(const auv_msgs::WorldWaypointReq::ConstPtr& sparusMergedWorldInfo);
	void sparusBatteryCallback(const cola2_msgs::BatteryLevel::ConstPtr& sparusBatteryInfo);
	void sparusRunningTimeCallback(const cola2_msgs::TotalTime::ConstPtr& sparusRunningTimeInfo);
	void sparusDiagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& sparusDiagnosticsInfo);
    void sparusCameraCallback(const sensor_msgs::Image::ConstPtr& msg);

    void armStateCallback(const sensor_msgs::JointState::ConstPtr& armStateMsg);
    void specParamsCallback(const std_msgs::Float32MultiArrayConstPtr& specificationParams);

    void vsInputImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void vsResultImageCallback(const sensor_msgs::Image::ConstPtr& msg);



private Q_SLOTS:

protected:
    bool eventFilter(QObject *, QEvent *);
    void updateROI(int x0, int y0, int x1, int y1);
    void mouseReleaseEvent(QMouseEvent *);

private:
	Ui::MainWindowDesign	ui;
	QNode 					qnode;

	QImage		imageTopic, g500Image, sparusImage;
    QPainter	painter;
    QPixmap		pixmapTopic, resultPixmapTopic, croppedPixmapTopic, g500Pixmap, sparusPixmap;

    int width, height;
    int x0,y0,x1,y1;

    void startROI(int x0, int y0);
    void endROI(int x1, int y1);
    void drawCurrentROI();
    void updatePoint(int x, int y);
    bool validPoint(int x, int y);
    void notifyPoint(int x, int y);
    bool pointIn(int x, int y);
    void showCropROI();

};

}  // namespace merbots_gui

#endif // merbots_gui_MAIN_WINDOW_H
