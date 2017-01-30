/**
 * @file /include/merbots_gui/main_window.hpp
 *
 * @brief Qt based gui for merbots_gui.
 *
 * @date November 2010
 **/
#ifndef merbots_gui_MAIN_WINDOW_H
#define merbots_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include <QtWebKit/QWebView>

#include "ui_main_window.h"
#include "qnode.hpp"

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
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	
    /******************************************
    ** Manual connections
    *******************************************/
    void g500LoadStream();
    void sparusLoadStream();

	/******************************************
	** Implemenation [Callbacks]
	*******************************************/
	//void g500BatteryChecker();


private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace merbots_gui

#endif // merbots_gui_MAIN_WINDOW_H
