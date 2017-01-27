/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
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
	ui.mainTabs->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).

    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    QObject::connect(ui.sparusLoadStreamButton, SIGNAL(clicked()), this, SLOT(sparusLoadStream()));
    QObject::connect(ui.sparusStreamIP, SIGNAL(returnPressed()), this, SLOT(sparusLoadStream_returnPressed()));
    QObject::connect(ui.g500LoadStreamButton, SIGNAL(clicked()), this, SLOT(g500LoadStream()));
    QObject::connect(ui.g500StreamIP, SIGNAL(returnPressed()), this, SLOT(g500LoadStream_returnPressed()));

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
}

MainWindow::~MainWindow() {}



/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}



/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/
void MainWindow::g500LoadStream()
{
    QString text = "http://" + ui.g500StreamIP->text() + ":8080/stream?topic=" \
     				+ ui.g500StreamTopic->text() + "&type=" + ui.g500StreamType->currentText();
    cout << "New G500 stream: " <<  text.toUtf8().constData() << endl;
    ui.g500StreamView->load(QUrl("http://www.google.com"));
    //ui.g500StreamView->load(text);
}

void MainWindow::g500LoadStream_returnPressed()
{
	g500LoadStream();
}

void MainWindow::sparusLoadStream()
{
    QString text = "http://" + ui.sparusStreamIP->text() + ":8080/stream?topic=" \
     				+ ui.sparusStreamTopic->text() + "&type=" + ui.sparusStreamType->currentText();
    cout << "New SPARUS stream: " <<  text.toUtf8().constData() << endl;
    ui.sparusStreamView->load(QUrl("http://www.google.com"));
    //ui.sparusStreamView->load(text);
}

void MainWindow::sparusLoadStream_returnPressed()
{
	sparusLoadStream();
}
/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
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

