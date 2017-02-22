/**
 * Copyright (c) 2014 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 *
 * Author:
 *     Juan Carlos García
 *
 * Date February 2017
 **/


#include "../include/merbots_gui/set_robot_pose.h"
//#include "../include/merbots_gui/ui_set_robot_pose.h"
#include "ui_set_robot_pose.h"


SetRobotPoseDlg::SetRobotPoseDlg(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SetRobotPoseDlg)
{
    ui->setupUi(this);
    ui->pushButton->setText("Done");
   	QObject::connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(testButtonClicked()));

}

SetRobotPoseDlg::~SetRobotPoseDlg()
{
    delete ui;
}

void SetRobotPoseDlg::testButtonClicked()
{
	qDebug() << "Done button";
}