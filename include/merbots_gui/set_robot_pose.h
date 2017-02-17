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


#ifndef SET_ROBOT_POSE_H
#define SET_ROBOT_POSE_H

#include <QDialog>
//#include "ui_set_robot_pose.h"


namespace Ui {
class set_robot_pose;
}

class set_robot_pose : public QDialog
{
    Q_OBJECT

public:
    explicit set_robot_pose(QWidget *parent = 0);
    ~set_robot_pose();

private:
    Ui::set_robot_pose *ui;
};

#endif // SET_ROBOT_POSE_H
