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


#ifndef SETROBOTPOSEDLG_H
#define SETROBOTPOSEDLG_H

#include <QDialog>
#include <QtGui/QMainWindow>
#include <QWidget>
#include <QtGui>
#include <QString>
#include <QtCore>



namespace Ui {
class SetRobotPoseDlg;
}

class SetRobotPoseDlg : public QDialog
{
    Q_OBJECT

public:
    explicit SetRobotPoseDlg(QWidget *parent = 0);
    ~SetRobotPoseDlg();

Q_SIGNALS:

public Q_SLOTS:
	void testButtonClicked();


private:
    Ui::SetRobotPoseDlg *ui;
};

#endif // SETROBOTPOSEDLG_H
