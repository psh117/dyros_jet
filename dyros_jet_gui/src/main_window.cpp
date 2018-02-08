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

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <QString>
#include <fstream>
#include <string>
#include "dyros_jet_gui/main_window.hpp"
#include <qpixmap.h>
#include <QQuaternion>
#include <QVector3D>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dyros_jet_gui {

const char *jointName[32] = { "HeadYaw", "HeadPitch"
                   , "WaistPitch", "WaistYaw"
                   , "R_ShoulderPitch", "R_ShoulderRoll", "R_ShoulderYaw", "R_ElbowRoll", "R_WristYaw", "R_WristRoll", "R_HandYaw", "R_Gripper"
                   , "L_ShoulderPitch", "L_ShoulderRoll", "L_ShoulderYaw", "L_ElbowRoll", "L_WristYaw", "L_WristRoll", "L_HandYaw", "L_Gripper"
                   , "R_HipYaw", "R_HipRoll", "R_HipPitch", "R_KneePitch", "R_AnklePitch", "R_AnkleRoll"
                   , "L_HipYaw", "L_HipRoll", "L_HipPitch", "L_KneePitch", "L_AnklePitch", "L_AnkleRoll"};

const char *disarranged_jointName[32] = {"L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll"
    ,"R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll"
    ,"WaistPitch","WaistYaw"
    ,"L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw"
    ,"R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw"
    ,"HeadYaw", "HeadPitch", "R_Gripper", "L_Gripper"};

const int MainWindow::head_id_index[2] = {29, 30};
const int MainWindow::waist_id_index[2] = {13, 14};
const int MainWindow::right_arm_id_index[8] = {22, 23, 24, 25, 26, 27, 28, 31};
const int MainWindow::left_arm_id_index[8] = {15, 16, 17, 18, 19, 20, 21, 32};
const int MainWindow::right_leg_id_index[6] = {7, 8, 9, 10, 11, 12};
const int MainWindow::left_leg_id_index[6] = {1, 2, 3, 4, 5, 6};

using namespace Qt;

// std::string motorID = {"R-SP", "L-SP", "R-SR", ""};
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode(argc,argv)
  , isConnected(false)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  ui.groupBox_3->setStyleSheet("#groupBox_3 {background-image: url(:/images/robot_with_no_background.png); background-repeat: none; background-position: center;}");

  ReadSettings();
  setWindowIcon(QIcon(":/images/icon_2.png"));
  ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  /*********************
  ** Logging
  **********************/
  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  QObject::connect(&qnode, SIGNAL(jointStateUpdated()), this, SLOT(updateJointView()));
  QObject::connect(&qnode, SIGNAL(recogInfoUpdated()), this, SLOT(updateRecogInfo()));

  /*********************
    ** Auto Start
    **********************/


  // | Creating UI


  QPalette* palette_estop = new QPalette();
  palette_estop->setColor(QPalette::Button, QColor(255,187,0));
  ui.button_estop->setAutoFillBackground(true);
  ui.button_estop->setPalette(*palette_estop);
  ui.button_estop->update();
  // -- Table
  if ( ui.checkbox_remember_settings->isChecked() ) {
    on_button_connect_clicked(true);
  }
//  QStringList horizonHeaderLabel;
//  horizonHeaderLabel.append("Name");
//  horizonHeaderLabel.append("Pos");
//  horizonHeaderLabel.append("Torque");
//  horizonHeaderLabel.append("Error");
//  ui.motor_table->setHorizontalHeaderLabels(horizonHeaderLabel);
  for(int i=0; i<32; i++)
  {
    jointID.push_back(i+1);
  }

  // -- ID Arrangement
    for(int i = 0; i < 32; i++)
    {
      if(i < 2) arranged_id[i] = head_id_index[i];
      else if(i < 4) arranged_id[i] = waist_id_index[i-2];
      else if(i < 12) arranged_id[i] = right_arm_id_index[i-4];
      else if(i < 20) arranged_id[i] = left_arm_id_index[i-12];
      else if(i < 26) arranged_id[i] = right_leg_id_index[i-20];
      else arranged_id[i] = left_leg_id_index[i-26];
    }
  // -- State

  autoMissionSelectVisible(0);
  QObject::connect(ui.button_power_on, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));

  QObject::connect(ui.button_auto, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));

  QObject::connect(ui.button_auto_door_init, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_auto_door_open, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_auto_door_push, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_auto_door_reach, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_auto_door_ready, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_auto_door_start, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));

  QObject::connect(ui.button_auto_valve_approach, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_auto_valve_close, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_auto_valve_init, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_auto_valve_reach, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_auto_valve_ready, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_auto_valve_start, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));

  QObject::connect(ui.button_auto_egress_init, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_auto_egress_egress, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_auto_egress_standby, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_auto_egress_start, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_auto_egress_guide, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_auto_egress_hello, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));

  QObject::connect(ui.button_manual, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));

  QObject::connect(ui.button_manual_joint_ctrl, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_manual_task_ctrl, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_manual_recog_ctrl,SIGNAL(clicked()),this,SLOT(stateButtonClicked()));

  QObject::connect(ui.button_mode_change, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));


  QObject::connect(ui.button_event, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));

  QObject::connect(ui.button_event_handclap, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_event_handclap_ready, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_event_handclap_do, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_event_handclap_end, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));

  QObject::connect(ui.button_event_handshake, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_event_handshake_turn, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_event_handshake_ready, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_event_handshake_do, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_event_handshake_end, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_event_handshake_motion1,SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_event_handshake_motion2,SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_event_handshake_motion3,SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_event_handshake_return,SIGNAL(clicked()), this, SLOT(stateButtonClicked()));

  QObject::connect(ui.button_event_hello, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_event_hello_ready, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_event_hello_do, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_event_hello_end, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
  QObject::connect(ui.button_event_hello_introduce, SIGNAL(clicked()),this,SLOT(stateButtonClicked()));
  QObject::connect(ui.button_event_hello_introduce_end, SIGNAL(clicked()),this,SLOT(stateButtonClicked()));



  QObject::connect(ui.button_scan,SIGNAL(clicked()),this,SLOT(on_button_scan_clicked()));

  // -- Joint Control Set
  QString tempName;
  for (int i = 0; i < 32; i++)
  {
    button_joint_ctrl[i][0] = new QPushButton("-1");
    button_joint_ctrl[i][1] = new QPushButton("+1");
    button_joint_ctrl[i][2] = new QPushButton("SET");
    doubleSpin_joint_ctrl[i] = new QDoubleSpinBox;

    if(i < 2) tempName = ui.h_table->item(i, 0)->text();
    else if(i < 4) tempName = ui.w_table->item(i-2, 0)->text();
    else if(i < 12) tempName = ui.ra_table->item(i-4, 0)->text();
    else if(i < 20) tempName = ui.la_table->item(i-12, 0)->text();
    else if(i < 26) tempName = ui.rl_table->item(i-20, 0)->text();
    else tempName = ui.ll_table->item(i-26, 0)->text();

    label_joint_ctrl[i] = new QLabel(tempName);

    //labels_joint_ctrl_ids[i].setText(text);

    button_joint_ctrl[i][0]->setMaximumWidth(40);
    button_joint_ctrl[i][1]->setMaximumWidth(40);
    button_joint_ctrl[i][2]->setMaximumWidth(60);

    button_joint_ctrl[i][0]->setObjectName(tr("%1").arg(i+1));
    button_joint_ctrl[i][1]->setObjectName(tr("%1").arg(i+1));
    button_joint_ctrl[i][2]->setObjectName(tr("%1").arg(i+1));

    doubleSpin_joint_ctrl[i]->setMaximumWidth(80);
    doubleSpin_joint_ctrl[i]->setValue(0.0);
    doubleSpin_joint_ctrl[i]->setMinimum(-20.0);
    doubleSpin_joint_ctrl[i]->setMaximum(20.0);

    label_joint_ctrl[i]->setMaximumWidth(500);
    label_joint_ctrl[i]->setAlignment(Qt::AlignCenter);


    ui.gridLayout_joint_ctrl->addWidget(button_joint_ctrl[i][0], i, 0, Qt::AlignTop);
    ui.gridLayout_joint_ctrl->addWidget(label_joint_ctrl[i]    , i, 1, Qt::AlignTop);
    ui.gridLayout_joint_ctrl->addWidget(button_joint_ctrl[i][1], i, 2, Qt::AlignTop);
    ui.gridLayout_joint_ctrl->addWidget(doubleSpin_joint_ctrl[i] , i, 3, Qt::AlignTop);
    ui.gridLayout_joint_ctrl->addWidget(button_joint_ctrl[i][2], i, 4, Qt::AlignTop);

    QObject::connect(button_joint_ctrl[i][0], SIGNAL(clicked()), this, SLOT(jointCtrlMinusClicked()));
    QObject::connect(button_joint_ctrl[i][1], SIGNAL(clicked()), this, SLOT(jointCtrlPlusClicked()));
    QObject::connect(button_joint_ctrl[i][2], SIGNAL(clicked()), this, SLOT(jointCtrlSetClicked()));
  }

  // -- Task Control Set
  for (int i = 0; i < 24; i++)
  {

    if(i < 12)
    {
      button_task_ctrl[i][0] = new QPushButton("<");
      button_task_ctrl[i][1] = new QPushButton(">");

      doubleSpin_task_ctrl[i] = new QDoubleSpinBox;
    }
    else
    {
      button_task_ctrl_2[i-12][0] = new QPushButton("<");
      button_task_ctrl_2[i-12][1] = new QPushButton(">");

      doubleSpin_task_ctrl_2[i-12] = new QDoubleSpinBox;
    }




    QString text = "";
    switch(i)
    {
    case 0:
      text = "left arm x";
      break;
    case 1:
      text = "left arm y";
      break;
    case 2:
      text = "left arm z";
      break;
    case 3:
      text = "left arm r";
      break;
    case 4:
      text = "left arm p";
      break;
    case 5:
      text = "left arm y";
      break;
    case 6:
      text = "left leg x";
      break;
    case 7:
      text = "left leg y";
      break;
    case 8:
      text = "left leg z";
      break;
    case 9:
      text = "left leg r";
      break;
    case 10:
      text = "left leg p";
      break;
    case 11:
      text = "left leg y";
      break;
    case 12:
      text = "right arm x";
      break;
    case 13:
      text = "right arm y";
      break;
    case 14:
      text = "right arm z";
      break;
    case 15:
      text = "right arm r";
      break;
    case 16:
      text = "right arm p";
      break;
    case 17:
      text = "right arm y";
      break;
    case 18:
      text = "right leg x";
      break;
    case 19:
      text = "right leg y";
      break;
    case 20:
      text = "right leg z";
      break;
    case 21:
      text = "right leg r";
      break;
    case 22:
      text = "right leg p";
      break;
    case 23:
      text = "right leg y";
      break;
    }


    if(i < 12)
    {
      label_task_ctrl[i] = new QLabel(text);
      //labels_joint_ctrl_ids[i].setText(text);

      button_task_ctrl[i][0]->setMaximumWidth(40);
      button_task_ctrl[i][1]->setMaximumWidth(40);

      button_task_ctrl[i][0]->setObjectName(tr("%1").arg(i+1));
      button_task_ctrl[i][1]->setObjectName(tr("%1").arg(i+1));

      doubleSpin_task_ctrl[i]->setMinimumWidth(80);
      doubleSpin_task_ctrl[i]->setValue(0.0);
      doubleSpin_task_ctrl[i]->setMinimum(0.0);
      doubleSpin_task_ctrl[i]->setMaximum(30.0);

      label_task_ctrl[i]->setMaximumWidth(200);
      //label_task_ctrl[i]->setAlignment(Qt::AlignCenter);
    }
    else
    {
      label_task_ctrl_2[i-12] = new QLabel(text);
      //labels_joint_ctrl_ids[i].setText(text);

      button_task_ctrl_2[i-12][0]->setMaximumWidth(40);
      button_task_ctrl_2[i-12][1]->setMaximumWidth(40);

      button_task_ctrl_2[i-12][0]->setObjectName(tr("%1").arg(i+1));
      button_task_ctrl_2[i-12][1]->setObjectName(tr("%1").arg(i+1));

      doubleSpin_task_ctrl_2[i-12]->setMinimumWidth(80);
      doubleSpin_task_ctrl_2[i-12]->setValue(0.0);
      doubleSpin_task_ctrl_2[i-12]->setMinimum(0.0);
      doubleSpin_task_ctrl_2[i-12]->setMaximum(30.0);

      label_task_ctrl_2[i-12]->setMaximumWidth(200);
      //label_task_ctrl[i]->setAlignment(Qt::AlignCenter);
    }




    if(i<6)
    {
      ui.gridLayout_task_ctrl->addWidget(label_task_ctrl[i]     , i, 0, Qt::AlignTop);
      ui.gridLayout_task_ctrl->addWidget(doubleSpin_task_ctrl[i], i, 1, Qt::AlignTop);
      ui.gridLayout_task_ctrl->addWidget(button_task_ctrl[i][0] , i, 2, Qt::AlignTop);
      ui.gridLayout_task_ctrl->addWidget(button_task_ctrl[i][1] , i, 3, Qt::AlignTop);
    }
    else if(i<12)
    {
      ui.gridLayout_task_ctrl->addWidget(label_task_ctrl[i]     , i+1, 0, Qt::AlignTop);
      ui.gridLayout_task_ctrl->addWidget(doubleSpin_task_ctrl[i], i+1, 1, Qt::AlignTop);
      ui.gridLayout_task_ctrl->addWidget(button_task_ctrl[i][0] , i+1, 2, Qt::AlignTop);
      ui.gridLayout_task_ctrl->addWidget(button_task_ctrl[i][1] , i+1, 3, Qt::AlignTop);
    }
    else if(i<18)
    {
      ui.gridLayout_task_ctrl_2->addWidget(label_task_ctrl_2[i-12]     , i-12, 0, Qt::AlignTop);
      ui.gridLayout_task_ctrl_2->addWidget(doubleSpin_task_ctrl_2[i-12], i-12, 1, Qt::AlignTop);
      ui.gridLayout_task_ctrl_2->addWidget(button_task_ctrl_2[i-12][0] , i-12, 2, Qt::AlignTop);
      ui.gridLayout_task_ctrl_2->addWidget(button_task_ctrl_2[i-12][1] , i-12, 3, Qt::AlignTop);
    }
    else
    {
      ui.gridLayout_task_ctrl_2->addWidget(label_task_ctrl_2[i-12]     , i-11, 0, Qt::AlignTop);
      ui.gridLayout_task_ctrl_2->addWidget(doubleSpin_task_ctrl_2[i-12], i-11, 1, Qt::AlignTop);
      ui.gridLayout_task_ctrl_2->addWidget(button_task_ctrl_2[i-12][0] , i-11, 2, Qt::AlignTop);
      ui.gridLayout_task_ctrl_2->addWidget(button_task_ctrl_2[i-12][1] , i-11, 3, Qt::AlignTop);
    }

    if(i < 12)
    {
      QObject::connect(button_task_ctrl[i][0], SIGNAL(clicked()), this, SLOT(taskCtrlMinusClicked()));
      QObject::connect(button_task_ctrl[i][1], SIGNAL(clicked()), this, SLOT(taskCtrlPlusClicked()));
    }
    else
    {
      QObject::connect(button_task_ctrl_2[i-12][0], SIGNAL(clicked()), this, SLOT(taskCtrlMinusClicked()));
      QObject::connect(button_task_ctrl_2[i-12][1], SIGNAL(clicked()), this, SLOT(taskCtrlPlusClicked()));
    }


  }
  label_task_ctrl[12] = new QLabel("");
  label_task_ctrl_2[12] = new QLabel("");

  ui.gridLayout_task_ctrl->addWidget(label_task_ctrl[12] , 6, 0, Qt::AlignTop);
  ui.gridLayout_task_ctrl_2->addWidget(label_task_ctrl_2[12], 6, 0, Qt::AlignTop);

  ui.gridLayout_task_ctrl->setColumnStretch(1,1);
  ui.gridLayout_task_ctrl_2->setColumnStretch(1,1);


  //ui.button_estop->text()
  updateUI();

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

void MainWindow::autoMissionSelectVisible(int mission)
{
  ui.button_auto_valve_approach->setVisible(mission==1);
  ui.button_auto_valve_close->setVisible(mission==1);
  ui.button_auto_valve_init->setVisible(mission==1);
  ui.button_auto_valve_reach->setVisible(mission==1);
  ui.button_auto_valve_ready->setVisible(mission==1);
  ui.button_auto_door_init->setVisible(mission==2);
  ui.button_auto_door_open->setVisible(mission==2);
  ui.button_auto_door_push->setVisible(mission==2);
  ui.button_auto_door_reach->setVisible(mission==2);
  ui.button_auto_door_ready->setVisible(mission==2);
  ui.button_auto_egress_init->setVisible(mission==3);
  ui.button_auto_egress_egress->setVisible(mission==3);
  ui.button_auto_egress_standby->setVisible(mission==3);
  ui.button_auto_egress_guide->setVisible(mission==3);
  ui.button_auto_egress_hello->setVisible(mission==3);
}
void MainWindow::updateUI()
{
  ui.groupBox_state->setEnabled(isConnected);
  ui.groupBox_joint_ctrl->setEnabled(isConnected);
  ui.groupBox_task_ctrl->setEnabled(isConnected);
  ui.groupBox_recog_ctrl->setEnabled(isConnected);
  ui.groupBox_walk_ctrl->setEnabled(isConnected);
  ui.groupBox_sensor_ctrl->setEnabled(isConnected);
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
  if ( ui.checkbox_use_environment->isChecked() ) {
    if ( !qnode.init() ) {
      showNoMasterMessage();
    } else {
      ui.button_connect->setEnabled(false);
      isConnected = true;
    }
  } else {
    if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
                      ui.line_edit_host->text().toStdString()) ) {
      showNoMasterMessage();
    } else {
      ui.button_connect->setEnabled(false);
      ui.line_edit_master->setReadOnly(true);
      ui.line_edit_host->setReadOnly(true);
      ui.line_edit_topic->setReadOnly(true);
      isConnected = true;
    }
  }
  updateUI();
}

/*

void MainWindow::on_manuButton_clicked(bool check )
{
    qnode.send_transition("manu_on");
}
void MainWindow::on_autoButton_clicked(bool check)
{
    qnode.send_transition("auto_on");
}

void MainWindow::on_button_joint_control_clicked(bool check)
{
    qnode.send_transition("activate_jctrl");
}

void MainWindow::on_button_power_on_clicked(bool check)
{
    qnode.send_transition("power_on");
}
*/
void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
  bool enabled;
  if ( state == 0 ) {
    enabled = true;
  } else {
    enabled = false;
  }
  ui.line_edit_master->setEnabled(enabled);
  ui.line_edit_host->setEnabled(enabled);
  //ui.line_edit_topic->setEnabled(enabled);
}

void MainWindow::on_button_walk_start_clicked()
{
  qnode.walk_cmd_msg_.walk_mode = 1;

  if(std::strcmp(ui.comboBox_compensator->currentText().toStdString().c_str(), "Hip Compensator") == 0)
  {
    qnode.walk_cmd_msg_.compensator_mode[0] = true;
    qnode.walk_cmd_msg_.compensator_mode[1] = false;
  }
  else if(std::strcmp(ui.comboBox_compensator->currentText().toStdString().c_str(), "External Encoder") == 0)
  {
    qnode.walk_cmd_msg_.compensator_mode[0] = false;
    qnode.walk_cmd_msg_.compensator_mode[1] = true;
  }
  else
  {
    qnode.walk_cmd_msg_.compensator_mode[0] = true;
    qnode.walk_cmd_msg_.compensator_mode[1] = true;
  }

  if(std::strcmp(ui.comboBox_method->currentText().toStdString().c_str(), "Inverse Kinematics") == 0)
  {
    qnode.walk_cmd_msg_.ik_mode = 0;
  } else qnode.walk_cmd_msg_.ik_mode = 1;

  if(std::strcmp(ui.comboBox_first_step->currentText().toStdString().c_str(), "Left") == 0)
  {
    qnode.walk_cmd_msg_.first_foot_step = false;
  } else qnode.walk_cmd_msg_.first_foot_step =true;

  if(std::strcmp(ui.comboBox_heel_toe->currentText().toStdString().c_str(), "Yes") == 0)
  {
    qnode.walk_cmd_msg_.heel_toe =true;
  } else qnode.walk_cmd_msg_.heel_toe = false;

  qnode.walk_cmd_msg_.x = ui.doubleSpinBox_walk_x->value();
  qnode.walk_cmd_msg_.y = ui.doubleSpinBox_walk_y->value();
  qnode.walk_cmd_msg_.z = ui.doubleSpinBox_walk_z->value();
  qnode.walk_cmd_msg_.height = ui.doubleSpinBox_walk_height->value();
  qnode.walk_cmd_msg_.theta = ui.doubleSpinBox_walk_theta->value();
  qnode.walk_cmd_msg_.step_length_x = ui.doubleSpinBox_walk_step_x->value();
  qnode.walk_cmd_msg_.step_length_y = ui.doubleSpinBox_walk_step_y->value();

  qnode.send_walk_ctrl();

  std::string tempString;

  if(ui.checkBox_send_data_cb->isChecked())
  {
    tempString = "clicked";
  } else tempString = "unclicked";

  qnode.controlbase_data_.data = tempString;

  qnode.send_data_cb();
}

void MainWindow::on_button_walk_init_clicked()
{
  const char *tempName[32] = {"L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll"
                               ,"R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll"
                               ,"WaistPitch","WaistYaw"
                               ,"L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw"
                               ,"R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw"
                               ,"HeadYaw", "HeadPitch", "R_Gripper", "L_Gripper"};

  const double tempNum[32] = {0 , 0.034906585 , -0.3490658504 , 0.6981317008 , -0.3490658504 , -0.034906585
                                   , 0 , -0.034906585 , 0.3490658504 , -0.6981317008 , 0.3490658504 , 0.034906585
                                   , 0 , 0
                                   , 0.6981317008 , -1.6580627893 , -1.3962634016 , -1.9198621771 , 0 , -1.2217304764 , -0.1745329252
                                   , -0.6981317008 , 1.6580627893 , 1.3962634016 , 1.9198621771 , 0 , 1.2217304764 , 1.7453292519
                                   , 0 , 0 , 0 , 0};

  const double tempDuration[32] = {5 , 5 , 5 , 5 , 5 , 5
                                   , 5 , 5 , 5 , 5 , 5 , 5
                                   , 5 , 5
                                   , 5 , 5 , 5 , 5 , 5 , 5 , 5
                                   , 5 , 5 , 5 , 5 , 5 , 5 , 5
                                   , 0 , 0 , 0 , 0};

  for(int i = 0; i < 32; i ++)
  {
    qnode.joint_cmd_msg_.name[i] = tempName[i];
    qnode.joint_cmd_msg_.position[i] = tempNum[i];
    qnode.joint_cmd_msg_.duration[i] = tempDuration[i];
  }
  qnode.publish_joint_ctrl();

  for(int i = 0; i < 32; i ++)
  {
    qnode.joint_cmd_msg_.name[i] = "";
  }
}

void MainWindow::on_button_walk_stop_clicked()
{

}

void MainWindow::on_button_walk_quit_clicked()
{


  //    thormang_ctrl_msgs::WalkingCmd msg;
  //    msg.command = "stop";
  //    msg.planner = ui.comboBox_walk_planner->currentText().toStdString();
  //    msg.walking_alg = ui.comboBox_walk_algorithm->currentText().toStdString();
  //    msg.x = ui.doubleSpinBox_walk_x->value();
  //    msg.y = ui.doubleSpinBox_walk_y->value();
  //    msg.height = ui.doubleSpinBox_walk_height->value();
  //    msg.theta = ui.doubleSpinBox_walk_theta->value();
  //    msg.imp_time = ui.doubleSpinBox_walk_imp_time->value();
  //    msg.recov_time= ui.doubleSpinBox_walk_recov_time->value();

  //    qnode.send_walking_cmd(msg);

  //    // temp stop state
      std::string state;
      state = "shutdown";
      qnode.send_transition(state);

}

void MainWindow::on_button_scan_clicked()
{
  //   thormang_ctrl_msgs::RecogCmd msg;
  //   msg.id = 2; // id 2 means rotate lidar
  //   msg.x = 0;
  //   msg.y = 0;
  //   msg.z = 0;
  //   msg.roll = 0;
  //   msg.pitch = 0;
  //   msg.yaw = 0;
  //   qnode.send_recog_cmd(msg);
}

void MainWindow::on_button_estop_clicked()
{
  std::string state;
  state = "shutdown";
  qnode.send_transition(state);
}

void MainWindow::on_button_ft_calib_clicked()
{
  //qnode.send_ft_calib(5.0);
}

void MainWindow::stateButtonClicked()
{
  QString objName = sender()->objectName();
  std::string state;
  if(objName.compare("button_power_on") == 0)  {
    state = "power_on";
  } else if (objName.compare("button_auto") == 0) {
    state = "auto_on";
  } else if (objName.compare("button_auto_valve_start") == 0) {
    state = "mission1";
    autoMissionSelectVisible(1);
  } else if (objName.compare("button_auto_valve_approach") == 0) {
    state = "v_approach";
  } else if (objName.compare("button_auto_valve_close") == 0) {
    state = "v_close";
  } else if (objName.compare("button_auto_valve_init") == 0) {
    state = "v_init";
  } else if (objName.compare("button_auto_valve_reach") == 0) {
    state = "v_reach";
  } else if (objName.compare("button_auto_valve_ready") == 0) {
    state = "v_ready";
  } else if (objName.compare("button_auto_door_start") == 0) {
    state = "mission2";
    autoMissionSelectVisible(2);
  } else if (objName.compare("button_auto_door_init") == 0) {
    state = "d_init";
  } else if (objName.compare("button_auto_door_open") == 0) {
    state = "d_open";
  } else if (objName.compare("button_auto_door_push") == 0) {
    state = "d_push";
  } else if (objName.compare("button_auto_door_reach") == 0) {
    state = "d_reach";
  } else if (objName.compare("button_auto_door_ready") == 0) {
    state = "d_ready";

  } else if (objName.compare("button_auto_egress_start") == 0) {
    state = "mission3";
    autoMissionSelectVisible(3);
  } else if (objName.compare("button_auto_egress_init") == 0) {
    state = "e_init";
  } else if (objName.compare("button_auto_egress_egress") == 0) {
    state = "e_egress";
  } else if (objName.compare("button_auto_egress_standby") == 0) {
    state = "e_standby";
  } else if (objName.compare("button_auto_egress_hello") == 0) {
    state = "e_hello";
  } else if (objName.compare("button_auto_egress_guide") == 0) {
    state = "e_guide";
  } else if (objName.compare("button_manual") == 0) {
    state = "manu_on";
  } else if (objName.compare("button_manual_joint_ctrl") == 0) {
    state = "activate_jctrl";
  } else if (objName.compare("button_manual_task_ctrl") == 0) {
    state = "activate_tctrl";
  } else if (objName.compare("button_mode_change") == 0) {
    state = "cmd_modechg";
  } else if (objName.compare("button_manual_recog_ctrl") == 0) {
    state = "activate_recog";
  } else if (objName.compare("button_event") == 0) {
    state = "event_on";
  } else if (objName.compare("button_event_handclap") == 0) {
    state = "handclap";
  } else if (objName.compare("button_event_handclap_ready") == 0) {
    state = "handclap_ready";
  } else if (objName.compare("button_event_handclap_do") == 0) {
    state = "handclap_do";
  } else if (objName.compare("button_event_handclap_end") == 0) {
    state = "handclap_end";
  } else if (objName.compare("button_event_handshake") == 0) {
    state = "handshake";
  }else if (objName.compare("button_event_handshake_turn") == 0) {
    state = "handshake_turn";
  }
  else if (objName.compare("button_event_handshake_ready") == 0) {
    state = "handshake_ready";
  } else if (objName.compare("button_event_handshake_do") == 0) {
    state = "handshake_do";
  } else if (objName.compare("button_event_handshake_end") == 0) {
    state = "handshake_end";
  } else if (objName.compare("button_event_handshake_motion1") == 0) {
    state = "handshake_motion1";
  } else if (objName.compare("button_event_handshake_motion2") == 0) {
    state = "handshake_motion2";
  }else if (objName.compare("button_event_handshake_motion3") == 0) {
    state = "handshake_motion3";
  }
  else if (objName.compare("button_event_handshake_return") == 0) {
    state = "handshake_return";
  }
  else if (objName.compare("button_event_hello") == 0) {
    state = "hello";
  } else if (objName.compare("button_event_hello_ready") == 0) {
    state = "hello_ready";
  } else if (objName.compare("button_event_hello_do") == 0) {
    state = "hello_do";
    //qnode.send_hello_count(ui.spin_hello_count->value());
  } else if (objName.compare("button_event_hello_end") == 0) {
    state = "hello_end";
  } else if (objName.compare("button_event_hello_introduce") == 0){
    state = "hello_introduce";
  } else if (objName.compare("button_event_hello_introduce_end") == 0){
    state = "hello_introduce_end";
  }
  qnode.send_transition(state);
}

void MainWindow::jointCtrlMinusClicked()
{
  int id = sender()->objectName().toInt();

  qnode.send_joint_ctrl(arranged_id[id-1], jointName[id-1], -1.0); // degree
}

void MainWindow::jointCtrlPlusClicked()
{
  int id = sender()->objectName().toInt();

  qnode.send_joint_ctrl(arranged_id[id-1], jointName[id-1], 1.0); // degree
}

void MainWindow::jointCtrlSetClicked()
{
  int id = sender()->objectName().toInt();
  double deg = doubleSpin_joint_ctrl[id-1]->value();

  qnode.send_joint_ctrl(arranged_id[id-1], jointName[id-1], deg); // degree
}

void MainWindow::taskCtrlMinusClicked()
{
  int id = sender()->objectName().toInt();
  double pos = -doubleSpin_task_ctrl[id-1]->value();
  QVector3D axis;
  QQuaternion quat;
  int offset = 1.0;


  for(int i = 0; i < 4; i++)
  {
    qnode.task_cmd_msg_.end_effector[i] = false;

    qnode.task_cmd_msg_.pose[i].position.x = 0.0;
    qnode.task_cmd_msg_.pose[i].position.y = 0.0;
    qnode.task_cmd_msg_.pose[i].position.z = 0.0;

    qnode.task_cmd_msg_.pose[i].orientation.x = 0.0;
    qnode.task_cmd_msg_.pose[i].orientation.y = 0.0;
    qnode.task_cmd_msg_.pose[i].orientation.z = 0.0;
    qnode.task_cmd_msg_.pose[i].orientation.w = 0.0;

    qnode.task_cmd_msg_.duration[i] = 0.0;
  }

  if(id <= 6)
  {
    qnode.task_cmd_msg_.end_effector[2] = true;
    qnode.task_cmd_msg_.mode[2] = 0;


    switch(id)
    {
    case 1:
      qnode.task_cmd_msg_.pose[2].position.x = pos;
      break;
    case 2:
      qnode.task_cmd_msg_.pose[2].position.y = pos;
      break;
    case 3:
      qnode.task_cmd_msg_.pose[2].position.z = pos;
      break;
    case 4:
      axis = QVector3D(1, 0, 0);
      qnode.task_cmd_msg_.pose[2].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[2].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[2].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[2].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    case 5:
      axis = QVector3D(0, 1, 0);
      qnode.task_cmd_msg_.pose[2].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[2].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[2].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[2].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    case 6:
      axis = QVector3D(0, 0, 1);
      qnode.task_cmd_msg_.pose[2].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[2].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[2].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[2].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    default:
      break;
    }
    qnode.task_cmd_msg_.duration[2] = 1 - pos*offset;
    qnode.send_task_ctrl();
  }
  else if(id <= 12)
  {
    qnode.task_cmd_msg_.end_effector[0] = true;
    qnode.task_cmd_msg_.mode[0] = 0;

    switch(id)
    {
    case 7:
      qnode.task_cmd_msg_.pose[0].position.x = pos;
      break;
    case 8:
      qnode.task_cmd_msg_.pose[0].position.y = pos;
      break;
    case 9:
      qnode.task_cmd_msg_.pose[0].position.z = pos;
      break;
    case 10:
      axis = QVector3D(1, 0, 0);
      qnode.task_cmd_msg_.pose[0].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[0].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[0].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[0].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    case 11:
      axis = QVector3D(0, 1, 0);
      qnode.task_cmd_msg_.pose[0].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[0].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[0].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[0].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    case 12:
      axis = QVector3D(0, 0, 1);
      qnode.task_cmd_msg_.pose[0].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[0].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[0].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[0].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    default:
      break;
    }
    qnode.task_cmd_msg_.duration[0] = 1 - pos*offset;
    qnode.send_task_ctrl();
  }
  else if(id <= 18)
  {
    qnode.task_cmd_msg_.end_effector[3] = true;
    qnode.task_cmd_msg_.mode[3] = 0;

    switch(id)
    {
    case 13:
      qnode.task_cmd_msg_.pose[3].position.x = pos;
      break;
    case 14:
      qnode.task_cmd_msg_.pose[3].position.y = pos;
      break;
    case 15:
      qnode.task_cmd_msg_.pose[3].position.z = pos;
      break;
    case 16:
      axis = QVector3D(1, 0, 0);
      qnode.task_cmd_msg_.pose[3].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[3].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[3].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[3].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    case 17:
      axis = QVector3D(0, 1, 0);
      qnode.task_cmd_msg_.pose[3].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[3].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[3].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[3].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    case 18:
      axis = QVector3D(0, 0, 1);
      qnode.task_cmd_msg_.pose[3].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[3].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[3].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[3].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    default:
      break;
    }
    qnode.task_cmd_msg_.duration[3] = 1 - pos*offset;
    qnode.send_task_ctrl();
  }
  else
  {
    qnode.task_cmd_msg_.end_effector[1] = true;
    qnode.task_cmd_msg_.mode[1] = 0;

    switch(id)
    {
    case 19:
      qnode.task_cmd_msg_.pose[1].position.x = pos;
      break;
    case 20:
      qnode.task_cmd_msg_.pose[1].position.y = pos;
      break;
    case 21:
      qnode.task_cmd_msg_.pose[1].position.z = pos;
      break;
    case 22:
      axis = QVector3D(1, 0, 0);
      qnode.task_cmd_msg_.pose[1].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[1].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[1].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[1].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    case 23:
      axis = QVector3D(0, 1, 0);
      qnode.task_cmd_msg_.pose[1].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[1].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[1].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[1].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    case 24:
      axis = QVector3D(0, 0, 1);
      qnode.task_cmd_msg_.pose[1].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[1].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[1].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[1].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    default:
      break;
    }
    qnode.task_cmd_msg_.duration[1] = 1 - pos*offset;
    qnode.send_task_ctrl();
  }
}

void MainWindow::taskCtrlPlusClicked()
{
  int id = sender()->objectName().toInt();
  double pos = doubleSpin_task_ctrl[id-1]->value();
  QVector3D axis;
  QQuaternion quat;
  int offset = 1.0;


  for(int i = 0; i < 4; i++)
  {
    qnode.task_cmd_msg_.end_effector[i] = false;

    qnode.task_cmd_msg_.pose[i].position.x = 0.0;
    qnode.task_cmd_msg_.pose[i].position.y = 0.0;
    qnode.task_cmd_msg_.pose[i].position.z = 0.0;

    qnode.task_cmd_msg_.pose[i].orientation.x = 0.0;
    qnode.task_cmd_msg_.pose[i].orientation.y = 0.0;
    qnode.task_cmd_msg_.pose[i].orientation.z = 0.0;
    qnode.task_cmd_msg_.pose[i].orientation.w = 0.0;

    qnode.task_cmd_msg_.duration[i] = 0.0;
  }

  if(id <= 6)
  {
    qnode.task_cmd_msg_.end_effector[2] = true;
    qnode.task_cmd_msg_.mode[2] = 0;


    switch(id)
    {
    case 1:
      qnode.task_cmd_msg_.pose[2].position.x = pos;
      break;
    case 2:
      qnode.task_cmd_msg_.pose[2].position.y = pos;
      break;
    case 3:
      qnode.task_cmd_msg_.pose[2].position.z = pos;
      break;
    case 4:
      axis = QVector3D(1, 0, 0);
      qnode.task_cmd_msg_.pose[2].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[2].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[2].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[2].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    case 5:
      axis = QVector3D(0, 1, 0);
      qnode.task_cmd_msg_.pose[2].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[2].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[2].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[2].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    case 6:
      axis = QVector3D(0, 0, 1);
      qnode.task_cmd_msg_.pose[2].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[2].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[2].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[2].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    default:
      break;
    }
    qnode.task_cmd_msg_.duration[2] = 1 + pos*offset;
    qnode.send_task_ctrl();
  }
  else if(id <= 12)
  {
    qnode.task_cmd_msg_.end_effector[0] = true;
    qnode.task_cmd_msg_.mode[0] = 0;

    switch(id)
    {
    case 7:
      qnode.task_cmd_msg_.pose[0].position.x = pos;
      break;
    case 8:
      qnode.task_cmd_msg_.pose[0].position.y = pos;
      break;
    case 9:
      qnode.task_cmd_msg_.pose[0].position.z = pos;
      break;
    case 10:
      axis = QVector3D(1, 0, 0);
      qnode.task_cmd_msg_.pose[0].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[0].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[0].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[0].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    case 11:
      axis = QVector3D(0, 1, 0);
      qnode.task_cmd_msg_.pose[0].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[0].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[0].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[0].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    case 12:
      axis = QVector3D(0, 0, 1);
      qnode.task_cmd_msg_.pose[0].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[0].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[0].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[0].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    default:
      break;
    }
    qnode.task_cmd_msg_.duration[0] = 1 + pos*offset;
    qnode.send_task_ctrl();
  }
  else if(id <= 18)
  {
    qnode.task_cmd_msg_.end_effector[3] = true;
    qnode.task_cmd_msg_.mode[3] = 0;

    switch(id)
    {
    case 13:
      qnode.task_cmd_msg_.pose[3].position.x = pos;
      break;
    case 14:
      qnode.task_cmd_msg_.pose[3].position.y = pos;
      break;
    case 15:
      qnode.task_cmd_msg_.pose[3].position.z = pos;
      break;
    case 16:
      axis = QVector3D(1, 0, 0);
      qnode.task_cmd_msg_.pose[3].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[3].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[3].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[3].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    case 17:
      axis = QVector3D(0, 1, 0);
      qnode.task_cmd_msg_.pose[3].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[3].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[3].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[3].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    case 18:
      axis = QVector3D(0, 0, 1);
      qnode.task_cmd_msg_.pose[3].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[3].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[3].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[3].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    default:
      break;
    }
    qnode.task_cmd_msg_.duration[3] = 1 + pos*offset;
    qnode.send_task_ctrl();
  }
  else
  {
    qnode.task_cmd_msg_.end_effector[1] = true;
    qnode.task_cmd_msg_.mode[1] = 0;

    switch(id)
    {
    case 19:
      qnode.task_cmd_msg_.pose[1].position.x = pos;
      break;
    case 20:
      qnode.task_cmd_msg_.pose[1].position.y = pos;
      break;
    case 21:
      qnode.task_cmd_msg_.pose[1].position.z = pos;
      break;
    case 22:
      axis = QVector3D(1, 0, 0);
      qnode.task_cmd_msg_.pose[1].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[1].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[1].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[1].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    case 23:
      axis = QVector3D(0, 1, 0);
      qnode.task_cmd_msg_.pose[1].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[1].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[1].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[1].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    case 24:
      axis = QVector3D(0, 0, 1);
      qnode.task_cmd_msg_.pose[1].orientation.x = quat.fromAxisAndAngle(axis, pos).x();
      qnode.task_cmd_msg_.pose[1].orientation.y = quat.fromAxisAndAngle(axis, pos).y();
      qnode.task_cmd_msg_.pose[1].orientation.z = quat.fromAxisAndAngle(axis, pos).z();
      qnode.task_cmd_msg_.pose[1].orientation.w = quat.fromAxisAndAngle(axis, pos).scalar();
      break;
    default:
      break;
    }
    qnode.task_cmd_msg_.duration[1] = 1 + pos*offset;
    qnode.send_task_ctrl();
  }
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
  ui.view_logging->scrollToBottom();
}

void MainWindow::setTable(int i, int column, QTableWidgetItem *newItem)
{

  if(i < 6) ui.ll_table->setItem(i, column, newItem);
  else if(i < 12) ui.rl_table->setItem(i-6, column, newItem);
  else if(i < 14) ui.w_table->setItem(i-12, column, newItem);
  else if(i < 21) ui.la_table->setItem(i-14, column, newItem);
  else if(i < 28) ui.ra_table->setItem(i-21, column, newItem);
  else if(i < 30) ui.h_table->setItem(i-28, column, newItem);
  else if(i == 30) ui.ra_table->setItem(7, column, newItem);
  else ui.la_table->setItem(7, column, newItem);

}

void MainWindow::updateJointView() {

  for (int i=0; i<32; i++)
  {

    double degree = qnode.joint_msg_.angle[i] * 57.295791433;
    if (fabs(degree) < 0.01) degree = 0.0; // +, - preventing

    QTableWidgetItem *newItem = new QTableWidgetItem(
                QString::number(degree,'f',2));


//    ui.ra_table->setItem(arranged_id[i],1,newItem);
    setTable(i, 1, newItem);
//    ui.motor_table->setItem(qnode.joint_msg_.id[i]-1, 1, newItem);
    newItem = new QTableWidgetItem(
                QString::number(qnode.joint_msg_.current[i],'f',2));


    const int lowThres = 20.0;
    const int highThres = 40.0;

    // Torque indicator
    double c = fabs(qnode.joint_msg_.current[i]);
    if(c < lowThres)
    {
        newItem->setBackgroundColor(QColor(255,255,255 / lowThres * (lowThres - c)));
    }
    else if(c > highThres)
    {
        newItem->setBackgroundColor(QColor(255,0,0));
    }
    else
    {
        newItem->setBackgroundColor(QColor(255,255 / (highThres-lowThres) * ((highThres-lowThres) - c + lowThres), 0));
    }

//    ui.motor_table->setItem(qnode.joint_msg_.id[i]-1, 2, newItem);
    setTable(i, 2, newItem);
    newItem = new QTableWidgetItem(
                QString::number(qnode.joint_msg_.error[i]));

    if(qnode.joint_msg_.error[i] != 0)
    {
       newItem->setBackgroundColor(Qt::yellow);
    }
//    ui.motor_table->setItem(qnode.joint_msg_.id[i]-1, 3, newItem);
    setTable(i, 3, newItem);

  }
}

void MainWindow::updateRecogInfo() {
/*
  ui.line_edit_scan_x->setText(QString::number(qnode.recog_info_msg.data[0] * 100,'f',2) + " cm");
  ui.line_edit_scan_y->setText(QString::number(qnode.recog_info_msg.data[1] * 100,'f',2) + " cm");
  ui.line_edit_scan_z->setText(QString::number(qnode.recog_info_msg.data[2] * 100,'f',2) + " cm");

  ui.line_edit_scan_roll->setText(QString::number(qnode.recog_info_msg.data[3],'f',2));
  ui.line_edit_scan_pitch->setText(QString::number(qnode.recog_info_msg.data[4],'f',2));
  ui.line_edit_scan_yaw->setText(QString::number(qnode.recog_info_msg.data[5],'f',2));
*/
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
  QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
  QSettings settings("Qt-Ros Package", "dyros_jet_gui");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
  QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
  QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
  //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
  ui.line_edit_master->setText(master_url);
  ui.line_edit_host->setText(host_url);
  //ui.line_edit_topic->setText(topic_name);
  bool remember = settings.value("remember_settings", false).toBool();
  ui.checkbox_remember_settings->setChecked(remember);
  bool checked = settings.value("use_environment_variables", false).toBool();
  ui.checkbox_use_environment->setChecked(checked);
  if ( checked ) {
    ui.line_edit_master->setEnabled(false);
    ui.line_edit_host->setEnabled(false);
    //ui.line_edit_topic->setEnabled(false);
  }
}

void MainWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "dyros_jet_gui");
  settings.setValue("master_url",ui.line_edit_master->text());
  settings.setValue("host_url",ui.line_edit_host->text());
  //settings.setValue("topic_name",ui.line_edit_topic->text());
  settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
  settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
  WriteSettings();
  QMainWindow::closeEvent(event);
}



void MainWindow::on_button_joint_save_clicked()
{

  /*
  std::string homeDir(getenv("HOME"));
  std::string fullDir = homeDir + "/DAQ/JointDatas/" + ui.line_edit_joint_file_name->text().toStdString();
  std::cout << fullDir;
  std::ofstream outFile(fullDir.c_str());

  outFile << qnode.joint_msg.angle[0];
  outFile << 1.0;
  for (int i=1; i<32; i++)
  {
    //outFile << "," << 1.0;
    outFile << "," << qnode.joint_msg.angle[i];
  }
  outFile << std::endl << std::endl;

  outFile << "ID\tAngle" << std::endl;
  for (int i=0; i<32; i++)
  {
    outFile << (int)qnode.joint_msg.id[i] << "\t" << qnode.joint_msg.angle[i] << std::endl;
    //outFile << i << "\t" << 1.0 << std::endl;
  }
*/
}

void MainWindow::on_button_motor_save_clicked()
{

  if(ui.h_table->item(0,1)->text() != NULL)
  {

  std::string homeDir(getenv("HOME"));
  std::string fullDir = homeDir + "/" + ui.line_edit_motor_save_file_name->text().toStdString(); //+ path of which data should be saved
  std::ofstream outFile(fullDir.c_str());

  outFile << "Data of Joint Position" << std::endl;
  outFile << "Name" << "\t" << "Pos" << std::endl;

    for(int i = 0; i <32; i++)
    {
      if(i < 2) outFile << ui.h_table->item(i, 0)->text().toStdString() << "\t" << ui.h_table->item(i, 1)->text().toStdString() << std::endl;
      else if(i < 4) outFile << ui.w_table->item(i-2, 0)->text().toStdString() << "\t" << ui.w_table->item(i-2, 1)->text().toStdString() << std::endl;
      else if(i < 12) outFile << ui.ra_table->item(i-4, 0)->text().toStdString() << "\t" << ui.ra_table->item(i-4, 1)->text().toStdString() << std::endl;
      else if(i < 20) outFile << ui.la_table->item(i-12, 0)->text().toStdString() << "\t" << ui.la_table->item(i-12, 1)->text().toStdString() << std::endl;
      else if(i < 26) outFile << ui.rl_table->item(i-20, 0)->text().toStdString() << "\t" << ui.rl_table->item(i-20, 1)->text().toStdString() << std::endl;
      else outFile << ui.ll_table->item(i-26, 0)->text().toStdString() << "\t" << ui.ll_table->item(i-26, 1)->text().toStdString() << std::endl;
    }

    outFile.close();
  } else QMessageBox::warning(this, "Error", "No Data Has Been Found!!!");
}

void MainWindow::on_all_checkBox_clicked()
{
  QCheckBox *checkbox_array[6] = {ui.h_checkBox, ui.w_checkBox, ui.ra_checkBox, ui.la_checkBox, ui.rl_checkBox, ui.ll_checkBox};

  if(ui.all_checkBox->isChecked())
  {
    for(int i = 0; i < 6; i ++)
    {
      checkbox_array[i]->setCheckable(false);
      checkbox_array[i]->setDisabled(true);
    }
  }
  else
  {
    for(int i = 0; i < 6; i++)
    {
      checkbox_array[i]->setCheckable(true);
      checkbox_array[i]->setDisabled(false);
    }
  }
}

void MainWindow::ConfineChecklist()
{
  int i;

  if(ui.all_checkBox->isChecked())
  {
    for(i = 0; i < 32; i ++) qnode.joint_cmd_msg_.name[i] = disarranged_jointName[i];
  }
  else
  {
    for(i = 0; i < 32; i++) qnode.joint_cmd_msg_.name[i] = ""; //Initiallize

    if(ui.h_checkBox->isChecked())
    {
      for(i = 0; i < 2; i++) qnode.joint_cmd_msg_.name[arranged_id[i]-1] = jointName[i];
    }
    if(ui.w_checkBox->isChecked())
    {
      for(i = 2; i < 4; i++) qnode.joint_cmd_msg_.name[arranged_id[i]-1] = jointName[i];
    }
    if(ui.ra_checkBox->isChecked())
    {
      for(i = 4; i < 12; i++) qnode.joint_cmd_msg_.name[arranged_id[i]-1] = jointName[i];
    }
    if(ui.la_checkBox->isChecked())
    {
      for(i = 12; i < 20; i++) qnode.joint_cmd_msg_.name[arranged_id[i]-1] = jointName[i];
    }
    if(ui.rl_checkBox->isChecked())
    {
      for(i = 20; i < 26; i++) qnode.joint_cmd_msg_.name[arranged_id[i]-1] = jointName[i];
    }
    if(ui.ll_checkBox->isChecked())
    {
      for(i = 26; i < 32; i++) qnode.joint_cmd_msg_.name[arranged_id[i]-1] = jointName[i];
    }
  }
}

void MainWindow::on_button_motor_load_clicked()
{

  std::string homeDir(getenv("HOME"));
  std::string fullDir = homeDir + "/" + ui.line_edit_motor_load_file_name->text().toStdString(); //+ path of which data should be saved
  std::ifstream inFile(fullDir.c_str());

  double loaded_pos[32];
  char name[32][10];
  char trash[30];


  if(inFile)
  {

    inFile.getline(trash, 30);
    inFile.getline(trash, 30);

    for(int i = 0; i < 32; i++)
    {

      inFile >> name[i] >> loaded_pos[i];

    }
    inFile.close();


  } else {
  QMessageBox::warning(this, "Error", "No Saved File Has Been Found!!!");
  inFile.close();
  }

  ConfineChecklist();

  for(int i = 0; i < 32; i++)
  {
    qnode.joint_cmd_msg_.position[arranged_id[i]-1] = loaded_pos[i] / 57.295791433;
    qnode.joint_cmd_msg_.duration[arranged_id[i]-1] = 5.0;
  }


  qnode.publish_joint_ctrl();
//  ROS_INFO("Help");
//  qnode.joint_cmd_msg_.enable[0] = true;
//  qnode.joint_cmd_msg_.position[0] = 3.141592;
//  qnode.joint_cmd_msg_.duration[0] = 3.0;
//  ConfineChecklist();
//  qnode.pubJointCommandMessage(arranged_id, loaded_pos);
}

}  // namespace dyros_jet_gui

