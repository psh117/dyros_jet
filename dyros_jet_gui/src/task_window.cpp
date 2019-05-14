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
#include "dyros_jet_gui/task_window.hpp"
#include <qpixmap.h>
#include <QQuaternion>
#include <QVector3D>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>



/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dyros_jet_gui {

const char *TaskWindow::disarranged_jointName[32] = {"L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll"
                                                     ,"R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll"
                                                     ,"WaistPitch","WaistYaw"
                                                     ,"L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw"
                                                     ,"R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw"
                                                     ,"HeadYaw", "HeadPitch", "R_Gripper", "L_Gripper"};

using namespace Qt;

// std::string motorID = {"R-SP", "L-SP", "R-SR", ""};
/*****************************************************************************
** Implementation [TaskWindow]
*****************************************************************************/

TaskWindow::TaskWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode(argc,argv)
  , isConnected(false)
{
  if ( qnode.init() ) isConnected = true;

  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  ReadSettings();
  setWindowIcon(QIcon(":/images/icon_2.png"));

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  /*********************
    ** Auto Start
    **********************/

  // | Creating UI

  // -- Graph
  time = 0.0;
  count = 0;
  space = 0;
  LF_select = "LF_Force_X";
  RF_select = "RF_Force_X";

  TimeCheck = false;
  timer = new QTimer(this);

  FT_LF = new QCustomPlot(ui.widget1);

  FT_LF->setGeometry(0, 0, 850, 180);
  FT_LF->xAxis->setRange(0, 30, AlignLeft);
  FT_LF->yAxis->setRange(-700.0, 700.0);
  FT_LF->xAxis->setLabel("time(sec)");
  FT_LF->yAxis->setLabel("Force(N)");

  FT_LF->addGraph();
  FT_LF->addGraph();
  FT_LF->addGraph();
  FT_LF->addGraph();
  FT_LF->addGraph();
  FT_LF->addGraph();
  FT_LF->graph()->setLineStyle(QCPGraph::lsLine);


  tracer_LF = new QCPItemTracer(FT_LF);
  tracer_LF->setGraph(FT_LF->graph(0));
  tracer_LF->setVisible(false);

  FT_RF = new QCustomPlot(ui.widget2);

  FT_RF->setGeometry(0, 0, 850, 180);
  FT_RF->xAxis->setRange(0, 30, AlignLeft);
  FT_RF->yAxis->setRange(-700.0, 700.0);
  FT_RF->xAxis->setLabel("time(sec)");
  FT_RF->yAxis->setLabel("Force(N)");

  FT_RF->addGraph();
  FT_RF->addGraph();
  FT_RF->addGraph();
  FT_RF->addGraph();
  FT_RF->addGraph();
  FT_RF->addGraph();
  FT_RF->graph()->setLineStyle(QCPGraph::lsLine);

  tracer_RF = new QCPItemTracer(FT_RF);
  tracer_RF->setGraph(FT_RF->graph(0));
  tracer_RF->setVisible(false);

  textLabel_LF = new QCPItemText(FT_LF);
  textLabel_LF->setVisible(false);

  textLabel_RF = new QCPItemText(FT_RF);
  textLabel_RF->setVisible(false);

  connect(FT_LF, SIGNAL(mousePress(QMouseEvent*)), this,SLOT(ClickedGraph_LF(QMouseEvent*)));
  connect(FT_RF, SIGNAL(mousePress(QMouseEvent*)), this,SLOT(ClickedGraph_RF(QMouseEvent*)));

  // -- Table
  for(int i=0; i<32; i++)
  {
    jointID.push_back(i+1);
  }
  // -- Task Control Set
  for (int i = 0; i < 24; i++)
  {
    label_task_cm = new QLabel(" (cm) ");
    if(i < 12)
    {
      button_task_ctrl[i][0] = new QPushButton("-");
      button_task_ctrl[i][1] = new QPushButton("+");

      doubleSpin_task_ctrl[i] = new QDoubleSpinBox;
    }
    else
    {
      button_task_ctrl_2[i-12][0] = new QPushButton("-");
      button_task_ctrl_2[i-12][1] = new QPushButton("+");

      doubleSpin_task_ctrl_2[i-12] = new QDoubleSpinBox;
    }

    QString text = "";
    switch(i)
    {
    case 0:
      text = "left arm x (cm)";
      break;
    case 1:
      text = "left arm y (cm)";
      break;
    case 2:
      text = "left arm z (cm)";
      break;
    case 3:
      text = "left arm r (deg)";
      break;
    case 4:
      text = "left arm p (deg)";
      break;
    case 5:
      text = "left arm y (deg)";
      break;
    case 6:
      text = "left leg x (cm)";
      break;
    case 7:
      text = "left leg y (cm)";
      break;
    case 8:
      text = "left leg z (cm)";
      break;
    case 9:
      text = "left leg r (deg)";
      break;
    case 10:
      text = "left leg p (deg)";
      break;
    case 11:
      text = "left leg y (deg)";
      break;
    case 12:
      text = "right arm x (cm)";
      break;
    case 13:
      text = "right arm y (cm)";
      break;
    case 14:
      text = "right arm z (cm)";
      break;
    case 15:
      text = "right arm r (deg)";
      break;
    case 16:
      text = "right arm p (deg)";
      break;
    case 17:
      text = "right arm y (deg)";
      break;
    case 18:
      text = "right leg x (cm)";
      break;
    case 19:
      text = "right leg y (cm)";
      break;
    case 20:
      text = "right leg z (cm)";
      break;
    case 21:
      text = "right leg r (deg)";
      break;
    case 22:
      text = "right leg p (deg)";
      break;
    case 23:
      text = "right leg y (deg)";
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

  updateUI();

}

TaskWindow::~TaskWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void TaskWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

void TaskWindow::updateUI()
{
  ui.groupBox_task_ctrl->setEnabled(isConnected);
  ui.groupBox_walk_ctrl->setEnabled(isConnected);
  ui.groupBox_sensor_ctrl->setEnabled(isConnected);
  ui.groupBox_task_ctrl_option->setEnabled(isConnected);
}

void TaskWindow::on_button_walk_start_clicked()
{
  qnode.walk_cmd_msg_.walk_mode = 1;

  qnode.walk_cmd_msg_.compensator_mode[0] = ui.checkBox_hip_compensator->isChecked();
  qnode.walk_cmd_msg_.compensator_mode[1] = ui.checkBox_ext_encoder_cmp->isChecked();

  qnode.walk_cmd_msg_.walking_pattern = ui.checkBox_DCM ->isChecked();
  //if(ui.check)
  /*
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
  }*/

  if(ui.comboBox_method->currentText().toStdString() == "Inverse Kinematics")
  {
    qnode.walk_cmd_msg_.ik_mode = dyros_jet_msgs::WalkingCommand::IK;
  }
  else if (ui.comboBox_method->currentText().toStdString() == "Jacobian")
  {
    qnode.walk_cmd_msg_.ik_mode = dyros_jet_msgs::WalkingCommand::JACOBIAN;
  }

  qnode.walk_cmd_msg_.first_foot_step = (ui.comboBox_first_step->currentText().toStdString() == "Right");

  qnode.walk_cmd_msg_.heel_toe = (ui.comboBox_heel_toe->currentText().toStdString() == "Yes");


  qnode.walk_cmd_msg_.x = ui.doubleSpinBox_walk_x->value();
  qnode.walk_cmd_msg_.y = ui.doubleSpinBox_walk_y->value();
  qnode.walk_cmd_msg_.z = ui.doubleSpinBox_walk_z->value();
  qnode.walk_cmd_msg_.height = ui.doubleSpinBox_walk_height->value();
  qnode.walk_cmd_msg_.theta = ui.doubleSpinBox_walk_theta->value();
  qnode.walk_cmd_msg_.step_length_x = ui.doubleSpinBox_walk_step_x->value();
  qnode.walk_cmd_msg_.step_length_y = ui.doubleSpinBox_walk_step_y->value();

  qnode.send_walk_ctrl();


  if(ui.checkBox_send_data_cb->isChecked())
  {
    qnode.controlbase_bool_.data = true;

  } else qnode.controlbase_bool_.data = false;

  qnode.send_bool_cb();
}

void TaskWindow::on_button_walk_init_wholebody_clicked()
{
  const double tempNum[32] = {0 , 0.034906585 , -0.034906585 , 0.733038285 , -0.6981317 , -0.034906585
                              , 0 , -0.034906585 , 0.0349065850 , -0.733038285 , 0.6981317 , 0.034906585
                              , 0 , 0
                              , 0.6981317008 , -1.6580627893 , -1.3962634016 , -1.9198621771 , 0 , -1.2217304764 , -0.1745329252
                              , -0.6981317008 , 1.6580627893 , 1.3962634016 , 1.9198621771 , 0 , 1.2217304764 , 0.17453292519
                              , 0 , 0 , 0 , 0};

  const double tempDuration[32] = {5 , 5 , 5 , 5 , 5 , 5
                                   , 5 , 5 , 5 , 5 , 5 , 5
                                   , 5 , 5
                                   , 5 , 5 , 5 , 5 , 5 , 5 , 5
                                   , 5 , 5 , 5 , 5 , 5 , 5 , 5
                                   , 0 , 0 , 0 , 0};

  for(int i = 0; i < 32; i ++)
  {
    qnode.joint_cmd_msg_.name[i] = disarranged_jointName[i];
    qnode.joint_cmd_msg_.position[i] = tempNum[i];
    qnode.joint_cmd_msg_.duration[i] = tempDuration[i];
  }
  qnode.publish_joint_ctrl();

  for(int i = 0; i < 32; i ++)
  {
    qnode.joint_cmd_msg_.name[i] = "";
  }
  qnode.walk_cmd_msg_.walk_mode = 0;
  qnode.send_walk_ctrl();
}

void TaskWindow::on_button_walk_init_lowerbody_clicked()
{
  const double tempNum[32] = {0 , 0.034906585 , -0.034906585 , 0.733038285 , -0.6981317 , -0.034906585
                              , 0 , -0.034906585 , 0.0349065850 , -0.733038285 , 0.6981317 , 0.034906585
                              , 0 , 0
                              , 0.6981317008 , -1.6580627893 , -1.3962634016 , -1.9198621771 , 0 , -1.2217304764 , -0.1745329252
                              , -0.6981317008 , 1.6580627893 , 1.3962634016 , 1.9198621771 , 0 , 1.2217304764 , 0.17453292519
                              , 0 , 0 , 0 , 0};

  const double tempDuration[32] = {5 , 5 , 5 , 5 , 5 , 5
                                   , 5 , 5 , 5 , 5 , 5 , 5
                                   , 5 , 5
                                   , 5 , 5 , 5 , 5 , 5 , 5 , 5
                                   , 5 , 5 , 5 , 5 , 5 , 5 , 5
                                   , 0 , 0 , 0 , 0};

  for(int i = 0; i < 32; i ++)
  {
    if(i < 12) qnode.joint_cmd_msg_.name[i] = disarranged_jointName[i];
    qnode.joint_cmd_msg_.position[i] = tempNum[i];
    qnode.joint_cmd_msg_.duration[i] = tempDuration[i];
  }
  qnode.publish_joint_ctrl();

  for(int i = 0; i < 32; i ++)
  {
    qnode.joint_cmd_msg_.name[i] = "";
  }

  qnode.walk_cmd_msg_.walk_mode = 0;
  qnode.send_walk_ctrl();

}

void TaskWindow::on_button_walk_stop_clicked()
{
  qnode.walk_cmd_msg_.walk_mode = 0;
  qnode.send_walk_ctrl();
}

void TaskWindow::on_button_ft_calib_clicked()
{
  qnode.send_ft_calib(5.0);
}

void TaskWindow::taskCtrlMinusClicked()
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
      qnode.task_cmd_msg_.pose[2].position.x = pos / 100.0;
      break;
    case 2:
      qnode.task_cmd_msg_.pose[2].position.y = pos / 100.0;
      break;
    case 3:
      qnode.task_cmd_msg_.pose[2].position.z = pos / 100.0;
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
    qnode.task_cmd_msg_.duration[2] = 1 - pos/15.0*offset;
    qnode.send_task_ctrl();
  }
  else if(id <= 12)
  {
    qnode.task_cmd_msg_.end_effector[0] = true;
    qnode.task_cmd_msg_.mode[0] = 0;

    switch(id)
    {
    case 7:
      qnode.task_cmd_msg_.pose[0].position.x = pos / 100.0;
      break;
    case 8:
      qnode.task_cmd_msg_.pose[0].position.y = pos / 100.0;
      break;
    case 9:
      qnode.task_cmd_msg_.pose[0].position.z = pos / 100.0;
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
    qnode.task_cmd_msg_.duration[0] = 1 - pos/15.0*offset;
    qnode.send_task_ctrl();
  }
  else if(id <= 18)
  {
    qnode.task_cmd_msg_.end_effector[3] = true;
    qnode.task_cmd_msg_.mode[3] = 0;

    switch(id)
    {
    case 13:
      qnode.task_cmd_msg_.pose[3].position.x = pos / 100.0;
      break;
    case 14:
      qnode.task_cmd_msg_.pose[3].position.y = pos / 100.0;
      break;
    case 15:
      qnode.task_cmd_msg_.pose[3].position.z = pos / 100.0;
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
    qnode.task_cmd_msg_.duration[3] = 1 - pos/15.0*offset;
    qnode.send_task_ctrl();
  }
  else
  {
    qnode.task_cmd_msg_.end_effector[1] = true;
    qnode.task_cmd_msg_.mode[1] = 0;

    switch(id)
    {
    case 19:
      qnode.task_cmd_msg_.pose[1].position.x = pos / 100.0;
      break;
    case 20:
      qnode.task_cmd_msg_.pose[1].position.y = pos / 100.0;
      break;
    case 21:
      qnode.task_cmd_msg_.pose[1].position.z = pos / 100.0;
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
    qnode.task_cmd_msg_.duration[1] = 1 - pos/15.0*offset;
    qnode.send_task_ctrl();
  }
}

void TaskWindow::taskCtrlPlusClicked()
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
      qnode.task_cmd_msg_.pose[2].position.x = pos / 100.0;
      break;
    case 2:
      qnode.task_cmd_msg_.pose[2].position.y = pos / 100.0;
      break;
    case 3:
      qnode.task_cmd_msg_.pose[2].position.z = pos / 100.0;
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
    qnode.task_cmd_msg_.duration[2] = 1 + pos/15.0*offset;
    qnode.send_task_ctrl();
  }
  else if(id <= 12)
  {
    qnode.task_cmd_msg_.end_effector[0] = true;
    qnode.task_cmd_msg_.mode[0] = 0;

    switch(id)
    {
    case 7:
      qnode.task_cmd_msg_.pose[0].position.x = pos / 100.0;
      break;
    case 8:
      qnode.task_cmd_msg_.pose[0].position.y = pos / 100.0;
      break;
    case 9:
      qnode.task_cmd_msg_.pose[0].position.z = pos / 100.0;
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
    qnode.task_cmd_msg_.duration[0] = 1 + pos/15.0*offset;
    qnode.send_task_ctrl();
  }
  else if(id <= 18)
  {
    qnode.task_cmd_msg_.end_effector[3] = true;
    qnode.task_cmd_msg_.mode[3] = 0;

    switch(id)
    {
    case 13:
      qnode.task_cmd_msg_.pose[3].position.x = pos / 100.0;
      break;
    case 14:
      qnode.task_cmd_msg_.pose[3].position.y = pos / 100.0;
      break;
    case 15:
      qnode.task_cmd_msg_.pose[3].position.z = pos / 100.0;
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
    qnode.task_cmd_msg_.duration[3] = 1 + pos/15.0 *offset;
    qnode.send_task_ctrl();
  }
  else
  {
    qnode.task_cmd_msg_.end_effector[1] = true;
    qnode.task_cmd_msg_.mode[1] = 0;

    switch(id)
    {
    case 19:
      qnode.task_cmd_msg_.pose[1].position.x = pos / 100.0;
      break;
    case 20:
      qnode.task_cmd_msg_.pose[1].position.y = pos / 100.0;
      break;
    case 21:
      qnode.task_cmd_msg_.pose[1].position.z = pos / 100.0;
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
    qnode.task_cmd_msg_.duration[1] = 1 + pos/15.0*offset;
    qnode.send_task_ctrl();
  }
}

void TaskWindow::on_pushButton_task_option_clicked()
{
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

  geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(ui.doubleSpinBox_r_ta->value() / 360 * 2 * 3.141592, ui.doubleSpinBox_p_ta->value() / 360 * 2 * 3.141592, ui.doubleSpinBox_y_ta_2->value() / 360 * 2 * 3.141592);

  if(std::strcmp(ui.comboBox_ee_type->currentText().toStdString().c_str(), "Left Arm") == 0)
  {
    qnode.task_cmd_msg_.end_effector[0] = false;
    qnode.task_cmd_msg_.end_effector[1] = false;
    qnode.task_cmd_msg_.end_effector[2] = true;
    qnode.task_cmd_msg_.end_effector[3] = false;
    if(strcmp(ui.comboBox_t_m->currentText().toStdString().c_str(), "Relative") == 0) qnode.task_cmd_msg_.mode[2] = 0;
    else qnode.task_cmd_msg_.mode[2] = 1;

    qnode.task_cmd_msg_.pose[2].position.x = ui.doubleSpinBox_x_ta->value() / 100.0;
    qnode.task_cmd_msg_.pose[2].position.y = ui.doubleSpinBox_y_ta->value() / 100.0;
    qnode.task_cmd_msg_.pose[2].position.z = ui.doubleSpinBox_z_ta->value() / 100.0;

    qnode.task_cmd_msg_.pose[2].orientation.x = quaternion.x;
    qnode.task_cmd_msg_.pose[2].orientation.y = quaternion.y;
    qnode.task_cmd_msg_.pose[2].orientation.z = quaternion.z;
    qnode.task_cmd_msg_.pose[2].orientation.w = quaternion.w;

    qnode.task_cmd_msg_.duration[2] = 3.0;
  }
  else if(std::strcmp(ui.comboBox_ee_type->currentText().toStdString().c_str(), "Right Arm") == 0)
  {
    qnode.task_cmd_msg_.end_effector[0] = false;
    qnode.task_cmd_msg_.end_effector[1] = false;
    qnode.task_cmd_msg_.end_effector[2] = false;
    qnode.task_cmd_msg_.end_effector[3] = true;
    if(strcmp(ui.comboBox_t_m->currentText().toStdString().c_str(), "Relative") == 0) qnode.task_cmd_msg_.mode[3] = 0;
    else qnode.task_cmd_msg_.mode[3] = 1;

    qnode.task_cmd_msg_.pose[3].position.x = ui.doubleSpinBox_x_ta->value() / 100.0;
    qnode.task_cmd_msg_.pose[3].position.y = ui.doubleSpinBox_y_ta->value() / 100.0;
    qnode.task_cmd_msg_.pose[3].position.z = ui.doubleSpinBox_z_ta->value() / 100.0;

    qnode.task_cmd_msg_.pose[3].orientation.x = quaternion.x;
    qnode.task_cmd_msg_.pose[3].orientation.y = quaternion.y;
    qnode.task_cmd_msg_.pose[3].orientation.z = quaternion.z;
    qnode.task_cmd_msg_.pose[3].orientation.w = quaternion.w;

    qnode.task_cmd_msg_.duration[3] = 3.0;
  }
  else if(std::strcmp(ui.comboBox_ee_type->currentText().toStdString().c_str(), "Left Leg") == 0)
  {
    qnode.task_cmd_msg_.end_effector[0] = true;
    qnode.task_cmd_msg_.end_effector[1] = false;
    qnode.task_cmd_msg_.end_effector[2] = false;
    qnode.task_cmd_msg_.end_effector[3] = false;
    if(strcmp(ui.comboBox_t_m->currentText().toStdString().c_str(), "Relative") == 0) qnode.task_cmd_msg_.mode[0] = 0;
    else qnode.task_cmd_msg_.mode[0] = 1;

    qnode.task_cmd_msg_.pose[0].position.x = ui.doubleSpinBox_x_ta->value() / 100.0;
    qnode.task_cmd_msg_.pose[0].position.y = ui.doubleSpinBox_y_ta->value() / 100.0;
    qnode.task_cmd_msg_.pose[0].position.z = ui.doubleSpinBox_z_ta->value() / 100.0;

    qnode.task_cmd_msg_.pose[0].orientation.x = quaternion.x;
    qnode.task_cmd_msg_.pose[0].orientation.y = quaternion.y;
    qnode.task_cmd_msg_.pose[0].orientation.z = quaternion.z;
    qnode.task_cmd_msg_.pose[0].orientation.w = quaternion.w;

    qnode.task_cmd_msg_.duration[0] = 3.0;
  }
  else
  {
    qnode.task_cmd_msg_.end_effector[0] = false;
    qnode.task_cmd_msg_.end_effector[1] = true;
    qnode.task_cmd_msg_.end_effector[2] = false;
    qnode.task_cmd_msg_.end_effector[3] = false;
    if(strcmp(ui.comboBox_t_m->currentText().toStdString().c_str(), "Relative") == 0) qnode.task_cmd_msg_.mode[1] = 0;
    else qnode.task_cmd_msg_.mode[1] = 1;

    qnode.task_cmd_msg_.pose[1].position.x = ui.doubleSpinBox_x_ta->value() / 100.0;
    qnode.task_cmd_msg_.pose[1].position.y = ui.doubleSpinBox_y_ta->value() / 100.0;
    qnode.task_cmd_msg_.pose[1].position.z = ui.doubleSpinBox_z_ta->value() / 100.0;

    qnode.task_cmd_msg_.pose[1].orientation.x = quaternion.x;
    qnode.task_cmd_msg_.pose[1].orientation.y = quaternion.y;
    qnode.task_cmd_msg_.pose[1].orientation.z = quaternion.z;
    qnode.task_cmd_msg_.pose[1].orientation.w = quaternion.w;

    qnode.task_cmd_msg_.duration[1] = 3.0;
  }

  qnode.send_task_ctrl();
}

void TaskWindow::UpdateGraph()
{
  double lf_state, rf_state;

  //lf_state = 500*sin(time*3.14/1.8);
  //rf_state = 500*cos(time*3.14/1.8);

  lf_state = qnode.ft_lf_msg_.wrench.force.z;
  rf_state = qnode.ft_rf_msg_.wrench.force.z;



  if(count > 1500)
  {
    space += 0.02;
    FT_LF->xAxis->setRange(space,30+space);
    FT_RF->xAxis->setRange(space,30+space);
  }






  //https://stackoverflow.com/questions/27417703/how-to-plot-large-time-series-with-qcustomplot-efficiently?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
  FT_LF->graph(0)->addData(time, qnode.ft_lf_msg_.wrench.force.x);
  FT_LF->graph(1)->addData(time, qnode.ft_lf_msg_.wrench.force.y);
  FT_LF->graph(2)->addData(time, qnode.ft_lf_msg_.wrench.force.z);
  FT_LF->graph(3)->addData(time, qnode.ft_lf_msg_.wrench.torque.x);
  FT_LF->graph(4)->addData(time, qnode.ft_lf_msg_.wrench.torque.y);
  FT_LF->graph(5)->addData(time, qnode.ft_lf_msg_.wrench.torque.z);



  FT_RF->graph(0)->addData(time, qnode.ft_rf_msg_.wrench.force.x);
  FT_RF->graph(1)->addData(time, qnode.ft_rf_msg_.wrench.force.y);
  FT_RF->graph(2)->addData(time, qnode.ft_rf_msg_.wrench.force.z);
  FT_RF->graph(3)->addData(time, qnode.ft_rf_msg_.wrench.torque.x);
  FT_RF->graph(4)->addData(time, qnode.ft_rf_msg_.wrench.torque.y);
  FT_RF->graph(5)->addData(time, qnode.ft_rf_msg_.wrench.torque.z);


  /*
  FT_LF->graph(0)->addData(time, 500*sin(time*3.14/1.8));

  FT_LF->graph(1)->addData(time, 400*sin(time*3.14/1.8));

  FT_LF->graph(2)->addData(time, 300*sin(time*3.14/1.8));

  FT_LF->graph(3)->addData(time, 200*sin(time*3.14/1.8));

  FT_LF->graph(4)->addData(time, 100*sin(time*3.14/1.8));

  FT_LF->graph(5)->addData(time, 50*sin(time*3.14/1.8));
*/




  for(int i=0;i<6;i++){
    FT_RF->graph(i)->data()->removeBefore(time - 30);
    FT_LF->graph(i)->data()->removeBefore(time-30);
    FT_LF->graph(i)->setVisible(FALSE);
    FT_RF->graph(i)->setVisible(FALSE);
  }






  if(LF_select=="LF_Force_X"){
    FT_LF->graph(0)->setVisible(TRUE);
  }
  else if(LF_select =="LF_Force_Y"){
    FT_LF->graph(1)->setVisible(TRUE);
  }
  else if(LF_select =="LF_Force_Z"){
    FT_LF->graph(2)->setVisible(TRUE);
  }
  else if(LF_select =="LF_Torque_X"){
    FT_LF->graph(3)->setVisible(TRUE);
  }
  else if(LF_select =="LF_Torque_Y"){
    FT_LF->graph(4)->setVisible(TRUE);
  }
  else if(LF_select =="LF_Torque_Z"){
    FT_LF->graph(5)->setVisible(TRUE);
  }
  else {
  ROS_ERROR("ERROR AT LF Source Selection");
  }


  if(RF_select=="RF_Force_X"){
    FT_RF->graph(0)->setVisible(TRUE);
  }
  else if(RF_select =="RF_Force_Y"){
    FT_RF->graph(1)->setVisible(TRUE);
  }
  else if(RF_select =="RF_Force_Z"){
    FT_RF->graph(2)->setVisible(TRUE);
  }
  else if(RF_select =="RF_Torque_X"){
    FT_RF->graph(3)->setVisible(TRUE);
  }
  else if(RF_select =="RF_Torque_Y"){
    FT_RF->graph(4)->setVisible(TRUE);
  }
  else if(RF_select =="RF_Torque_Z"){
    FT_RF->graph(5)->setVisible(TRUE);
  }
  else {
  ROS_ERROR("ERROR AT RF Source Selection");
  }



  FT_LF->replot();
  FT_RF->replot();

  time += 0.02;
  count += 1;
}

void TaskWindow::on_pushButton_ft_start_clicked()
{
  if(!TimeCheck)
  {
    timer->start(20);
    connect(timer,SIGNAL(timeout()), this, SLOT(UpdateGraph()));
  }
  TimeCheck = true;
}

void TaskWindow::on_pushButton_ft_stop_clicked()
{
  TimeCheck = false;
  disconnect(timer, SIGNAL(timeout()), this, SLOT(UpdateGraph()));
  timer->stop();
}

void TaskWindow::on_pushButton_ft_reset_clicked()
{
  if(!TimeCheck)
  {
    time = 0.0;
    count = 0;
    space = 0;

  for(int i=0;i<6;i++){
    FT_LF->graph(i)->data()->clear();
  }

    FT_LF->xAxis->setRange(0,30,AlignLeft);
    tracer_LF->setVisible(false);
    textLabel_LF->setVisible(false);


    for(int i=0;i<6;i++){
      FT_RF->graph(i)->data()->clear();
    }
    FT_RF->xAxis->setRange(0,30,AlignLeft);
    tracer_RF->setVisible(false);
    textLabel_RF->setVisible(false);

    FT_LF->replot();
    FT_RF->replot();
  }
}

void TaskWindow::ClickedGraph_LF(QMouseEvent* event)
{
  double x = FT_LF->xAxis->pixelToCoord(event->pos().x());
  if(x < time)
  {
    tracer_LF->setVisible(true);
    textLabel_LF->setVisible(true);

    double x_dec2 = floor(10.*(x+0.05))/10.;
    tracer_LF->setGraphKey(x_dec2);

    double y = tracer_LF->position->value();
    double y_dec2 = floor(10.*(y+0.05))/10.;

    textLabel_LF->position->setCoords(x,y);
    textLabel_LF->setText(QString("(%1, %2)").arg(x_dec2).arg(y_dec2));
  }
}

void TaskWindow::ClickedGraph_RF(QMouseEvent* event)
{
  double x = FT_RF->xAxis->pixelToCoord(event->pos().x());
  if(x < time)
  {
    tracer_RF->setVisible(true);
    textLabel_RF->setVisible(true);

    double x_dec2 = floor(10.*(x+0.05))/10.;
    tracer_RF->setGraphKey(x_dec2);

    double y = tracer_RF->position->value();
    double y_dec2 = floor(10.*(y+0.05))/10.;

    textLabel_RF->position->setCoords(x,y);
    textLabel_RF->setText(QString("(%1, %2)").arg(x_dec2).arg(y_dec2));
  }
}
/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void TaskWindow::on_actionAbout_triggered() {
  QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void TaskWindow::ReadSettings() {
  QSettings settings("Qt-Ros Package", "dyros_jet_gui");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
}

void TaskWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "dyros_jet_gui");
  //settings.setValue("topic_name",ui.line_edit_topic->text());
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
}

void TaskWindow::closeEvent(QCloseEvent *event)
{
  WriteSettings();
  QMainWindow::closeEvent(event);
}

}  // namespace dyros_jet_gui

void dyros_jet_gui::TaskWindow::on_comboBox_LF_currentIndexChanged(const QString &arg1)
{
  LF_select = arg1.toStdString().c_str();
}

void dyros_jet_gui::TaskWindow::on_comboBox_RF_currentIndexChanged(const QString &arg1)
{
  RF_select = arg1.toStdString().c_str();
}


void dyros_jet_gui::TaskWindow::on_button_hand_command_clicked()
{
  qnode.hand_cmd_msg_.position[0] = ui.doubleSpinBox_hand_finger1->value() * M_PI / 180.;
  qnode.hand_cmd_msg_.position[1] = ui.doubleSpinBox_hand_finger2->value() * M_PI / 180.;
  qnode.hand_cmd_msg_.position[2] = ui.doubleSpinBox_hand_thumb_fe->value() * M_PI / 180.;
  qnode.hand_cmd_msg_.position[3] = ui.doubleSpinBox_hand_thumb_aa->value() * M_PI / 180.;
  qnode.send_hand_cmd();
}

void dyros_jet_gui::TaskWindow::on_button_hand_preset_open_clicked()
{
  qnode.hand_cmd_msg_.position[0] = 0 * M_PI / 180.;
  qnode.hand_cmd_msg_.position[1] = 0 * M_PI / 180.;
  qnode.hand_cmd_msg_.position[2] = 0 * M_PI / 180.;
  qnode.hand_cmd_msg_.position[3] = 0 * M_PI / 180.;
  qnode.send_hand_cmd();
}

void dyros_jet_gui::TaskWindow::on_button_hand_preset_drill_power_clicked()
{
  qnode.hand_cmd_msg_.position[0] = 80 * M_PI / 180.;
  qnode.hand_cmd_msg_.position[1] = 80 * M_PI / 180.;
  qnode.hand_cmd_msg_.position[2] = 80 * M_PI / 180.;
  qnode.hand_cmd_msg_.position[3] = 15 * M_PI / 180.;
  qnode.send_hand_cmd();
}

void dyros_jet_gui::TaskWindow::on_button_hand_preset_box_pinch_clicked()
{
  qnode.hand_cmd_msg_.position[0] = 45 * M_PI / 180.;
  qnode.hand_cmd_msg_.position[1] = 45 * M_PI / 180.;
  qnode.hand_cmd_msg_.position[2] = 45 * M_PI / 180.;
  qnode.hand_cmd_msg_.position[3] = -10 * M_PI / 180.;
  qnode.send_hand_cmd();
}
