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
#include "dyros_jet_gui/status_window.hpp"
#include "rt_dynamixel_msgs/ModeSetting.h"
#include "rt_dynamixel_msgs/MotorSetting.h"
#include <qpixmap.h>
#include <QQuaternion>
#include <QVector3D>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dyros_jet_gui {

using namespace Qt;

// std::string motorID = {"R-SP", "L-SP", "R-SR", ""};
/*****************************************************************************
** Implementation [StatusWindow]
*****************************************************************************/

StatusWindow::StatusWindow(int argc, char** argv, QWidget *parent)
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


  QObject::connect(&qnode, SIGNAL(stateUpdated()), this, SLOT(updateStateView()));
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

  for(int i=0; i<32; i++)
  {
    jointID.push_back(i+1);
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

  updateUI();

}

StatusWindow::~StatusWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void StatusWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

void StatusWindow::autoMissionSelectVisible(int mission)
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
void StatusWindow::updateUI()
{
  ui.groupBox_state->setEnabled(isConnected);
  ui.groupBox_recog_ctrl->setEnabled(isConnected);
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void StatusWindow::on_button_scan_clicked()
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

void StatusWindow::on_button_estop_clicked()
{
  std::cout << "ESTOP...";
  qnode.changeDxlMode(rt_dynamixel_msgs::ModeSetting::Request::SETTING);
  qnode.setTorque(0);
  std::cout << " DONE" << std::endl;
  std::cout << "SHUTDOWN..." ;
  usleep(1000000);
  qnode.shutdown();
  std::cout << " DONE" << std::endl;
}

void StatusWindow::on_button_torque_on_clicked()
{
  qnode.changeDxlMode(rt_dynamixel_msgs::ModeSetting::Request::SETTING);
  qnode.setTorque(1);
  qnode.changeDxlMode(rt_dynamixel_msgs::ModeSetting::Request::CONTROL_RUN);
}

void StatusWindow::stateButtonClicked()
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

  //ui.line_edit_current_state->setText(objName);
}


/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */


void StatusWindow::updateRecogInfo() {
/*
  ui.line_edit_scan_x->setText(QString::number(qnode.recog_info_msg.data[0] * 100,'f',2) + " cm");
  ui.line_edit_scan_y->setText(QString::number(qnode.recog_info_msg.data[1] * 100,'f',2) + " cm");
  ui.line_edit_scan_z->setText(QString::number(qnode.recog_info_msg.data[2] * 100,'f',2) + " cm");

  ui.line_edit_scan_roll->setText(QString::number(qnode.recog_info_msg.data[3],'f',2));
  ui.line_edit_scan_pitch->setText(QString::number(qnode.recog_info_msg.data[4],'f',2));
  ui.line_edit_scan_yaw->setText(QString::number(qnode.recog_info_msg.data[5],'f',2));
*/
}

void StatusWindow::updateStateView() {
  ui.line_edit_current_state->setText(qnode.getCurrentState().c_str());
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void StatusWindow::on_actionAbout_triggered() {
  QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void StatusWindow::ReadSettings() {
  QSettings settings("Qt-Ros Package", "dyros_jet_gui");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
}

void StatusWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "dyros_jet_gui");
  //settings.setValue("topic_name",ui.line_edit_topic->text());
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
}

void StatusWindow::closeEvent(QCloseEvent *event)
{
  WriteSettings();
  QMainWindow::closeEvent(event);
}

}  // namespace dyros_jet_gui


