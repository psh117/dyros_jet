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
#include "dyros_jet_gui/joint_window.hpp"
#include <qpixmap.h>
#include <QQuaternion>
#include <QVector3D>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dyros_jet_gui {

const char *JointWindow::arranged_jointName[32] = { "HeadYaw", "HeadPitch"
                   , "WaistPitch", "WaistYaw"
                   , "R_ShoulderPitch", "R_ShoulderRoll", "R_ShoulderYaw", "R_ElbowRoll", "R_WristYaw", "R_WristRoll", "R_HandYaw", "R_Gripper"
                   , "L_ShoulderPitch", "L_ShoulderRoll", "L_ShoulderYaw", "L_ElbowRoll", "L_WristYaw", "L_WristRoll", "L_HandYaw", "L_Gripper"
                   , "R_HipYaw", "R_HipRoll", "R_HipPitch", "R_KneePitch", "R_AnklePitch", "R_AnkleRoll"
                   , "L_HipYaw", "L_HipRoll", "L_HipPitch", "L_KneePitch", "L_AnklePitch", "L_AnkleRoll"};

const char *JointWindow::disarranged_jointName[32] = {"L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll"
    ,"R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll"
    ,"WaistPitch","WaistYaw"
    ,"L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw"
    ,"R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw"
    ,"HeadYaw", "HeadPitch", "R_Gripper", "L_Gripper"};

const int JointWindow::head_id_index[2] = {29, 30};
const int JointWindow::waist_id_index[2] = {13, 14};
const int JointWindow::right_arm_id_index[8] = {22, 23, 24, 25, 26, 27, 28, 31};
const int JointWindow::left_arm_id_index[8] = {15, 16, 17, 18, 19, 20, 21, 32};
const int JointWindow::right_leg_id_index[6] = {7, 8, 9, 10, 11, 12};
const int JointWindow::left_leg_id_index[6] = {1, 2, 3, 4, 5, 6};

using namespace Qt;

// std::string motorID = {"R-SP", "L-SP", "R-SR", ""};
/*****************************************************************************
** Implementation [JointWindow]
*****************************************************************************/

JointWindow::JointWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode(argc,argv)
  , isConnected(false)
{
  if ( !qnode.init() ) {
    showNoMasterMessage();
  } else {
    isConnected = true;
  }
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  ui.groupBox_3->setStyleSheet("#groupBox_3 {background-image: url(:/images/robot_with_no_background.png); background-repeat: none; background-position: center;}");

  ReadSettings();
  setWindowIcon(QIcon(":/images/icon_2.png"));
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  /*********************
  ** Logging
  **********************/
  QObject::connect(&qnode, SIGNAL(jointStateUpdated()), this, SLOT(updateJointView()));

  /*********************
    ** Auto Start
    **********************/


  // | Creating UI

  // -- Table


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

  // -- Joint Control Set
  QString tempName;
  int h = 26;
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

    button_joint_ctrl[i][0]->setMaximumWidth(60);
    button_joint_ctrl[i][1]->setMaximumWidth(60);
    button_joint_ctrl[i][2]->setMaximumWidth(80);
    button_joint_ctrl[i][0]->setMaximumHeight(h);
    button_joint_ctrl[i][1]->setMaximumHeight(h);
    button_joint_ctrl[i][2]->setMaximumHeight(h);

    button_joint_ctrl[i][0]->setObjectName(tr("%1").arg(i+1));
    button_joint_ctrl[i][1]->setObjectName(tr("%1").arg(i+1));
    button_joint_ctrl[i][2]->setObjectName(tr("%1").arg(i+1));

    doubleSpin_joint_ctrl[i]->setMaximumWidth(100);
    doubleSpin_joint_ctrl[i]->setValue(0.0);
    doubleSpin_joint_ctrl[i]->setMinimum(-20.0);
    doubleSpin_joint_ctrl[i]->setMaximum(20.0);
    doubleSpin_joint_ctrl[i]->setMaximumHeight(h);

    label_joint_ctrl[i]->setMinimumWidth(130);
    label_joint_ctrl[i]->setMaximumWidth(500);
    label_joint_ctrl[i]->setMaximumHeight(h);
    label_joint_ctrl[i]->setAlignment(Qt::AlignCenter);

if(i < 2)
{
  ui.gridLayout_joint_ctrl->addWidget(button_joint_ctrl[i][0], i, 0, Qt::AlignTop);
  ui.gridLayout_joint_ctrl->addWidget(label_joint_ctrl[i]    , i, 1, Qt::AlignTop);
  ui.gridLayout_joint_ctrl->addWidget(button_joint_ctrl[i][1], i, 2, Qt::AlignTop);
  ui.gridLayout_joint_ctrl->addWidget(doubleSpin_joint_ctrl[i] , i, 3, Qt::AlignTop);
  ui.gridLayout_joint_ctrl->addWidget(button_joint_ctrl[i][2], i, 4, Qt::AlignTop);
}
else if(i < 4)
{
  ui.gridLayout_joint_ctrl_2->addWidget(button_joint_ctrl[i][0], i-2, 0, Qt::AlignTop);
  ui.gridLayout_joint_ctrl_2->addWidget(label_joint_ctrl[i]    , i-2, 1, Qt::AlignTop);
  ui.gridLayout_joint_ctrl_2->addWidget(button_joint_ctrl[i][1], i-2, 2, Qt::AlignTop);
  ui.gridLayout_joint_ctrl_2->addWidget(doubleSpin_joint_ctrl[i] , i-2, 3, Qt::AlignTop);
  ui.gridLayout_joint_ctrl_2->addWidget(button_joint_ctrl[i][2], i-2, 4, Qt::AlignTop);
}
else if(i < 12)
{
  ui.gridLayout_joint_ctrl_3->addWidget(button_joint_ctrl[i][0], i-4, 0, Qt::AlignTop);
  ui.gridLayout_joint_ctrl_3->addWidget(label_joint_ctrl[i]    , i-4, 1, Qt::AlignTop);
  ui.gridLayout_joint_ctrl_3->addWidget(button_joint_ctrl[i][1], i-4, 2, Qt::AlignTop);
  ui.gridLayout_joint_ctrl_3->addWidget(doubleSpin_joint_ctrl[i] , i-4, 3, Qt::AlignTop);
  ui.gridLayout_joint_ctrl_3->addWidget(button_joint_ctrl[i][2], i-4, 4, Qt::AlignTop);
}
else if(i < 20)
{
  ui.gridLayout_joint_ctrl_4->addWidget(button_joint_ctrl[i][0], i-12, 0, Qt::AlignTop);
  ui.gridLayout_joint_ctrl_4->addWidget(label_joint_ctrl[i]    , i-12, 1, Qt::AlignTop);
  ui.gridLayout_joint_ctrl_4->addWidget(button_joint_ctrl[i][1], i-12, 2, Qt::AlignTop);
  ui.gridLayout_joint_ctrl_4->addWidget(doubleSpin_joint_ctrl[i] , i-12, 3, Qt::AlignTop);
  ui.gridLayout_joint_ctrl_4->addWidget(button_joint_ctrl[i][2], i-12, 4, Qt::AlignTop);
}
else if(i < 26)
{
  ui.gridLayout_joint_ctrl_5->addWidget(button_joint_ctrl[i][0], i-20, 0, Qt::AlignTop);
  ui.gridLayout_joint_ctrl_5->addWidget(label_joint_ctrl[i]    , i-20, 1, Qt::AlignTop);
  ui.gridLayout_joint_ctrl_5->addWidget(button_joint_ctrl[i][1], i-20, 2, Qt::AlignTop);
  ui.gridLayout_joint_ctrl_5->addWidget(doubleSpin_joint_ctrl[i] , i-20, 3, Qt::AlignTop);
  ui.gridLayout_joint_ctrl_5->addWidget(button_joint_ctrl[i][2], i-20, 4, Qt::AlignTop);
}
else
{
  ui.gridLayout_joint_ctrl_6->addWidget(button_joint_ctrl[i][0], i-26, 0, Qt::AlignTop);
  ui.gridLayout_joint_ctrl_6->addWidget(label_joint_ctrl[i]    , i-26, 1, Qt::AlignTop);
  ui.gridLayout_joint_ctrl_6->addWidget(button_joint_ctrl[i][1], i-26, 2, Qt::AlignTop);
  ui.gridLayout_joint_ctrl_6->addWidget(doubleSpin_joint_ctrl[i] , i-26, 3, Qt::AlignTop);
  ui.gridLayout_joint_ctrl_6->addWidget(button_joint_ctrl[i][2], i-26, 4, Qt::AlignTop);
}

    QObject::connect(button_joint_ctrl[i][0], SIGNAL(clicked()), this, SLOT(jointCtrlMinusClicked()));
    QObject::connect(button_joint_ctrl[i][1], SIGNAL(clicked()), this, SLOT(jointCtrlPlusClicked()));
    QObject::connect(button_joint_ctrl[i][2], SIGNAL(clicked()), this, SLOT(jointCtrlSetClicked()));
  }

  updateUI();
}

JointWindow::~JointWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void JointWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

void JointWindow::updateUI()
{
  ui.groupBox_joint_ctrl->setEnabled(isConnected);
  ui.groupBox_save_load->setEnabled(isConnected);
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void JointWindow::jointCtrlMinusClicked()
{
  int id = sender()->objectName().toInt();

  qnode.send_joint_ctrl(arranged_id[id-1], arranged_jointName[id-1], -1.0); // degree
}

void JointWindow::jointCtrlPlusClicked()
{
  int id = sender()->objectName().toInt();

  qnode.send_joint_ctrl(arranged_id[id-1], arranged_jointName[id-1], 1.0); // degree
}

void JointWindow::jointCtrlSetClicked()
{
  int id = sender()->objectName().toInt();
  double deg = doubleSpin_joint_ctrl[id-1]->value();

  qnode.send_joint_ctrl(arranged_id[id-1], arranged_jointName[id-1], deg); // degree
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */

void JointWindow::setTable(int i, int column, QTableWidgetItem *newItem)
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

void JointWindow::updateJointView() {

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

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void JointWindow::on_actionAbout_triggered() {
  QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void JointWindow::ReadSettings() {
  QSettings settings("Qt-Ros Package", "dyros_jet_gui");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
}

void JointWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "dyros_jet_gui");
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
}

void JointWindow::closeEvent(QCloseEvent *event)
{
  WriteSettings();
  QMainWindow::closeEvent(event);
}

void JointWindow::on_button_motor_save_clicked()
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

void JointWindow::on_all_checkBox_clicked()
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

void JointWindow::ConfineChecklist()
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
      for(i = 0; i < 2; i++) qnode.joint_cmd_msg_.name[arranged_id[i]-1] = arranged_jointName[i];
    }
    if(ui.w_checkBox->isChecked())
    {
      for(i = 2; i < 4; i++) qnode.joint_cmd_msg_.name[arranged_id[i]-1] = arranged_jointName[i];
    }
    if(ui.ra_checkBox->isChecked())
    {
      for(i = 4; i < 12; i++) qnode.joint_cmd_msg_.name[arranged_id[i]-1] = arranged_jointName[i];
    }
    if(ui.la_checkBox->isChecked())
    {
      for(i = 12; i < 20; i++) qnode.joint_cmd_msg_.name[arranged_id[i]-1] = arranged_jointName[i];
    }
    if(ui.rl_checkBox->isChecked())
    {
      for(i = 20; i < 26; i++) qnode.joint_cmd_msg_.name[arranged_id[i]-1] = arranged_jointName[i];
    }
    if(ui.ll_checkBox->isChecked())
    {
      for(i = 26; i < 32; i++) qnode.joint_cmd_msg_.name[arranged_id[i]-1] = arranged_jointName[i];
    }
  }
}

void JointWindow::on_button_motor_load_clicked()
{

  std::string homeDir(getenv("HOME"));
  std::string fullDir = homeDir + "/" + ui.line_edit_motor_load_file_name->text().toStdString(); //+ path of which data should be saved
  std::ifstream inFile(fullDir.c_str());

  double current_angle;
  double offset = 0.5;
  double difference;
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
    current_angle = qnode.joint_msg_.angle[arranged_id[i]-1];
    difference = current_angle - (loaded_pos[i] / 57.295791433);
    if(difference > 0) qnode.joint_cmd_msg_.duration[arranged_id[i]-1] = 0.5 + difference*offset;
    else qnode.joint_cmd_msg_.duration[arranged_id[i]-1] = 0.5 - difference*offset;
  }

  qnode.publish_joint_ctrl();
}

void JointWindow::on_basic_button_clicked()
{
  ui.groupBox_3->setStyleSheet("#groupBox_3 {background-image: url(:/images/robot_with_no_background.png); background-repeat: none; background-position: center;}");
}

void JointWindow::on_detail_button_clicked()
{
  ui.groupBox_3->setStyleSheet("#groupBox_3 {background-image: url(:/images/icon_2.png); background-repeat: none; background-position: center;}");
}

}  // namespace dyros_jet_gui


