/**
 * @file /include/dyros_jet_gui/joint_window.hpp
 *
 * @brief Qt based gui for dyros_jet_gui.
 *
 * @date November 2010
 **/
#ifndef dyros_jet_gui_JOINT_WINDOW_H
#define dyros_jet_gui_JOINT_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <QtGui/QMainWindow>
#include <vector>
#include "ui_joint_window.h"
#include "joint_qnode.hpp"
#include <QPushButton>
#include <QVarLengthArray>
#include <QLabel>
#include <QLineEdit>
#include <QDoubleSpinBox>
#endif

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace dyros_jet_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class JointWindow : public QMainWindow {
Q_OBJECT

public:
  JointWindow(int argc, char** argv, QWidget *parent = 0);
  ~JointWindow();

  void ReadSettings(); // Load up qt program settings at startup
  void WriteSettings(); // Save qt program settings when closing

  void closeEvent(QCloseEvent *event); // Overloaded function
  void showNoMasterMessage();
    void autoMissionSelectVisible(int mission);
    void updateUI();

public Q_SLOTS:
  /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/
    void on_actionAbout_triggered();
    /******************************************
    ** Code based UI connections
    *******************************************/
    void jointCtrlMinusClicked();
    void jointCtrlPlusClicked();
    void jointCtrlSetClicked();
    /******************************************
    ** Manual connections
    *******************************************/
    /*void updateLoggingView();*/ // no idea why this can't connect automatically
    void updateJointView();
    void setTable(int i, int column, QTableWidgetItem *newItem);
    void on_button_motor_save_clicked();
    void on_button_motor_load_clicked();
    void ConfineChecklist();
    void on_all_checkBox_clicked();
    void on_basic_button_clicked();
    void on_detail_button_clicked();

private:
  Ui::JointWindow ui;
  JointQNode qnode;
    std::vector<int> jointID;
    bool isConnected;

public:
  static const int head_id_index[2];
  static const int waist_id_index[2];
  static const int right_arm_id_index[8];
  static const int left_arm_id_index[8];
  static const int right_leg_id_index[6];
  static const int left_leg_id_index[6];

  static const char *arranged_jointName[32];
  static const char *disarranged_jointName[32];

  int arranged_id[32];

    // -- UI
    // ---- Joint Ctrl
    QPushButton *button_joint_ctrl[32][3];
    QDoubleSpinBox *doubleSpin_joint_ctrl[32];
    QLabel *label_joint_ctrl[32];

};
}  // namespace dyros_jet_gui

#endif // dyros_jet_gui_JOINT_WINDOW_H
