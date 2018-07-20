/**
 * @file /include/dyros_jet_gui/main_window.hpp
 *
 * @brief Qt based gui for dyros_jet_gui.
 *
 * @date November 2010
 **/
#ifndef dyros_jet_gui_STATUS_WINDOW_H
#define dyros_jet_gui_STATUS_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <QtGui/QMainWindow>
#include <vector>
#include "ui_status_window.h"
#include "status_qnode.hpp"
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
class StatusWindow : public QMainWindow {
Q_OBJECT

public:
  StatusWindow(int argc, char** argv, QWidget *parent = 0);
  ~StatusWindow();

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

    void on_button_scan_clicked();
    void on_button_estop_clicked();
    //void on_button_torque_on_clicked();
    /******************************************
    ** Code based UI connections
    *******************************************/
    void stateButtonClicked();
    /******************************************
    ** Manual connections
    *******************************************/
    void updateRecogInfo();
    void updateStateView();

private:
  Ui::StatusWindow ui;
  StatusQNode qnode;
    std::vector<int> jointID;
    bool isConnected;

public:

    // -- UI
    QPushButton *button_recog_ctrl;

};
}  // namespace dyros_jet_gui

#endif // dyros_jet_gui_STATUS_WINDOW_H
