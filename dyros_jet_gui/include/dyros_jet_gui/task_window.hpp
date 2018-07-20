/**
 * @file /include/dyros_jet_gui/task_window.hpp
 *
 * @brief Qt based gui for dyros_jet_gui.
 *
 * @date November 2010
 **/
#ifndef dyros_jet_gui_TASK_WINDOW_H
#define dyros_jet_gui_TASK_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <QtGui/QMainWindow>
#include <qapplication.h>
#include <vector>
#include "ui_task_window.h"
#include "task_qnode.hpp"
#include <QPushButton>
#include <QVarLengthArray>
#include <QLabel>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QPalette>
#include <geometry_msgs/WrenchStamped.h>
#include <qwidget.h>
#include <QTimer>
#include <QTime>
#include "qcustomplot/qcustomplot.h"
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
class TaskWindow : public QMainWindow {
Q_OBJECT

public:
  TaskWindow(int argc, char** argv, QWidget *parent = 0);
  ~TaskWindow();

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

    // on_{the name of button}_clicked() --> Automatically slot(connect to UI)

    void on_button_walk_start_clicked();
    void on_button_walk_init_wholebody_clicked();
    void on_button_walk_init_lowerbody_clicked();
    void on_button_walk_stop_clicked();
    void on_button_ft_calib_clicked();

    void on_button_hand_command_clicked();
    void on_button_hand_preset_open_clicked();
    void on_button_hand_preset_drill_power_clicked();
    void on_button_hand_preset_box_pinch_clicked();


    void on_pushButton_ft_start_clicked();
    void on_pushButton_ft_stop_clicked();
    void on_pushButton_ft_reset_clicked();
    void UpdateGraph();
    void ClickedGraph_LF(QMouseEvent* event);
    void ClickedGraph_RF(QMouseEvent* event);

    void on_comboBox_LF_currentIndexChanged(const QString &arg1);
    void on_comboBox_RF_currentIndexChanged(const QString &arg1);
    /******************************************
    ** Code based UI connections
    *******************************************/

    void taskCtrlMinusClicked();
    void taskCtrlPlusClicked();
    void on_pushButton_task_option_clicked();

    /******************************************
    ** Manual connections
    *******************************************/

private:
  Ui::TaskWindow ui;
  TaskQNode qnode;
    std::vector<int> jointID;
    bool isConnected;

public:
  static const char *disarranged_jointName[32];

    // -- UI
    // ---- Task Ctrl
    QPushButton *button_task_ctrl[12][2];
    QPushButton *button_task_ctrl_2[12][2];
    QDoubleSpinBox *doubleSpin_task_ctrl[12];
    QDoubleSpinBox *doubleSpin_task_ctrl_2[12];
    QLabel *label_task_ctrl[13];
    QLabel *label_task_ctrl_2[13];
    QLabel *label_task_cm;

    QCustomPlot* FT_LF;
    QCustomPlot* FT_RF;

    QTimer* timer;
    QCPItemText *textLabel_LF;
    QCPItemText *textLabel_RF;

    QCPItemTracer* tracer_LF;
    QCPItemTracer* tracer_RF;

    double time;
    bool TimeCheck;
    int count;
    double space;

    std::string LF_select;
    std::string RF_select;


};
}  // namespace dyros_jet_gui

#endif // dyros_jet_gui_TASK_WINDOW_H
