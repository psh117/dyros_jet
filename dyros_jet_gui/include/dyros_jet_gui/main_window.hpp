/**
 * @file /include/dyros_jet_gui/main_window.hpp
 *
 * @brief Qt based gui for dyros_jet_gui.
 *
 * @date November 2010
 **/
#ifndef dyros_jet_gui_MAIN_WINDOW_H
#define dyros_jet_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <QtGui/QMainWindow>
#include <vector>
#include "ui_main_window.h"
#include "qnode.hpp"
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
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

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
    void on_button_connect_clicked(bool check);

    void on_checkbox_use_environment_stateChanged(int state);
    void on_button_walk_start_clicked();
    void on_button_walk_init_clicked();
    void on_button_walk_stop_clicked();

    void on_button_scan_clicked();
    void on_button_estop_clicked();
    void on_button_ft_calib_clicked();
    void on_button_joint_save_clicked();
    /******************************************
    ** Code based UI connections
    *******************************************/
    void stateButtonClicked();
    void jointCtrlMinusClicked();
    void jointCtrlPlusClicked();
    void jointCtrlSetClicked();

    void taskCtrlMinusClicked();
    void taskCtrlPlusClicked();
    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void updateJointView();
    void updateRecogInfo();


private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    std::vector<int> jointID;
    bool isConnected;


    // -- UI
    // ---- Joint Ctrl
    QPushButton *button_joint_ctrl[32][3];
    QDoubleSpinBox *doubleSpin_joint_ctrl[32];
    QLabel *label_joint_ctrl[32];

    // ---- Task Ctrl
    QPushButton *button_task_ctrl[12][2];
    QDoubleSpinBox *doubleSpin_task_ctrl[12];
    QLabel *label_task_ctrl[13];

    QPushButton *button_recog_ctrl;

};
}  // namespace dyros_jet_gui

#endif // dyros_jet_gui_MAIN_WINDOW_H
