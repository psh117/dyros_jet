/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "dyros_jet_gui/joint_window.hpp"
#include "dyros_jet_gui/task_window.hpp"
#include "dyros_jet_gui/status_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);

      dyros_jet_gui::JointWindow w1(argc,argv);
      dyros_jet_gui::TaskWindow w2(argc,argv);
      dyros_jet_gui::StatusWindow w3(argc,argv);

      int height_offset = 60;
      int width_offset = 100;

      if(QDesktopWidget().height()==1440){


        w1.setGeometry(0, 0, 1230, 690);
        w2.setGeometry(0, 790, 1230, 690);
        w3.setGeometry(1290, 0, 1230, 690);



      }
      else if(QDesktopWidget().height()==2160){

        w1.setGeometry(0, 0, 1870, 1050);
        w2.setGeometry(0, 1110, 1870, 1050);
        w3.setGeometry(1970, 0, 1870, 1050);



      }




      //w1.setGeometry(0, 0, QDesktopWidget().width()/2, QDesktopWidget().height()/2 - height_offset);
      //w2.setGeometry(0, QDesktopWidget().height()/2 + height_offset, QDesktopWidget().width()/2, QDesktopWidget().height()/2 -height_offset);
      //w3.setGeometry(QDesktopWidget().width()/2 + width_offset, QDesktopWidget().height()/2 + height_offset, QDesktopWidget().width()/2 - width_offset, QDesktopWidget().height()/2 - height_offset);





      w1.show();
      w2.show();
      w3.show();



    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
