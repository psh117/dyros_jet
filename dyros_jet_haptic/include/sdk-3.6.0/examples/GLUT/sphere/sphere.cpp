//  (C) 2001-2015 Force Dimension
//  All Rights Reserved.
//
//  Version 3.6.0



#include <stdio.h>
#include <string.h>
#include <iostream>
using namespace std;

#include "CMacrosGL.h"
using namespace Eigen;

#include "dhdc.h"



// menu options
const int OPTION_FULLSCREEN     = 1;
const int OPTION_WINDOWDISPLAY  = 2;

// size of sphere
const double SphereRadius = 0.03;

// size of finger
const double FingerRadius = 0.005;

// status flags
bool SimulationOn;
bool SimulationFinished;

// device-specific globals
int      FingerCount;
Vector3d FingerPosGlobal[2];
bool     HasRot;
bool     HasGrip;

// position of sphere in global world coordinates
Vector3d SpherePosGlobal;

// text overlay globals
double LastTime;
double Freq;
char   Perf[50];
bool   ShowRate = true;

// object properties
const double Stiffness = 1000.0;



// GLUT display callback
void
UpdateDisplay (int val)
{
  if (SimulationOn) glutPostRedisplay ();

  glutTimerFunc (30, UpdateDisplay, 0);
}



// GLUT/OpenGL rendering function
void
Draw(void)
{
  int i;

  cMatrixGL      mat;
  GLUquadricObj *sphere;
  double         deviceRotGlobal[3][3];

  // clean up
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // retrieve orientation frame (identity for 3-dof devices)
  dhdGetOrientationFrame (deviceRotGlobal);

  // render sphere
  glEnable  (GL_COLOR_MATERIAL);
  glColor3f (0.1f, 0.3f, 0.5f);
  mat.set (SpherePosGlobal);
  mat.glMatrixPushMultiply ();
  sphere = gluNewQuadric ();
  gluSphere (sphere, SphereRadius, 32, 32);
  mat.glMatrixPop ();

  // render finger(s)
  for (i=0; i<FingerCount; i++) {
    glColor3f (0.8f, 0.8f, 0.8f);
    mat.set (FingerPosGlobal[i], deviceRotGlobal);
    mat.glMatrixPushMultiply ();
    sphere = gluNewQuadric ();
    gluSphere (sphere, FingerRadius, 32, 32);

    // render a small frame
    if (HasRot) {
      glDisable  (GL_LIGHTING);
      glBegin    (GL_LINES);
      glColor3f (0.45f, 0.45f, 0.45f);
      glVertex3d (0.00,  0.000,  0.000);
      glVertex3d (0.02,  0.000,  0.000);
      glVertex3d (0.02, -0.004,  0.000);
      glVertex3d (0.02,  0.004,  0.000);
      glVertex3d (0.02,  0.000, -0.004);
      glVertex3d (0.02,  0.000,  0.004);
      glEnd();
      glEnable  (GL_LIGHTING);
    }

    mat.glMatrixPop ();
  }

  // text overlay
  if (ShowRate) {
    if (dhdGetTime() - LastTime > 0.1) {
      Freq     = dhdGetComFreq();
      LastTime = dhdGetTime ();
      sprintf (Perf, "%0.03f kHz", Freq);
    }
    glDisable (GL_LIGHTING);
    glColor3f  (1.0, 1.0, 1.0);
    glRasterPos3f (0.0f, -0.01f, -0.1f);
    for (char *c=Perf; *c != '\0'; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
    glEnable  (GL_LIGHTING);
  }

  glutSwapBuffers ();

  GLenum err = glGetError();
  if (err != GL_NO_ERROR) printf ("error:  %s\n", gluErrorString(err));
}



// exit callback
void
Close (void)
{
  // finish haptic loop
  SimulationOn = false;
  while (!SimulationFinished) dhdSleep (0.1);

  // close device
  dhdClose ();
}



// haptic thread
void*
HapticsLoop (void* pUserData)
{
  static double t0 = dhdGetTime ();
  double        t  = t0 + 0.001;

  int      i;
  Vector3d newFingerPosGlobal;
  Vector3d newFingerPosLocal;
  Vector3d forceGlobal[2];

  // start haptic simulation
  SimulationOn       = true;
  SimulationFinished = false;

  // start with no force
  forceGlobal[0].setZero ();
  forceGlobal[1].setZero ();

  // enable force
  dhdEnableForce (DHD_ON);

  // main haptic simulation loop
  while (SimulationOn) {

    t  = dhdGetTime ();
    t0 = t;

    double x, y, z;

    // adapt behavior to device capabilities
    if (HasGrip) {
      dhdGetGripperThumbPos (&x, &y, &z);
      FingerPosGlobal[0] << x, y, z;

      dhdGetGripperFingerPos (&x, &y, &z);
      FingerPosGlobal[1] << x, y, z;
    }
    else {
      dhdGetPosition (&x, &y, &z);
      FingerPosGlobal[0] << x, y, z;
    }

    // compute interaction between sphere and each finger
    for (i=0; i<FingerCount; i++) {

      // compute penetration
      Vector3d dir  = (FingerPosGlobal[i] - SpherePosGlobal).normalized();
      double   dist = (FingerPosGlobal[i] - SpherePosGlobal).norm() - SphereRadius - FingerRadius;

      // compute force
      if (dist < 0.0) forceGlobal[i] = -dist * Stiffness * dir;
      else            forceGlobal[i].setZero ();
    }

    // compute projected force on each gripper finger
    Vector3d force;
    double   gripperForce = 0.0;
    if (HasGrip) {

      // compute total force
      force = forceGlobal[0] + forceGlobal[1];
      Vector3d gripdir = FingerPosGlobal[1] - FingerPosGlobal[0];

      // if force is not null
      if (gripdir.norm() > 0.00001) {

        // project force on mobile gripper finger (forceGlobal[1]) onto gripper opening vector (gripdir)
        gripdir.normalize ();
        Vector3d gripper = (forceGlobal[1].dot (gripdir) / (gripdir.squaredNorm())) * gripdir;
        gripperForce = gripper.norm();

        // compute the direction of the force based on the angle between
        // the gripper force vector (gripper) and the gripper opening vector (gripdir)
        if (force.norm() > 0.001) {
          double cosangle = gripdir.dot (gripper) / (gripdir.norm()*gripper.norm());
          if      (cosangle >  1.0) cosangle =  1.0;
          else if (cosangle < -1.0) cosangle = -1.0;
          double angle = acos(cosangle);
          if ((angle > M_PI/2.0) || (angle < -M_PI/2.0)) gripperForce = -gripperForce;
        }
      }

      // invert if necessary for left-handed devices
      if (dhdIsLeftHanded()) gripperForce = -gripperForce;
    }
    else force = forceGlobal[0];

    // apply all forces at once
    dhdSetForceAndGripperForce (force(0), force(1), force(2), gripperForce);
  }

  // close connection with haptic device
  dhdClose ();

  // simulation is now exiting
  SimulationFinished = true;

  // return
  return NULL;
}



// window resize GLUT callback
void
OnWindowResize (int w,
                int h)
{
  double glAspect = ((double)w / (double)h);

  glViewport      (0, 0, w, h);
  glMatrixMode    (GL_PROJECTION);
  glLoadIdentity  ();
  gluPerspective  (60, glAspect, 0.01, 10);

  gluLookAt       (0.2, 0, 0,
                   0, 0, 0,
                   0, 0, 1);

  glMatrixMode    (GL_MODELVIEW);
  glLoadIdentity  ();
}



// GLUT window keypress callback
void
OnKey (unsigned char key,
       int           x,
       int           y)
{
  // detect exit requests
  if ((key == 27) || (key == 'q')) {
    SimulationOn = false;
    while (!SimulationFinished) dhdSleep (0.01);
    exit(0);
  }

  // toggle refresh rate display
  if (key == 'r') ShowRate = !ShowRate;
}



// GLUT right-click menu callback
void
RightClickMenu (int value)
{
  switch (value) {
  case OPTION_FULLSCREEN:
    glutFullScreen ();
    break;
  case OPTION_WINDOWDISPLAY:
    glutReshapeWindow      (512, 512);
    glutInitWindowPosition (0, 0);
    break;
  }

  glutPostRedisplay ();
}



// GLUT initialization
int
InitGLUT (int &argc, char *argv[])
{
  // initialization
  glutInit (&argc, argv);

  // window size and position
  int WINDOW_SIZE_W = 512;
  int WINDOW_SIZE_H = 512;
  int screenW       = glutGet (GLUT_SCREEN_WIDTH);
  int screenH       = glutGet (GLUT_SCREEN_HEIGHT);
  int windowPosX    = (screenW - WINDOW_SIZE_W) / 2;
  int windowPosY    = (screenH - WINDOW_SIZE_H) / 2;

  // create window
  glutInitWindowSize      (WINDOW_SIZE_W, WINDOW_SIZE_H);
  glutInitWindowPosition  (windowPosX, windowPosY);
  glutInitDisplayMode     (GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
  glutCreateWindow        (argv[0]);
  glutDisplayFunc         (Draw);
  glutKeyboardFunc        (OnKey);
  glutReshapeFunc         (OnWindowResize);
  glutSetWindowTitle      ("Force Dimension - Sphere Example");

  // create right-click menu
  glutCreateMenu          (RightClickMenu);
  glutAddMenuEntry        ("Full Screen",    OPTION_FULLSCREEN);
  glutAddMenuEntry        ("Window Display", OPTION_WINDOWDISPLAY);
  glutAttachMenu          (GLUT_RIGHT_BUTTON);

  // start in window mode
  glutReshapeWindow      (512, 512);
  glutInitWindowPosition (0, 0);

  // set material properties
  GLfloat mat_ambient[]  = {0.5f, 0.5f, 0.5f};
  GLfloat mat_diffuse[]  = {0.5f, 0.5f, 0.5f};
  GLfloat mat_specular[] = {0.5f, 0.5f, 0.5f};
  glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
  glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
  glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
  glMaterialf  (GL_FRONT_AND_BACK, GL_SHININESS, 1.0);

  // set light source
  GLfloat ambient[]  = {0.5f, 0.5f, 0.5f, 1.0f};
  GLfloat diffuse[]  = {0.8f, 0.8f, 0.8f, 1.0f};
  GLfloat specular[] = {1.0f, 1.0f, 1.0f, 1.0f};
  glLightfv (GL_LIGHT0, GL_AMBIENT, ambient);
  glLightfv (GL_LIGHT0, GL_DIFFUSE, diffuse);
  glLightfv (GL_LIGHT0, GL_SPECULAR, specular);
  glLightf  (GL_LIGHT0, GL_CONSTANT_ATTENUATION, 1.0);
  glLightf  (GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.0);
  glLightf  (GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.0);

  GLfloat lightPos[]  = {2.0, 0.0, 0.0, 1.0f};
  GLfloat lightDir[]  = {-1.0, 0.0, 0.0, 1.0f};
  glLightfv (GL_LIGHT0, GL_POSITION, lightPos);
  glLightfv (GL_LIGHT0, GL_SPOT_DIRECTION, lightDir);
  glLightf  (GL_LIGHT0, GL_SPOT_CUTOFF, 180);
  glLightf  (GL_LIGHT0, GL_SPOT_EXPONENT, 1.0);
  glEnable  (GL_LIGHTING);
  glEnable  (GL_LIGHT0);

  return 0;
}



// haptic devices initialization
int
InitHaptics ()
{
  if (dhdOpen () >= 0) {
    printf ("%s device detected\n", dhdGetSystemName());

    // set device capabilities
    FingerCount = 1;
    HasRot      = dhdHasWrist ();
    HasGrip     = dhdHasGripper ();

    if (HasGrip) FingerCount = 2;
    else         FingerCount = 1;
  }

  else {
    printf ("no device detected\n");
    dhdSleep (2.0);
    exit (0);
  }

  printf ("\n");

  // register exit callback
  atexit (Close);

  return 0;
}



// simulation initialization
int
InitSimulation ()
{
  int i;

  glEnable (GL_DEPTH_TEST);
  glClearColor (0.0, 0.0, 0.0, 1.0);

  for (i=0; i<FingerCount; i++) FingerPosGlobal[i].setZero();

  SpherePosGlobal.setZero();

  return 0;
}



void
StartSimulation ()
{
  // create a high priority haptic thread
#if defined(WIN32) || defined(WIN64)
  DWORD ThreadId;
  CreateThread (NULL, 0, (LPTHREAD_START_ROUTINE)(HapticsLoop), NULL, NULL, &ThreadId);
  SetThreadPriority(&ThreadId, THREAD_PRIORITY_ABOVE_NORMAL);
#else
  pthread_t handle;
  pthread_create (&handle, NULL, HapticsLoop, NULL);
  struct sched_param sp;
  memset (&sp, 0, sizeof(struct sched_param));
  sp.sched_priority = 10;
  pthread_setschedparam (handle, SCHED_RR, &sp);
#endif

  // update display
  glutTimerFunc (30, UpdateDisplay, 0);

  // display instructions
  printf ("\n");
  printf ("commands:\n\n");
  printf ("   'r' to toggle display of haptic rate\n");
  printf ("   'q' to quit\n");
  printf ("\n\n");

  // start main graphic rendering loop
  glutMainLoop ();
}



int
main (int   argc,
      char *argv[])
{
  // message
  int major, minor, release, revision;
  dhdGetSDKVersion (&major, &minor, &release, &revision);
  cout << endl;
  cout << "Force Dimension - OpenGL Sphere Example " << major << "." << minor << "." << release << "." << revision << endl;
  cout << "(C) 2001-2015 Force Dimension" << endl;
  cout << "All Rights Reserved." << endl << endl;

  // initialize GLUT
  InitGLUT (argc, argv);

  // initialize haptic devices
  InitHaptics ();

  // initialize simulation objects
  InitSimulation ();

  // start simulation
  StartSimulation ();

  return 0;
}
