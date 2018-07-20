//  (C) 2001-2015 Force Dimension
//  All Rights Reserved.
//
//  Version 3.6.0



#include <math.h>
#include <memory.h>
#include <stdlib.h>
#include <stdio.h>

#include "CMacrosGL.h"
using namespace Eigen;

#if defined(LINUX) || defined(MACOSX)
#include <pthread.h>
#endif

#include "dhdc.h"



// window display options
const int OPTION_FULLSCREEN     = 1;
const int OPTION_WINDOWDISPLAY  = 2;

// object stiffness
double Stiffness   = 2012;
double TorqueGain  =    1.0;

// set size of cube (half edge)
const float CubeSize = 0.06f;

// status flags
bool SimulationOn;
bool SimulationFinished;
int  Status[DHD_MAX_STATUS];
bool HapticsON = false;
bool TorquesON = true;
bool ForcesON  = true;

// text overlay globals
double LastTime;
double Freq;
char   Perf[50];
bool   ShowRate = true;

// white diffuse light
GLfloat LightAmbient[]  = {0.5f, 0.5f, 0.5f, 1.0f};
GLfloat LightDiffuse[]  = {0.8f, 0.8f, 0.8f, 1.0f};
GLfloat LightSpecular[] = {1.0f, 1.0f, 1.0f, 1.0f};

// light source position
GLfloat LightPosition[] = {1.0f, 0.5f, 0.8f, 0.0f};

// normals for the 6 faces of a cube
GLfloat N[6][3]   = { {-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0},
                      {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, -1.0} };

// vertex indices for the 6 faces of a cube
GLint Faces[6][4] = { {0, 1, 2, 3}, {3, 2, 6, 7}, {7, 6, 5, 4},
                      {4, 5, 1, 0}, {5, 6, 2, 1}, {7, 4, 0, 3} };

// will be filled in with X,Y,Z vertices
GLfloat V[8][3];



// graphics initialization
void
InitGraphics ()
{
  // compute size of half edge of cube
  static const float size = CubeSize / 2.0f;

  // setup cube vertex data
  V[0][0] = V[1][0] = V[2][0] = V[3][0] = -size;
  V[4][0] = V[5][0] = V[6][0] = V[7][0] =  size;
  V[0][1] = V[1][1] = V[4][1] = V[5][1] = -size;
  V[2][1] = V[3][1] = V[6][1] = V[7][1] =  size;
  V[0][2] = V[3][2] = V[4][2] = V[7][2] =  size;
  V[1][2] = V[2][2] = V[5][2] = V[6][2] = -size;

  // enable a single OpenGL light. 
  glLightfv (GL_LIGHT0, GL_AMBIENT,  LightAmbient);
  glLightfv (GL_LIGHT0, GL_DIFFUSE,  LightDiffuse);
  glLightfv (GL_LIGHT0, GL_SPECULAR, LightSpecular);
  glLightfv (GL_LIGHT0, GL_POSITION, LightPosition);
  glEnable  (GL_LIGHT0);
  glEnable  (GL_LIGHTING);

  // use depth buffering for hidden surface elimination.
  glEnable (GL_DEPTH_TEST);
}



// window resize callback
void
RezizeWindow (int w,
              int h)
{
  double glAspect = ((double)w / (double)h);

  glViewport     (0, 0, w, h);
  glMatrixMode   (GL_PROJECTION);
  glLoadIdentity ();
  gluPerspective (60, glAspect, 0.01, 10);

  gluLookAt      (0.2, 0.0, 0.0,
                  0.0, 0.0, 0.0,
                  0.0, 0.0, 1.0);

  glMatrixMode   (GL_MODELVIEW);
  glLoadIdentity ();
}



// scene rendering timer callback
void
UpdateDisplay (int val)
{
  if (SimulationOn) glutPostRedisplay ();

  glutTimerFunc (30, UpdateDisplay, 0);
}



// scene rendering callback
void
Draw (void)
{
  int            i;
  double         posX, posY, posZ;
  cMatrixGL      mat;
  GLUquadricObj *sphere;
  double         deviceRot[3][3];
  Vector3d       devicePos;

  // clean up
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // get info from device
  dhdGetPosition (&posX, &posY, &posZ);
  devicePos << posX, posY, posZ;
  dhdGetOrientationFrame (deviceRot);

  // render cube
  mat.set (devicePos, deviceRot);
  mat.glMatrixPushMultiply ();

  glEnable  (GL_COLOR_MATERIAL);
  glColor3f (0.1f, 0.3f, 0.5f);

  for (i=0; i<6; i++) {
    glBegin     (GL_QUADS);
    glNormal3fv (&N[i][0]);
    glVertex3fv (&V[Faces[i][0]][0]);
    glVertex3fv (&V[Faces[i][1]][0]);
    glVertex3fv (&V[Faces[i][2]][0]);
    glVertex3fv (&V[Faces[i][3]][0]);
    glEnd       ();
  }

  mat.glMatrixPop ();

  // render sphere at center of workspace
  glColor3f (0.8f, 0.8f, 0.8f);
  sphere = gluNewQuadric ();
  gluSphere (sphere, 0.005, 64, 64);

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
  if (err != GL_NO_ERROR) printf ("error: %s\n", gluErrorString(err));
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



// keyboard input callback
void
Key (unsigned char key,
     int           x,
     int           y)
{
  switch (key) {
  case 27:  
  case 'q':
    SimulationOn = false;
    while (!SimulationFinished) dhdSleep (0.01);
    exit(0);
  case 't':
    TorquesON = !TorquesON;
    break;
  case 'f':
    ForcesON = !ForcesON;
    if (ForcesON) dhdEnableForce (DHD_ON);
    else          dhdEnableForce (DHD_OFF);
    break;
  }
}



// right-click menu callback
void
SetOther (int value)
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



// collision model
void
ComputeInteractions (Vector3d& pos,
                     Matrix3d& rot,
                     Vector3d& force,
                     Vector3d& torque)
{
  static const float size = CubeSize / 2.0f;

  Matrix3d localRotTrans;
  Vector3d localPos;
  Vector3d localForce  (0, 0, 0);
  Vector3d localTorque (0, 0, 0);

  // compute position of device in locale coordinates of cube
  localRotTrans =   rot.transpose ();
  localPos      = - localRotTrans * pos;

  // compute interaction force and torque
  if ((localPos(0) < size) && (localPos(0) > -size) &&
      (localPos(1) < size) && (localPos(1) > -size) &&
      (localPos(2) < size) && (localPos(2) > -size))
  {
    double   depth = size;
    double   t_depth;
    Vector3d normal (0,0,1);
    Vector3d nForce;

    // check all size walls of cube
    t_depth = fabs( size - localPos(0)); if (t_depth < depth) { depth = t_depth; normal <<  1, 0, 0; }
    t_depth = fabs(-size - localPos(0)); if (t_depth < depth) { depth = t_depth; normal << -1, 0, 0; }
    t_depth = fabs( size - localPos(1)); if (t_depth < depth) { depth = t_depth; normal <<  0, 1, 0; }
    t_depth = fabs(-size - localPos(1)); if (t_depth < depth) { depth = t_depth; normal <<  0,-1, 0; }
    t_depth = fabs( size - localPos(2)); if (t_depth < depth) { depth = t_depth; normal <<  0, 0, 1; }
    t_depth = fabs(-size - localPos(2)); if (t_depth < depth) { depth = t_depth; normal <<  0, 0,-1; }

    // compute reaction force
    localForce  = -depth * Stiffness * normal;
    nForce      = - localForce;
    localTorque = -TorqueGain * localPos.cross (nForce);
  }

  // convert results in global coordinates
  force  = rot * localForce;
  torque = rot * localTorque;
}



// haptic thread
void*
HapticsLoop (void* pUserData)
{
  double   posX, posY, posZ;
  Vector3d deviceForce;
  Vector3d deviceTorque;
  Vector3d devicePos;
  double   r[3][3];
  Matrix3d deviceRot;
  bool     force;
  bool     btnDown;

  // start haptic simulation
  force              = false;
  btnDown            = false;
  SimulationOn       = true;
  SimulationFinished = false;

  // enable force
  dhdEnableForce (DHD_ON);

  // main haptic simulation loop
  while (SimulationOn) {

    // init variables
    posX = posY = posZ = 0.0;

    // read position of haptic device
    dhdGetPosition (&posX, &posY, &posZ);
    devicePos << posX,  posY,  posZ;

    // read orientation of haptic device
    dhdGetOrientationFrame (r);
    deviceRot << r[0][0], r[0][1], r[0][2],
                 r[1][0], r[1][1], r[1][2],
                 r[2][0], r[2][1], r[2][2];

    // compute forces and torques
    deviceForce.setZero ();
    deviceTorque.setZero ();
    ComputeInteractions (devicePos, deviceRot, deviceForce, deviceTorque);

    // only enable forces once the device if in free space
    if (!HapticsON) {
      if (deviceForce.norm() == 0) {
        HapticsON = true;
      }
      else {
        deviceForce.setZero();
        deviceTorque.setZero();
      }
    }

    // disable torques if required
    if (!TorquesON) deviceTorque.setZero();

    // send forces to device
    dhdSetForceAndTorqueAndGripperForce (deviceForce(0),  deviceForce(1),  deviceForce(2),
                                         deviceTorque(0), deviceTorque(1), deviceTorque(2),
                                         0.0);
  }

  // close connection with haptic device
  dhdClose ();

  // simulation is now exiting
  SimulationFinished = true;

  // return
  return NULL;
}



// entry point
int
main (int    argc,
      char **argv)
{
  // message
  int major, minor, release, revision;
  dhdGetSDKVersion (&major, &minor, &release, &revision);
  printf ("\n");
  printf ("Force Dimension - Torques Example %d.%d.%d.%d\n", major, minor, release, revision);
  printf ("(C) 2001-2015 Force Dimension\n");
  printf ("All Rights Reserved.\n\n");

  // open the first available device
  if (dhdOpen () < 0) {
    printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
    dhdSleep (2.0);
    return -1;
  }

  // identify device
  printf ("%s device detected\n\n", dhdGetSystemName());

  // register exit callback
  atexit (Close);

  // adjust stiffness for some devices
  switch (dhdGetSystemType ()) {
  case DHD_DEVICE_SIGMA331:
  case DHD_DEVICE_SIGMA331_LEFT:
  case DHD_DEVICE_SIGMA331S:
  case DHD_DEVICE_SIGMA331S_LEFT:
  case DHD_DEVICE_SIGMA33P:
  case DHD_DEVICE_SIGMA33P_LEFT:
    Stiffness    = 6000;
    TorqueGain  =     1.0;
    break;
  }

  // display instructions
  printf ("\n");
  printf ("commands:\n\n");
  printf ("   't' to enable or disable torques on wrist actuated devices\n");
  printf ("   'f' to enable or disable forces\n");
  printf ("   'q' to quit\n");
  printf ("\n\n");

  // initialization
  glutInit (&argc, argv);
  int WINDOW_SIZE_W = 512;
  int WINDOW_SIZE_H = 512;
  int screenW       = glutGet (GLUT_SCREEN_WIDTH);
  int screenH       = glutGet (GLUT_SCREEN_HEIGHT);
  int windowPosX    = (screenW - WINDOW_SIZE_W) / 2;
  int windowPosY    = (screenH - WINDOW_SIZE_H) / 2;

  // configure window
  glutInitWindowSize     (WINDOW_SIZE_W, WINDOW_SIZE_H);
  glutInitWindowPosition (windowPosX, windowPosY);
  glutInitDisplayMode    (GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
  glutCreateWindow       (argv[0]);
  glutDisplayFunc        (Draw);
  glutKeyboardFunc       (Key);
  glutReshapeFunc        (RezizeWindow);
  glutSetWindowTitle     ("Force Dimension - Torques Example");

  // configure right-click menu
  glutCreateMenu   (SetOther);
  glutAddMenuEntry ("Full Screen", OPTION_FULLSCREEN);
  glutAddMenuEntry ("Window Display", OPTION_WINDOWDISPLAY);
  glutAttachMenu   (GLUT_RIGHT_BUTTON);

  // initialize graphics scene
  InitGraphics ();

  // create a high priority haptic thread

#if defined(WIN32) || defined(WIN64)
  DWORD ThreadId;
  CreateThread (NULL, 0, (LPTHREAD_START_ROUTINE)(HapticsLoop), NULL, NULL, &ThreadId);
  SetThreadPriority(&ThreadId, THREAD_PRIORITY_ABOVE_NORMAL);
#endif

#if defined(LINUX) || defined(MACOSX)
  pthread_t          handle;
  pthread_create (&handle, NULL, HapticsLoop, NULL);
  struct sched_param sp;
  memset (&sp, 0, sizeof(struct sched_param));
  sp.sched_priority = 10;
  pthread_setschedparam (handle, SCHED_RR, &sp);
#endif

  // start in window mode
  glutReshapeWindow      (512, 512);
  glutInitWindowPosition (0, 0);

  // update display
  glutTimerFunc (30, UpdateDisplay, 0);

  // start main graphic rendering loop
  glutMainLoop ();

  return 0;
}

