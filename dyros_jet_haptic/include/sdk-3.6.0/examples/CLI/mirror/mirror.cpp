//  (C) 2001-2015 Force Dimension
//  All Rights Reserved.
//
//  Version 3.6.0



#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "drdc.h"

#define DEFAULT_K    500.0
#define MIN_SCALE      0.2
#define MAX_SCALE      5.0

#define MIN(a,b) ((a)<(b))?(a):(b)
#define MAX(a,b) ((a)>(b))?(a):(b)



int
main (int  argc,
      char **argv)
{
  double mx, my, mz;
  double sp[DHD_MAX_DOF];
  double fx, fy, fz;
  double time;
  double refTime = dhdGetTime ();
  double K       = DEFAULT_K;
  double scale   = 1.0;
  int    done    = 0;
  int    master, slave;


  // message
  int major, minor, release, revision;
  dhdGetSDKVersion (&major, &minor, &release, &revision);
  printf ("Force Dimension - Master Slave Example %d.%d.%d.%d\n", major, minor, release, revision);
  printf ("(C) 2001-2015 Force Dimension\n");
  printf ("All Rights Reserved.\n\n");

  // open and initialize 2 devices
  for (int dev=0; dev<2; dev++) {

    // open device
    if (drdOpenID (dev) < 0) {
      printf ("error: not enough devices found\n");
      dhdSleep (2.0);
      for (int j=0; j<=dev; j++) drdClose (j);
      return -1;
    }

    // exclude some device types that have not been fully tested with 'mirror'
    bool incompatible = false;
    switch (dhdGetSystemType ()) {
    case DHD_DEVICE_SIGMA331:
    case DHD_DEVICE_SIGMA331_LEFT:
      incompatible = true;
      break;
    }

    // check that device is supported
    if (incompatible || !drdIsSupported()) {
      printf ("error: unsupported device (%s)\n", dhdGetSystemName(dev));
      dhdSleep (2.0);
      for (int j=0; j<=dev; j++) drdClose (j);
      return -1;
    }

    // initialize Falcon by hand if necessary
    if (!drdIsInitialized() && dhdGetSystemType() == DHD_DEVICE_FALCON) {
      printf ("please initialize Falcon device...\r"); fflush(stdout);
      while (!drdIsInitialized()) dhdSetForce (0.0, 0.0, 0.0);
      printf ("                                  \r");
      dhdSleep (0.5);
    }

    // initialize if necessary
    if (!drdIsInitialized (dev) && (drdAutoInit (dev) < 0)) {
      printf ("error: initialization failed (%s)\n", dhdErrorGetLastStr ());
      dhdSleep (2.0);
      for (int j=0; j<=dev; j++) drdClose (j);
      return -1;
    }

    // start robot control loop
    if (drdStart (dev) < 0) {
      printf ("error: control loop failed to start properly (%s)\n", dhdErrorGetLastStr ());
      dhdSleep (2.0);
      for (int j=0; j<=dev; j++) drdClose (j);
      return -1;
    }
  }

  // default role assignment
  master = 0;
  slave  = 1;

  // prefer Falcon as master 
  if (dhdGetSystemType (0) != DHD_DEVICE_FALCON && dhdGetSystemType (1) == DHD_DEVICE_FALCON) {
    master = 1;
    slave  = 0;
  }

  // give preference to omega.3 as slave
  if (dhdGetSystemType (0) == DHD_DEVICE_OMEGA3 && dhdGetSystemType (1) != DHD_DEVICE_OMEGA3) {
    master = 1;
    slave  = 0;
  }

  // if a device is virtual, make it the master
  if (dhdGetComMode (1) == DHD_COM_MODE_VIRTUAL) {
    master = 1;
    slave  = 0;
  }

  ushort mastersn, slavesn;
  dhdGetSerialNumber (&mastersn, master);
  dhdGetSerialNumber (&slavesn, slave);
  printf ("%s haptic device [sn: %04d] as master\n", dhdGetSystemName(master), mastersn);
  printf ("%s haptic device [sn: %04d] as slave\n", dhdGetSystemName(slave), slavesn);

  // display instructions
  printf ("\n");
  printf ("press 'd' to decrease scaling factor\n");
  printf ("      'u' to increase scaling factor\n");
  printf ("      ',' to decrease virtual stiffness\n");
  printf ("      '.' to increase virtual stiffness\n");
  printf ("      'q' to quit\n\n");

  // center both devices
  drdMoveToPos (0.0, 0.0, 0.0, false, master);
  drdMoveToPos (0.0, 0.0, 0.0, true,  slave);
  while (drdIsMoving (master) || drdIsMoving (slave)) drdWaitForTick (master);

  // stop regulation on master, stop motion filters on slave
  drdStop (true, master);
  dhdSetForce (0.0, 0.0, 0.0, master);
  drdEnableFilter (false, slave);

  // master slave loop
  while (!done) {

    // send the slave to master pos
    dhdGetPosition (&mx, &my, &mz, master);
    drdTrackPos    (scale*mx, scale*my, scale*mz, slave);

    // compute force and render on master
    drdGetPositionAndOrientation (sp, NULL, slave);
    fx = -K * (mx-sp[0]/scale); fy = -K * (my-sp[1]/scale); fz = -K * (mz-sp[2]/scale);
    dhdSetForce (fx, fy, fz, master);

    // print stats and check for exit condition
    time = dhdGetTime ();
    if (time-refTime > 0.04) {
      printf ("scale = %0.03f | K = %04d | master %0.02f kHz | slave %0.02f kHz            \r", scale, (int)K, dhdGetComFreq (master), drdGetCtrlFreq (slave));
      refTime = time;
      if (!drdIsRunning (slave)) done = -1;
      if (dhdKbHit ()){
        switch (dhdKbGet ()) {
        case 'q': done = 1;   break;
        case ',': K -= 0.1*K; break;
        case '.': K += 0.1*K; break;
        case 'd': scale = MIN(MAX_SCALE, scale-0.005*scale); break;
        case 'u': scale = MAX(MIN_SCALE, scale+0.005*scale); break;
        }
      }
    }
  }

  // report exit cause
  printf ("                                                                           \r");
  if (done == -1) printf ("\nregulation finished abnormally on slave device\n");
  else            printf ("\nexiting on user request\n");

  // close the connection
  drdClose (slave);
  drdClose (master);

  // exit
  printf ("\ndone.\n");
  return 0;
}
