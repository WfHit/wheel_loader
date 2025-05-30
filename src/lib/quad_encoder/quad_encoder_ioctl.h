#pragma once

#include <nuttx/sensors/ioctl.h>

/* Custom IOCTL commands for quadrature encoder */
#define QEIOC_SETMODE        _QEIOC(0x0010)  /* Set counting mode */
#define QEIOC_GETVELOCITY    _QEIOC(0x0011)  /* Get velocity */
#define QEIOC_SETRESOLUTION  _QEIOC(0x0012)  /* Set encoder resolution */
#define QEIOC_GETRESOLUTION  _QEIOC(0x0013)  /* Get encoder resolution */
#define QEIOC_SETFILTER      _QEIOC(0x0014)  /* Set filter level */
#define QEIOC_ENABLEINDEX    _QEIOC(0x0015)  /* Enable index pulse */
#define QEIOC_DISABLEINDEX   _QEIOC(0x0016)  /* Disable index pulse */
#define QEIOC_GETANGLE       _QEIOC(0x0017)  /* Get angle in degrees */
#define QEIOC_GETRPM         _QEIOC(0x0018)  /* Get RPM */
