//! This file contains C code from rtkcmn.c, part of
//! RTKLIB: An Open Source Program Package for GNSS Positioning

/*
The RTKLIB software package is distributed under the following BSD 2-clause license
and additional exclusive clauses. Users are permitted to develop, produce or sell their
own non-commercial or commercial products utilizing, linking or including RTKLIB as
long as they comply with the license.

------------------------------------------------------------------------------------------

Copyright (c) 2007-2013, T. Takasu, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

 -  Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
 -  Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
 -  The software package includes some companion executive binaries or shared
    libraries necessary to execute APs on Windows.
    These licenses succeed to the original ones of these software.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <time.h>
#include <stdint.h>

namespace ze {

typedef struct {      /* time struct */
  time_t time;        /* time (s) expressed by standard time_t */
  double sec;         /* fraction of second under 1 s */
} gtime_t;

/* convert calendar day/time to time -------------------------------------------
* convert calendar day/time to gtime_t struct
* args   : double *ep       I   day/time {year,month,day,hour,min,sec}
* return : gtime_t struct
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
gtime_t epoch2time(const double *ep);

/* add time --------------------------------------------------------------------
* add time to gtime_t struct
* args   : gtime_t t        I   gtime_t struct
*          double sec       I   time to add (s)
* return : gtime_t struct (t+sec)
*-----------------------------------------------------------------------------*/
gtime_t timeadd(gtime_t t, double sec);

/* time difference -------------------------------------------------------------
* difference between gtime_t structs
* args   : gtime_t t1,t2    I   gtime_t structs
* return : time difference (t1-t2) (s)
*-----------------------------------------------------------------------------*/
double timediff(gtime_t t1, gtime_t t2);

/* gps time to time ------------------------------------------------------------
* convert week and tow in gps time to gtime_t struct
* args   : int    week      I   week number in gps time
*          double sec       I   time of week in gps time (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
gtime_t gpst2time(int week, double sec);

/* gpstime to utc --------------------------------------------------------------
* convert gpstime to utc considering leap seconds
* args   : gtime_t t        I   time expressed in gpstime
* return : time expressed in utc
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
gtime_t gpst2utc(gtime_t t);

/* gpstime to utc --------------------------------------------------------------
* convert gpstime to utc considering leap seconds
* args   : gps week, gps seconds
* return : time expressed in utc, in nanoseconds
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
int64_t gps2UtcNSecs(int gps_week, double gps_secs);

} // ze namespace
