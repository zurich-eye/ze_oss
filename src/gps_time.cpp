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

#include <math.h>
#include <ze/common/time.h>
#include <ze/common/gps_time.h>

namespace ze {

constexpr double gpst0[]={1980,1, 6,0,0,0}; /* gps time reference */

constexpr double leaps[][7]={ /* leap seconds {y,m,d,h,m,s,utc-gpst,...} */
                              {2015,1,1,0,0,0,-17},
                              {2012,1,1,0,0,0,-16},
                              {2009,1,1,0,0,0,-15},
                              {2006,1,1,0,0,0,-14},
                              {1999,1,1,0,0,0,-13},
                              {1997,7,1,0,0,0,-12},
                              {1996,1,1,0,0,0,-11},
                              {1994,7,1,0,0,0,-10},
                              {1993,7,1,0,0,0, -9},
                              {1992,7,1,0,0,0, -8},
                              {1991,1,1,0,0,0, -7},
                              {1990,1,1,0,0,0, -6},
                              {1988,1,1,0,0,0, -5},
                              {1985,7,1,0,0,0, -4},
                              {1983,7,1,0,0,0, -3},
                              {1982,7,1,0,0,0, -2},
                              {1981,7,1,0,0,0, -1}
                            };

gtime_t epoch2time(const double *ep)
{
  const int doy[]={1,32,60,91,121,152,182,213,244,274,305,335};
  gtime_t time={0};
  int days,sec,year=(int)ep[0],mon=(int)ep[1],day=(int)ep[2];

  if (year<1970||2099<year||mon<1||12<mon) return time;

  /* leap year if year%4==0 in 1901-2099 */
  days=(year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+(year%4==0&&mon>=3?1:0);
  sec=(int)floor(ep[5]);
  time.time=(time_t)days*86400+(int)ep[3]*3600+(int)ep[4]*60+sec;
  time.sec=ep[5]-sec;
  return time;
}

gtime_t timeadd(gtime_t t, double sec)
{
  double tt;

  t.sec+=sec; tt=floor(t.sec); t.time+=(int)tt; t.sec-=tt;
  return t;
}

double timediff(gtime_t t1, gtime_t t2)
{
  return difftime(t1.time,t2.time)+t1.sec-t2.sec;
}

gtime_t gpst2time(int week, double sec)
{
  gtime_t t=epoch2time(gpst0);

  if (sec<-1E9||1E9<sec) sec=0.0;
  t.time+=86400*7*week+(int)sec;
  t.sec=sec-(int)sec;
  return t;
}

gtime_t gpst2utc(gtime_t t)
{
  gtime_t tu;
  size_t i;

  for (i=0;i<sizeof(leaps)/sizeof(*leaps);i++) {
    tu=timeadd(t,leaps[i][6]);
    if (timediff(tu,epoch2time(leaps[i]))>=0.0) return tu;
  }
  return t;
}

int64_t gps2UtcNSecs(int gps_week, double gps_secs)
{
  gtime_t unix_time = gpst2utc(
        gpst2time(gps_week, gps_secs));
  return static_cast<int64_t>(
        ze::secToNanosec(
          unix_time.sec+static_cast<double>(unix_time.time)));
}

} // ze namespace
