/*
   1024x768.S: EDID data set for standard 1024x768 60 Hz monitor

   Copyright (C) 2011 Carsten Emde <C.Emde@osadl.org>

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   as published by the Free Software Foundation; either version 2
   of the License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA.
*/

/* EDID */
#define VERSION 1
#define REVISION 3

/* Display */
#define CLOCK 60503 /* kHz */
#define XPIX 1280
#define YPIX 768
#define XY_RATIO XY_RATIO_16_10
#define XBLANK 33
#define YBLANK 0
#define XOFFSET 9
#define XPULSE 8
#define YOFFSET (63+0)
#define YPULSE (63+3)
#define DPI 187
#define VFREQ 60 /* Hz */
#define TIMING_NAME "Linux XGA"
/* No ESTABLISHED_TIMINGx_BITS */
#define HSYNC_POL 1
#define VSYNC_POL 1
#define CRC 0xb9

#include "edid.S"
