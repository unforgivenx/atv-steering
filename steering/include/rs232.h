/*
***************************************************************************
*
* Author: Teunis van Beelen
*
* Copyright (C) 2005, 2006, 2007, 2008, 2009, 2010, 2011, 2012, 2013 Teunis van Beelen
*
* teuniz@gmail.com
*
***************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*
***************************************************************************
*
* This version of GPL is at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
*
***************************************************************************
*/

/* last revision: February 1, 2013 */

/* For more info and how to use this libray, visit: http://www.teuniz.net/RS-232/ */
/*

Comport Numbers
	Linux 	windows
0 	ttyS0 	COM1
1 	ttyS1 	COM2
2 	ttyS2 	COM3
3 	ttyS3 	COM4
4 	ttyS4 	COM5
5 	ttyS5 	COM6
6 	ttyS6 	COM7
7 	ttyS7 	COM8
8 	ttyS8 	COM9
9 	ttyS9 	COM10
10 	ttyS10 	COM11
11 	ttyS11 	COM12
12 	ttyS12 	COM13
13 	ttyS13 	COM14
14 	ttyS14 	COM15
15 	ttyS15 	COM16
16 	ttyUSB0 n.a.
17 	ttyUSB1 n.a.
18 	ttyUSB2 n.a.
19 	ttyUSB3 n.a.
20 	ttyUSB4 n.a.
21 	ttyUSB5 n.a.
22 	ttyAMA0 n.a.
23 	ttyAMA1 n.a.
24 	ttyACM0 n.a.
25 	ttyACM1 n.a.
26 	rfcomm0 n.a.
27 	rfcomm1 n.a.
28 	ircomm0 n.a.
29 	ircomm1 n.a.

Possible Baudrates
Linux 	windows
50 	n.a.
75 	n.a.
110 	110
134 	n.a.
150 	n.a.
200 	n.a.
300 	300
600 	600
1200 	1200
1800 	n.a.
2400 	2400
4800 	4800
9600 	9600
19200 	19200
38400 	38400
57600 	57600
115200 	115200
230400 	128000
460800 	256000
500000 	500000
576000 	n.a.
921600 	n.a.
1000000 	1000000

*/



#ifndef rs232_INCLUDED
#define rs232_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>



#ifdef __linux__

#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>

#else

#include <windows.h>

#endif

int RS232_OpenComport(int, int);
int RS232_PollComport(int, unsigned char *, int);
int RS232_SendByte(int, unsigned char);
int RS232_SendBuf(int, unsigned char *, int);
void RS232_CloseComport(int);
void RS232_cputs(int, const char *);
int RS232_IsCTSEnabled(int);
int RS232_IsDSREnabled(int);
void RS232_enableDTR(int);
void RS232_disableDTR(int);
void RS232_enableRTS(int);
void RS232_disableRTS(int);


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif


