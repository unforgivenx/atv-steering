
// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "math.h"
#include "rs232.h"
#include<string.h>

#define MAX_STRING_LEN 80

#define _nodeID 0x3F
#define _startOfFrame 0xA5

int cport_nr;
const double max_bus_volt = 88.3;
int lims_found;
int rl_lim, ph_lim; 

// CRC parameters

const int order = 16;
const unsigned long polynom = 0x1021;
const int direct = 1;
const unsigned long crcinit = 0x0000;
const unsigned long crcxor = 0x0;
const int refin = 0;
const int refout = 0;

// internal global values:

unsigned long crcmask;
unsigned long crchighbit;
unsigned long crcinit_direct;
unsigned long crcinit_nondirect;
unsigned long crctab[256];

// subroutines

unsigned long reflect (unsigned long crc, int bitnum) {

	// reflects the lower 'bitnum' bits of 'crc'

	unsigned long i, j=1, crcout=0;

	for (i=(unsigned long)1<<(bitnum-1); i; i>>=1) {
		if (crc & i) crcout|=j;
		j<<= 1;
	}
	return (crcout);
}



void generate_crc_table() {

	// make CRC lookup table used by table algorithms

	int i, j;
	unsigned long bit, crc;

	for (i=0; i<256; i++) {

		crc=(unsigned long)i;
		if (refin) crc=reflect(crc, 8);
		crc<<= order-8;

		for (j=0; j<8; j++) {

			bit = crc & crchighbit;
			crc<<= 1;
			if (bit) crc^= polynom;
		}			

		if (refin) crc = reflect(crc, order);
		crc&= crcmask;
		crctab[i]= crc;
	}
}


		
unsigned long crctablefast (unsigned char* p, unsigned long len) {

	// fast lookup table algorithm without augmented zero bytes, e.g. used in pkzip.
	// only usable with polynom orders of 8, 16, 24 or 32.

	unsigned long crc = crcinit_direct;

	if (refin) crc = reflect(crc, order);

	if (!refin) while (len--) crc = (crc << 8) ^ crctab[ ((crc >> (order-8)) & 0xff) ^ *p++];
	else while (len--) crc = (crc >> 8) ^ crctab[ (crc & 0xff) ^ *p++];

	if (refout^refin) crc = reflect(crc, order);
	crc^= crcxor;
	crc&= crcmask;

	return(crc);
}



unsigned long crctable (unsigned char* p, unsigned long len) {

	// normal lookup table algorithm with augmented zero bytes.
	// only usable with polynom orders of 8, 16, 24 or 32.

	unsigned long crc = crcinit_nondirect;

	if (refin) crc = reflect(crc, order);

	if (!refin) while (len--) crc = ((crc << 8) | *p++) ^ crctab[ (crc >> (order-8))  & 0xff];
	else while (len--) crc = ((crc >> 8) | (*p++ << (order-8))) ^ crctab[ crc & 0xff];

	if (!refin) while (++len < order/8) crc = (crc << 8) ^ crctab[ (crc >> (order-8))  & 0xff];
	else while (++len < order/8) crc = (crc >> 8) ^ crctab[crc & 0xff];

	if (refout^refin) crc = reflect(crc, order);
	crc^= crcxor;
	crc&= crcmask;

	return(crc);
}



unsigned long crcbitbybit(unsigned char* p, unsigned long len) {

	// bit by bit algorithm with augmented zero bytes.
	// does not use lookup table, suited for polynom orders between 1...32.

	unsigned long i, j, c, bit;
	unsigned long crc = crcinit_nondirect;

	for (i=0; i<len; i++) {

		c = (unsigned long)*p++;
		if (refin) c = reflect(c, 8);

		for (j=0x80; j; j>>=1) {

			bit = crc & crchighbit;
			crc<<= 1;
			if (c & j) crc|= 1;
			if (bit) crc^= polynom;
		}
	}	

	for (i=0; i<order; i++) {

		bit = crc & crchighbit;
		crc<<= 1;
		if (bit) crc^= polynom;
	}

	if (refout) crc=reflect(crc, order);
	crc^= crcxor;
	crc&= crcmask;

	return(crc);
}



unsigned long crcbitbybitfast(unsigned char* p, unsigned long len) {

	// fast bit by bit algorithm without augmented zero bytes.
	// does not use lookup table, suited for polynom orders between 1...32.

	unsigned long i, j, c, bit;
	unsigned long crc = crcinit_direct;

	for (i=0; i<len; i++) {

		c = (unsigned long)*p++;
		if (refin) c = reflect(c, 8);

		for (j=0x80; j; j>>=1) {

			bit = crc & crchighbit;
			crc<<= 1;
			if (c & j) bit^= crchighbit;
			if (bit) crc^= polynom;
		}
	}	

	if (refout) crc=reflect(crc, order);
	crc^= crcxor;
	crc&= crcmask;

	return(crc);
}


void send_command(unsigned char _cb, unsigned char _index, unsigned char _offset, unsigned char _data_words, long _datas)
{
	unsigned char header_sec[6] = {_startOfFrame, _nodeID, _cb, _index, _offset, _data_words};
	unsigned short header_crc = crctablefast((unsigned char *)header_sec, 6);
	unsigned char header_crc_bytes[2];

	header_crc_bytes[0] = (header_crc >> 8) & 0xFF;
	header_crc_bytes[1] = header_crc & 0xFF;

	unsigned char data_crc_bytes[2];
	unsigned char data_bytes[_data_words * 2];
	int i;

	//printf("CRC: %02X:%02X\r\n", header_crc_bytes[0],  header_crc_bytes[1]);

	for (i=1; i<=_data_words * 2; i++)
	{
		data_bytes[_data_words * 2 - i] = (_datas >> (8*(_data_words * 2-i))) & 0xFF;
	}

	/*for (i = 0; i < _data_words * 2; i++)
	{
	    if (i > 0) printf(":");
	    printf("%02X", data_bytes[i]);
	}
	printf("\n");*/

	unsigned short data_crc = crctablefast((unsigned char *)data_bytes, _data_words * 2);

	data_crc_bytes[0] = (data_crc >> 8) & 0xFF;
	data_crc_bytes[1] = data_crc & 0xFF;

	//printf("CRC: %02X:%02X\r\n", data_crc_bytes[0],  data_crc_bytes[1]);

	if (_cb == 2)
	{	
		unsigned char complete_msg[10+_data_words * 2];
		complete_msg[0] = _startOfFrame;
		complete_msg[1] = _nodeID;
		complete_msg[2] = _cb;
		complete_msg[3] = _index;
		complete_msg[4] = _offset;
		complete_msg[5] = _data_words;
		complete_msg[6] = header_crc_bytes[0];
		complete_msg[7] = header_crc_bytes[1];

		for (i=0; i<_data_words * 2; i++)
		{
		complete_msg[i + 8] = data_bytes[i];
		}
		complete_msg[_data_words * 2 + 8] = data_crc_bytes[0];
		complete_msg[_data_words * 2 + 9] = data_crc_bytes[1];
		for (i = 0; i < sizeof(complete_msg); i++)
			{
			    if (i > 0) printf(":");
			    printf("%02X", complete_msg[i]);
			}
		printf("\n");

		RS232_SendBuf(cport_nr, complete_msg, sizeof(complete_msg));

	}
	else
	{
		unsigned char complete_msg[8];
		complete_msg[0] = _startOfFrame;
		complete_msg[1] = _nodeID;
		complete_msg[2] = _cb;
		complete_msg[3] = _index;
		complete_msg[4] = _offset;
		complete_msg[5] = _data_words;
		complete_msg[6] = header_crc_bytes[0];
		complete_msg[7] = header_crc_bytes[1];
		for (i = 0; i < sizeof(complete_msg); i++)
			{
			    if (i > 0) printf(":");
			    printf("%02X", complete_msg[i]);
			}
			printf("\n");
		
		RS232_SendBuf(cport_nr, complete_msg, sizeof(complete_msg));
	}
	ROS_INFO("Command Sent!");
	
}

int read_reply()
{
	unsigned char buf[4096];
	int i;
	usleep(100000);
	int n = RS232_PollComport(cport_nr, buf, 4095);

	/*if(n > 0)
	{
		buf[n] = 0;   // always put a "null" at the end of a string!

		for(i=0; i < n; i++)
		{
			if(buf[i] < 32)  // replace unreadable control-codes by dots
			{
			  buf[i] = '.';
			}
		}
		printf("received %i bytes: %x\n", n, (unsigned int)buf[1]);
	}*/
	printf("received %i bytes: %x\n", n, (unsigned int)buf[1]);
	for (i = 0; i < n; i++)
	{
	    if (i > 0) printf(":");
	    printf("%02X", buf[i]);
	}
	printf("\n");

	if (buf[0] != _startOfFrame && buf[1] != 0xFF)
	return -1;

	int data_words = buf[5];
	//printf("data_words: %d\n",data_words);
	if (buf[2] == 2) // message contains data
	{
		int i;
		int data=0;
		for (i=0; i<data_words*2; i++)
		{
			data = data + (buf[8+i] << 8*i);
		}
		return data;
	}
	else if (buf[2] == 0) // message contains no data
	{
		if (buf[3] == 6) return -2;
		if (buf[3] != 1) return -1;
		
		if (data_words == 0) return 1; //acknowledge
		
		return -1;
	}
	return 0;
}

int enable_bridge()
{
        send_command(2, 1, 0, 1, 0);
	int reply = read_reply();
	return reply;
}

int write_access()
{
        send_command(2, 7, 0, 1, 0xF);
	int reply = read_reply();
	return reply;
}

int disable_bridge()
{
        send_command(2, 1, 0, 1, 1);
	int reply = read_reply();
	return reply;
}

int set_current(double current)
{
	current = (current * pow(2, 15) / (20.0));
        send_command(2, 0x45, 0, 2, (int)current);
	int reply = read_reply();	
	return reply;
}

double get_current()
{
        send_command(1, 0x10, 3, 1, 0);
	int reply = read_reply();
	double replyf = reply * 20.0 / (double)pow(2,13);
	return replyf;
}

double get_bus_voltage()
{
        send_command(1, 15, 0, 1, 0);
	int reply = read_reply();
	printf("Reply: %d\n",reply);
	double replyf = (double)reply * 1.05 * max_bus_volt / (double)pow(2,14);
	return replyf;
}

double get_dc_bus_ov()
{
        send_command(1, 216, 83, 1, 0);
	int reply = read_reply();
	double replyf = reply / 10.0;
	return replyf;
}


double read_digital()
{
        send_command(1, 35, 0, 1, 0);
	int reply = read_reply();
	return reply;
}

double read_digital_pin(int pin)
{
        send_command(1, 35, 0, 1, 0);
	int reply = read_reply();
	int pin_val = (reply & ( 1 << pin )) >> pin;
	return pin_val;
}

double read_analog(int pin)
{
        send_command(1, 26, pin-1, 1, 0);
	int reply = read_reply();
	double replyf = (double)reply * 20 / (double)pow(2,14);
	return replyf;
}

double read_encoder()
{
        send_command(1, 14, 0, 2, 0);
	int reply = read_reply();
	return reply;
}

double read_position()
{
        send_command(1, 18, 0, 2, 0);
	int reply = read_reply();
	return reply;
}

int set_position(double pos)
{
        send_command(2, 0x45, 0, 2, pos);
	int reply = read_reply();	
	return reply;
}

int set_steering(double perc)
{
	if (lims_found == 1)
	{
	int br_val = (ph_lim-rl_lim)*perc/100.0 + rl_lim;
        set_position(br_val);
	return 1;
	}
	else return 0;
}


int find_limits()
{

int ph_found, rl_found = 0;

int br_rl = -1, br_ph = -1;
int pre_pos = read_position();
ROS_INFO("Initial encoder: %d", pre_pos);
	while (lims_found != 1)
	{
		if (ph_found != 1)
		{
			if (read_digital_pin(0) == 1)
			{
				ph_found = 1;
				ph_lim = read_position();
				set_position(ph_lim);
				ROS_INFO("PH found as %d", ph_lim);
			} else {
				set_position(pre_pos-1000000);
				ROS_INFO("PH commanded to %d", pre_pos-1000000);
			}
		}
		else if (rl_found != 1)
		{
			if (read_digital_pin(1) == 1)
			{
				rl_found = 1;
				rl_lim = read_position();
				set_position((rl_lim+ph_lim)/2);
				lims_found = 1;
				ROS_INFO("RL found as %d", rl_lim);
			} else {
				set_position(pre_pos+1000000);
				ROS_INFO("RL commanded to %d", pre_pos+1000000);
			}
		}
	}


return 0;
}


int fl=0;
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  if (fl ==0) {if (disable_bridge() == 1) {fl=1;printf("Acknowledged! \r\n");}}
  if (fl ==1) {if (enable_bridge() == 1) {fl=0;printf("Acknowledged! \r\n");}}
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void steeringCallback(const std_msgs::Int16::ConstPtr& msg)
{
  set_steering(msg->data);
  ROS_INFO("I heard perc: [%d]", msg->data);
}

int main(int argc, char **argv)
{

int i;
	cport_nr=2;  
      
	/* 
	0 : /dev/ttyO0
	1 : /dev/ttyO1
	2 : /dev/ttyO2
	3 : /dev/ttyO3
	4 : /dev/ttyO4
	5 : /dev/ttyO5
	*/

	int bdrate=115200;       /* 9600 baud */

  	unsigned char buf[4096];
	unsigned long bit, crc;


	// at first, compute constant bit masks for whole CRC and CRC high bit

	crcmask = ((((unsigned long)1<<(order-1))-1)<<1)|1;
	crchighbit = (unsigned long)1<<(order-1);
	
	// generate lookup table

	generate_crc_table();


	// compute missing initial CRC value

	if (!direct) {

		crcinit_nondirect = crcinit;
		crc = crcinit;
		for (i=0; i<order; i++) {

			bit = crc & crchighbit;
			crc<<= 1;
			if (bit) crc^= polynom;
		}
		crc&= crcmask;
		crcinit_direct = crc;
	}

	else {

		crcinit_direct = crcinit;
		crc = crcinit;
		for (i=0; i<order; i++) {

			bit = crc & 1;
			if (bit) crc^= polynom;
			crc >>= 1;
			if (bit) crc|= crchighbit;
		}	
		crcinit_nondirect = crc;
	}


  if(RS232_OpenComport(cport_nr, bdrate))
  {
    printf("Can not open comport\n");

    return(0);
  }

int ch, n2;
double n3;

  ros::init(argc, argv, "steering");

  ros::NodeHandle n;

// %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n.subscribe("steering", 1, chatterCallback);
  ros::Subscriber sub2 = n.subscribe("steering_perc", 1, steeringCallback);
// %EndTag(SUBSCRIBER)%
write_access();	
enable_bridge();
find_limits();	
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%

