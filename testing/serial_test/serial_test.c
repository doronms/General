/*
 * main.c
 *
 *  Created on: Apr 14, 2016
 *      Author: shimon
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <stddef.h>
#include <sched.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/time.h>
#include <time.h>
#include <ctype.h>



#define NUM_UARTS		11
#define MAXBUF			120
#define MAX_CYCLES		1

int fd;
int pid;
int result = 0;
int prio_max, prio_min;
//static struct timeval tv1_start;
//static struct timeval tv1_end;
static timer_t timer1;
static int quit_flag = 0;
static unsigned int timers_reload[NUM_UARTS] = { 10, 10, 20, 50, 1 };		// milli seconds
static unsigned int timers_val[NUM_UARTS];
static unsigned int timers_timeouts[NUM_UARTS];
static unsigned int tickcntr = 0;
static unsigned int tx_buf_size[NUM_UARTS] = { 50, 50, 100, 150, 20 };
char txbuf[MAXBUF] = "send from ttyP2P uart ip and is very good for me and i what to kill me self";
char rdbuf [MAXBUF] = {0};
struct timeval tvw_start;
struct timeval tvw_end;
struct timeval tvr_start;
struct timeval tvr_end;
int bytes_written;
int bytes_read;



int set_interface_attribs (int fd, int speed, int parity)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
	   printf("error %s from tcgetattr", strerror(errno));
	   return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 1;            // read doesn't block
	tty.c_cc[VTIME] = 0;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
									// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		printf("error %s from tcsetattr", strerror(errno));
		return -1;
	}
	return 0;
}

void set_blocking (int fd, int should_block)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);

	if (tcgetattr (fd, &tty) != 0)
	{
			printf("error %s from tggetattr", strerror(errno));
			return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
		printf("error %s setting term attributes", strerror(errno));
}


int main(int argc, char* argv[])
{
	int i;
	int j;
	int id;
	char fdstr[80];
	unsigned int wr_time_usec;
	unsigned int rd_time_usec;
	int pid;
	int channel_no;
	char channel_str[80];
	int res;


	if (argc < 2)
	{
		printf("usage serial_test <channel number>\n");
		printf("<channel number> shall be between 0 and %d.\n", NUM_UARTS-1);
		exit (-1);
	}

	strcpy(channel_str, argv[1]);
	channel_no = atoi(channel_str);

	if ((channel_no < 0) || (channel_no > NUM_UARTS-1))
	{
		printf("Uart channel shall be between 0 to %d.\n", NUM_UARTS-1);
		exit (-1);
	}

	sprintf(fdstr, "/dev/ttyP2P%d", channel_no);
//	fd = open (fdstr, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	fd = open(fdstr, O_RDONLY | O_NOCTTY);
	if (fd < 0)
	{
		printf("error %d opening: %s\n", errno, strerror (errno));
		return -1;
	}

	printf("serial port initialized\n");
	
	close(fd);
	printf("serial port close\n");
	return 0; /*shemie */

	set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 0);                // set no blocking

//	set_sched_policry_and_priority();

	pid = getpid();
	
	while(1)
	{
		printf("tx buffer: %s\n", txbuf);
		bytes_written = write (fd, txbuf, sizeof(txbuf));
		printf("serial_test pid = %d, UART %d, %d bytes written\n", pid, channel_no, bytes_written);

		usleep(100);

		bytes_read = read (fd, rdbuf, sizeof(rdbuf));
		printf("serial_test pid = %d, UART %d, %d bytes read\n", pid, channel_no, bytes_read);

		if (bytes_read > 0)
		{
			printf("rxbuf : ", rdbuf);
			for (i=0; i<bytes_read; i++)
			{
				printf("%c", rdbuf[i]);
			}
			printf("\n");
		}
		usleep(5000);	
	//	sleep(1);
//	sleep (2);	
	}

	close(fd);
	return 0;
}



