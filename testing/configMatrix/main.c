/*
 * main.c
 *
 *  Created on: Jan 1, 2009
 *      Author: bsp
 */
#include <fcntl.h> /*for using read write permissions flags when openning the files*/
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include "public.h"


int main(void)
{
	int fd;
	AssignmentRequest request;
	int i=0, result = 0;
	request.enable = 1;
	request.gain = 1;
	request.input = 1;
	request.output = 8;
	fd = open("/dev/videoMatrix", O_RDWR);
/*
	for(i = 0; i < 16; i++)
	{
		request.output = i;
		result = ioctl(fd,IOCTL_SET_ASSIGNMENT, &request);
		if(result < 0)
		{
			printf("ioctl failed\n");
		}
		usleep(1000);
	}*/

		result = ioctl(fd,IOCTL_SET_ASSIGNMENT, &request);
		if(result < 0)
		{
			printf("ioctl failed\n");
		}
}

