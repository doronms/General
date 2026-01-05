//
// public.h for pcidrv driver
//

#pragma once

#define VIDEO_MATRIX_DIMENTION	16	



#pragma pack(push, 1)

// Input buffer for IOCTL_SET_ASSIGNMENT
typedef struct _AssignmentRequest
{
	unsigned long input;
	unsigned long output;
	unsigned long gain;
	unsigned long enable;
} AssignmentRequest;



#define	IOCTL_SET_ASSIGNMENT					0x000A
#define IOCTL_GET_ADD_RECOGNITION				0x000B

// Output buffer for IOCTL_WRITE_REG
// There is no data in response to IOCTL_WRITE_REG

/**************  ERRORS  ******************/
#define ERROR_SUCCESS 			0
#define ERROR_ID_NOT_READY		-2
#define ERROR_ID_WRONG_INPUT 		-3

#pragma pack(pop)
