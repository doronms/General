// alt_timer_ioctl.h
#ifndef _ALT_TIMER_IOCTL_H_
#define _ALT_TIMER_IOCTL_H_

#define ALT_TIMER_MAGIC 'T'
#define WAIT_FOR_INT1PPS _IOR(ALT_TIMER_MAGIC, 1, int)
#define WAIT_FOR_INT1PPM _IOR(ALT_TIMER_MAGIC, 2, int)

#endif
