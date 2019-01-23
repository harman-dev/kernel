/*
 * Public include for Harman faceplate driver
 *
 */

#ifndef _FACEPLATE_H
#define _FACEPLATE_H

#define BIT_ROTARY_VOLUME           0x00000001
#define BIT_SLIDER                  0x00000002
#define BIT_TUNER                   0x00000004
#define BIT_POWER                   0x00000008
#define BIT_PROXIMITY               0x00000010
#define BIT_FACEPLATE_KEYS          0x00000020

#define WAIT_FOR_POWER_KEY_EVENT     _IOR('q', 1, unsigned char)
#define DISABLE_FACEPLATE_FUNCTION   _IOW('q', 2, unsigned int )
#define ENABLE_FACEPLATE_FUNCTION    _IOW('q', 3, unsigned int )
#define GET_POWER_BUTTON_VALUE       _IOR('q', 4, unsigned char)


#endif /* _FACEPLATE_H */
