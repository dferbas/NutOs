#ifndef _STM32_OTG_H_
#define _STM32_OTG_H_

#include <sys/device.h>

#define IOCTL_ADD_IFACE_DESC    1
#define IOCTL_DEL_IFACE_DESC    2
#define IOCTL_ADD_STR_DESC  3
#define IOCTL_DEL_STR_DESC  4
#define IOCTL_USB_DEV_RESET 5

#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05

typedef void (* EP_CALLBACK)(void*);

extern NUTDEVICE devStm32Otg;

#endif
