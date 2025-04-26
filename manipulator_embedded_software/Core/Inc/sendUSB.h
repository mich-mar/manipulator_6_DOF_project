#ifndef USB_UTILS_H
#define USB_UTILS_H

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdint.h>

void sendUSBmsg(char *message);

#endif // USB_UTILS_H
