#include "sendUSB.h"

void sendUSBmsg(char *message)
{
    CDC_Transmit_FS((uint8_t *)message, strlen(message));
}