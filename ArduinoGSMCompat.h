#ifndef _ARDUINO_GSM_COMPAT_H_
#define _ARDUINO_GSM_COMPAT_H_

//#define GSM_ERROR 	false
#define GSM_READY 	true

#define attachGPRS	gprsConnect

#define GPRS_READY	true
#define GPRS_KO		false

#define GSM_DEFAULT_STREAM	SerialUSB1

#define TINY_GSM_MODEM_U201

static const uint32_t rates[] = { 115200, 9600, 57600, 19200, 38400, 74400, 74880, 230400, 460800, 921600 };
static int i = 0;

void changeBaudRate() {
	GSM_DEFAULT_STREAM.begin(rates[i++ % (sizeof(rates)/sizeof(rates[0]))]);
}

#endif