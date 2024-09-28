/*!
 * @file PM1006.h
 *
 * @mainpage Arduino PM1006 library for PM1006 sensors
 *
 * @section intro_sec Introduction
 *
 * Driver for the PM1006 PM2.5 sensor as used in the IKEA VINDRIKTNING.
 *
 * Communication with this sensor is via serial UART and operates at 5 Volt Logic Level
 * so ensure you use logic level conversion for 3v3 devices.
 * It provides inhalable particulate matter measurements for 2.5um
 * size particles particles.
 *
 * NOTE : This library uses a blocking call to read the serial uart buffer. The maxium wait time
 *        is goverened by the value of CMD_TIMEOUT and is set to 1000mS max
 *
 * Documentation for the Cubic sensor can be found on Adafruit's website:
 * https://cdn-learn.adafruit.com/assets/assets/000/122/217/original/PM1006_LED_PARTICLE_SENSOR_MODULE_SPECIFICATIONS-1.pdf?1688148991
 *
 * @section author Author
 *
 * Written by Kevin Lutzer, modified by Steve Quinn
 *
 * @section license License
 *
 * Apache license.
 * See the LICENSE file for details.
 *
 */
#ifndef __PM1006_H__
#define __PM1006_H__

#include "Stream.h"
#include "Arduino.h"

#define PM1006_DF1  ((uint8_t) 3)
#define PM1006_DF2  ((uint8_t) 4)
#define PM1006_DF3  ((uint8_t) 5)
#define PM1006_DF4  ((uint8_t) 6)
#define PM1006_DF5  ((uint8_t) 7)
#define PM1006_DF6  ((uint8_t) 8)
#define PM1006_DF7  ((uint8_t) 9)
#define PM1006_DF8  ((uint8_t) 10)
#define PM1006_DF9  ((uint8_t) 11)
#define PM1006_DF10 ((uint8_t) 12)
#define PM1006_DF11 ((uint8_t) 13)
#define PM1006_DF12 ((uint8_t) 14)
#define PM1006_DF13 ((uint8_t) 15)
#define PM1006_DF14 ((uint8_t) 16)
#define PM1006_DF15 ((uint8_t) 17)


class PM1006 {

public:

  static const int BAUD_RATE = 9600;
  const int CMD_TIMEOUT = 1000;

  const uint8_t CMD_TAKE_MEASUREMENT_LEN = 5;
  const unsigned char CMD_TAKE_MEASUREMENT[5] = {0x11, 0x02, 0x0B, 0x01, 0xE1};

  // From the datasheet there is only one command that is avaliable to take the
  // appropriate readings. The response is 20 bytes long including the header and checksum
  const uint8_t CMD_TAKE_MEASUREMENT_RESP_LEN = 20;

  PM1006(Stream *stream);
  ~PM1006();

  bool takeMeasurement();

  int getPM2_5();

private:

  Stream *_stream = NULL;
  bool sendCMD(const unsigned char * txBuf, uint8_t txLen, unsigned char * rxBuf, uint8_t rxLen);
  bool isValidMeasurementFrame(uint8_t * buf);

  int _lastPM2_5 = -1;
};

#endif
