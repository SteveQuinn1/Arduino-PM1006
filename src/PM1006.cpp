/*!
 * @file PM1006.cpp
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

#include "PM1006.h"
#include "Stream.h"

/*!
 *  @brief class constructor for using hardware or software serial
 *  @param stream is the serial instance to use. Stream can be either of
 *  type HardwareSerial or Software Serial. The
 */
PM1006::PM1006(Stream *stream) {
  _stream = stream;
}

PM1006::~PM1006() {
  if (_stream) {
    delete _stream;
  }
}

/*!
* @brief returns the last measured PM2.5 reading in ug/(m^3).
* @returns -1 if there has been no measurement taken or the value
* from the last successful measurement.
*/
int PM1006::getPM2_5() {
  return this->_lastPM2_5;
}


/*!
 * @brief takes a measurement for PM2.5, PM1.0, and PM10 concentrations
 * @return true if successful, false if otherwise
*/
bool PM1006::takeMeasurement() {
  uint8_t rxBuf[CMD_TAKE_MEASUREMENT_RESP_LEN];
  if(!this->sendCMD(CMD_TAKE_MEASUREMENT, CMD_TAKE_MEASUREMENT_LEN, rxBuf, CMD_TAKE_MEASUREMENT_RESP_LEN)) {
    return false;
  }

  if(!this->isValidMeasurementFrame(rxBuf)) {
    return false;
  }

  /*
    From page 7 of the data sheet response is in the following format:
    0x16, 0x11, 0x0B, DF1-DF4 DF5-DF8 DF9-DF12 DF13 DF14 DF15 DF16 [CS]
    PM2.5(ug/m3)= DF3*256 + DF4
  */
  this->_lastPM2_5 = (rxBuf[PM1006_DF3] << 8) | rxBuf[PM1006_DF4];

  return true;
}

/*!
 * @brief checks to see if the buffer paramter contains the correct start bytes
 * to signify a valid measurement frame.
 * @param buf is the data to be checked
*/
bool PM1006::isValidMeasurementFrame(uint8_t * buf) {
    return buf[0] == 0x16 && buf[1] == 0x11 && buf[2] == 0x0B;
}

/*!
 * @brief sends a command to the PM1006 sensor and reads the returned data. This command will fail
 * and timeout if the sensor does not
 * @param txBuf a buffer that contains the data to be sent to the sensor.
 * @param txLen the length of the txBuf buffer.
 * @param rxBuf a buffer that will be filled with data that is return
 * @param rxLen the length of the rxBuf buffer.
 * @return true on success and false otherwise.
*/
bool PM1006::sendCMD(const unsigned char * txBuf, uint8_t txLen, unsigned char * rxBuf, uint8_t rxLen) {

  // Clear the RX buffer
  while (this->_stream->available()) {
    this->_stream->read();
  }

  // Send the command to get the measurement
  this->_stream->write(txBuf, txLen);

  int i = 0;
  unsigned long start = millis();
  while (((millis() - start) < CMD_TIMEOUT) && i < rxLen) {
    while (this->_stream->available()) {
      rxBuf[i] = this->_stream->read();
      i++;
    }
    yield();
  }

  // // Failed because of timeout
  if (i < rxLen) {
    return false;
  }

  return true;
}
