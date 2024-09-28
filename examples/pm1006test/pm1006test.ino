/**
 * PM1006 Driver Test
 * Author: Original by Kevin Lutzer, modified by Steve Quinn for PM1006
*/

#include <PM1006.h>
#include <HardwareSerial.h>

#define PM1006_RX_PIN 4
#define PM1006_TX_PIN 5
#define PM1006_FAN_PIN 10

#define SAMPLE_RATE 2000 // ms

PM1006 * pm1006;

void setup() {
    // Setup and turn on fan
    pinMode(PM1006_FAN_PIN, OUTPUT);
    digitalWrite(PM1006_FAN_PIN, HIGH);

    // Setup the serial logger
    Serial.begin(115200);

    // Setup and create instance of the PM1006 driver
    // The baud rate for the serial connection must be PM1006::BAUD_RATE.
    Serial1.begin(PM1006::BAUD_RATE, SERIAL_8N1, PM1006_RX_PIN, PM1006_TX_PIN);
    pm1006 = new PM1006(&Serial1);
}

void loop() {
    if(!pm1006->takeMeasurement()) {
        Serial.println("Failed to take measurement");
    } else {
        Serial.print("PM2.5 = ");
        Serial.print(pm1006->getPM2_5());
        Serial.println(" ug/m3");

    }
    
    Serial.println();
    
    delay(SAMPLE_RATE);
}