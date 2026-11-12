// Firmware for Seeed XIAO nRF52840 Sense
// Reads the onboard LSM6DS3 accelerometer and streams "x,y,z" (in g)
// over USB serial at ~100 Hz.
//
// This version also powers up the IMU explicitly and prints a short
// diagnostic (I2C scan) at startup, to help if "IMU init failed" appears.
//
// Arduino IDE setup:
//   1. File > Preferences > Additional Boards Manager URLs, add:
//      https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
//   2. Tools > Board > Boards Manager: install "Seeed nRF52 mbed-enabled Boards"
//   3. Tools > Board: select "Seeed XIAO BLE Sense - nRF52840"
//   4. Library Manager: install "Seeed Arduino LSM6DS3" (by Seeed Studio)
//   5. Upload this sketch to BOTH boards.

#include <LSM6DS3.h>
#include <Wire.h>

LSM6DS3 imu(I2C_MODE, 0x6A);  // onboard IMU address

const unsigned long SAMPLE_PERIOD_MS = 10;  // 100 Hz

void powerOnImu() {
  // On the XIAO Sense, the IMU has its own power-enable pin.
  // Some board-package versions leave it off, which makes init fail.
#ifdef PIN_LSM6DS3TR_C_POWER
  pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
  digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
  Serial.println("INFO IMU power pin enabled");
#else
  Serial.println("INFO no IMU power pin defined by this board package");
#endif
  delay(100);  // give the sensor time to start
}

void scanBus() {
  Serial.println("INFO scanning I2C bus (Wire)...");
  Wire.begin();
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("INFO   device at 0x");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) Serial.println("INFO   no devices found on Wire");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);  // wait until the NUC opens the port
  }

  powerOnImu();
  scanBus();

  if (imu.begin() != 0) {
    while (true) {
      Serial.println("ERR IMU init failed");
      Serial.println("ERR expected a device at 0x6A in the scan above.");
      Serial.println("ERR If none was found, check the board is the 'Sense' version.");
      delay(2000);
    }
  }
  Serial.println("OK XIAO accel ready");
}

void loop() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < SAMPLE_PERIOD_MS) return;
  last = now;

  Serial.print(imu.readFloatAccelX(), 4);
  Serial.print(',');
  Serial.print(imu.readFloatAccelY(), 4);
  Serial.print(',');
  Serial.println(imu.readFloatAccelZ(), 4);
}