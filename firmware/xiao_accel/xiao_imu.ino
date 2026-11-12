// IMU firmware for Seeed XIAO nRF52840 Sense
// (replaces xiao_accel.ino and xiao_accel_ble.ino)
//
// What it does:
//   * Samples the onboard LSM6DS3 accelerometer AND gyroscope at 104 Hz,
//     each sample stamped with the board's clock in milliseconds.
//   * Measures the gyro's resting offset at power-up (keep it still!).
//   * Keeps every sample in a memory buffer until it has been sent.
//     While nothing is connected (e.g. underwater) the buffer fills up;
//     when a connection comes back, the backlog is sent first, then live
//     data continues. If the buffer fills, the OLDEST samples are dropped,
//     so you always keep the most recent ~40 seconds.
//   * Sends over Bluetooth (to ble_bridge.py) and/or USB serial.
//
// USB serial line format (115200 baud):
//   t_ms,ax,ay,az,gx,gy,gz,backlog
//   accel in g, gyro in degrees/second, backlog = samples still waiting
//
// LEDs: red = calibrating (keep still), blue = Bluetooth connected.
//
// BEFORE UPLOADING:
//   * Libraries: "Seeed Arduino LSM6DS3" and "ArduinoBLE"
//   * Board: Seeed nRF52 mbed-enabled Boards > Seeed XIAO BLE Sense - nRF52840
//   * Set BOARD_ID: 1 for the first board, 2 for the second.

#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <Wire.h>

#define BOARD_ID 1          // <-- change to 2 for the second board

#define SAMPLE_HZ 104       // matches one of the IMU's native rates
#define BUFFER_SAMPLES 4096 // 16 bytes each = 64 KB = ~39 s at 104 Hz
#define MAX_SEND_PER_LOOP 4 // how fast the backlog is sent out

#define IMU_SERVICE_UUID "8f1c0001-4b6e-4a8e-9f3a-2d6c5e7a1b00"
#define IMU_DATA_UUID    "8f1c0003-4b6e-4a8e-9f3a-2d6c5e7a1b00"

const uint32_t PERIOD_US = 1000000UL / SAMPLE_HZ;

// One stored sample (16 bytes).
struct __attribute__((packed)) Sample {
  uint32_t t_ms;
  int16_t ax, ay, az;  // milli-g
  int16_t gx, gy, gz;  // 0.1 degrees/second
};

// What goes over Bluetooth (18 bytes, fits one notification).
struct __attribute__((packed)) Packet {
  Sample s;
  uint16_t backlog;
};

LSM6DS3 imu(I2C_MODE, 0x6A);
BLEService imuService(IMU_SERVICE_UUID);
BLECharacteristic imuData(IMU_DATA_UUID, BLERead | BLENotify, sizeof(Packet), true);

static Sample ring[BUFFER_SAMPLES];
static uint32_t head = 0;    // next slot to write
static uint32_t count = 0;   // samples waiting to be sent
static uint32_t dropped = 0; // samples lost because the buffer was full

static float gyroBias[3] = {0, 0, 0};
static uint32_t nextUs = 0;
static char deviceName[20];

// ---------- LEDs (active-low on the XIAO) ----------
void ledRed(bool on) {
#ifdef LED_RED
  digitalWrite(LED_RED, on ? LOW : HIGH);
#endif
}
void ledBlue(bool on) {
#ifdef LED_BLUE
  digitalWrite(LED_BLUE, on ? LOW : HIGH);
#endif
}

// ---------- buffer ----------
void pushSample(const Sample &s) {
  ring[head] = s;
  head = (head + 1) % BUFFER_SAMPLES;
  if (count < BUFFER_SAMPLES) count++;
  else dropped++;  // overwrote the oldest sample
}

const Sample &oldestSample() {
  return ring[(head + BUFFER_SAMPLES - count) % BUFFER_SAMPLES];
}

int16_t toI16(float v) {
  if (v > 32767.0f) v = 32767.0f;
  if (v < -32768.0f) v = -32768.0f;
  return (int16_t)lroundf(v);
}

// ---------- sensor ----------
void calibrateGyro() {
  // Average the gyro while the board sits still; that average is its offset.
  ledRed(true);
  const int N = 200;
  float sum[3] = {0, 0, 0};
  for (int i = 0; i < N; i++) {
    sum[0] += imu.readFloatGyroX();
    sum[1] += imu.readFloatGyroY();
    sum[2] += imu.readFloatGyroZ();
    delay(5);
  }
  for (int k = 0; k < 3; k++) gyroBias[k] = sum[k] / N;
  ledRed(false);
}

void takeSample() {
  Sample s;
  s.t_ms = millis();
  s.ax = toI16(imu.readFloatAccelX() * 1000.0f);
  s.ay = toI16(imu.readFloatAccelY() * 1000.0f);
  s.az = toI16(imu.readFloatAccelZ() * 1000.0f);
  s.gx = toI16((imu.readFloatGyroX() - gyroBias[0]) * 10.0f);
  s.gy = toI16((imu.readFloatGyroY() - gyroBias[1]) * 10.0f);
  s.gz = toI16((imu.readFloatGyroZ() - gyroBias[2]) * 10.0f);
  pushSample(s);
}

// ---------- output ----------
void printSample(const Packet &p) {
  Serial.print(p.s.t_ms);           Serial.print(',');
  Serial.print(p.s.ax / 1000.0f, 3); Serial.print(',');
  Serial.print(p.s.ay / 1000.0f, 3); Serial.print(',');
  Serial.print(p.s.az / 1000.0f, 3); Serial.print(',');
  Serial.print(p.s.gx / 10.0f, 1);   Serial.print(',');
  Serial.print(p.s.gy / 10.0f, 1);   Serial.print(',');
  Serial.print(p.s.gz / 10.0f, 1);   Serial.print(',');
  Serial.println(p.backlog);
}

void sendBuffered() {
  bool ble = imuData.subscribed();
  bool usb = (bool)Serial;
  if (!ble && !usb) return;  // nobody listening: keep buffering

  for (int i = 0; i < MAX_SEND_PER_LOOP && count > 0; i++) {
    Packet p;
    p.s = oldestSample();
    p.backlog = (count - 1 > 65535) ? 65535 : (uint16_t)(count - 1);
    if (ble && !imuData.writeValue((const uint8_t *)&p, sizeof p)) {
      return;  // Bluetooth busy; try again next loop, sample stays buffered
    }
    if (usb) printSample(p);
    count--;  // sent, remove from buffer
  }
}

// ---------- setup / loop ----------
void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 2000) delay(10);  // don't block on battery

#ifdef LED_RED
  pinMode(LED_RED, OUTPUT);
#endif
#ifdef LED_BLUE
  pinMode(LED_BLUE, OUTPUT);
#endif
  ledRed(false);
  ledBlue(false);

#ifdef PIN_LSM6DS3TR_C_POWER
  pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
  digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
  delay(100);
#endif

  // Ranges suited to fast arm movement (e.g. swimming strokes).
  imu.settings.accelRange = 16;        // +/-16 g
  imu.settings.accelSampleRate = 104;  // Hz
  imu.settings.gyroRange = 2000;       // +/-2000 degrees/second
  imu.settings.gyroSampleRate = 104;   // Hz

  if (imu.begin() != 0) {
    while (true) {
      if (Serial) Serial.println("ERR IMU init failed");
      ledRed(true); delay(500); ledRed(false); delay(500);
    }
  }

  if (Serial) Serial.println("INFO calibrating gyro - keep the board still");
  delay(500);
  calibrateGyro();

  if (!BLE.begin()) {
    while (true) {
      if (Serial) Serial.println("ERR Bluetooth init failed");
      delay(1000);
    }
  }
  snprintf(deviceName, sizeof deviceName, "XIAO-ACCEL-%d", BOARD_ID);
  BLE.setLocalName(deviceName);
  BLE.setDeviceName(deviceName);
  BLE.setConnectionInterval(6, 12);  // 7.5-15 ms, keeps up with 104 Hz
  imuService.addCharacteristic(imuData);
  BLE.addService(imuService);
  BLE.setAdvertisedService(imuService);
  BLE.advertise();

  if (Serial) {
    Serial.print("OK ");
    Serial.print(deviceName);
    Serial.println(" ready (accel+gyro, 104 Hz, buffered)");
  }
  nextUs = micros();
}

void loop() {
  BLE.poll();
  ledBlue(BLE.connected());

  uint32_t now = micros();
  if ((int32_t)(now - nextUs) >= 0) {
    nextUs += PERIOD_US;
    if ((int32_t)(now - nextUs) > (int32_t)(10 * PERIOD_US)) {
      nextUs = now + PERIOD_US;  // fell far behind; resync the schedule
    }
    takeSample();
  }

  sendBuffered();
}
