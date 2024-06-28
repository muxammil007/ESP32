#include <SPI.h>

// Define the pins for SPI
#define BMP_SCK 18
#define BMP_MISO 19
#define BMP_MOSI 23
#define BMP_CS 5

// Define BMP280 Registers
#define BMP280_REG_ID 0xD0
#define BMP280_REG_RESET 0xE0
#define BMP280_REG_STATUS 0xF3
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_PRESS_LSB 0xF8
#define BMP280_REG_PRESS_XLSB 0xF9
#define BMP280_REG_TEMP_MSB 0xFA
#define BMP280_REG_TEMP_LSB 0xFB
#define BMP280_REG_TEMP_XLSB 0xFC

// Calibration parameters
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

void setup() {
  Serial.begin(115200);
  SPI.begin(BMP_SCK, BMP_MISO, BMP_MOSI, BMP_CS);
  pinMode(BMP_CS, OUTPUT);
  digitalWrite(BMP_CS, HIGH);
  
  // Check the BMP280 ID
  uint8_t id = readRegister8(BMP280_REG_ID);
  Serial.print("BMP280 ID: 0x");
  Serial.println(id, HEX);
  
  // Reset the sensor
  writeRegister8(BMP280_REG_RESET, 0xB6);
  delay(100);

  // Read calibration data
  readCalibrationData();
  
  // Configure the sensor
  writeRegister8(BMP280_REG_CTRL_MEAS, 0x27); // Normal mode, temp and pressure over-sampling rate = 1
  writeRegister8(BMP280_REG_CONFIG, 0xA0); // Standby time = 1000 ms

  Serial.println("BMP280 Initialized");
}

void loop() {
  int32_t temperature = readTemperature();
  uint32_t pressure = readPressure();
  
  Serial.print("Temperature: ");
  Serial.print(temperature / 100.0);
  Serial.println(" *C");
  
  Serial.print("Pressure: ");
  Serial.print(pressure / 100.0);
  Serial.println(" hPa");
  
  delay(2000);
}

uint8_t readRegister8(uint8_t reg) {
  digitalWrite(BMP_CS, LOW);
  SPI.transfer(reg | 0x80); // Set MSB to 1 to indicate read operation
  uint8_t value = SPI.transfer(0);
  digitalWrite(BMP_CS, HIGH);
  return value;
}

void writeRegister8(uint8_t reg, uint8_t value) {
  digitalWrite(BMP_CS, LOW);
  SPI.transfer(reg & 0x7F); // Set MSB to 0 to indicate write operation
  SPI.transfer(value);
  digitalWrite(BMP_CS, HIGH);
}

void readCalibrationData() {
  dig_T1 = read16_LE(0x88);
  dig_T2 = readS16_LE(0x8A);
  dig_T3 = readS16_LE(0x8C);
  dig_P1 = read16_LE(0x8E);
  dig_P2 = readS16_LE(0x90);
  dig_P3 = readS16_LE(0x92);
  dig_P4 = readS16_LE(0x94);
  dig_P5 = readS16_LE(0x96);
  dig_P6 = readS16_LE(0x98);
  dig_P7 = readS16_LE(0x9A);
  dig_P8 = readS16_LE(0x9C);
  dig_P9 = readS16_LE(0x9E);
}

uint16_t read16_LE(uint8_t reg) {
  return (readRegister8(reg) | (readRegister8(reg + 1) << 8));
}

int16_t readS16_LE(uint8_t reg) {
  return (int16_t)read16_LE(reg);
}

int32_t readTemperature() {
  int32_t var1, var2;
  int32_t adc_T = (readRegister8(BMP280_REG_TEMP_MSB) << 12) | (readRegister8(BMP280_REG_TEMP_LSB) << 4) | (readRegister8(BMP280_REG_TEMP_XLSB) >> 4);

  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  int32_t t_fine = var1 + var2;
  return (t_fine * 5 + 128) >> 8;
}

uint32_t readPressure() {
  int32_t var1, var2;
  int32_t adc_P = (readRegister8(BMP280_REG_PRESS_MSB) << 12) | (readRegister8(BMP280_REG_PRESS_LSB) << 4) | (readRegister8(BMP280_REG_PRESS_XLSB) >> 4);
  int32_t t_fine = readTemperature(); // call readTemperature to get t_fine value

  var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dig_P6);
  var2 = var2 + ((var1 * ((int32_t)dig_P5)) << 1);
  var2 = (var2 >> 2) + (((int32_t)dig_P4) << 16);
  var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((int32_t)dig_P1)) >> 15);
  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  uint32_t p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (p < 0x80000000) {
    p = (p << 1) / ((uint32_t)var1);
  } else {
    p = (p / (uint32_t)var1) * 2;
  }
  var1 = (((int32_t)dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
  var2 = (((int32_t)(p >> 2)) * ((int32_t)dig_P8)) >> 13;
  p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
  return p;
}
