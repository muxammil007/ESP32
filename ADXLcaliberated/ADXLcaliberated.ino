#include <SPI.h>

// Memory register addresses
const int DEVID_AD = 0x00;
const int DEVID_MST = 0x01;
const int PARTID = 0x02;
const int XDATA3 = 0x08;
const int XDATA2 = 0x09;
const int XDATA1 = 0x0A;
const int YDATA3 = 0x0B;
const int YDATA2 = 0x0C;
const int YDATA1 = 0x0D;
const int ZDATA3 = 0x0E;
const int ZDATA2 = 0x0F;
const int ZDATA1 = 0x10;
const int RANGE = 0x2C;
const int POWER_CTL = 0x2D;

// Device values
const int RANGE_10G = 0x01;
const int RANGE_20G = 0x02;
const int RANGE_40G = 0x03;
const int MEASURE_MODE = 0x06; // Only accelerometer

// Operations
const int READ_BYTE = 0x01;
const int WRITE_BYTE = 0x00;

// Pins used for the connection with the sensor
const int CHIP_SELECT_PIN = 2;

void setup() {
  Serial.begin(115200);
  SPI.begin(18, 19, 23, CHIP_SELECT_PIN);  // SCLK, MISO, MOSI, SS

  // Initialize the data ready and chip select pins
  pinMode(CHIP_SELECT_PIN, OUTPUT);
  digitalWrite(CHIP_SELECT_PIN, HIGH);

  // Read and print device IDs
  uint8_t devid_ad = readRegister(DEVID_AD); // Device ID
  uint8_t devid_mst = readRegister(DEVID_MST); // MEMS ID
  uint8_t part_id = readRegister(PARTID);   // PART ID

  Serial.print("Device ID AD: 0x"); Serial.println(devid_ad, HEX);
  Serial.print("Device ID MST: 0x"); Serial.println(devid_mst, HEX);
  Serial.print("Part ID: 0x"); Serial.println(part_id, HEX);
  
  if (devid_ad != 0xAD || devid_mst != 0x1D || part_id != 0xED) {
    Serial.println("Error: Device ID mismatch");
    while (1);
  } else {
    Serial.println("Device ID matched successfully.");
  }

  // Configure ADXL357:
  writeRegister(RANGE, RANGE_10G); // Set range to ±10g
  writeRegister(POWER_CTL, MEASURE_MODE); // Enable measure mode

  // Give the sensor time to set up
  delay(100);
}

const float SENSITIVITY_10G = 51200.0; // Sensitivity for ±10g range in LSB/g

void loop() {
  int axisAddresses[] = {XDATA3, XDATA2, XDATA1, YDATA3, YDATA2, YDATA1, ZDATA3, ZDATA2, ZDATA1};
  int axisMeasures[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int dataSize = 9;

  // Read accelerometer data
  readMultipleData(axisAddresses, dataSize, axisMeasures);

  // Debug print raw data
  // for (int i = 0; i < dataSize; i++) {
  //   Serial.print(axisMeasures[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();

  // Combine data to form 20-bit values
  long xdata = ((long)axisMeasures[0] << 12) | ((long)axisMeasures[1] << 4) | ((long)(axisMeasures[2] >> 4) & 0x0F);
  long ydata = ((long)axisMeasures[3] << 12) | ((long)axisMeasures[4] << 4) | ((long)(axisMeasures[5] >> 4) & 0x0F);
  long zdata = ((long)axisMeasures[6] << 12) | ((long)axisMeasures[7] << 4) | ((long)(axisMeasures[8] >> 4) & 0x0F);

  // Apply two's complement for 20-bit signed integers
  if (xdata & 0x80000) xdata |= ~((1 << 20) - 1); // Sign extend to 32 bits
  if (ydata & 0x80000) ydata |= ~((1 << 20) - 1); // Sign extend to 32 bits
  if (zdata & 0x80000) zdata |= ~((1 << 20) - 1); // Sign extend to 32 bits

  // Debug print combined data
  // Serial.print("xdata: "); Serial.print(xdata); Serial.print(" ");
  // Serial.print("ydata: "); Serial.print(ydata); Serial.print(" ");
  // Serial.print("zdata: "); Serial.println(zdata);

  // Convert raw data to g-units
  float x_g = (float)xdata / SENSITIVITY_10G;
  float y_g = (float)ydata / SENSITIVITY_10G;
  float z_g = (float)zdata / SENSITIVITY_10G;

  // Print axis in g-units
  Serial.print(x_g, 2);
  Serial.print("\t");
  Serial.print(y_g, 2);
  Serial.print("\t");
  Serial.print(z_g, 2);
  Serial.print("\n");

  // Next data in 100 milliseconds
  delay(100);
}

/* 
 * Write registry in specific device address
 */
void writeRegister(byte thisRegister, byte thisValue) {
  byte dataToSend = (thisRegister << 1) | WRITE_BYTE;
  digitalWrite(CHIP_SELECT_PIN, LOW);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(dataToSend);
  SPI.transfer(thisValue);
  SPI.endTransaction();
  digitalWrite(CHIP_SELECT_PIN, HIGH);
}

/* 
 * Read registry in specific device address
 */
unsigned int readRegister(byte thisRegister) {
  unsigned int result = 0;
  byte dataToSend = (thisRegister << 1) | READ_BYTE;

  digitalWrite(CHIP_SELECT_PIN, LOW);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(dataToSend);
  result = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(CHIP_SELECT_PIN, HIGH);
  return result;
}

/* 
 * Read multiple registries
 */
void readMultipleData(int *addresses, int dataSize, int *readedData) {
  digitalWrite(CHIP_SELECT_PIN, LOW);
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
  for (int i = 0; i < dataSize; i++) {
    byte dataToSend = (addresses[i] << 1) | READ_BYTE;
    SPI.transfer(dataToSend);
    readedData[i] = SPI.transfer(0x00);
  }
  SPI.endTransaction();
  digitalWrite(CHIP_SELECT_PIN, HIGH);
}
