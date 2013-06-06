#include <Wire.h>
#include "MPU6050.h"
#include "aJSON.h"

// JSON message is of the format:
// {"compass":{"heading":119.00000},"accelero":{"x":0.04712,"y":0.00049,"z":0.97757},"gyro":{"x":-0.39674,"y":-1.95318,"z":-1.65563}}

#define ACCELEROMETER_RANGE 2.0 // +- 2g
#define GYROSCOPE_RANGE 250.0   // +- 250 deg/s

int interval = 0;
int i;

aJsonStream serial_stream(&Serial);

//// compass
#define MAX_HEADING_HISTORY 5

int HMC6352Address = 0x42;
// This is calculated in the setup() function
int slaveAddress;
//int ledPin = 13;
//boolean ledState = false;
byte headingData[2];
int headingValue;
int headingHistory[MAX_HEADING_HISTORY];
long headingAvg = -1L;
int headingMax = -1;
int headingMin = 10000;

//// accelero / gyro
#define MAX_AG_HISTORY 5
#define AX 0
#define AY 1
#define AZ 2
#define GX 3
#define GY 4
#define GZ 5

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

bool ag_connected = false;
// int16_t ax, ay, az;
// int16_t gx, gy, gz;
int agValue[6];
int agHistory[6][MAX_AG_HISTORY];
long agAvg[6];
int agMax[6];
int agMin[6];

// --------------------------------------------------------------------
void PushFront(int val, int* valueList, int len)
{
  for (int i = len-2; i >= 0; i--) 
  {
    valueList[i+1] = valueList[i];
  }
  valueList[0] = val;
}

long calculateHeadingAvg()
{
  long nHeadingAvg = 0L;
  
  for (int i = 0; i < MAX_HEADING_HISTORY; i++)
  {
    if (headingHistory[i] == -1.0) return -1.0;
    
    nHeadingAvg += headingHistory[i];
  }
  
  return (long) nHeadingAvg /  MAX_HEADING_HISTORY;
}

void calculateAgAvg(long* avg)
{
  long average;
  for (int j = 0; j < 6; ++j)
  {
    average = 0L;
    for (int i = 0; i < MAX_AG_HISTORY; i++)
    {
      if (agHistory[j][i] == -1.0) {
        average = -MAX_AG_HISTORY;
        break;
      }
      
      average += agHistory[j][i];
    }
    
    avg[j] = (long) average /  MAX_AG_HISTORY;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  setupCompass();
  setupAG();

  // createJson();
}

void setupAG() {
  // initialize device
  // Serial.println("Initializing Accelero/Gyro MPU6050...");
  accelgyro.initialize();

  // verify connection
  // Serial.println("Testing connection...");
  ag_connected = accelgyro.testConnection();
  // Serial.println(ag_connected ? "MPU6050 connection successful" : "MPU6050 connection failed");

  resetAG();
}

void setupCompass() {
  // Shift the device's documented slave address (0x42) 1 bit right
  // This compensates for how the TWI library only wants the
  // 7 most significant bits (with the high bit padded with 0)
  slaveAddress = HMC6352Address >> 1;   // This results in 0x21 as the address to pass to TWI

  resetCompass();
}

void loop()
{
  if (Serial.available()) {
    byte b = Serial.read();
    if (b == 'r') {
      resetCompass();
      resetAG();

      Serial.println("reset\n\n\n\n\n\n\n\n\n\n");
      delay(1000);
    } else if (b == 'd') {
      interval -= 50;
      if (interval < 0) {
        interval = 0;
      }
      Serial.print("*** Interval: ");
      Serial.print(interval);
      Serial.print(" ***");
    } else if (b == 'u') {
      interval += 50;
      Serial.print("*** Interval: ");
      Serial.print(interval);
      Serial.print(" ***");
    } else if (b == 'i') {
      interval += 500;
      Serial.print("*** Interval: ");
      Serial.print(interval);
      Serial.print(" ***");
    } else if (b == 's') {
      interval = 0;
      Serial.print("*** Interval: ");
      Serial.print(interval);
      Serial.print(" ***");
    }
  }

  readCompass();
  readAG();

  // debugCompass();
  // debugAG();
  // Serial.println("");

  sendData();

  delay(interval);
} 

void resetAG() {
  for (i = 0; i < 6; ++i)
  {
    agValue[i] = 0;
    for (int j = 0; j < MAX_AG_HISTORY; ++j)
    {
      agHistory[i][j] = -1;
    }
    agAvg[i] = -1L;
    agMax[i] = -1000000;
    agMin[i] = 1000000;
  }
}

void readAG() {

  if (ag_connected) {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&agValue[AX], &agValue[AY], &agValue[AZ], &agValue[GX], &agValue[GY], &agValue[GZ]);

    for (int j = 0; j < 6; ++j)
    {
      // agValue[j] = agValue[j] / (32767 / 2) * 9.81;

      PushFront(agValue[j], agHistory[j], MAX_AG_HISTORY);

      if (agValue[j] > agMax[j]) {
        agMax[j] = agValue[j];
      }

      if (agValue[j] < agMin[j]) {
        agMin[j] = agValue[j];
      }
    }

    calculateAgAvg(agAvg);
  }

    
}

void debugAG() {
  for (int j = 0; j < 3; ++j)
  {
    Serial.print(formatAcceleroValue(agValue[j])); Serial.print("\t");
    Serial.print("["); Serial.print(formatAcceleroValue(agAvg[j])); Serial.print("]"); Serial.print("\t");
    Serial.print("{"); Serial.print(formatAcceleroValue(agMin[j])); Serial.print(","); Serial.print(formatAcceleroValue(agMax[j])); Serial.print("}"); Serial.print("\t");
  }
  for (int j = 3; j < 6; ++j)
  {
    Serial.print(formatGyroValue(agValue[j])); Serial.print("\t");
    Serial.print("["); Serial.print(formatGyroValue(agAvg[j])); Serial.print("]"); Serial.print("\t");
    Serial.print("{"); Serial.print(formatGyroValue(agMin[j])); Serial.print(","); Serial.print(formatGyroValue(agMax[j])); Serial.print("}"); Serial.print("\t");
  }
}

void resetCompass() {
  for (int i = 0; i < MAX_HEADING_HISTORY; i++) {
    headingHistory[i] = -1;
  }

  headingAvg = -1L;
  headingMax = -1000000;
  headingMin = 1000000;
}

void readCompass() {

  // for (int j = 0; j < MAX_HEADING_HISTORY; j++) {
    // Flash the LED on pin 13 just to show that something is happening
    // Also serves as an indication that we're not "stuck" waiting for TWI data
  //  ledState = !ledState;
  //  if (ledState) {
  //    digitalWrite(ledPin,HIGH);
  //  }
  //  else
  //  {
  //    digitalWrite(ledPin,LOW);
  //  }
    // Send a "A" command to the HMC6352
    // This requests the current heading data
    Wire.beginTransmission(slaveAddress);
  //  Wire.send("A");              // The "Get Data" command
    Wire.write("A");              // The "Get Data" command
    Wire.endTransmission();
    delay(10);                   // The HMC6352 needs at least a 70us (microsecond) delay
    // after this command.  Using 10ms just makes it safe
    // Read the 2 heading bytes, MSB first
    // The resulting 16bit word is the compass heading in 10th's of a degree
    // For example: a heading of 1345 would be 134.5 degrees
    Wire.requestFrom(slaveAddress, 2);        // Request the 2 byte heading (MSB comes first)
    i = 0;
    while(Wire.available() && i < 2)
    { 
  //    headingData[i] = Wire.receive();
      headingData[i] = Wire.read();
      i++;
    }
    headingValue = headingData[0]*256 + headingData[1];  // Put the MSB and LSB together

    PushFront(headingValue, headingHistory, MAX_HEADING_HISTORY);

    if (headingValue > headingMax) {
      headingMax = headingValue;
    }

    if (headingValue < headingMin) {
      headingMin = headingValue;
    }
  // }

  headingAvg = calculateHeadingAvg();


}

void debugCompass() {
  // // Serial.print("Current heading: ");
  // Serial.print(headingValue / 10.0);
  // // Serial.print(int (headingValue / 10));     // The whole number part of the heading
  // // Serial.print(".");
  // // Serial.print(int (headingValue % 10));     // The fractional part of the heading
  // // Serial.print(" degrees");
  // if (headingAvg != -1) {
  //   // Serial.print(headingAvg / 10.0);
  //   Serial.print("\t");
  //   Serial.print("Avg: ");
  //   Serial.print(headingAvg / 10.0);     // The whole number part of the heading
  //   Serial.println(" ");
  //   Serial.print("\t");
  //   Serial.print("Min: ");
  //   Serial.print(headingMin / 10.0);     // The whole number part of the heading
  //   Serial.println(" ");
  //   Serial.print("\t");
  //   Serial.print("Max: ");
  //   Serial.print(headingMax / 10.0);     // The whole number part of the heading
  // }
  Serial.print(formatCompassValue(headingValue)); Serial.print("\t");
  Serial.print("["); Serial.print(formatCompassValue(headingAvg)); Serial.print("]"); Serial.print("\t");
  Serial.print("{"); Serial.print(formatCompassValue(headingMin)); Serial.print(","); Serial.print(formatCompassValue(headingMax)); Serial.print("}"); Serial.print("\t");
}

// JSON message is of the format:
// {"compass":{"heading":119.00000},"accelero":{"x":0.04712,"y":0.00049,"z":0.97757},"gyro":{"x":-0.39674,"y":-1.95318,"z":-1.65563}}

aJsonObject *json, *compass, *accelero, *gyro;
void createJson() {
  json = aJson.createObject();

  compass = aJson.createObject();
  aJson.addNumberToObject(compass, "heading", formatCompassValue(headingValue));
  aJson.addItemToObject(json, "compass", compass);

  accelero = aJson.createObject();
  aJson.addNumberToObject(accelero, "x", formatAcceleroValue(agValue[AX]));
  aJson.addNumberToObject(accelero, "y", formatAcceleroValue(agValue[AY]));
  aJson.addNumberToObject(accelero, "z", formatAcceleroValue(agValue[AZ]));
  aJson.addItemToObject(json, "accelero", accelero);

  gyro = aJson.createObject();
  aJson.addNumberToObject(gyro, "x", formatGyroValue(agValue[GX]));
  aJson.addNumberToObject(gyro, "y", formatGyroValue(agValue[GY]));
  aJson.addNumberToObject(gyro, "z", formatGyroValue(agValue[GZ]));
  aJson.addItemToObject(json, "gyro", gyro);
}

float formatAcceleroValue(int value) {
  return float(value / 32767.0 * ACCELEROMETER_RANGE);
}

float formatGyroValue(int value) {
  return float(value / 32767.0 * GYROSCOPE_RANGE);
}

float formatCompassValue(int value) {
  return float(value / 10.0);
}

void sendData() {
  createJson();
  // aJson.getObjectItem(compass, "theta")->valuefloat = headingValue;

  aJson.print(json, &serial_stream);
  Serial.println();
  aJson.deleteItem(json);
}