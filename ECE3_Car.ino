#include <ECE3.h>

uint16_t sensorValues[8];
float weightedValues[8];
uint16_t MIN_VALUES[8] = {641,	619,	619,	550,	642,	665,	620,	641};
uint16_t MAX_VALUES[8] = {1859,	1881,	1881,	1300,	1422,	1835,	1847,	1442};

float errorValue;
float totalWeightedSum;

void setup() {
  // put your setup code here, to run once:
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(1000);
  for (int i = 0; i < 8; i++) {
    sensorRange[i] = (float)(MAX_VALUES[i] - MIN_VALUES[i]);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  ECE3_read_IR(sensorValues);
  for(int i=0;i < 8; i++) {
    sensorValues[i] -= MIN_VALUES[i];
    weightedValues[i] *= (1000.0 / MAX_VALUES[i]);
  }
  errorValue = ( -8.0*weightedValues[0] - 4*weightedValues[1] - 2*weightedValues[2] - weightedValues[3] + weightedValues[4] + 2*weightedValues[5] + 4*weightedValues[6] + 8*weightedValues[7]);
  Serial.println("Sensor Values: ");
  for(int i=0;i < 8; i++) {
    Serial.print(sensorValues[i] + " ")
  }
  Serial.println(errorValue);
  delay(1000);
}
