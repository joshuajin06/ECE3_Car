#include <ECE3.h>
#include <stdio.h>

const int left_nslp_pin=31; // nslp HIGH ==> awake & ready for PWM
const int right_nslp_pin=11; // nslp HIGH ==> awake & ready for PWM
const int left_dir_pin=29;
const int right_dir_pin=30;
const int left_pwm_pin=40;
const int right_pwm_pin=39;

int left_wheel_speed=50;
int right_wheel_speed=50;
int max_speed = 110;
int min_speed = 0;

uint16_t sensorValues[8];
float weightedValues[8];
uint16_t MIN_VALUES[8] = {641,	619,	619,	550,	642,	665,	620,	641};
uint16_t MAX_VALUES[8] = {1859,	1881,	1881,	1300,	1422,	1835,	1847,	1442};

float errorValue;
float k_p = 0.04;
float k_d = 0.000;
float k_val;
float p_left;
float p_right;
float prev_error;
float d_val;
unsigned long lastTime = 0;

int crosspiece_counter = 0;
bool turn_255_happened = false;
unsigned long turn_timer = 0;

float proportional_control(float error, float k_p) {
  //if the car is to the right of the track, error is positive
  //if the car is to the left of the track, error is negative
  //this is assuming car is facing forward
  return error * k_p;
}

void crosspiece_control(float sum) {
  Serial.println(weightedValues[0]);
  Serial.println(sum);
  if(sum > 8000) {
    Serial.println("sum greater than 8000");
    crosspiece_counter += 1;
    if(turn_255_happened == false && crosspiece_counter == 2) {
      Serial.println("first if statement");
      crosspiece_counter = 1;
      unsigned long cur_time = millis();
      digitalWrite(left_dir_pin,LOW);
      while((millis()-cur_time) < turn_timer) {
          Serial.println("second if statement");
          analogWrite(left_pwm_pin, 50);
          analogWrite(right_pwm_pin, 50);
      }
      Serial.println("third if statement");
      turn_255_happened = true;
      crosspiece_counter = 0;
      digitalWrite(left_dir_pin, HIGH);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  ECE3_Init();
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);


  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_nslp_pin,HIGH);


  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(2000);
  lastTime = micros();
}

void loop() {
  // put your main code here, to run repeatedly:

  ECE3_read_IR(sensorValues);

  for(int i=0; i < 8; i++) {
    weightedValues[i] = (float) (sensorValues[i] - MIN_VALUES[i]) * 1000 / MAX_VALUES[i];
    // Serial.println(weightedValues[i]);
  }

  unsigned long now = micros();
  float dt = (now - lastTime) / 1e6;
  float sum = 0;
  for(int i=0; i < 8;i ++) {
    sum += weightedValues[i];
  }

  errorValue = ( -8.0*weightedValues[0] - 4*weightedValues[1] - 2*weightedValues[2] - weightedValues[3] + weightedValues[4] + 2*weightedValues[5] + 4*weightedValues[6] + 8*weightedValues[7]) / 4;
  
  float baseSpeed = left_wheel_speed - 0.1 * abs(errorValue);
  baseSpeed = constrain(baseSpeed, 20, 50);
  
  k_val = errorValue * k_p;
  d_val = k_d * (errorValue - prev_error)/dt;
  // right_wheel_speed += k_val;
  // left_wheel_speed -= k_val;
  p_left = baseSpeed - k_val - d_val;
  p_right = baseSpeed + k_val + d_val;

  // Apply speed limits (clamping)
  if (p_left < min_speed) {
    p_left = min_speed;

  } else if (p_left > max_speed) {
    p_left = max_speed;
  }

  if (p_right < min_speed) {
    p_right = min_speed;
  } else if (p_right > max_speed) {
    p_right = max_speed;
  }

  crosspiece_control(sum);

  // left_wheel_speed = left_wheel_speed - k_val < max_speed ? left_wheel_speed - k_val : max_speed;
  // right_wheel_speed = right_wheel_speed + k_val < max_speed ? right_wheel_speed + k_val : max_speed;

  analogWrite(left_pwm_pin, p_left);
  analogWrite(right_pwm_pin, p_right);

  // Serial.print("Left Wheel Speed: ");
  // Serial.println(left_wheel_speed);
  // Serial.print("Right Wheel Speed: ");
  // Serial.println(right_wheel_speed);
  prev_error = errorValue;
  lastTime = now;
}
