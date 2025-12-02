#include <ECE3.h>
#include <stdio.h>

const int left_nslp_pin=31; // nslp HIGH ==> awake & ready for PWM
const int right_nslp_pin=11; // nslp HIGH ==> awake & ready for PWM
const int left_dir_pin=29;
const int right_dir_pin=30;
const int left_pwm_pin=40;
const int right_pwm_pin=39;


int left_wheel_speed=20;
int right_wheel_speed=20;
int max_speed = 110;
int min_speed = 0;

uint16_t sensorValues[8];
float weightedValues[8];
uint16_t MIN_VALUES[8] = {641,	619,	619,	550,	642,	665,	620,	641};
uint16_t MAX_VALUES[8] = {1859,	1881,	1881,	1300,	1422,	1835,	1847,	1442};

float errorValue;
float k_p = 0.04;
float k_d = 0.001;
float k_val;
float p_left;
float p_right;
float prev_error;
float d_val;
unsigned long lastTime = 0;

int K = 1;

bool end_program = false;


// -------- crosspiece control variables ---------------
int debounce_count = 0; 

// 0 = Start, 1 = Donut, 2 = Horseshoe Start, 3 = Horseshoe End, 4 = Finish
int track_state = 0; 

// 'turn_255_happened' replaces 'turn_255_happened'
bool turn_255_happened = false;

// --- CONSTANTS (No more magic numbers) ---
const unsigned long spin_duration = 1600; // Time to spin for 225 degrees
const int BLACK_THRESHOLD = 2100;          // Sensor value for black line

bool crosspiece_detected() {
  int count = 0;
  int sensor_threshold = 4; // Require 4 sensors to be dark

  for (int i = 0; i < 8; ++i){
    // Using the constant defined above instead of hardcoded 2200
    if (sensorValues[i] > BLACK_THRESHOLD){
      count++;
    }
  }

  // Special Case: The 3rd marker (End of Horseshoe) is tricky/thin?
  if(track_state == 3){
    sensor_threshold = 3;
  }
  if(track_state == 1){
    sensor_threshold = 5;
  }

  if(count >= sensor_threshold){
    return true;
  }
  return false;
}

// Helper function to handle the 225 degree turn
void perform_spin() {
    unsigned long start_time = millis();
    
    // Pause briefly before turn
    delay(80);
    
    // Set motors to spin Counter-Clockwise
    // Left Wheel Reverses (HIGH), Right Wheel Forward (LOW)
    digitalWrite(left_dir_pin, HIGH);
    
    // Spin for the exact duration
    while((millis() - start_time) < spin_duration) {
        analogWrite(left_pwm_pin, 50);
        analogWrite(right_pwm_pin, 50);
    }
    
    // Reset Left Wheel to Forward (LOW)
    digitalWrite(left_dir_pin, LOW);
    
    turn_255_happened = true;
    
    // Reset debounce so we don't accidentally trigger again immediately
    debounce_count = 0; 
}

void crosspiece_control() {

  // Do we see a black bar?
  if(crosspiece_detected()) {
    debounce_count++; // Increment noise filter

    // Have we seen it 2 times in a row?
    if(debounce_count == 2){
      
      track_state++; // We passed a marker, move to next state

      // --- STATE 1: THE 225 DEGREE TURN ---
      if(track_state == 1) {
        if(!turn_255_happened) {
           perform_spin();
        }

        left_wheel_speed=50;
        right_wheel_speed=50;
        
        // After spin, increase K gain (aggressive turning for horseshoe?)
        K = 2; 

        // weightedValues[5] = 665;
        // weightedValues[6] = 620;
        // weightedValues[7] = 641;
        
        // LED Debug: ON-OFF-OFF
        digitalWrite(75, HIGH); 
        digitalWrite(76, LOW); 
        digitalWrite(77, LOW);
      }
      
      // --- STATE 2: HORSESHOE START  ---
      else if(track_state == 2) {
        // LED Debug: OFF-ON-OFF
        digitalWrite(75, LOW); 
        digitalWrite(76, HIGH); 
        digitalWrite(77, LOW);
        
        K = 1; // Reset K to normal
      }
      
      // --- STATE 3: HORSESHOE END ---
      else if(track_state == 3) {
        // left_wheel_speed=30;
        // right_wheel_speed=30;


        // weightedValues[4] = 0;
        // weightedValues[5] = 0;
        // weightedValues[6] = 0;
        // weightedValues[7] = 0;
        // weightedValues[4] = 642;
        // weightedValues[5] = 665;
        // weightedValues[6] = 620;
        // weightedValues[7] = 641;

        // bool firmly_on_left_line = (weightedValues[3] > 500) || (weightedValues[4] > 500);

        // if (firmly_on_left_line) {
        //   // We are safe on the main line. Put blinders on the right side.
        //   // This makes the car ignore the split track on the right.
        //   weightedValues[0] = 600;
        //   weightedValues[1] = 600; 
        // }


        // LED Debug: OFF-OFF-ON
        digitalWrite(75, LOW); 
        digitalWrite(76, LOW); 
        digitalWrite(77, HIGH);

        left_wheel_speed=80;
        right_wheel_speed=80;

        k_p = 0.06;
        
        // K stays at 1
      }
      
      // --- STATE 4: END OF TRACK ---
      else if(track_state == 4){
        // LED Debug: ON-OFF-OFF
        digitalWrite(75, HIGH); 
        digitalWrite(76, LOW); 
        digitalWrite(77, LOW);
        delay(200);
        
        end_program = true;           // Successfully ends run
      }
    }
  } 
  else {
    // If we don't see the line, reset the noise filter
    debounce_count = 0;
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

  // Serial.println(sensorValues[0]);

  float sum = 0;
  for(int i=0; i < 8; i++) {
    weightedValues[i] = (float) (sensorValues[i] - MIN_VALUES[i]) * 1000 / MAX_VALUES[i];
    // Serial.println(weightedValues[i]);
  }

  unsigned long now = micros();
  float dt = (now - lastTime) / 1e6;
  for(int i=2; i < 5;i ++) {
    sum += weightedValues[i];
  }

  crosspiece_control();
  if (track_state == 3 || track_state == 1) {
  // if (track_state == 3) {
      
      // Check if the CENTER sensors see the line (Indices 3 and 4 are the middle)
      bool firmly_on_main_line = (weightedValues[3] > 500) || (weightedValues[4] > 500) || (weightedValues[5] > 500);
    
      if (firmly_on_main_line) {
        // "Blinders on" - Ignore the split track on the RIGHT.
        weightedValues[0] = 0;
        weightedValues[1] = 0; 
        weightedValues[2] = 400; 
      }
  }
  // if (track_state == 1) {
      
  //     // Check if the CENTER sensors see the line (Indices 3 and 4 are the middle)
  //     bool firmly_on_main_line = (weightedValues[3] > 500) || (weightedValues[4] > 500) || (weightedValues[5] > 500);
    
  //     if (firmly_on_main_line) {
  //       // "Blinders on" - Ignore the split track on the RIGHT.
  //       weightedValues[0] = 0;
  //       weightedValues[1] = 0; 
  //       // weightedValues[2] = 0; 
  //     }
  // }
  errorValue = ( -8*weightedValues[0] - 4*weightedValues[1] - 2 * K * weightedValues[2] - K * weightedValues[3] + weightedValues[4] + 2*weightedValues[5] + 4*weightedValues[6] + 8*weightedValues[7]) / 4;
  

  float baseSpeed = left_wheel_speed - 0.1 * abs(errorValue);
  baseSpeed = constrain(baseSpeed, 20, 50);
  
  k_val = errorValue * k_p;
  d_val = k_d * (errorValue - prev_error)/dt;
  // right_wheel_speed += k_val;
  // left_wheel_speed -= k_val;
  p_left = baseSpeed - k_val - d_val;
  // p_left = baseSpeed - k_val;
  p_right = baseSpeed + k_val + d_val;
  // p_right = baseSpeed + k_val;

  // Apply speed limits (clamping)
  if (p_left < 0) {
    p_left = min_speed;

  } else if (p_left > max_speed) {
    p_left = max_speed;
  }

  if (p_right < min_speed) {
    p_right = min_speed;
  } else if (p_right > max_speed) {
    p_right = max_speed;
  }


  if(end_program == true){
    while (true) {
      analogWrite(left_pwm_pin, 0);  // ADD THIS: Turn off motors!
      analogWrite(right_pwm_pin, 0); // ADD THIS: Turn off motors!
    }
  }

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
