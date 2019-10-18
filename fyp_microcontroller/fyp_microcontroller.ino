// TODO: MAKE SURE STEPPER MOTOR CONNECTIONS ARE CORRECT
// ASSUMED STEPPER IN RAIL 1 IS A/NOT_A
// TODO: MAKE VARIABLES STATIC

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define KP  0.2
#define KI  0.001
#define KD  0.0001

#define LED_PIN           13
#define STEPPER_ENABLE1   4
#define STEPPER_IN1_1     5
#define STEPPER_IN1_2     6
#define STEPPER_ENABLE2   7
#define STEPPER_IN2_1     8
#define STEPPER_IN2_2     9
#define LINEAR_ACT_ENABLE 10
#define LINEAR_ACT_IN1    11
#define LINEAR_ACT_IN2    12
#define LINEAR_POT        A3

#define MAX_ELEVATION_RANGING_SAMPLES 1500
#define ELEVATION_RANGING_TOLERANCE   2
#define OFFSET_ORIENTATION_Y          0
#define ELEVATION_CONFIRMED_LIMIT     30

#define STEPS_PER_STEPPER_REV 200
#define STEPPER_TEETH         10
#define DRIVEN_GEAR_TEETH     42
#define GEARBOX_GEAR_RATIO    0.04 //25 revs on input shaft --> 1 rev on output shaft

#define CONTROL_INTERVAL      10 
#define STEPPER_PHASE_DELAY   3

#define BUFFER_INDEX_AZ_HUNDRED 0
#define BUFFER_INDEX_AZ_TEN 1
#define BUFFER_INDEX_AZ_ONE 2
#define BUFFER_INDEX_AZ_DEC 3
#define BUFFER_INDEX_EL_TEN 4
#define BUFFER_INDEX_EL_ONE 5
#define BUFFER_INDEX_EL_DEC 6

const boolean ANTICLOCKWISE       = 1;
const boolean CLOCKWISE           = 0;
const boolean POSITIVE_POLARITY   = 1;
const boolean NEGATIVE_POLARITY   = 0;

const uint16_t IMU_SAMPLERATE_DELAY = 100;
unsigned long prev_control_time = millis();
unsigned long prev_step_time = millis();
unsigned long current_time = millis();
volatile byte imu_sample_counter = 0;
const float dt = (CONTROL_INTERVAL/1000.0);


float desired_azimuth = 0.0;
float desired_elevation = 0.0;
int desired_elevation_pot = 0;
double antenna_orientation_x, antenna_orientation_y, antenna_orientation_z;
boolean elevation_achieved = false;
boolean azimuth_achieved = false;
float elevation_integral_term = 0.0;
float elevation_derivative_term = 0.0;
float prev_elevation_error = 0.0;

float min_elevation, max_elevation;
int min_pot_value, max_pot_value;

volatile byte phase_number = 1;
int desired_step_number = 0;
volatile int step_number = 0;
boolean stepper_motor_enabled = false;
int steps_per_antenna_rev;
byte stepper_counter = 0;

const byte buffer_size = 7;
char received_buffer[buffer_size];
int buffer_index = 0;
boolean new_command_received = false;
boolean message_being_received = false;
const char start_char = '<';
const char end_char = '>';

Adafruit_BNO055 IMU = Adafruit_BNO055(55, 0x28);

void setupPins()
{
  pinMode(LED_PIN,OUTPUT);
  pinMode(LINEAR_ACT_ENABLE,OUTPUT); 
  pinMode(LINEAR_ACT_IN1,OUTPUT); 
  pinMode(LINEAR_ACT_IN2,OUTPUT);
  pinMode(LINEAR_POT, INPUT);
  pinMode(STEPPER_ENABLE1,OUTPUT);
  pinMode(STEPPER_IN1_1,OUTPUT);
  pinMode(STEPPER_IN1_2,OUTPUT);
  pinMode(STEPPER_ENABLE2,OUTPUT);
  pinMode(STEPPER_IN2_1,OUTPUT);
  pinMode(STEPPER_IN2_2,OUTPUT);
  

  digitalWrite(LED_PIN, LOW);
  digitalWrite(LINEAR_ACT_ENABLE, LOW);
  digitalWrite(LINEAR_ACT_IN1, HIGH);
  digitalWrite(LINEAR_ACT_IN2, LOW);
  digitalWrite(STEPPER_ENABLE1, LOW);
  digitalWrite(STEPPER_IN1_1, HIGH);
  digitalWrite(STEPPER_IN1_2, LOW);
  digitalWrite(STEPPER_ENABLE2, LOW);
  digitalWrite(STEPPER_IN2_1, HIGH);
  digitalWrite(STEPPER_IN2_2, LOW);
}

void setupIMU()
{
  Serial.println("Initialising IMU...");
  if(!IMU.begin()) digitalWrite(LED_PIN, HIGH);
  delay(1000);
}

boolean IMUReady(unsigned long prev_time)
{
  if((millis()-prev_time) >= IMU_SAMPLERATE_DELAY) return true;
  else return false;
}

void getAntennaOrientation()
{
  sensors_event_t orientationData;
  IMU.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  setAntennaOrientation(&orientationData);
}

void setAntennaOrientation(sensors_event_t* event)
{
  
  antenna_orientation_x = event->orientation.x;
  antenna_orientation_y = (-1*(event->orientation.y) + OFFSET_ORIENTATION_Y);
  antenna_orientation_z = event->orientation.z;
}

int calculateStepsPerRev(int steps_per_stepper_rev, int num_stepper_teeth, int num_driven_teeth, float gearbox_gear_ratio)
{
  
  float belt_system_gear_ratio = (float) num_stepper_teeth / num_driven_teeth; 
  int steps_per_driven_rev = steps_per_stepper_rev / belt_system_gear_ratio;
  int steps_per_rev = steps_per_driven_rev * (float)(1/gearbox_gear_ratio);
  return steps_per_rev;
}

void setLinearActuatorSpeed(unsigned int duty_cycle)
{
  analogWrite(LINEAR_ACT_ENABLE, map(duty_cycle,0,100,0,255));
}

void extendLinearActuator()
{
  digitalWrite(LINEAR_ACT_IN1, HIGH);
  digitalWrite(LINEAR_ACT_IN2, LOW);
}

void retractLinearActuator()
{
  digitalWrite(LINEAR_ACT_IN1, LOW);
  digitalWrite(LINEAR_ACT_IN2, HIGH);
}

void getElevationRange()
{
  boolean min_elevation_found = false;
  boolean max_elevation_found = false;
  byte confirmed_count = 0;
  int num_of_samples = 0;
  unsigned int pot_val;
  unsigned int prev_pot_val = 2000;
  setLinearActuatorSpeed(100);
  extendLinearActuator();
  delay(500);
  while(!min_elevation_found && (num_of_samples < MAX_ELEVATION_RANGING_SAMPLES))
  {
    pot_val = analogRead(LINEAR_POT);
    if((pot_val < (prev_pot_val+ELEVATION_RANGING_TOLERANCE)) && (pot_val > (prev_pot_val-ELEVATION_RANGING_TOLERANCE)))
    {
     confirmed_count++;
     if(confirmed_count > ELEVATION_CONFIRMED_LIMIT) 
     {
      min_elevation = 0.00;
      min_pot_value = pot_val;
      for(int i = 0; i < 10; i++)
      {
        delay(IMU_SAMPLERATE_DELAY);
        getAntennaOrientation();
        min_elevation += antenna_orientation_y;
      }
      min_elevation = min_elevation/10.0;
      min_elevation_found = true;
      Serial.println(min_elevation);
     }
    }
    else
    {
      prev_pot_val = pot_val;
      confirmed_count = 0;
    }
    num_of_samples++;
    delay(10);
  }
  confirmed_count = 0;
  num_of_samples = 0;
  prev_pot_val = 2000;
  setLinearActuatorSpeed(100);
  retractLinearActuator();
  delay(500);
  while(!max_elevation_found && (num_of_samples < MAX_ELEVATION_RANGING_SAMPLES))
  {
    pot_val = analogRead(LINEAR_POT);
    if((pot_val < (prev_pot_val+ELEVATION_RANGING_TOLERANCE)) && (pot_val > (prev_pot_val-ELEVATION_RANGING_TOLERANCE)))
    {
     confirmed_count++;
     if(confirmed_count > ELEVATION_CONFIRMED_LIMIT) 
     {
      max_elevation = 0.00;
      max_pot_value = pot_val;
      for(int i = 0; i < 10; i++)
      {
        delay(IMU_SAMPLERATE_DELAY);
        getAntennaOrientation();
        max_elevation += antenna_orientation_y;
      }
      max_elevation = max_elevation/10.0;
      max_elevation_found = true;
      Serial.println(max_elevation);
     }
    }
    else
    {
      prev_pot_val = pot_val;
      confirmed_count = 0;
    }
    num_of_samples++;
    delay(10);
  }
}

int mapElevationToPotVal(float elevation)
{
  return map(elevation, min_elevation, max_elevation, min_pot_value, max_pot_value);
}

void elevationControlLoop()
{
  int current_position = analogRead(LINEAR_POT);
  float current_error = (float)(desired_elevation_pot - current_position); 
  int duty_cycle = (KP * current_error) + (KI * elevation_integral_term);
  if(duty_cycle > 100)
  {
    duty_cycle = 100;
  }
  else if(duty_cycle < -100)
  {
    duty_cycle = -100;
  }
  else 
  {
    elevation_integral_term += (current_error * dt);
    if(elevation_integral_term > 100.0) elevation_integral_term = 100.0;
    else if(elevation_integral_term < -100.0) elevation_integral_term = -100.0;
  }

  //Serial.print("desired ");
  //Serial.print(desired_elevation_pot);
  //Serial.print(" current ");
  //Serial.print(current_position);
  //Serial.print(" duty ");
  //Serial.print(duty_cycle);
  //Serial.print(" current_error ");
  //Serial.print(current_error);
  //Serial.print(" integral ");
  //Serial.println(elevation_integral_term);
  if(duty_cycle<0)
  {
    setLinearActuatorSpeed(abs(duty_cycle));
    extendLinearActuator();
  }
  else
  {
    setLinearActuatorSpeed(duty_cycle);
    retractLinearActuator();
  }
}


void disableLinearActuator()
{
  setLinearActuatorSpeed(0);
}

void enableStepperMotor()
{
  
  //digitalWrite(STEPPER_ENABLE1, HIGH);
  //digitalWrite(STEPPER_ENABLE2, HIGH);
  stepper_motor_enabled = true;
}

void setMotorPolarity(byte A_polarity, byte B_polarity, byte not_A_polarity, byte not_B_polarity)
{
  if(A_polarity == POSITIVE_POLARITY)
  {
    digitalWrite(STEPPER_IN1_1, HIGH);
    digitalWrite(STEPPER_IN1_2, LOW);
  }
  else
  {
    digitalWrite(STEPPER_IN1_1, LOW);
    digitalWrite(STEPPER_IN1_2, HIGH);
  }
  if(B_polarity == POSITIVE_POLARITY) 
  {
    digitalWrite(STEPPER_IN2_1, HIGH);
    digitalWrite(STEPPER_IN2_2, LOW);
  }
  else
  {
    digitalWrite(STEPPER_IN2_1, LOW);
    digitalWrite(STEPPER_IN2_2, HIGH);
  }
}

void rotateStepperOneStep(int rotation_clockwise)
{

  if(!stepper_motor_enabled) enableStepperMotor();
  if(rotation_clockwise)
  {
    phase_number++;
    if(phase_number > 4) phase_number = 1;
  }
  else
  {
    phase_number--;
    if(phase_number < 1) phase_number = 4;
  }
  switch(phase_number)
  {
    case 1:
      setMotorPolarity(POSITIVE_POLARITY, POSITIVE_POLARITY, NEGATIVE_POLARITY, NEGATIVE_POLARITY);
      break;
    case 2:
      setMotorPolarity(NEGATIVE_POLARITY, POSITIVE_POLARITY, POSITIVE_POLARITY, NEGATIVE_POLARITY);
      break;
    case 3:
      setMotorPolarity(NEGATIVE_POLARITY, NEGATIVE_POLARITY, POSITIVE_POLARITY, POSITIVE_POLARITY);
      break;
    case 4:
      setMotorPolarity(POSITIVE_POLARITY, NEGATIVE_POLARITY, NEGATIVE_POLARITY, POSITIVE_POLARITY);
      break;
  }
  step_number++;
  
}

int calculateNewDesiredStep(float bearing, int current_step)
{
  int CW_bearing_step = map((int)(bearing*10.0), 0, 3600, 0, 21000);
  int ACW_bearing_step = map((int)((bearing-360.0)*10.0), 0, -3600, 0, -21000);
  int ACW_step_difference = abs((ACW_bearing_step - current_step));
  int CW_step_difference = abs((CW_bearing_step - current_step));
  if(ACW_step_difference < CW_step_difference)
  {
    return ACW_bearing_step;
  }
  else
  {
    return CW_bearing_step;
  }
}

void updateStepper()
{
  if(desired_step_number < step_number)
  {
    rotateStepperOneStep(ANTICLOCKWISE);
  }
  else if(desired_step_number > step_number)
  {
    rotateStepperOneStep(CLOCKWISE);
  }
}

void disableStepperMotor()
{
  digitalWrite(STEPPER_ENABLE1, LOW);
  digitalWrite(STEPPER_ENABLE2, LOW);
  stepper_motor_enabled = false;
}

void checkUARTRecv() 
{
  char received_data;
  while((Serial.available() > 0) && new_command_received == false) {
    received_data = Serial.read();
    if(received_data == start_char) 
    {
      message_being_received = true;
    }
    else 
    {
      if(message_being_received) 
      {
        if((received_data != end_char)) 
        {
          received_buffer[buffer_index] = received_data;
          buffer_index++;
          if(buffer_index > buffer_size) 
          {
            buffer_index = buffer_size - 1;
            digitalWrite(LED_PIN, HIGH);
          }
        }
        else 
        {
          buffer_index = 0;
          message_being_received = false;
          new_command_received = true;
        }
      }

    }

  }
}

void parseReceivedData() 
{
  desired_azimuth = (float)((received_buffer[BUFFER_INDEX_AZ_HUNDRED]-'0')*100) +  (float)((received_buffer[BUFFER_INDEX_AZ_TEN]-'0')*10) + (float)((received_buffer[BUFFER_INDEX_AZ_ONE]-'0')) + (((float)(received_buffer[BUFFER_INDEX_AZ_DEC]-'0'))/10);
  desired_elevation = (float)((received_buffer[BUFFER_INDEX_EL_TEN]-'0')*10) + (float)((received_buffer[BUFFER_INDEX_EL_ONE]-'0')) + (((float)(received_buffer[BUFFER_INDEX_EL_DEC]-'0'))/10);
  new_command_received = false;
}

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  setupPins();
  setupIMU();
  //getElevationRange();
  //desired_elevation_pot = mapElevationToPotVal(0.0);
  steps_per_antenna_rev = calculateStepsPerRev(STEPS_PER_STEPPER_REV, STEPPER_TEETH, DRIVEN_GEAR_TEETH, GEARBOX_GEAR_RATIO);
}

void loop() 
{
  checkUARTRecv();
  current_time = millis();
  
  if(new_command_received) {
    parseReceivedData();
    desired_elevation_pot = mapElevationToPotVal(desired_elevation);
    desired_step_number = calculateNewDesiredStep(desired_azimuth, step_number);
  }
  
  if((current_time - prev_control_time) >= CONTROL_INTERVAL)
  {
    prev_control_time = current_time;
    //elevationControlLoop();
    imu_sample_counter++;
    if(imu_sample_counter == (IMU_SAMPLERATE_DELAY/CONTROL_INTERVAL))
    {
      imu_sample_counter = 0;
      getAntennaOrientation();
      Serial.println("");
      Serial.println(desired_step_number);
      Serial.println(step_number);
      Serial.println("");
    }
    if((current_time - prev_step_time) >= STEPPER_PHASE_DELAY)
    {
      updateStepper();
      prev_step_time = current_time;
    }
  }
  //for (int i = 0; i < steps_per_antenna_rev; i++) {
  //  rotateStepperOneStep(CLOCKWISE);
   // delay(3);
  //}
  //delay(1000);
  //getAntennaOrientation();
  
}
