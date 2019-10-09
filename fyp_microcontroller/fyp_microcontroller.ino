// TODO: MAKE SURE STEPPER MOTOR CONNECTIONS ARE CORRECT
// ASSUMED STEPPER IN RAIL 1 IS A/NOT_A

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

#define STEPS_PER_STEPPER_REV 200
#define STEPPER_TEETH         10
#define DRIVEN_GEAR_TEETH     42
#define GEARBOX_GEAR_RATIO    0.04 //25 revs on input shaft --> 1 rev on output shaft

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

float desired_azimuth = 0.0;
float desired_elevation = 0.0;

volatile byte phase_number = 1;
volatile int step_number = 0;
boolean stepper_motor_enabled = false;
int steps_per_antenna_rev;

const byte buffer_size = 7;
char received_buffer[buffer_size];
int buffer_index = 0;
boolean new_command_received = false;
boolean message_being_received = false;
const char start_char = '<';
const char end_char = '>';

void setupPins()
{
  pinMode(LED_PIN,OUTPUT);
  pinMode(LINEAR_ACT_ENABLE,OUTPUT); 
  pinMode(LINEAR_ACT_IN1,OUTPUT); 
  pinMode(LINEAR_ACT_IN2,OUTPUT);
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

int calculateStepsPerRev(int steps_per_stepper_rev, int num_stepper_teeth, int num_driven_teeth, float gearbox_gear_ratio)
{
  
  float belt_system_gear_ratio = (float) num_stepper_teeth / num_driven_teeth; 
  int steps_per_driven_rev = steps_per_stepper_rev / belt_system_gear_ratio;
  int steps_per_rev = steps_per_driven_rev * (float)(1/gearbox_gear_ratio);
  return steps_per_rev;
}

void enableLinearActuator()
{
  digitalWrite(LINEAR_ACT_ENABLE, HIGH);
}

void extendLinearActuator()
{
  enableLinearActuator();
  digitalWrite(LINEAR_ACT_IN1, HIGH);
  digitalWrite(LINEAR_ACT_IN2, LOW);
}

void retractLinearActuator()
{
  enableLinearActuator();
  digitalWrite(LINEAR_ACT_IN1, LOW);
  digitalWrite(LINEAR_ACT_IN2, HIGH);
}

void disableLinearActuator()
{
  digitalWrite(LINEAR_ACT_ENABLE, LOW);
}

void enableStepperMotor()
{
  digitalWrite(STEPPER_ENABLE1, HIGH);
  digitalWrite(STEPPER_ENABLE2, HIGH);
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
  steps_per_antenna_rev = calculateStepsPerRev(STEPS_PER_STEPPER_REV, STEPPER_TEETH, DRIVEN_GEAR_TEETH, GEARBOX_GEAR_RATIO);
}

void loop() 
{
  // put your main code here, to run repeatedly:
  checkUARTRecv();
  if(new_command_received) {
    parseReceivedData();
    //Serial.println(desired_azimuth);
    //Serial.println(desired_elevation);   
  }
  
  
}
