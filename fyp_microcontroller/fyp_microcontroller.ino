#define LED_PIN           13
#define LINEAR_ACT_ENABLE 4
#define LINEAR_ACT_IN1    5
#define LINEAR_ACT_IN2    6
#define STEPPER_ENABLE1   7
#define STEPPER_IN1_1     8
#define STEPPER_IN1_2     9
#define STEPPER_ENABLE2   10
#define STEPPER_IN2_1     11
#define STEPPER_IN2_2     12   

#define BUFFER_INDEX_AZ_HUNDRED 0
#define BUFFER_INDEX_AZ_TEN 1
#define BUFFER_INDEX_AZ_ONE 2
#define BUFFER_INDEX_AZ_DEC 3
#define BUFFER_INDEX_EL_TEN 4
#define BUFFER_INDEX_EL_ONE 5
#define BUFFER_INDEX_EL_DEC 6

float desired_azimuth = 0.0;
float desired_elevation = 0.0;

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
  digitalWrite(STEPPER_IN1_1, LOW);
  digitalWrite(STEPPER_IN1_2, HIGH);
  digitalWrite(STEPPER_ENABLE2, LOW);
  digitalWrite(STEPPER_IN2_1, LOW);
  digitalWrite(STEPPER_IN2_2, HIGH);
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

void checkUARTRecv() 
{
  char received_data;
  while((Serial.available() > 0) && new_command_received == false) {
    received_data = Serial.read();
    if(received_data == start_char) {
      message_being_received = true;
    }
    else {
      if(message_being_received) {
        if((received_data != end_char)) {
          received_buffer[buffer_index] = received_data;
          buffer_index++;
          if(buffer_index > buffer_size) {
            buffer_index = buffer_size - 1;
            digitalWrite(LED_PIN, HIGH);
          }
        }
        else {
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
