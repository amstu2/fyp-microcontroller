#define LED_PIN 13
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

void checkUARTRecv() {
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

void parseReceivedData() {
  desired_azimuth = (float)((received_buffer[BUFFER_INDEX_AZ_HUNDRED]-'0')*100) +  (float)((received_buffer[BUFFER_INDEX_AZ_TEN]-'0')*10) + (float)((received_buffer[BUFFER_INDEX_AZ_ONE]-'0')) + (((float)(received_buffer[BUFFER_INDEX_AZ_DEC]-'0'))/10);
  desired_elevation = (float)((received_buffer[BUFFER_INDEX_EL_TEN]-'0')*10) + (float)((received_buffer[BUFFER_INDEX_EL_ONE]-'0')) + (((float)(received_buffer[BUFFER_INDEX_EL_DEC]-'0'))/10);
  new_command_received = false;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  checkUARTRecv();
  if(new_command_received) {
    parseReceivedData();
    Serial.println(desired_azimuth);
    Serial.println(desired_elevation);
  }
  
}
