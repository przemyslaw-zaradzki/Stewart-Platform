#include <ArduinoJson.h>
#include <Servo.h>

#define loop_time 40
#define analog_read_time 3
using namespace std;


Servo s1, s2, s3, s4, s5, s6;
int a[6] = {90, 90, 90, 90, 90, 90};
const int min_angle = 45;
const int max_angle = 135;


int ball_pos_x[3];
int ball_pos_y[3];
int ball_median_x;
int ball_median_y;
int i = 0;
int time_int = 0;

unsigned long time_start, time_stop;

DynamicJsonDocument doc(256);
DeserializationError error;

char raport[256];

void setup() {
  Serial.begin(57600);
  Serial.setTimeout(loop_time-12);
  s6.attach(6);
  s5.attach(7);
  s4.attach(9);
  s3.attach(10);
  s2.attach(11);
  s1.attach(4);

  s1.write(a[0]);
  s2.write(a[1]);
  s3.write(a[2]);
  s4.write(a[3]);
  s5.write(a[4]);
  s6.write(a[5]);

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
}


void loop() {
  time_start = millis();
  error = deserializeJson(doc, Serial);
  
  digitalWrite(2, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1);
  ball_pos_x[0] = analogRead(A1);    
  digitalWrite(2, LOW);
  delay(1);
  digitalWrite(3, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1);
  ball_pos_y[0] = analogRead(A2);
  digitalWrite(3, LOW);
  delay(1);

  digitalWrite(2, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1);
  ball_pos_x[1] = analogRead(A1);    
  digitalWrite(2, LOW);
  delay(1);
  digitalWrite(3, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1);
  ball_pos_y[1] = analogRead(A2);
  digitalWrite(3, LOW);
  delay(1);

  digitalWrite(2, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1);
  ball_pos_x[2] = analogRead(A1);    
  digitalWrite(2, LOW);
  delay(1);
  digitalWrite(3, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1);
  ball_pos_y[2] = analogRead(A2);
  digitalWrite(3, LOW);
  delay(1);


  if(ball_pos_x[0]>=ball_pos_x[1] and ball_pos_x[0]<=ball_pos_x[2]){
    ball_median_x=ball_pos_x[0];
  }
  if(ball_pos_x[0]<=ball_pos_x[1] and ball_pos_x[0]>=ball_pos_x[2]){
    ball_median_x=ball_pos_x[0];
  }

  if(ball_pos_x[1]>=ball_pos_x[0] and ball_pos_x[1]<=ball_pos_x[2]){
    ball_median_x=ball_pos_x[1];
  }
  if(ball_pos_x[1]<=ball_pos_x[0] and ball_pos_x[1]>=ball_pos_x[2]){
    ball_median_x=ball_pos_x[1];
  }

  if(ball_pos_x[2]>=ball_pos_x[0] and ball_pos_x[2]<=ball_pos_x[1]){
    ball_median_x=ball_pos_x[2];
  }
  if(ball_pos_x[2]<=ball_pos_x[0] and ball_pos_x[2]>=ball_pos_x[1]){
    ball_median_x=ball_pos_x[2];
  }


  if(ball_pos_y[0]>=ball_pos_y[1] and ball_pos_y[0]<=ball_pos_y[2]){
    ball_median_y=ball_pos_y[0];
  }
  if(ball_pos_y[0]<=ball_pos_y[1] and ball_pos_y[0]>=ball_pos_y[2]){
    ball_median_y=ball_pos_y[0];
  }

  if(ball_pos_y[1]>=ball_pos_y[0] and ball_pos_y[1]<=ball_pos_y[2]){
    ball_median_y=ball_pos_y[1];
  }
  if(ball_pos_y[1]<=ball_pos_y[0] and ball_pos_y[1]>=ball_pos_y[2]){
    ball_median_y=ball_pos_y[1];
  }

  if(ball_pos_y[2]>=ball_pos_y[0] and ball_pos_y[2]<=ball_pos_y[1]){
    ball_median_y=ball_pos_y[2];
  }
  if(ball_pos_y[2]<=ball_pos_y[0] and ball_pos_y[2]>=ball_pos_y[1]){
    ball_median_y=ball_pos_y[2];
  }
  
  // Opróżnienie bufora
  while(Serial.available() > 0) {
    Serial.read();
  }

  time_stop = millis() - time_start;
  if (time_stop >  32767 ) {
    time_int = 32767;
  }
  else {
    time_int = int(time_stop);
  }

  switch (error.code()) {
    case DeserializationError::Ok:
  
      a[0] = doc["a1"];
      a[1] = doc["a2"];
      a[2] = doc["a3"];
      a[3] = doc["a4"];
      a[4] = doc["a5"];
      a[5] = doc["a6"];

      if(
        a[0]>min_angle and a[0]<max_angle and
        a[1]>min_angle and a[1]<max_angle and
        a[2]>min_angle and a[2]<max_angle and
        a[3]>min_angle and a[3]<max_angle and
        a[4]>min_angle and a[4]<max_angle and
        a[5]>min_angle and a[5]<max_angle
        )
        {
          s1.write(a[0]);
          s2.write(a[1]);
          s3.write(a[2]);
          s4.write(a[3]);
          s5.write(a[4]);
          s6.write(a[5]);
        }
            
      sprintf(raport,
            "{\"a1\":%i,\"a2\":%i,\"a3\":%i,\"a4\":%i,\"a5\":%i,\"a6\":%i,\"x\":%i,\"y\":%i,\"i\":%i,\"t\":%i,\"e\":\"DS\"}",
            a[0], a[1], a[2], a[3], a[4], a[5], ball_median_x, ball_median_y, i, time_int);
      Serial.println(raport);
      break;
    case DeserializationError::EmptyInput:
      sprintf(raport,
            "{\"a1\":%i,\"a2\":%i,\"a3\":%i,\"a4\":%i,\"a5\":%i,\"a6\":%i,\"x\":%i,\"y\":%i,\"i\":%i,\"t\":%i,\"e\":\"EI\"}",
            a[0], a[1], a[2], a[3], a[4], a[5], ball_median_x, ball_median_y, i, time_int);
      Serial.println(raport);
      break;
    case DeserializationError::IncompleteInput:
      sprintf(raport,
            "{\"a1\":%i,\"a2\":%i,\"a3\":%i,\"a4\":%i,\"a5\":%i,\"a6\":%i,\"x\":%i,\"y\":%i,\"i\":%i,\"t\":%i,\"e\":\"II\"}",
            a[0], a[1], a[2], a[3], a[4], a[5], ball_median_x, ball_median_y, i, time_int);
      Serial.println(raport);
      break;
    case DeserializationError::InvalidInput:
      sprintf(raport,
            "{\"a1\":%i,\"a2\":%i,\"a3\":%i,\"a4\":%i,\"a5\":%i,\"a6\":%i,\"x\":%i,\"y\":%i,\"i\":%i,\"t\":%i,\"e\":\"IV\"}",
            a[0], a[1], a[2], a[3], a[4], a[5], ball_median_x, ball_median_y, i, time_int);
      Serial.println(raport);
      break;
    case DeserializationError::NoMemory:
      sprintf(raport,
            "{\"a1\":%i,\"a2\":%i,\"a3\":%i,\"a4\":%i,\"a5\":%i,\"a6\":%i,\"x\":%i,\"y\":%i,\"i\":%i,\"t\":%i,\"e\":\"NM\"}",
            a[0], a[1], a[2], a[3], a[4], a[5], ball_median_x, ball_median_y, i, time_int);    
      Serial.println(raport);
      break;
    case DeserializationError::TooDeep:
      sprintf(raport,
            "{\"a1\":%i,\"a2\":%i,\"a3\":%i,\"a4\":%i,\"a5\":%i,\"a6\":%i,\"x\":%i,\"y\":%i,\"i\":%i,\"t\":%i,\"e\":\"TD\"}",
            a[0], a[1], a[2], a[3], a[4], a[5], ball_median_x, ball_median_y, i, time_int);    
      Serial.println(raport);
      break;
    default:
      sprintf(raport,
            "{\"a1\":%i,\"a2\":%i,\"a3\":%i,\"a4\":%i,\"a5\":%i,\"a6\":%i,\"x\":%i,\"y\":%i,\"i\":%i,\"t\":%i,\"e\":\"D\"}",
            a[0], a[1], a[2], a[3], a[4], a[5], ball_median_x, ball_median_y, i, time_int);    
      Serial.println(raport);
      break;
  }

  i++;
  if (i > 1000) {
    i = 0;
  }

  time_stop = millis() - time_start;
  if (time_stop >  32767 ) {
    time_int = 32767;
  }
  else {
    time_int = int(time_stop);
  }
  if (time_int < loop_time) {
    delay(loop_time-time_int);
  }
}
