#include <SoftwareSerial.h>

int pul = 3;
int dir = 4;
int en = 5;
int inp = 6;
int alm = 7;
String input;

int alm_buf = 0;
int inp_buf = 0;

// period
int T = 400;

void setup() {
  // put your setup code here, to run once:

pinMode(pul, OUTPUT);
pinMode(dir, OUTPUT);
pinMode(en, OUTPUT);
pinMode(inp, INPUT);
pinMode(alm, INPUT);
Serial.begin(9600);

digitalWrite(dir, 0);
//analogWrite(pul, 127);
digitalWrite(en, 1);

}


void loop() {
  digitalWrite(pul, 0);
  delayMicroseconds(T/2);
  digitalWrite(pul, 1);
  delayMicroseconds(T/2);
}