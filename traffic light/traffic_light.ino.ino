#include <SoftwareSerial.h>

SoftwareSerial mySerial(12, 13); // RX, TX
int red = 11 ;
int yellow = 10;
int green = 9;


void setup()
{
  pinMode(red, OUTPUT);
  pinMode(yellow, OUTPUT);
  pinMode(green, OUTPUT);
  
  mySerial.begin(9600);
  mySerial.println("Traffic Light");
  mySerial.println("1=RED");
  mySerial.println("2=YELLOW");
  mySerial.println("3=GREEN");
  mySerial.println("4 and 5 TEST MODE");
}

void loop()
{
  int no = mySerial.read();
  if (no == '1')
  {
    digitalWrite(red,HIGH);
    digitalWrite(yellow, LOW);
    digitalWrite(green, LOW);
    if (digitalRead(red) == HIGH)
    {
      mySerial.println("RED");
    }

  }
  if (no == '2')
  {
    digitalWrite(yellow, HIGH);
    digitalWrite(red,LOW);
    digitalWrite(green, LOW);
    if (digitalRead(yellow) == HIGH)
    {
      mySerial.println("YELLOW");
    }
  }
  if (no == '3')
  {
    digitalWrite(green, HIGH);
    digitalWrite(red,LOW);
    digitalWrite(yellow, LOW);
    if (digitalRead(green) == HIGH)
    {
      mySerial.println("GREEN");
    }

  }

  if (no == '4')
  {
    digitalWrite(red, LOW);
    digitalWrite(yellow, LOW);
    digitalWrite(green, LOW);
    mySerial.println("TEST MODE ALL LIGHTS OFF");
  }
  if (no == '5')
  {
    digitalWrite(red, HIGH);
    digitalWrite(yellow, HIGH);
    digitalWrite(green, HIGH);
    mySerial.println("TEST MODE ALL LIGHTS ON");
  }
}
