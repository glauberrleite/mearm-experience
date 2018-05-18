#include <Servo.h> 
Servo base;
Servo left;
Servo right;
Servo hand;
int basePin = 3;
int rightPin = 5;
int leftPin = 6;
int handPin = 9;
int startingTheta[] = {0, 0, 0};
int * theta;

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void setup(){

    Serial.begin(9600);
    
    base.attach(basePin);
    left.attach(leftPin);
    right.attach(rightPin);
    hand.attach(handPin);

    theta = startingTheta;
    setTheta(theta);
    setHand(0);
}

void setBase(int angle) {
  if (angle > 60)
    angle = 60;
  else if (angle < -60)
    angle = -60;
  
  int value = map(angle, -60, 60, 36, 143);
  
  base.write(value);
}

void setRight(int angle) {
  if (angle > 180)
    angle = 180;
  else if (angle < -45)
    angle = -45;
  
  int value = map(angle, -45, 180, 10, 150);

  right.write(value);
}

void setLeft(int angle) {
  if (angle > 20)
    angle = 20;
  else if (angle < -70)
    angle = -70;
  
  int value = map(angle, -70, 20, 70, 180);

  left.write(value);
}

void setTheta(int theta[]) {  
  setBase(theta[0]);
  delay(500);
  setRight(theta[1]);
  delay(500);
  setLeft(theta[2]);
  delay(500);
}

void setHand(int closure) {
  if (closure > 100)
    closure = 100;
  else if (closure < 0)
    closure = 0;
  
  int value = map(closure, 0, 100, 110, 145);

  hand.write(value);
}

String readSerialString() {
  
  String content = "";
  char character;

  while (Serial.available() > 0) {
    character = Serial.read();
    if (character != '\n') {
      content.concat(character);
    }

    delay(20);
  }

  return content;
}

void loop(){
    if (Serial.available() > 0) { 

        String content = readSerialString();

        String cmd = getValue(content, ',', 0);

        String show = "";
        
        if (cmd.equals("h")) {
          if (getValue(content, ',', 1).equals("c")) {
            show = "Close Hand";
            setHand(100);
          } else {
            show = "Open Hand";
            setHand(0);
          }          
          
        } else if (cmd.equals("t")) {
          theta[0] = getValue(content, ',', 1).toInt();
          theta[1] = getValue(content, ',', 2).toInt();
          theta[2] = getValue(content, ',', 3).toInt();

          show = "Angles moved to: t1 = ";
          show.concat(theta[0]);
          show.concat(", t2 = ");
          show.concat(theta[1]);
          show.concat(", t3 = ");
          show.concat(theta[2]);

          setTheta(theta);
        } else show = "Received an unkown command";

        Serial.println(show);
    }
}
