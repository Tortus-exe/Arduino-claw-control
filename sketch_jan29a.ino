#include <Servo.h>

//  [[PINS]]
#define closenessPins 10  // attach LEDs from furthest to closest:
/*  OG setup:
 *    Green 1: 10 (furthest)
 *    Green 2: 11
 *    Yellow: 12
 *    Red: 13 (closest, triggers claw)
 */
#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 3 //attach pin D3 Arduino to pin Trig of HC-SR04
#define servoSignal 9 //servo on D7
#define joystickSW 5 //escape button
#define joystickY A1  //joystick VRy
#define joystickX A0  // joystick VRx

//  [[ROLLING AVERAGE VALUES]]
#define MULTIPLIER  16
#define CACHELENGTH 5

//  [[SERVO STATES]]
#define CLOSED 180
#define OPEN 0
#define CLOSEDTIME 5000

//  [[FLASHING STATES]]
#define FLASHTIMER 100
#define DEADZONE 70

typedef struct {
  int center_y;
  int center_x;
  
  int y;
  int x;  // X and y coords
  double angle;  //angle of the input
  double r;  //magnitude of the input
} JoyCoord;

double distance;     // for output of ping()
Servo outputServo;  //servo object
JoyCoord* input = (JoyCoord*)malloc(sizeof(JoyCoord));

int thresholds[] = {20, 15, 10, 5}; // closer to closest
int selectedThreshold = 0;

void writeCalibration(int pins, double distance);
void calibrateJoy(JoyCoord* joyToCalibrate, int Y_pin, int X_pin);
void readJoy(JoyCoord* output, int Y_pin, int X_pin);
double ping();
double smoothPing();
void writeClosenessPins(int dist, int pins);
void closeClaw(int dist);

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  pinMode(joystickSW, INPUT_PULLUP);  //inverted input
  pinMode(closenessPins, OUTPUT);
  pinMode(closenessPins+1, OUTPUT);
  pinMode(closenessPins+2, OUTPUT);
  pinMode(closenessPins+3, OUTPUT);
  outputServo.attach(servoSignal);
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed

  calibrateJoy(input, joystickY, joystickX);

  do {
    //begin calibration loop
    distance = smoothPing();
    writeClosenessPins(distance, closenessPins);
    readJoy(input, joystickY, joystickX);
    writeCalibration(closenessPins, distance);
    delay(100);
  } while(digitalRead(joystickSW));
  Serial.print(thresholds[0]);
  Serial.print(" ");
  Serial.print(thresholds[1]);
  Serial.print(" ");
  Serial.print(thresholds[2]);
  Serial.print(" ");
  Serial.println(thresholds[3]);
}
void loop() {
  distance = smoothPing();
  
  writeClosenessPins(distance, closenessPins);
  if(distance < thresholds[3])
    closeClaw(distance);
  Serial.print(digitalRead(joystickSW));
  Serial.print(" Distance: ");
  Serial.println(distance);
  delay(50);  //so we don't overping
}


void writeCalibration(int pins, double distance) {
  static bool directionPressed = false;
  static int timer = FLASHTIMER;
  static bool on = false;
  static int slxn = 0;
  //flash the currently selected 
  if(timer--) {
    timer=(distance < thresholds[slxn]) ? FLASHTIMER : FLASHTIMER/2;
    on=!on;
    digitalWrite(pins+slxn, on);
  } 
  //input logic
  if(!directionPressed) {
    if(input->r>DEADZONE) {
      if(abs(input->x)<abs(input->y)) {
        //horizontal input
        if(input->x>0) {(++slxn)&=3;}
        else {(--slxn)&=3;}
      } else {
        //vertical input
        if(input->y>0) {++thresholds[slxn];}
        else {--thresholds[slxn];}
        Serial.println(input->y);
      }
    }
    directionPressed = true;
  }
  directionPressed=!(input->r<DEADZONE);
}


void closeClaw(int dist) {
  int timer = 0;
  //when entering this subroutine, polling will be suspended. 
  outputServo.write(CLOSED);
  do {
    delay(10);
  }while(++timer<CLOSEDTIME/10 && digitalRead(joystickSW));
  outputServo.write(OPEN);
}

void writeClosenessPins(int dist, int pins) {
  for(int i = 0; i < 4; ++i) {
    if(dist < thresholds[i]) {
      digitalWrite(pins+i, HIGH);
    } else {
      digitalWrite(pins+i, LOW);
    }
  }//clear all the pins
}

double smoothPing() {
  //use a moving average to smoothen the ping
  static int remainingCacheSpaces = CACHELENGTH;
  static double average = 0;
  
  average -= average/CACHELENGTH;
  average += ping()/CACHELENGTH;
  if(remainingCacheSpaces!=0) {
    remainingCacheSpaces--; //remove a cache space if we aren't already full
  }
  return average;
}

double ping() {
    // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  double distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  return distance;
}

void calibrateJoy(JoyCoord* joyToCalibrate, int Y_pin, int X_pin) {
    joyToCalibrate->center_y = analogRead(Y_pin);
    joyToCalibrate->center_x = analogRead(X_pin);
}

void readJoy(JoyCoord* output, int Y_pin, int X_pin) {
  // read the analog input:
  output->y = analogRead(Y_pin)-output->center_y;
  output->x = analogRead(X_pin)-output->center_x;
  output->r = sqrt(pow(output->y,2) + pow(output->x,2));
  output->angle = atan((double)output->y/output->x);
}
