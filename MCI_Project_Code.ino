//#include <QTRSensors.h>

//>>>>>>> For Arduino
// Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;
// Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;
// UltraSonic connections
//int Trig = 5
//int Echo = 13

int bluetoothCommand;


// IR Sensor class
//QTRSensors qtr;
// number of sensors
const uint8_t SensorCount = 5;
// values read from the sensor array
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];


// values required for PID Control and some motor constants
uint16_t position;
float Kp = 0;
float Ki = 0;
float Kd = 0;
uint8_t multiP = 1;
uint8_t multiI = 1;
uint8_t multiD = 1;
float Pvalue;
float Ivalue;
float Dvalue;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 230;




void setup() {
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  // Bluetooth interfacing
  Serial.begin(9600);


  // Configure the IR Sensors
//  qtr.setTypeAnalog();
//  qtr.setSensorPins((const uint8_t[]){ 26, 27, 14, 12, 13 }, SensorCount);
//  delay(500);
//  for (uint16_t i = 0; i < 400; i++) {
//    qtr.calibrate();
//  }
//  for (uint8_t i = 0; i < SensorCount; i++) {
//    threshold[i] = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i]) / 2;
//    Serial.print(threshold[i]);
//    Serial.print("  ");
//  }
//  Serial.println();

  delay(1000);
}

void loop() {

  while (Serial.available() > 0) {
    // read the value received through the bluetooth module into the variable
    bluetoothCommand = Serial.read();

    // 0 --> stop
    // 1 --> move forward
    // 2 --> move backwards
    // 3 --> turn right
    // 4 --> turn left

    // print the value onto the serial monitor for debugging purposes
    Serial.println(bluetoothCommand);
  }

  setSpeedM1(150);
  setSpeedM2(150);
  forwardM1();
  forwardM2();


  // set_Speed(50);
  // forward();
  // delay(2000);
  //  right();
  //  delay(200);

  //  set_Speed(0);
  //  delay(100);
  //  backward();
  //  delay(2000);
  //  left();
  //  delay(200);
//  moveCar();
}

// Car Moving Function
//void moveCar(){
//  // read calibrated sensor values and obtain a measure of the line position
//  // from 0 to 4000 (for a white line, use readLineWhite() instead)
//  position = qtr.readLineBlack(sensorValues);
//  error = 2000 - position;
//  while(sensorValues[0]>=980 && sensorValues[1]>=980 && sensorValues[2]>=980 && sensorValues[3]>=980 && sensorValues[4]>=980){ // A case when the line follower leaves the line
//    if(previousError>0){       //Turn left if the line was to the left before
//      motor_drive(-230,230);
//    }
//    else{
//      motor_drive(230,-230); // Else turn right
//    }
//    position = qtr.readLineBlack(sensorValues);
//  }
//  
//  PID_Linefollow(error);
//  //PID_Linefollow(error);
//}


// PID Controller line following algorithm
void PID_Linefollow(int error) {
  P = error;
  I += error;
  D = error - previousError;

  Pvalue = (Kp / pow(10, multiP)) * P;
  Ivalue = (Ki / pow(10, multiI)) * I;
  Dvalue = (Kd / pow(10, multiD)) * D;

  float PIDvalue = Pvalue + Ivalue + Dvalue;
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < -255) {
    lsp = -255;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < -255) {
    rsp = -255;
  }
  motor_drive(lsp, rsp);
}

void motor_drive(int left, int right) {

  if (right > 0) {
    setSpeedM2(right);
    forwardM2();
  } else {
    setSpeedM2(right);
    backwardM2();
  }


  if (left > 0) {
    setSpeedM1(left);
    forwardM1();
  } else {
    setSpeedM1(left);
    backwardM1();
  }
}



void setSpeedM1(int a) {
  analogWrite(enA, a);
}
void setSpeedM2(int a) {
  analogWrite(enB, a);
}

void forwardM1() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}
void forwardM2() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void backwardM1() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}
void backwardM2() {
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void deadStop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}



//int dist () {
//  digitalWrite(Trig, LOW);
//  delayMicroseconds(2);
//  digitalWrite(Trig, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(Trig, LOW);
//  duration = pulseIn(Echo, HIGH);
//  distance = duration * 0.0344 / 2;
//  delay(100);
//  return distance;
//}

// void right() {
//   digitalWrite(in1, LOW);
//   digitalWrite(in2, HIGH);
//   digitalWrite(in3, HIGH);
//   digitalWrite(in4, LOW);
// }

// void left() {
//   digitalWrite(in1, HIGH);
//   digitalWrite(in2, LOW);
//   digitalWrite(in3, LOW);
//   digitalWrite(in4, HIGH);
// }
// void move(int command, int speed)
// {
//   if (command == 0)
//   {
//     set_Speed(0);
//     stop();
//   }
//   else if (command == 1)
//   {
//     set_Speed(speed);
//     forward();
//   }
//   else if (command == 2)
//   {
//     set_Speed(speed);
//     backward();
//   }
//   else if (command == 3)
//   {
//     set_Speed(speed);
//     right();
//   }
//   else if (command == 3)
//   {
//     set_Speed(speed);
//     left();
//   }
// }
