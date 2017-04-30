#define FULL_SPEED 135
#define TURN_SPEED 130
#define TURN_TICK 68
#define UTURN_TICK 187
#define NOWALLS 20
#define SKRT 75
#define SKRRT 270

//ML 150 175
//MR 100 130


double Kp = .25;
double Kd = .0625;

int offSet = 0;

char ch = ' ';

byte state = LOW;

long randNum = 0;

int ranSeed = 28;

int p = 0;
int d = 0;
int totalError = 0;
int newError = 0;
int oldError = 0;

int receiverL = A8;
int emitterL = 23;

int receiverML = A5;
int emitterML = 18;

int receiverM = A2;
int emitterM = 17;

int receiverMR = A0;
int emitterMR = 15;

int receiverR = A6;
int emitterR = 21;

int errorL = 0;
int errorML = 0;
int errorM = 0;
int errorMR = 0;
int errorR = 0;

int centerL = 0;
int centerML = 0;
int centerM = 0;
int centerMR = 0;
int centerR = 0;

int sensorReadingL;
int sensorReadingML;
int sensorReadingM;
int sensorReadingMR;
int sensorReadingR;

int motorENL = 0;
int motorENR = 13;

int motorLogicL1 = 3;
int motorLogicL2 = 4;

int motorLogicR1 = 9;
int motorLogicR2 = 10;

int encoderLA = 2;
int encoderRA = 12;

volatile long tickRA = 0;
volatile long tickLA = 0;

bool hasFrontWall() {
  regulateSensorM();
  if(sensorReadingM > 20)
    return true;
  else
    return false; 
}

bool hasLeftWall() {
  regulateSensorL();
  if(sensorReadingL > NOWALLS)
    return true;
   else
    return false;
}

bool hasRightWall() {
  regulateSensorR();
  if (sensorReadingR > NOWALLS)
    return true;
  else
    return false;
}

void printTicks() {
  Serial3.print("RA: ");
  Serial3.print(tickRA);
  Serial3.print(" LA: ");
  Serial3.print(tickLA);
  Serial3.println(" ");
}

void printIR() {
  
  Serial3.print("L: ");
  Serial3.print(sensorReadingL);
  Serial3.print(" ML: ");
  Serial3.print(sensorReadingML);
  Serial3.print(" M: ");
  Serial3.print(sensorReadingM);
  Serial3.print(" MR: ");
  Serial3.print(sensorReadingMR);
  Serial3.print(" R: ");
  Serial3.print(sensorReadingR);
  Serial3.println(" ");
  delay(100);
}

void getIR() {

  digitalWrite(emitterL, HIGH);
  digitalWrite(emitterML, HIGH);
  digitalWrite(emitterM, HIGH);
  digitalWrite(emitterMR, HIGH);
  digitalWrite(emitterR, HIGH);

  sensorReadingL = analogRead(receiverL);
  sensorReadingMR = analogRead(receiverMR);
  sensorReadingM = analogRead(receiverM);
  sensorReadingML = analogRead(receiverML);
  sensorReadingR = analogRead(receiverR);

  sensorReadingL = sensorReadingL - errorL;
  sensorReadingML = sensorReadingML - errorML - 30;
  sensorReadingM = sensorReadingM - errorM;
  sensorReadingMR = sensorReadingMR - errorMR;
  sensorReadingR = sensorReadingR - errorR;
  
}

void IRinit() {
  int temp1 = 0;
  int temp2 = 0;
  centerL = analogRead(receiverL) - errorL;
  centerML = analogRead(receiverML) - errorML;
  centerM = analogRead(receiverM) - errorM;
  centerMR = analogRead(receiverMR) - errorMR;
  centerR = analogRead(receiverR) - errorR;
  
  if (sensorReadingML > sensorReadingMR) {
    
  }
}

void removeNoise(int &errorL, int errorML, int &errorM, int errorMR, int &errorR) {

  digitalWrite(emitterL, LOW);
  digitalWrite(emitterML, LOW);
  digitalWrite(emitterM, LOW);
  digitalWrite(emitterMR, LOW);
  digitalWrite(emitterR, LOW);

  errorL = analogRead(receiverL);
  errorML = analogRead(receiverML);
  errorM = analogRead(receiverM);
  errorMR = analogRead(receiverMR);
  errorR = analogRead(receiverR);

  delay(100);
}

void regulateSensorL() {
  digitalWrite(emitterL, HIGH);
  sensorReadingL = analogRead(receiverL) - errorL;
  sensorReadingL = sensorReadingL - centerL;
  if(sensorReadingL < 0)
    sensorReadingL = ~sensorReadingL + 1;
}

void regulateSensorML() {
  digitalWrite(emitterML, HIGH);
  sensorReadingML = analogRead(receiverML) - errorML;
  sensorReadingML = sensorReadingML - centerML;
  sensorReadingML = sensorReadingML - 35;
  if(sensorReadingML < 0)
    sensorReadingML = ~sensorReadingML + 1;
}

void regulateSensorM() {
  digitalWrite(emitterM, HIGH);
  sensorReadingM = analogRead(receiverM) - errorM;
  if(sensorReadingM < 0)
    sensorReadingM = ~sensorReadingM + 1;
}

void regulateSensorMR() {
  digitalWrite(emitterMR, HIGH);
  sensorReadingMR = analogRead(receiverMR) - errorMR;
  sensorReadingMR = sensorReadingMR - centerMR;
  if(sensorReadingMR < 0)
    sensorReadingMR = ~sensorReadingMR + 1;
}

void regulateSensorR() {
  digitalWrite(emitterR, HIGH);
  sensorReadingR = analogRead(receiverR) - errorR;
  sensorReadingR = sensorReadingR - centerR;
  if(sensorReadingR < 0)
    sensorReadingR = ~sensorReadingR + 1;
}

void regulateAll() {
  regulateSensorL();
  regulateSensorML();
  regulateSensorM();
  regulateSensorMR();
  regulateSensorR();
}

void forward(int spdL, int spdR) {    
   //RIGHT
   digitalWrite(motorENR, HIGH);
   analogWrite(motorLogicR1, spdR);
   analogWrite(motorLogicR2, 0);

   //LEFT
   digitalWrite(motorENL, HIGH);
   analogWrite(motorLogicL1, 0);
   analogWrite(motorLogicL2, spdL);
}

void breakMotors() {
    //RIGHT
    digitalWrite(motorENR, HIGH);
    analogWrite(motorLogicR1, HIGH);
    analogWrite(motorLogicR2, HIGH);

    //LEFT
    digitalWrite(motorENL, HIGH);
    analogWrite(motorLogicL1, HIGH);
    analogWrite(motorLogicL2, HIGH);
}

void disableMotors() {
   digitalWrite(motorENR, LOW);
   digitalWrite(motorENL, LOW);
}

void PD() {
  
  regulateAll();
  
  if(hasRightWall() && hasLeftWall()) {
    if (sensorReadingML > sensorReadingMR) {
      newError = sensorReadingML - sensorReadingMR;
      p = newError * Kp;
      d = (newError - oldError) * Kd;
      totalError = p + d;
      forward(FULL_SPEED, FULL_SPEED - totalError);
     }
     else if (sensorReadingML < sensorReadingMR) {
      newError = sensorReadingMR - sensorReadingML;
      p = newError * Kp;
      d = (newError - oldError) * Kd;
      totalError = p + d;
      forward(FULL_SPEED - totalError, FULL_SPEED);
     }
   }
}

void moveUp(int ticksLA, int ticksRA) {

  //regulateSensorM();
  
  while(tickLA < ticksLA || tickRA < ticksRA) {
  
    regulateSensorM();

    if (sensorReadingM > 150) { //DON'T HIT THE WALL
        breakMotors();
        disableMotors();
        break;
    }
  
    //RIGHT
    digitalWrite(motorENR, HIGH);
    analogWrite(motorLogicR1, FULL_SPEED);
    analogWrite(motorLogicR2, 0);

    //LEFT
    digitalWrite(motorENL, HIGH);
    analogWrite(motorLogicL1, 0);
    analogWrite(motorLogicL2, FULL_SPEED);
   }
  
  breakMotors();
  disableMotors();
}

void left90(int eLA, int eRA) {
  while (tickRA < eRA || tickLA < eLA) {
    digitalWrite(motorENR, HIGH);
    analogWrite(motorLogicR1, 130);
    analogWrite(motorLogicR2, 0);
    
    digitalWrite(motorENL, HIGH);
    analogWrite(motorLogicL1, 130);
    analogWrite(motorLogicL2, 0);
 }
 disableMotors();
 delay(150);
 printTicks();
}

void right90(int eLA, int eRA) {
  while (tickRA < eRA || tickLA < eLA) {
    digitalWrite(motorENR, HIGH);
    analogWrite(motorLogicR1, 0);
    analogWrite(motorLogicR2, 130);
    
    digitalWrite(motorENL, HIGH);
    analogWrite(motorLogicL1, 0);
    analogWrite(motorLogicL2, 130);
  }
  disableMotors();
  delay(150);
  printTicks();
}

void uturn(int eLA, int eRA) {
   while (tickRA < eRA || tickLA < eLA) {
    digitalWrite(motorENR, HIGH);
    analogWrite(motorLogicR1, 0);
    analogWrite(motorLogicR2, 130);
    
    digitalWrite(motorENL, HIGH);
    analogWrite(motorLogicL1, 0);
    analogWrite(motorLogicL2, 130);
  }
  disableMotors();
  delay(150);
  printTicks();
}

void resetTicks() {
  tickLA = 0;
  tickRA = 0;
}

void randomSolve() {

  regulateSensorM();
  if(sensorReadingM > 250) {
    breakMotors();
    disableMotors();
  }
  
  //dead end
  if (hasLeftWall() && sensorReadingM > 250 && hasRightWall()) {
    Serial3.println("1");
    resetTicks();
    uturn(UTURN_TICK, UTURN_TICK);
  }
  //has a left and right wall but NO front wall
  else if (hasLeftWall() && !hasFrontWall() && hasRightWall()) {
    Serial3.println("2");
    PD(); //go straight
  }
  //has a ONLY a FRONT wall
  else if (!hasLeftWall() && hasFrontWall() && !hasRightWall()) {
    Serial3.println("3");
    regulateSensorM();
    regulateSensorML();
    regulateSensorMR();
    if (sensorReadingM > 250) {
      breakMotors();
      disableMotors();
    }
    if (sensorReadingMR < 120 || sensorReadingML < 120) {
      resetTicks();
      moveUp(SKRT, SKRT);
      delay(150);
    }
    randNum = random(2);
    if (randNum == 1) {
      resetTicks();
      left90(TURN_TICK, TURN_TICK); //turn left
      resetTicks();
      moveUp(SKRRT, SKRRT);
    }
    else if (randNum == 0) {
      resetTicks();
      right90(TURN_TICK, TURN_TICK);
      resetTicks();
      moveUp(SKRRT, SKRRT);
    }
  }
  //has ONLY a RIGHT wall
  else if (!hasLeftWall() && !hasFrontWall() && hasRightWall()) {
    Serial3.println("4");
    regulateSensorML();
    if (sensorReadingML < 120) {
      Serial3.println("skrt skrt");
      resetTicks();
      moveUp(SKRT, SKRT);
      delay(150);
    }
    randNum = random(2);
    if (randNum == 1) {
      resetTicks();
      left90(TURN_TICK, TURN_TICK); //turn left
      resetTicks();
      moveUp(SKRRT, SKRRT);
    }
    else if (randNum == 0) {
      moveUp(SKRRT, SKRRT);
    }
  }
  //has ONLY a LEFT wall
  else if (hasLeftWall() && !hasFrontWall() && !hasRightWall()) {
    Serial3.println("5");
    regulateSensorMR();
    if (sensorReadingMR < 110) {
      Serial3.println("here");
      resetTicks();
      moveUp(SKRT, SKRT);
      delay(150);
    }
    randNum = random(2);
    if (randNum == 1) {
      resetTicks();
      right90(TURN_TICK, TURN_TICK);
      resetTicks();
      moveUp(SKRRT, SKRRT);
    }
    else if (randNum == 0) {
      moveUp(SKRRT, SKRRT);
    }
  }
  //has a left wall and front wall but NO right wall
  else if (hasLeftWall() && hasFrontWall() && !hasRightWall()) {
    Serial3.println("6");
    regulateSensorM();
    regulateSensorMR();
    if (sensorReadingM > 250) {
      breakMotors();
      disableMotors();
    }
    if (sensorReadingMR < 120) {
      Serial3.println("This one too");
      resetTicks();
      moveUp(SKRT, SKRT);
      delay(150);
    }
    resetTicks();
    right90(TURN_TICK, TURN_TICK); //turn right
    resetTicks();
    moveUp(SKRRT, SKRRT);
  }
  //has a front wall and right wall but NO left wall
  else if (!hasLeftWall() && hasFrontWall() && hasRightWall()) {
    Serial3.println("7");
    regulateSensorM();
    regulateSensorML();
    if(sensorReadingM > 250) {
      breakMotors();
      disableMotors();
    }
    if (sensorReadingML < 120) {
      resetTicks();
      moveUp(SKRT, SKRT);
      delay(150);
    }
    resetTicks();
    left90(TURN_TICK, TURN_TICK); //turn left
    resetTicks();
    moveUp(SKRRT, SKRRT);
  }
  //NO walls
  else if (!hasLeftWall() && !hasFrontWall() && !hasRightWall()) {
    Serial3.println("8");
    regulateSensorML();
    regulateSensorMR();
    if (sensorReadingML < 120 || sensorReadingMR < 120) {
      resetTicks();
      moveUp(SKRT, SKRT);
      delay(150);
    }
    randNum = random(2);
    if (randNum == 1) {
      resetTicks();
      left90(TURN_TICK, TURN_TICK); //turn left
      resetTicks();
      moveUp(SKRRT, SKRRT);
    }
    else if (randNum == 0) {
      resetTicks();
      right90(TURN_TICK, TURN_TICK);
      resetTicks();
      moveUp(SKRRT, SKRRT);
    }
  }

}

void riseRA() {
  tickRA++;
}

void riseLA(){
  tickLA++;
}

void setup() {

  Serial3.begin(9600);

  randomSeed(500);
  
  pinMode(receiverL, INPUT);
  pinMode(emitterL, OUTPUT);
  pinMode(receiverML, INPUT);
  pinMode(emitterML, OUTPUT);
  pinMode(receiverM, INPUT);
  pinMode(emitterM, OUTPUT);
  pinMode(receiverMR, INPUT);
  pinMode(emitterMR, OUTPUT);
  pinMode(receiverR, INPUT);
  pinMode(emitterR, OUTPUT);

  pinMode(motorENL, OUTPUT);
  pinMode(motorLogicL1, OUTPUT);
  pinMode(motorLogicL2, OUTPUT);
  digitalWrite(motorENL, LOW);

  pinMode(motorENR, OUTPUT);
  pinMode(motorLogicR1, OUTPUT);
  pinMode(motorLogicR2, OUTPUT);
  digitalWrite(motorENR, LOW);

  pinMode(encoderLA, INPUT);
  pinMode(encoderRA, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderLA), riseLA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRA), riseRA, RISING);

  removeNoise(errorL, errorML, errorM, errorMR, errorR);
  IRinit();
}

void loop() {
  /*
    if (Serial3.available() > 0) {
    ch = Serial3.read();
    if (ch == 'R') {
      state = HIGH;
    }
    else {
      state = LOW;
    }
  }
  */
  
  //if (state == HIGH) {
    //PD();
    randomSolve();
    //getIR();
    //printIR();
  //}
  //else if (state == LOW) {
    //tickRA = 0;
    //tickLA = 0;
    //disableMotors();
  //}
}
