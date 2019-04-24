#include <Pixy2.h>
#include <SPI.h>
/*
   Only incudes stock code for moving the robot
*/

const int pinEnc_L = 2;
const int pinEnc_R = 3;
const int pinON = 6;         // connect pin 6 to ON/OFF switch, active HIGH
const int pinCW_L = 7;    // connect pin 7 to clock-wise PMOS gate
const int pinCC_L = 8;    // connect pin 8 to counter clock-wise PMOS gate
const int pinRef_L = 9; // connect pin 9 to speed reference (left)
const int pinRef_R = 10; //connect pin 10 to speed reference (right)
const int pinCW_R = 11; //connect pin 12 to clock-wise PMOS gate 
const int pinCC_R = 12; //connect pin 13 to counter clock-wise PMOS gate
const int pinLED = 13;
// encoder counter variable
volatile int enc_count_L = 0; // "volatile" means the variable is stored in RAM
volatile int enc_count_R = 0; //sets encoder count to zero 

Pixy2 pixy;

// setup pins and initial values
void setup() {
  Serial.begin(115200);
  Serial.println("Starting");


//This line breaks the ability for the robot to drive
  pixy.init();
  
pinMode(pinON, INPUT); 
  pinMode(pinCW_L, OUTPUT); //sets pins 7-12 as outputs 
  pinMode(pinCC_L, OUTPUT);
  pinMode(pinRef_L, OUTPUT);
  pinMode(pinCW_R, OUTPUT);
  pinMode(pinCC_R, OUTPUT);
  pinMode(pinRef_R, OUTPUT);
  pinMode(pinLED, OUTPUT);            // on-board LED

  digitalWrite(pinLED, LOW);          // turn LED off
  digitalWrite(pinCW_L, LOW);  // stop clockwise
  digitalWrite(pinCC_L, LOW);  // stop counter-clockwise

  digitalWrite(pinCW_R, LOW);  // stop clockwise
  digitalWrite(pinCC_R, LOW);  // stop counter-clockwise

  analogWrite(pinRef_L, 200); // set speed reference, duty-cycle = 50/255
  analogWrite(pinRef_R, 200); // set speed reference, duty-cycle = 50/255
  Serial.print("Starting");

  /*
    Connect left-wheel encoder output to pin 2 (external Interrupt 0) via a 1k resistor
    Rising edge of the encoder signal triggers an interrupt
    count_Left is the interrupt service routine attached to Interrupt 0 (pin 2)
  */
  attachInterrupt(0, count_Left, FALLING); //attaches interrupt to falling edge encoder pulse
  attachInterrupt(1, count_Right, FALLING);
}

/*
  Interrupt 0 service routine
  Increment enc_count_Left on each falling edge of the
  encoder signal connected to pin 2
*/
void count_Left() {
  enc_count_L++;
}

void count_Right() {
  enc_count_R++;
}

void loop() {
  delay(1000);
  turnRight(1.08,80);
  delay(1000);
  turnLeft(1.11, 80);

  /**
   * This chunk of code should make the robot turn to face the detected object
   */
  //get the detected objects from the pixy
//  pixy.ccc.getBlocks();
//
//  if (pixy.ccc.numBlocks) {
//    if (pixy.ccc.blocks[0].m_x < 100) {
//      Serial.println("turning Left");
//      turnLeft(.2, 160);
//    } else if (pixy.ccc.blocks[0].m_x > 200) {
//      Serial.println("Turning Right");
//      turnRight(.2, 160);
//    }
//  } else {
//    digitalWrite(pinCW_R, LOW);
//    digitalWrite(pinCC_L, LOW);
//    digitalWrite(pinCW_L, LOW);
//    digitalWrite(pinCC_R, LOW);
//    Serial.println("Stopped");
//  }




  //  if (digitalRead(pinON)) {
  //    int speed = 80; //speed to be used is 80/255
  //    delay(1000);
  //    forward(24, speed); //move forward 24 inches
  //    delay(1000);
  //    turnRight(1.08, speed); //turn right first parameter is tuned to make it turn exactly 90 degrees
  //    delay(1000);
  //    forward(24, speed); //move forward 24 inches
  //    delay(1000);
  //    turnLeft(1.14, speed); //turn left first parameter is tuned to make it turn exactly 90 degrees
  //    delay(2000);
  //    digitalWrite(pinCW_R, LOW); //right wheel stop spinning clockwise
  //    digitalWrite(pinCC_L, LOW); //left wheel stop spinning counter-clockwise
  //    digitalWrite(pinCW_L, LOW); //left wheel stop spinning clockwise
  //    digitalWrite(pinCC_R, LOW); //right wheel stop spinning counter-clockwise
  //  }else{
  //    digitalWrite(pinCW_R, LOW);
  //    digitalWrite(pinCC_L, LOW);
  //    digitalWrite(pinCW_L, LOW);
  //    digitalWrite(pinCC_R, LOW);
  //  }

}


void forward(float dist, int sp) { //distance is in inches and sp is speed from 0-255
  enc_count_R = 0;
  enc_count_L = 0;
  float oneInch = (13 * 52) / (2 * PI * 2.55906); //math for one inch in encoder pulses
  analogWrite(pinRef_L, sp);//set pwm wave something out of 255
  analogWrite(pinRef_R, sp);
  while (enc_count_R  < dist * oneInch  || enc_count_L  < dist * oneInch) { //while it isn’t at the correct distance keep going
    digitalWrite(pinCW_R, HIGH);//set output pins
    digitalWrite(pinCC_L, HIGH);
    digitalWrite(pinCW_L, LOW);
    digitalWrite(pinCC_R, LOW);
    if (enc_count_R  > dist * oneInch ) { //if one wheel finishes before the other then turn it off
      digitalWrite(pinCW_R, LOW);
    }
    if (enc_count_L > dist * oneInch) {
      digitalWrite(pinCC_L, LOW);
    }
    Serial.println(enc_count_L);
    Serial.println(enc_count_R);
    Serial.println(oneInch * dist);
    Serial.println();

  }
  digitalWrite(pinCW_R, LOW);
  digitalWrite(pinCC_L, LOW);
  digitalWrite(pinCW_L, LOW);
  digitalWrite(pinCC_R, LOW);
}

void turnRight(float deg, int sp) { //deg is in degrees and sp is speed from 0-255
  enc_count_R = 0;
  enc_count_L = 0;
  float oneInch = (13 * 52) / (2 * PI * 2.55906) ;
  double dist = deg * oneInch / 3 + 1;  //math to make it work gotten by trial and error
  analogWrite(pinRef_L, sp);
  analogWrite(pinRef_R, sp);
  while (enc_count_R  < dist * oneInch  || enc_count_L  < dist * oneInch) { //same logic as in forward
    digitalWrite(pinCW_R, HIGH);
    digitalWrite(pinCC_L, LOW);
    digitalWrite(pinCW_L, HIGH);
    digitalWrite(pinCC_R, LOW);
    if (enc_count_R  > dist * oneInch ) {
      digitalWrite(pinCW_R, LOW);
    }
    if (enc_count_L > dist * oneInch) {
      digitalWrite(pinCC_L, LOW);
    }
  }
  digitalWrite(pinCW_R, LOW);
  digitalWrite(pinCC_L, LOW);
  digitalWrite(pinCW_L, LOW);
  digitalWrite(pinCC_R, LOW);
}

void turnLeft(float deg, int sp) { //deg is in degrees and sp is speed from 0-255
  enc_count_R = 0;
  enc_count_L = 0;
  float oneInch = (13 * 52) / (2 * PI * 2.55906);
  double dist = deg * oneInch / 3 +1;//same logic as in turnRight
  analogWrite(pinRef_L, sp);
  analogWrite(pinRef_R, sp);
  while (enc_count_R  < dist * oneInch  || enc_count_L  < dist * oneInch) { //same logic as in forward
    digitalWrite(pinCW_R, LOW);
    digitalWrite(pinCC_L, HIGH);
    digitalWrite(pinCW_L, LOW);
    digitalWrite(pinCC_R, HIGH);
    if (enc_count_R  > dist * oneInch ) {
      digitalWrite(pinCW_R, LOW);
    }
    if (enc_count_L > dist * oneInch) {
      digitalWrite(pinCC_L, LOW);
    }
  }
  digitalWrite(pinCW_R, LOW);
  digitalWrite(pinCC_L, LOW);
  digitalWrite(pinCW_L, LOW);
  digitalWrite(pinCC_R, LOW);
}
