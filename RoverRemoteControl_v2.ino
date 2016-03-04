/*  rcTiming.ino -- JW, 30 November 2015 --
   Uses pin-change interrupts on A0-A4 to time RC pulses

   Ref: http://arduino.stackexchange.com/questions/18183/read-rc-receiver-channels-using-interrupt-instead-of-pulsein

*/

#include <Streaming.h>  // The Streaming library
#include <AFMotor.h>    // AdaFruit H-Bridge library

static   byte rcOld;                  // Prev. states of inputs
volatile unsigned long rcRises[4];    // times of prev. rising edges
volatile unsigned long rcTimes[4];    // recent pulse lengths
volatile unsigned int  rcChange = 0;  // Change-counter

unsigned int speed = 0;

volatile unsigned int Xmin = 1060, Xmax = 1940;
volatile unsigned int Ymin = 1104, Ymax = 1860;
volatile int X = 0;
volatile int Y = 0;

volatile int Lspeed = 0;  // Left motor speed
volatile int Rspeed = 0;  // Right motos speed

volatile byte Ldirection = FORWARD;
volatile byte Rdirection = FORWARD;

volatile byte DEADZONE = 64;


AF_DCMotor motor1(1, MOTOR12_64KHZ); // create motor #1, 64KHz pwm
AF_DCMotor motor2(2, MOTOR12_64KHZ); // create motor #2, 64KHz pwm
AF_DCMotor motor3(3, MOTOR12_64KHZ); // create motor #3, 64KHz pwm
AF_DCMotor motor4(4, MOTOR12_64KHZ); // create motor #4, 64KHz pwm



// Be sure to call setup_rcTiming() from setup()
void setup_rcTiming() {
  rcOld = 0;
  pinMode(A0, INPUT);  // pin 14, A0, PC0, for pin-change interrupt
  pinMode(A1, INPUT);  // pin 15, A1, PC1, for pin-change interrupt
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  PCMSK1 |= 0x0F;       // Four-bit mask for four channels
  PCIFR  |= 0x02;       // clear pin-change interrupts if any
  PCICR  |= 0x02;       // enable pin-change interrupts
}

// Define the service routine for PCI vector 1
ISR(PCINT1_vect) {
  byte rcNew = PINC & 15;   // Get low 4 bits, A0-A3
  byte changes = rcNew ^ rcOld; // Notice changed bits
  byte channel = 0;
  unsigned long now = micros(); // micros() is ok in int routine
  while (changes) {
    if ((changes & 1)) {  // Did current channel change?
      if ((rcNew & (1 << channel))) { // Check rising edge
        rcRises[channel] = now;     // Is rising edge
      } else {              // Is falling edge
        rcTimes[channel] = now - rcRises[channel];
      }
    }
    changes >>= 1;      // shift out the done bit
    ++channel;
    ++rcChange;
  }
  rcOld = rcNew;        // Save new state
}


void actuateMotors() {

  if (Y > DEADZONE && (-DEADZONE < X < DEADZONE) ) { // FORWARDS!!

    Lspeed = Y;
    Rspeed = Y;

    Ldirection = FORWARD;
    Rdirection = FORWARD;

    motor1.setSpeed(Lspeed);  // LBM
    motor2.setSpeed(Rspeed);  // RBM
    motor3.setSpeed(Rspeed);  // RFM
    motor4.setSpeed(Lspeed);  // LFM
    motor1.run(Ldirection);      // LBM Run
    motor2.run(Rdirection);      // RBM Run
    motor3.run(Rdirection);      // RFM Run
    motor4.run(Ldirection);      // LFM Run

    if ( X > DEADZONE) {

      Lspeed = Y;
      Rspeed = Y - X;

      if (Rspeed < 0) {
        Rdirection = BACKWARD;
        Rspeed = abs(Rspeed);
      }

      motor1.setSpeed(Lspeed);  // LBM
      motor2.setSpeed(Rspeed);  // RBM
      motor3.setSpeed(Rspeed);  // RFM
      motor4.setSpeed(Lspeed);  // LFM
      motor1.run(Ldirection);      // LBM Run
      motor2.run(Rdirection);      // RBM Run
      motor3.run(Rdirection);      // RFM Run
      motor4.run(Ldirection);      // LFM Run
    }

    if ( X < -DEADZONE) {

      Lspeed = Y + X;
      Rspeed = Y;

      if (Lspeed < 0) {
        Ldirection = BACKWARD;
        Lspeed = abs(Lspeed);
      }

      motor1.setSpeed(Lspeed);  // LBM
      motor2.setSpeed(Rspeed);  // RBM
      motor3.setSpeed(Rspeed);  // RFM
      motor4.setSpeed(Lspeed);  // LFM
      motor1.run(Ldirection);      // LBM Run
      motor2.run(Rdirection);      // RBM Run
      motor3.run(Rdirection);      // RFM Run
      motor4.run(Ldirection);      // LFM Run
    }
  }

  else if (Y < -DEADZONE && (-DEADZONE < X < DEADZONE)) { // BACKWARDS!!
    Y = abs(Y); // same thing, just opposite direction

    Lspeed = Y;
    Rspeed = Y;

    Ldirection = BACKWARD;
    Rdirection = BACKWARD;

    motor1.setSpeed(Lspeed);  // LBM
    motor2.setSpeed(Rspeed);  // RBM
    motor3.setSpeed(Rspeed);  // RFM
    motor4.setSpeed(Lspeed);  // LFM
    motor1.run(Ldirection);      // LBM Run
    motor2.run(Rdirection);      // RBM Run
    motor3.run(Rdirection);      // RFM Run
    motor4.run(Ldirection);      // LFM Run

    if ( X > DEADZONE) {

      Lspeed = Y;
      Rspeed = Y - X;

      if (Rspeed < 0) {
        Rdirection = FORWARD;
        Rspeed = abs(Rspeed);
      }

      motor1.setSpeed(Lspeed);  // LBM
      motor2.setSpeed(Rspeed);  // RBM
      motor3.setSpeed(Rspeed);  // RFM
      motor4.setSpeed(Lspeed);  // LFM
      motor1.run(Ldirection);      // LBM Run
      motor2.run(Rdirection);      // RBM Run
      motor3.run(Rdirection);      // RFM Run
      motor4.run(Ldirection);      // LFM Run
    }

    if ( X < -DEADZONE) {

      Lspeed = Y + X;
      Rspeed = Y;

      if (Lspeed < 0) {
        Ldirection = FORWARD;
        Lspeed = abs(Lspeed);
      }

      motor1.setSpeed(Lspeed);  // LBM
      motor2.setSpeed(Rspeed);  // RBM
      motor3.setSpeed(Rspeed);  // RFM
      motor4.setSpeed(Lspeed);  // LFM
      motor1.run(Ldirection);      // LBM Run
      motor2.run(Rdirection);      // RBM Run
      motor3.run(Rdirection);      // RFM Run
      motor4.run(Ldirection);      // LFM Run
    }
  }

  else if ( (-DEADZONE < Y < DEADZONE) && ( X > DEADZONE) ) {

    Lspeed = X;
    Rspeed = X;
    Ldirection = FORWARD;
    Rdirection = BACKWARD;

    motor1.setSpeed(Lspeed);  // LBM
    motor2.setSpeed(Rspeed);  // RBM
    motor3.setSpeed(Rspeed);  // RFM
    motor4.setSpeed(Lspeed);  // LFM
    motor1.run(Ldirection);      // LBM Run
    motor2.run(Rdirection);      // RBM Run
    motor3.run(Rdirection);      // RFM Run
    motor4.run(Ldirection);      // LFM Run
  }

  else if ( (-DEADZONE < Y < DEADZONE) && ( X < -DEADZONE) ) {

    X = abs(X);

    Lspeed = X;
    Rspeed = X;
    Ldirection = BACKWARD;
    Rdirection = FORWARD;

    motor1.setSpeed(Lspeed);  // LBM
    motor2.setSpeed(Rspeed);  // RBM
    motor3.setSpeed(Rspeed);  // RFM
    motor4.setSpeed(Lspeed);  // LFM
    motor1.run(Ldirection);      // LBM Run
    motor2.run(Rdirection);      // RBM Run
    motor3.run(Rdirection);      // RFM Run
    motor4.run(Ldirection);      // LFM Run
  }

  else if ( (-DEADZONE < Y < DEADZONE) && (-DEADZONE < X < DEADZONE)) { // Mneh... still in "deadzone"

    Lspeed = 0;
    Rspeed = 0;

    motor1.setSpeed(Lspeed);     // LBM
    motor2.setSpeed(Rspeed);     // RBM
    motor3.setSpeed(Rspeed);     // RFM
    motor4.setSpeed(Lspeed);     // LFM
  }

  //Serial << "Moving..." << "X: " << X << "\tY: " << Y << "\t\tLspeed:" << Lspeed << "\tRspeed:" << Rspeed << "\tLirection:" << Ldirection << "\tRdirection:" << Rdirection << endl;

}






void setup() {
  Serial.begin(115200);


  // Show us the motors are running
  Serial.println("Test Motors");

  motor1.setSpeed(0);     // set the speed to 0/255
  motor2.setSpeed(0);     // set the speed to 0/255
  motor3.setSpeed(0);     // set the speed to 0/255
  motor4.setSpeed(0);     // set the speed to 0/255

  // Start RC Timing Tests
  Serial.println("Starting RC Timing Test");
  setup_rcTiming();

}

void loop() {

  unsigned long rcT[4]; // copy of recent pulse lengths
  unsigned int rcN;
  if (rcChange) {

    // Data is subject to races if interrupted, so off interrupts
    cli();                // Disable interrupts
    rcN = rcChange;
    rcChange = 0;         // Zero the change counter
    X = rcT[0] = rcTimes[0];  // mapping right joystick X
    Y = rcT[1] = rcTimes[1];  // mapping right joystick Y
    rcT[2] = rcTimes[2];
    rcT[3] = rcTimes[3];
    sei();                // reenable interrupts

    // Extend min-max values dynamically
    // First assign X and Y
    //X = rcT[0]; Y = rcT[1];

    // Then map the value to the required range...
    X = map(X, Xmin, Xmax, -255, 255); Y = map(Y, Ymin, Ymax, -255, 255);
    // And constrain it...
    X = constrain(X, -255, 255); Y = constrain(Y, -255, 255);

    actuateMotors(); // X and Y are global variables, so no need to pass the values...

    /*
        Serial << "t=" << millis() << " rcT[0]:" << rcT[0] << " rcT[1]:" << rcT[1]
               << " rcT[2]:" << rcT[2] << " rcT[3]" << rcT[3] << " rcN:" << rcN << endl
               << "\tXmin:" << Xmin << " X:" << X << " Xmax:" << Xmax << endl
               << "\tYmin:" << Ymin << " Y:" << Y << " Ymax:" << Ymax << endl << endl;
    */
  }
  sei();            // reenable interrupts
}
