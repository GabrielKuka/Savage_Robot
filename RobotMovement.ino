/*
  This template was provided by Nathan Beeten on October 28, 2016
  It has been modified by Gabriel Kuka on 11, 2016
  Actual version of the code is 2.2!
  This code was created and modified in order to control a savage soccer robot.
*/

#include <Arduino.h>
#include <XBOXRECV.h> //import the wireless coms library
#include <DCMOTOR.h> //import the motor control library

USB Usb; //Create Objects for program to use
XBOXRECV Xbox(&Usb);
// Pwm pin 11 is left for servos!!!!!
// Motor 1 pins
uint8_t dirPin1_M1 = 8;
uint8_t dirPin2_M1 = 7;
uint8_t enablePin_M1 = 6;

// Motor 2 pins
uint8_t dirPin1_M2 = 2;
uint8_t dirPin2_M2 = 4;
uint8_t enablePin_M2 = 5;

// Motor 3 pins (the roller)
uint8_t dirPin1_M3 = 9;
uint8_t dirPin2_M3 = 10;
uint8_t enablePin_M3 = 3;

// Motor objects
DCMOTOR Motor1;
DCMOTOR Motor2;
DCMOTOR Motor3;

// Variable that hold the speed of the motor that is used for the roller
int rollerMotorSpeed = 0;

// Joystick directions
int16_t hatXInput;
int16_t hatYInput;

// Letter button variables
int16_t buttonA;
int16_t buttonB;
int16_t buttonX;
int16_t buttonY;

// Bumper and trigger buttons
int16_t rightBumper_R1;
int16_t leftBumper_L1;
int16_t rightTrigger_R2;
int16_t leftTrigger_L2;

// These variables will hold the intensity of left and right joystick direction
int16_t sideIntensity;
int16_t fbIntensity;

// These variables will hold the intensity of l2 and r2 buttons
int8_t r2_Intensity;
int8_t l2_Intensity;


bool motorDir;

uint8_t pwmMap(uint16_t input);

uint8_t xboxPort = 0;//The port on the receiver that the controller is connected to

void setup() {//The setup code initializes the rest of your program
  Serial.begin(115200);
  if (Usb.Init() == -1) {//check whether the USB connects
    Serial.print(F("\r\nOSC did not start"));
    while (1); //Stop the program if we didn't connect
  }

  // Attach motors to pin in the arduino
  Motor1.attach( dirPin1_M1, dirPin2_M1, enablePin_M1 );
  Motor2.attach( dirPin1_M2, dirPin2_M2, enablePin_M2 );
  Motor3.attach( dirPin1_M3, dirPin2_M3, enablePin_M3 );

  Serial.print(F("\r\nXbox Wireless Receiver Library Started"));
}

void loop() {// The loop runs repeatedly from top to bottom after the setup
  Usb.Task();
  if (Xbox.XboxReceiverConnected)  {
    if (Xbox.Xbox360Connected[xboxPort]) {

      // Connects variables to the directions of the joystick of xbox controller
      hatXInput = Xbox.getAnalogHat(LeftHatX, xboxPort);
      hatYInput = Xbox.getAnalogHat(LeftHatY, xboxPort);

      // Connects variables to the specific buttons in the xbox controller
      buttonA = Xbox.getButtonClick(A, xboxPort);
      buttonB = Xbox.getButtonClick(B, xboxPort);
      buttonX = Xbox.getButtonClick(X, xboxPort);
      buttonY = Xbox.getButtonClick(Y, xboxPort);

      // Connects variables with bumper and trigger buttons at the xbox controller
      rightBumper_R1 = Xbox.getButtonClick(R1, xboxPort);
      leftBumper_L1 = Xbox.getButtonClick(L1, xboxPort);
      rightTrigger_R2 = Xbox.getButtonPress(R2, xboxPort);
      leftTrigger_L2 = Xbox.getButtonPress(L2, xboxPort);

      // Assigns the value of the intensity of the right joystick
      sideIntensity = abs(hatXInput);
      fbIntensity = abs(hatYInput);

      // Holds the value that shows how much trigger buttons are pressed
      r2_Intensity = abs(rightTrigger_R2);
      l2_Intensity = abs(leftTrigger_L2);

      checkMovement();

      if (buttonA) {
        Serial.println("Button A pressed");
      }
      if (buttonB) {
        Serial.println("Button B pressed");
        checkBasicMovement(5); // Stop robot if instructions above don't work
      }
      if (buttonX) {
        Serial.println("Button X pressed");
        setRollerState(); // Start or Stop rotating the roller
      }
      if (buttonY) {
        Serial.println("Button Y pressed");
      }


    }
  }
}

uint8_t pwmMap(uint16_t input) {
  uint8_t temp = (input - 1) / 128; //overflows at full neg without the -1
  Serial.print("Intensity: ");
  Serial.print(input);
  Serial.print("   Pwm Out: ");
  Serial.print(temp);
  Serial.print("      ");
  return temp;
}

void checkMovement() {
  
  
  if ( rightTrigger_R2 > 0  &&  hatXInput > 7500 ) {        //  <- If R2 is pressed and joystick moves right
    checkBasicMovement(3);                                  //  <- Move forward
    delay(200);                                             //  <- Delay 0.3 seconds from extra current drain
    checkBasicMovement(2);                                  //  <- Move right~~~
  } else if ( rightTrigger_R2 > 0  &&  hatXInput < -7500 ) {//  <- If R2 is pressed and joystick moves left
    checkBasicMovement(3);                                  //  <- Move forward
    delay(200);                                             //  <- Delay 0.3 seconds from extra current drain
    checkBasicMovement(1);                                  //  <- Move left~~~
  } else if ( leftTrigger_L2 > 0  &&  hatXInput > 7500 ) {  //  <- If L2 is pressed and joystick moves right
    checkBasicMovement(4);                                  //  <- Move backward
    delay(200);                                             //  <- Delay 0.3 seconds from extra current drain
    checkBasicMovement(2);                                  //  <- Move right~~~
  } else if ( leftTrigger_L2 > 0  &&  hatXInput < -7500 ) { //  <- If L2 is pressed and joystick moves left
    checkBasicMovement(4);                                  //  <- Move backward
    delay(200);                                             //  <- Delay 0.3 seconds from extra current drain
    checkBasicMovement(1);                                  //  <- Move left~~~
  } else if (rightTrigger_R2 > 0) {                         //  <- If R2 is pressed
    checkBasicMovement(3);                                  //  <- Move forward~~~
  } else if (leftTrigger_L2 > 0) {                          //  <- If L2 is pressed
    checkBasicMovement(4);                                  //  <- Move backward~~~
  } else if (hatXInput > 7500) {                            //  <- If joystick moves right
    checkBasicMovement(2);                                  //  <- Move right~~~
  } else if (hatXInput < -7500 ) {                          //  <- If joystick moves left
    checkBasicMovement(1);                                  //  <- Move left~~~
  } else if (r2_Intensity == 0 && l2_Intensity == 0) {      //  <- If none of the trigger buttons is pressed
    checkBasicMovement(5);                                  //  <- Stop robot~~~
  } else if ( rightTrigger_R2 > 0 && leftTrigger_L2 > 0 ) { //  <- If both of the trigger buttons are pressed at the same time
    checkBasicMovement(5);                                  //  <- Stop robot~~~
  }
}

void checkBasicMovement(int movement) {
  switch (movement) {
    
    case 1: // <- Turn robot left
    
      Serial.print("Turns left    ");
      Motor2.motorRunCW(pwmMap(sideIntensity));
      Motor1.motorRunCW(pwmMap(sideIntensity));
      Serial.println("");
      
      break;
      
    case 2: // <- Turn robot right
    
      Serial.print("Turns right    ");
      Motor2.motorRunCCW(pwmMap(sideIntensity));
      Motor1.motorRunCCW(pwmMap(sideIntensity));
      Serial.println("");
      
      break;

    case 3: // <- Moves robot forward

      Serial.print("Forward   ");
      Serial.print(rightTrigger_R2);
      Serial.println("");

      Motor2.motorRunCCW(r2_Intensity);
      Motor1.motorRunCW(r2_Intensity);

      break;

    case 4: // Moves robot backward

      Serial.print("Backward   ");
      Serial.print(leftTrigger_L2);
      Serial.println("");

      Motor2.motorRunCW(l2_Intensity);
      Motor1.motorRunCCW(l2_Intensity);

      break;
      
    default: // The defualt state will stop the motors from rotating
      Motor2.motorBrake();
      Motor1.motorBrake();
  }
}

void setRollerState() {
  if (getRollerState()) {
    rollerMotorSpeed = 150;
    Motor3.motorRunCCW(rollerMotorSpeed);
  } else {
    rollerMotorSpeed = 0;
    Motor3.motorBrake();
  }

}

boolean getRollerState() {
  if (rollerMotorSpeed == 0) {
    return true;
  }
  else {
    return false;
  }
}
