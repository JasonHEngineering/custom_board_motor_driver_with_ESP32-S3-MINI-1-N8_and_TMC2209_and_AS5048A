// Include the AccelStepper Library
#include <AccelStepper.h>

// Define pin connections

// For my custom ESP32-C3FH4 with DRV8825
// #define stepper_1_stepPin 6
// #define stepper_1_dirPin 7

// For my custom ESP32-S3-MINI-1-N8 with TMC2209
#define stepper_1_stepPin 47
#define stepper_1_dirPin 26
#define stepper_1_MS1 15
#define stepper_1_MS2 16

// Define motor interface type
#define motorInterfaceType 1 //AccelStepper::DRIVER (1) means a stepper driver (with Step and Direction pins)
#define no_of_steppers 1

#define movesteps 256000

//define an array of stepper pointers. initialize the pointer to NULL (null pointer) for safety (see below)
AccelStepper *steppers[no_of_steppers] = {NULL};

// Creates an instance
// AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

void setup() {

  // Create stepper instances
  steppers[0] = new AccelStepper(motorInterfaceType, stepper_1_stepPin, stepper_1_dirPin);

  // Configure micro-stepping
  // MS1 - LOW, MS2 - LOW ==> 1/8
  // MS1 - LOW, MS2 - HIGH ==> 1/64
  // MS1 - HIGH, MS2 - LOW ==> 1/32
  // MS1 - HIGH, MS2 - HIGH ==> 1/16

 if (steppers[0]) {
    pinMode(stepper_1_MS1, OUTPUT);
    digitalWrite(stepper_1_MS1, HIGH);
    pinMode(stepper_1_MS2, OUTPUT);
    digitalWrite(stepper_1_MS2, LOW);
 }

  // Configure each stepper as needed
 if (steppers[0]) {
    steppers[0]->setMaxSpeed(32000); //steps per second, not more than 32000 for TMC2209 
    steppers[0]->setAcceleration(8000.0); //desired acceleration in steps per second per second, not more than 8000 for TMC2209 
    steppers[0]->moveTo(movesteps);   
 }
  
 
}

void loop() {

	if (steppers[0]->distanceToGo() == 0) { 
	  // Change direction once the motor reaches target position
		steppers[0]->moveTo(-steppers[0]->currentPosition());

	}

	// Move the motor one step
	steppers[0]->run();
    
}
