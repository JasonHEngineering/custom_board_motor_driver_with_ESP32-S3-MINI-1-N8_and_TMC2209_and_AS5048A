// V1 pure reading of AS5048A only
// V2 with stepper
// V3 with PID - slow 

#include <AccelStepper.h>
#include <SPI.h>

/*PINS
   Arduino SPI pins
   MOSI = 11, MISO = 12, SCK = 13, CS = 10

   STM32 SPI pins
   MOSI = PA7, MISO = PA6, SCK = PA5, CS = PA4

   ESP32_S3_DevKit_C1_N16R8 SPI pins
   MOSI = 11, MISO = 12, SCK = 13, CS = per assignment as needed

   ESP32-S3-MINI-1-N8 custom board SPI pins
   MOSI = 11, MISO = 12, SCK = 13, CS = per assignment as needed
*/

// SPI lines for my custom ESP32-S3-MINI-1-N8
#define SPI_CS 38
#define SPI_MISO 37
#define SPI_SCLK 36
#define SPI_MOSI 35


// For my custom ESP32-S3-MINI-1-N8 with TMC2209
#define stepper_1_stepPin 47
#define stepper_1_dirPin 26
#define stepper_1_MS1 15
#define stepper_1_MS2 16

// Define motor interface type
#define motorInterfaceType 1 //AccelStepper::DRIVER (1) means a stepper driver (with Step and Direction pins)
#define no_of_steppers 1

// #define movesteps 256000

//define an array of stepper pointers. initialize the pointer to NULL (null pointer) for safety (see below)
AccelStepper *steppers[no_of_steppers] = {NULL};

float axis_degAngle[no_of_steppers] = {0}; //Angle in degrees
float accepted_axis_degAngle[no_of_steppers] = {0}; //Angle in degrees
const int windowSize = 1; // Adjust the window size based on your requirements
int currentIndex = 0;
uint16_t rawData = 0; //bits from the encoder (16 bits, but top 2 has to be discarded)
uint16_t command = 0b1111111111111111; //read command (0xFFF)
float sensorValues[no_of_steppers][windowSize];
const float start_axis_degAngle[no_of_steppers] = {177.74}; //Angle in degrees, encoder values when in kinematic zeroed position - 19-Apr-25

#define STEPS_PER_REV 6400   // Steps per full revolution (adjust to your setup) 32*200=6400

// float struct to store angles 
struct {
   float angles_array[no_of_steppers]; // unit: °
} encoder; 



// PID loop constants (tune for your system)
float Kp = 1508.0;
float Kd = 0.02;
float lastError = 0;
unsigned long lastTime = 0;

// Target angle in degrees (0 to 360)
float targetAngle = 160.0;



void setup() {

  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI); //SCLK, MISO, MOSI, SS
  pinMode(SPI_CS, OUTPUT); // set all CS pin as output
  digitalWrite(SPI_CS, HIGH);
  Serial.begin(9600); // start serial for output
  delay(100);


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
    //steppers[0]->moveTo(movesteps);   
 }

}

void loop() {
  // put your main code here, to run repeatedly:
  read_angle_Register(); //read the position of the magnet
  //report_angle();

	// Move the motor one step
	// steppers[0]->run();


  // Time delta
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // Read encoder angle
  float currentAngle = getMovingAverage(0); //read the position of the magnet
  if (currentAngle < 0) currentAngle += 360;  // Normalize

  // Error (shortest direction)
  float error = targetAngle - currentAngle;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  // Derivative
  float derivative = (error - lastError) / dt;
  lastError = error;

  // PID control output (no integral used here)
  float output = Kp * error + Kd * derivative;

  // Convert angle error to stepper steps
  float stepsToMove = (output / 360.0) * STEPS_PER_REV;

  // Move stepper (relative move)
  steppers[0]->setSpeed(stepsToMove);
  steppers[0]->runSpeed();

  // Debug
  Serial.print("Target: "); Serial.print(targetAngle);
  Serial.print(" | Current: "); Serial.print(currentAngle);
  Serial.print(" | Error: "); Serial.print(error);
  Serial.print(" | Output: "); Serial.println(output);
  
  //delay(1);
  delayMicroseconds(50);  // Delay for 50 micro seconds

}

void read_angle_Register()
{
  for (int i = 0; i <= (no_of_steppers-1); i++) {
    
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
    //SPI.setClockDivider(SPI_CLOCK_DIV64);
      
    //--sending the command
    digitalWrite(SPI_CS, LOW);
    SPI.transfer16(command);
    digitalWrite(SPI_CS, HIGH);
  
    //--receiving the reading
    digitalWrite(SPI_CS, LOW);
    rawData = SPI.transfer16(command);
    digitalWrite(SPI_CS, HIGH);
    
    SPI.endTransaction();
  
    rawData = rawData & 0b0011111111111111; //removing the top 2 bits (PAR and EF)
  
    axis_degAngle[i] = (float)rawData / 16383.0 * 360.0; //16384 = 2^14, 360 = 360 degrees, however for real resolution, take 2^14 - 1 = 16383
    updateMovingAverage(i, axis_degAngle[i]);

    // Note: if getMovingAverage(i) == 360.00, it is likely to be false/no reading
    if ((axis_degAngle[i] != 0.00) && (getMovingAverage(i) != 0.00)){
      accepted_axis_degAngle[i] = getMovingAverage(i);
      encoder.angles_array[i] = fmod(((accepted_axis_degAngle[i]-start_axis_degAngle[i])+180.0),360.0)-180.0;

    }
  }
}

void updateMovingAverage(int axis, float newValue) {
  // Update the array with the new value
  sensorValues[axis][currentIndex] = newValue;

  // Move to the next index in a circular manner
  if (axis==(no_of_steppers-1)){
    currentIndex = (currentIndex + 1) % windowSize;
  }
}


float getMovingAverage(int axis) {
  // Calculate the average of the values in the array
  float sum = 0;
  for (int i = 0; i < windowSize; i++) {
    sum += sensorValues[axis][i];
  }
  float local_window = windowSize;
  float moving_average = sum / local_window;
  return moving_average;
}



void report_angle()
{
  for(int j = 0; j <= (no_of_steppers-1); j++) {
      //Serial.println(String(j+1) + " " + getMovingAverage(j));
      Serial.println(String("Angle: ") + getMovingAverage(j));
      //Serial.println(String(j+1) + " " + encoder.angles_array[j]);
    }
    Serial.flush();
}

