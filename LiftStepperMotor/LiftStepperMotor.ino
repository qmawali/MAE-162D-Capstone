#include <AccelStepperWithDistance.h>

#define STEP_PIN 2
#define DIR_PIN 3
#define ENABLE_PIN 4

#define MICROSTEPS 16            // Microsteps per step
#define STEPS_PER_ROTATION 200   // Steps per full rotation
#define DISTANCE_PER_ROTATION 10 // Distance covered per full rotation (in cm)


AccelStepperWithDistance lift(AccelStepper::DRIVER, STEP_PIN, DIR_PIN, ENABLE_PIN, 5);

void setup() {
  // Set up the lift
  lift.setMicroStep(MICROSTEPS);
  lift.setStepsPerRotation(STEPS_PER_ROTATION);
  lift.setDistancePerRotation(DISTANCE_PER_ROTATION);

  // Set the maximum speed 
  lift.setMaxSpeed(1000); 

  // Enable the motor
  lift.enableOutputs();

  // Move the lift to the initial position (e.g., bottom)
  lift.setCurrentPosition(0); // Set initial position to bottom
}

void loop() {
  // Example: move the lift based on input from serial monitor
  if (Serial.available() > 0) {
    int targetHeight = Serial.parseInt();
    
    // Move the lift to the specified height
    lift.moveToDistance(targetHeight);
    lift.runToPosition();
  }
}