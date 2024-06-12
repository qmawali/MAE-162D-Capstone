# MAE-162D-Capstone
Software for BOTanist

Arduino connects to 2xPWM Motor Driver with two motors per output channel for drivetrain + 4 IR sensors for double-sided line-following.
Lift motor controlled by Nema 17, through TB6600 and Arduino
Arduino BLE address must be updated in Raspi code
Raspi controls pump through L298N, Ultrasonic sensor, and rack stepper motor through ULN2003 driver

## The line_following code uses the following pseudocode:

IR Input Right and left have value 1 for black color and 0 for white color
White is the surrounding space and black is the line

If Left and Right IR are 0 move forward
If left and Right IR are 1 stop
If left is 1 and Right 0 turn left
If left is 0 and Right is 1 turn right

To move forward turn on all four motors forward
To stop turn off all four motors
To turn right right motors go backward and left motors go forward
To turn left left motors go backward and right motors go forward
