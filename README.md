# MAE-162D-Capstone
Software for BOTanist

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
