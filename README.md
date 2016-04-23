# Claptrap
Main control sketch for Claptrap 

This is the main control code for the Claptrap Cosplay vehicle that is currently under construction by David Williams (Thevolget) 
and Caela Waller (CJ).  The above code controls the movement of the vehicle via a PSX controller, and controls are defined as 
follows:

    LEFT ANALOG STICK - THROTTLE TO MOTORS
    RIGHT ANALOG STICK - PAN/TILT FOR ARM MOVEMENT
    L1/R1 - ARM SELECT
    L2/R2 - ELBOW SELECT
    CROSS - MOVE ELBOW DOWN
    TRIANGLE - MOVE ELBOW UP
    START - ENABLE/DISABLE BRAKE
    L3 - ENABLE SERVO PAN/TILT LOCK
    SQUARE - DISABLE SERVO PAN/TILT LOCK
  
Arm movements are made via pressing the appropriate 'ARM' button, then moving the right analog stick to adjust the position.  Pressing
down on the right analog stick will enable the pan/tilt hold mode, which will allow the arms to be positioned and hold the position.  
To disable this mode, press the square button.  Elbow movements are controlled via picking the appropriate 'ELBOW' and using the 
triangle and cross buttons to increase or reduce the servo position.  Throttle is controlled via the left analog stick and the motor
can be disabled by pressing the START button.  

The vehicle will start with brake mode enabled by default, as to prevent the controller from being accidentally hit or from a bad 
calibration from the stick.  When ready to move, press the start button to enable.
