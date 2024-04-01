# Arduino SumoBot
This Project contains a program written in C++/Arduino that controls an AREXX AAR-04 to stay within a circle and tries to push off its foe. 

The script has following features:
- In order to be unpredictive, it randomly changes its heading using a random number generator (seeded from the analog pin 0).
- At all times it probes for the line (of the ring) with two light sensors and turns around if detected
- Using an ultrasonic range finder, it detects objects within a certain range and either attacks or defends according to the relative velocity of the object.

## Normal Behaviour:
If the robot is within the circle and has not detected an enemy within a certain range, the robot moves in random curves for a random amount of time. While doing so, at least one motor is always at full speed to be as agile as possible.  

## Line Detection Behaviour:
If a line is detected, it stops and turns in the direction with the smaller angle to the line to take the shortest time to get back to the field. It then turns for a random amount of time for unpredictability. Afterwards, it continues with its normal behaviour.

## Attack Strategy:
If the enemy is detected by the range finder, the script calculates if is is moving towards the enemy. If this is NOT the case, the robot tries to push the foe out of the circle with full speed. It only stops if a line is detected and resumes with its normal behaviour.

## Defence Strategy:
If the enemy is detected and moving towards the robot at a certain velocity, the robot quickly turns for a random duration and moves to another direction. Then it continues with its normal behaviour. 

This is a small project for a lecture from Johannes Kepler University.

## The Robot
It seems like the AREXX AAR-04 arduino robot development kit is already discontinued, nontheless one can easily modify the code to work with other platforms.
The manual to the robot can be found here: https://www.manualslib.com/manual/2066732/Arexx-Aar-04.html

© Michael Duschek
