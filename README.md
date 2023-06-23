# LevitadorNeumatico
 Project involving the levitation of a ping pong ball in a tube using a fan extracted from a hairdryer, with PID control.

This is the final project for the ME4250 Mechatronics course. The objective is to control a system using PID. In this case, we chose to control the height of a ball in a tube, with the reference being the position of the hand, so that the ball has to follow the hand.

An Arduino UNO was used as the microcontroller, and an L298N was used to control the fan. Additionally, HC-SR04 Ultrasonic sensors were used to determine the position of the ball in the tube and the position of the hand in relation to the tube.

The tube was divided into 3 control zones, each with a different PID, to improve the system's response.