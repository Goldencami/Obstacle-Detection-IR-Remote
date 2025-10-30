# Obstacle-Detection-IR-Remote
A fun project I decided to do with the Arudino Kit. The project is composed of different parts, the goal was to implement as much of my knowledge into making something fun.

## Ultrasonic Sensor
The main part of the project is composed of an ultrasonic sensor to detect an obstacle (e.g. my hand) to recreate the obstacle detection of a smart car to avoid incidents. Depending on the distance the sensor is detecting, the red LED blinks at a certain rate. The closer the obstacle, the faster the LED blinks to recreate a warning signal for the user. A corresponding output message will be displayed on the LCD screen:
- "No Obstacle"
- "!!Warning!!"
- "!!!Obstacle!!!"

If an obstacle happens to be detected, at a predetermined distance, the system locks. Showing all the LEDs blinking and displaying a message to unlock the system by either pressing the button on the bread board or a button on the remote.

## Photoresistor
A photoresistor was used as a smart room light by lighting the green LED according to the luminosity it receives. The less light it receives, the more power the green LED receives.

## LCD Screen
An LCD screen was used as a visual representation to display the distance, the intensity of the luminosity and combined with the IR Remote to simulate changing screens.

## Demo Video
https://github.com/user-attachments/assets/cf0a05ec-2b35-4497-bc21-bf849099a7ee
