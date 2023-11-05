# ECE198 Wildfire Smoke Detection Project
## Project Description
This project is a solution to wildfire smoke in schools in Canadian forest fire prone regions. It is designed for the ECE198 course and uses a Nucleo F401 board. 
The main goal of the project is to develop a system that can detect wildfire smoke and activate an external ventilation system to purify the air in a classroom.
## Prerequisites
To run this project, you will need:
STM32 Nucleo F401RE board
STM32CubeIDE
Adafruit SCD40 CO2 Sensor
Adafruit VOC Sensor
Adafruit PM2.5 Sensor
## Installation
To install and run this project, follow these steps:
Clone the repository to your local machine.
Open the project in STM32CubeIDE.
Build the project using the STM32CubeIDE.
Flash the built program onto the Nucleo F401RE board.
Usage
Once the program is flashed onto the Nucleo F401RE board, it will start monitoring the environment for smoke particles. If smoke is detected, the system will flash a specific warning light (red, yellow, or green) and activate 
an external exhaust system. 

## Build Instructions
![image](https://github.com/GeeseGoo/ECE198/assets/77707222/6f158231-3fc2-4ddf-8c16-ee334a807d8b)
Follow this wiring diagram to connect each part. 
Ensure that the sensors and microcontrollers are stored in the room where people are working, away from any windows or vents. The LED part of the circuit should be placed in an easily accessible area to ensure people will be able to see clearly. Connecting the relay to a fan or ventilation system should be done by a trained electrician. This can be done by cutting the input power line of the fan in any place, and attaching the cut ends to opposite loose wires in the relay.

## Usage Guide
After the product has been installed, usage is very simple. When entering the area monitored by the product, take a look at the LEDs to ensure none are in the red. If any lights are in the yellow, it is recommended to look at the LEDs again less than 60 minutes apart until the light returns to green to ensure the 1-hour exposure recommendations outlined in the requirements section of the report are met. If the product is not connected to any ventilation system, it is recommended that a fan is turned on or a window is opened to increase airflow in the room until the LEDs return to green. If any of the LEDs are ever displaying red, it is recommended that the area be evacuated until the LEDs return to yellow or green. To access the measurements and statistics stored on the microchip, it can be plugged into a computer via the USB connection and the data can be exported. After the data is exported, it is recommended that the reset button be pressed to wipe the memory of the microchip and allow for more storage. The sleep button on the product can also be pressed to turn it on and off without having to unplug it.

## Tests
### Test 1
Must inform users if CO2 levels are below 800 ppm, if CO2 levels are between 800 and 1200 ppm, or if CO2 levels are above 1200 ppm. These safe ranges were sourced from [10].  
We will test the co2 sensing capabilities of our device by creating a known amount of co2 using baking soda and vinegar. The environmental parameters of the test will be the baseline normal atmospheric level of 418.56 ppm of co2 [26]. The test set up will be done by enclosing our device in a 1 L container along with a baking soda and vinegar reaction to produce CO2. We will conduct 3 separate versions of the test with 3 separate quantifiable measurements standards and pass criteria. The test inputs will be the amount of baking soda and vinegar mixed to produce CO2. 1 with CO2 at the normal atmospheric level of 418.56 ppm. In test 2, we will increase the amount of CO2 by 581.44 ppm to get 1000 ppm of CO2 in the air, which is the average safe value of CO2 as shown by [10]. In Test 3 we will further increase the amount of CO2 by 1081.44 ppm to get 1500 ppm of CO2. A test will pass if it outputs the correct LED signal as shown by requirement 1. We chose 1500 ppm for the last test because it allows an error rate of + or - 300 ppm, which is an acceptable sensor accuracy for our application. To find the test inputs, aka the amount of baking soda and vinegar to use in the experiment, we can use the stoichiometric equation NaHCO3 + CH3COOH → CO2 + H2O + CH3COONa. We will be taking all the gasses at STP. : 
For test 1, 581.44ppm of CO2 will be created in a 1 L box. 
581.44ppm CO2 = 581.44 mL/L CO2 = 581.44 mL of CO2 needed
581.44 mL CO2 / 22.4 L/mol = 0.02597 mol CO2
Because the equation is 1: 1, the amount of baking soda and vinegar needed is 0.02597 mols each.
Test 2 and 3 will function in a similar way.

### Test 2

Using the safe range of PM 2.5 as shown in [11], must inform users if PM 2.5 levels are below 10 ppm, if PM2.5 levels are between 10 and 75 ppm, or if PM2.5 levels are above 75 ppm. 
We will be testing the PM 2.5-sensing capabilities of our device using tea candles, with one tea candle releasing 50 ppm of PM2.5 in a one-liter box [27]. The environmental parameter of the test will be the baseline normal atmospheric level of PM2.5 in Waterloo, 12ppm. [28]. The test set up will be done by enclosing our device in a 1 L box alongside either one or two tea candles, and the test inputs will be the amount of tea candles used. We will conduct 3 separate versions of the test with 3 separate quantifiable measurements standards and pass criteria. The first version will be conducted at the baseline pm2.5 levels in Waterloo, and the test will pass if the sensor accurately determines that the air is safe. The second test will be done with one tea candle so the pm2.5 levels are at 62 ppm, and the test will pass if the device determines that the air is somewhat hazardous. The final test will be done with 2 tea candles so that the pm2.5 levels are greater than 100, and the test will pass if the sensor determines that the air is hazardous. 

### Test 3
Using the safe levels shown in [12], must inform users if ozone levels are above 63 ppm, if ozone levels are between 50 and 63 ppm, or if ozone levels are below 50 ppm.
We will be testing the ozone-sensing capabilities of our device using an ozone generator. The environmental parameter of the test will be the baseline normal atmospheric level of ground level ozone in Waterloo, 22 ppm. [29]. The test set up will be done by enclosing our device in a 1 L box alongside a precise ozone generator. and the test inputs will be the amount of tea candles used. We will conduct 3 separate versions of the test with 3 separate quantifiable measurements standards and pass criteria. The first version will be conducted at the baseline ground ozone levels in Waterloo, and the test will pass if the sensor accurately determines that the air is safe. The second test will be done with the ozone generator generating 35 ppm of ozone to get 57 ppm of ozone overall, and the test will pass if the device determines that the air is somewhat hazardous. The final test will be done with the ozone generator generating 50 ppm of ozone, and the test will pass if the sensor determines that the air is hazardous. 

### Test 4
must inform users if TVOC (total volatile organic compounds) levels are above 2.2 ppm, if TVOC levels are between 2.2 and 0.65 ppm, or if TVOC levels are below 0.65 ppm. Values taken from [13]
We will be testing the TVOC capabilities of our device by evaporating formaldehyde into a 1 L box with our device. The environmental parameter of the test will be the baseline normal atmospheric level of TVOCs in Waterloo, 0.2 ppm [29]. The test set up will be done by enclosing our device in a sealed 1 L box alongside a jar of formaldehyde until all the formaldehyde evaporates. and the test inputs will be the amount of formaldehyde. We will conduct 3 separate versions of the test with 3 separate quantifiable measurements standards and pass criteria. The first version will be conducted at the baseline ground TVOC levels in Waterloo, and the test will pass if the sensor accurately determines that the air is safe. The second test will be done with an amount of formaldehyde evaporated to get 1.4 ppm of TVOCs in the air overall, and the test will pass if the device determines that the air is somewhat hazardous. The final test will be done with even more formaldehyde so that the TVOC count is up to 2.5 ppm, and the test will pass if the sensor determines that the air is hazardous. 

### Test 5
Scale power to an external ventilation system using as necessary to maintain safe air conditions. For demonstration purposes, power will be provided to a small fan to circulate air at a rate between 0.23 to 0.28 m3/min as shown in [14].
Similar to the previous test involving CO2, we will be analyzing the fan utilization and response based on pollutants sensed in the atmosphere by the device. However, this time we will activate the ventilation system and let it respond to the device’s sensors. The environmental parameter of the test will be the normal atmospheric level of CO2, 418.56 ppm [26] The test set up will be done by enclosing our device in a 1 L container with a fan cut out to exhaust air, along with a baking soda and vinegar reaction to produce CO2. We will conduct 3 separate versions of the test with 3 separate quantifiable measurements standards and different pass criterias. The test inputs will be the amount of baking soda and vinegar mixed to produce CO2. The first test will have no CO2 added. To pass this test, the device shouldn’t turn on the exhaust ventilation to save energy. The second test will involve an increase in the amount of CO2 to 1000 ppm, an intermediate value. To pass this test, the ventilation system should immediately turn on and exhaust the CO2 from the box. The third test will involve increasing the amount of CO2 to 1500 ppm, and to pass the ventilation system should also return the CO2 levels back to baseline.  

## Test History
### Test standard # 1: Device is sealed in box along with known amount of CO2 generated through baking soda and vinegar reaction.

2023-11-05T 19:06:23Z

Design Version 1.0

Sub-test 1 results: Device passes the test by accurately determining atmospheric levels of CO2 to be safe.

Sub-test 2 results: Device passes the test. After 0.02597 mols of baking soda and 0.02597 of 5% vinegar are reacted in the chamber, 581.44 ppm of CO2 is generated to create around 1000 ppm of CO2 total. Device accurately determines that this level is semi-hazardous.

Sub-test 3 results: Device passes the test. After 1 mol of baking soda and 1 mol of vinegar are reacted in the chamber, device determines that air is hazardous. 

### Test Standard # 5: Measure efficacy of air cleansing system.

2023-11-05T 19:07:12Z

Design Version 1.0

Sub-test 1 results: Device passes the test. With no additional CO2 added to the chamber, the device correctly determines that the air is safe and does not activate the ventilation system, conserving energy.

Sub-test 2 results: Device passes the test. After 0.02597 mols of baking soda and 0.02597 of 5% vinegar are reacted in the chamber, 581.44 ppm of CO2 is generated to create around 1000 ppm of CO2 total. The device accurately determines that this level is semi-hazardous and activates the ventilation system to exhaust the CO2 from the box.

Sub-test 3 results: Device passes the test. After 1 mol of baking soda and 1 mol of vinegar are reacted in the chamber, the CO2 level increases to 1500 ppm. The device determines that the air is hazardous and activates the ventilation system. The ventilation system successfully returns the CO2 levels back to baseline, demonstrating its efficacy in maintaining safe air conditions.

