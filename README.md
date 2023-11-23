# Suspension Travel & Tyre temp sensor for RaceChrono
A modified https://github.com/MagnusThome/RejsaRubberTrac with the following changes

* Uses a potentiometer instead of a distance sensor to measure the suspension travel
* Uses the newer MLX90640 sensor

# Usage
1. Set the name to match the corner you are going to place each device

   ``#define DEVICE_NAME "RearLeftSensor"``
2. Take note of the potentiometer value at ride height and at full droop. Also measure the travel from
ride height to droop. Then set the following values accordingly:

    ``#define DROOP_TRAVEL -33 //  the travel from ride height to droop in mm``

    ``#define SENSOR_RIDE_HEIGHT 398``

    ``#define SENSOR_DROOP 312``

3. Add the device as "RejsaRubberTrac" in "other devices" under RaceChrone setup.

## Temp sensor setup
1. If you have the temp sensor connected, uncomment the following line

   ``//#define TEMP_SENSOR "tempSensor"``
2. Place the temp sensor so that the reference pin points to the back of the car.
3. On right hand side tyres uncomment following line

   ``//#define RIGHT_SIDE true``
4. Temp 1 is the inside temp, temp 8 is the outside temp. 

## Safety notice
This is experimental, use it with caution and at your own risk!

## License
This is [licensed](LICENSE) under the [MIT Licence](http://en.wikipedia.org/wiki/MIT_License).
