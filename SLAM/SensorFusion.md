# Sensor Fusion
Using more data sources so that we have a better understanding of the environment

Two parts: Sense and Perceive.

Sensing is collecting data and perceiving is interpreting it.

For example Having multiple sensors for the same data reduces the noise by sqrt(n) times given the noise is not correlated.

Another: A gyro can be fused with a magnetometer for a compass in a phone. Direction and magnetic field would change only if the phone has moved, eliminating noise.

### Advantages
<ol>
	<li>Accuracy of data</li>
	<li>Reliability (If one sensor fails)</li>
	<li>Measuring more data types: Having 2 cameras can help measure depth</li>
	<li>Increasing coverage area.</li>
</ol>

Fusion mainly happens through Kalman filters.


# Sensor Fusion for Orientation
We are using a magnetometer and accelerometer.

There are 2 main issues with this setup.
<ol>
	<li>Magnetometer is disturbed other magnetic or magnet materials in the environment</li>
	<li>Gravitational magnetic fields are not in the horizontal plain</li>
	<li>Acceloremeter readings are disturbed by linear acceleration of the body</li>
</ol>

## Eliminating magnetic distortion due to magnets/magnetic materials
Magnet(Hard Iron) produces creates a circle with centre offset from the origin.

Soft Iron produces an oval

Callibrating mechanism can be used to convert these into a perfect sphere centred at origin.

## Eliminating Linear acceleration
If the acceleration is due to the system actuators then this can be easily eliminated by using a model to calculate acceleration.

If it is not then a threshold can be used which eliminates readings above g. This is not very accurate for an accelerating object.

## Using a gyro
Gyro can be independently used to measure the orientation as it returns the angular velocity which is multiplied by time to get orientation.

This system needs an initial position to fucntion and the sensor is very noisy and has a bias.

An integrating filter can be applied but the reading smoothly drifts over time due to the bias.

## Using sensor fusion
We have a trust line which decides which sensor system is given more emphasis.

Kalman filter decides this ratio automatically.

Acc/mag is used to set initial position. Mainly the gyro is used and acc/mag is used to remove bias slowly over time.



# Using GPS and IMU fusion
GPS returns the velocity and position. The drawbacks are that the frequency is low ~ 5Hz and the precison is also low ~ few metres. 

The perception of the state of the system gets corrupted between these updates. So we use both systems.

### Importance of Initialization
It is important to provide an initial bias to the system as it might result into the filter not converging at all.

<b>Prediction:</b> Kalman filter uses a model that is based both on prediction and measurements. This helps to eliminate noise.