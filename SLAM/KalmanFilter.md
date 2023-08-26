# Kalman Filters

## State Observers
Basically an error based system.

For example we have a quantity that we cannot measure directly but another measurable which is related to the quantity of interest.

We have another actuator input which affects both these quantities.

We build a mathematical nmodel that takes the actuator as input and gives the measurable(Tm) and immeasurable(Tim) as output.

These are just estimates as theoretical does not always coincide with reality. We try to tweak the mathematical model so that Tm converges to the actual measured Tm.

Doing so, Tim would also converge to the actual Tim.

In case we assume a car as a system wherein we have to find the position of the car, the GPS position becomes Tm and predicted becomes Tim.

The predicted and measured data and variance can be converted to probability distributions.

Kalman filter multiplies both the graphs to create a distribution with high probability and less variance.

<img src='Kalman.jpeg'>

Kalman Filter: 

<img src='Kalman 2.png'>

Here, Ax<sub>k-1</sub> + Bu<sub>k</sub> was a previous estimate (x<sub>k</sub>) (priori estimate) and can then be denoted by x<sup>- </sup>

The new estimate obtained is called the posterio estimate

