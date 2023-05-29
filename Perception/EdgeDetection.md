# Edge Detection
## Causes of Edges
<ol>
	<li>Surface Normal Discontinuity (Sudden change in orientation of planes)</li>
	<li>Depth Discontinuity (difference in distance from camera to two objects)</li>
	<li>Surface Reflectance Discontinuity (Change in material)</li>
	<li>Illuminaiton Discontinuity (Shadows or change in light intensity)</li>
</ol>

## What should an edge detector do?
It should provide: 
<ol>
	<li>Edge Postion</li>
	<li>Edge Magnitude (lateral width)</li>
	<li>Edge orientation</li>
</ol>

It shall have:
<ol>
	<li>High sensitivity</li>
	<li>Excellent noise cancellation</li>
	<li>Good Localisation</li>
</ol>


## Gradient Operator

An edge is a rapid change in light intensity in a small region

For a 1D image we simply apply a gradient operator to the given function and the peaks of the gradient map shall give us the edge. The height of the peaks is symbolic of the strength of the edges.

For a 2D image we the gradient operator transforms into a 2 element vector given by [I<sub>x</sub>, I<sub>y</sub>]. Where I is the intensity map of the image. 

The gradient magnitude is given by ||▽I|| = sqrt(I<sub>x</sub><sup>2</sup> + I<sub>y</sub><sup>2</sup>) and the direction by 𝜭 = tan<sup>-1</sup>(I<sub>y</sub>/I<sub>x</sub>)

This gradient operator can be implemented as convolutions:
<img src="Gradient.png">

#### Edge Thresholding:
If the magnitude is greater than a certain T then it is an edge.

Hysterisis: 
Not an edge if < T<sub>0</sub>
Edge if > T<sub>1</sub>
If between T<sub>0</sub> and T<sub>1</sub>, edge if connected to another edge.

