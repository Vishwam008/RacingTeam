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



## Laplacian Operator and 2nd derivative
We will now use the second derivative to detect edges. For this we use the Laplacian operator:
<img src="https://lh3.googleusercontent.com/drive-viewer/AFGJ81rVadytNEEFQh1fN_2fKzGFtAdRySoSx5JL-xrjcIXXUjHiy2SnD6I0Ht0D92MVRwToxSrVzdbz7L-M3fHo2n4u7GIw=s2560">
At any edge the laplacian changes suddenly from a positive to a negative value called zero-crossings.

To an image with discrete pixels this can be applied using the follwing formula: 
<img src="https://lh3.googleusercontent.com/drive-viewer/AFGJ81p925uO58K2770-FCSMlsiqOf09vkldFK_h2Yrz0k39z-4E5y6DghdUSv_qoifKZQhUtEGKhMnu4wHIZZAROsElR3us=s1600">

As a convolution it can be represented as:
<img src="https://lh3.googleusercontent.com/drive-viewer/AFGJ81oqMJMyg7HTE6rErlfntok5PIz8t0mZFiiYwJJzOdqmj5N86iISH7OSKWawW4fNO64DerKMabfNS1jtO_9hmUMptxaFmw=s1600">

However, the image on the right only takes into account the pixels in 4 directions. This is incorrect as edges can also be at 45°. So we have a more accurate convolution on the right which takes into account diagonal pixels too.

### Noise Cancellation
Noise can very disruptive to filters using derivatives as they juct amplify the noise. Noise can be eliminated by first applying the Gaussian filter to smooth the curve and then the derivative/Laplacian.

Rather than applying one by one, these can be combined by matrix multiplication of both the convolutions.

## Difference between Laplacian and Derivative
<ol>
	<li>Laplacian requires only one operation while derivative requires two</li>
	<li>Derivative can tell the strength, direction and location of the image, laplacian only location</li>
</ol>

We thus need a combination of both to have both of their benefits.

## Canny Edge Detector
It includes the following steps:
<ol>
	<li>Smoothen Image with 2d Gaussian filter n<sub>𝜎</sub> * I</li>
	<li>Apply the Sobel derivative filter ∇n<sub>𝜎</sub> * I</li>
	<li>Obtain the Gradient orientation: </li>
	<img src="https://lh3.googleusercontent.com/drive-viewer/AFGJ81pk6k-DUnX2YzcRn7dB2jPeozjAq-shft3dWhA1eWMcX9n7QA1VLjBsqX_m1NiA93kLV_7q6wrWvF9I6czhIA-CFSkwVg=s2560">
	<li>Compute 1D Laplacian in the direction of the gradient.</li>
	<img src="https://lh3.googleusercontent.com/drive-viewer/AFGJ81pv8vGIU2OQTI16O9jgPVFQmRkfmn7lEsyMFQyIiRGFa5EUuMgH-ChHBpmya5wPmtvOHVpNG0KoO0t8FeHgBkB0frmsTA=s2560">
</ol>

The smoothening can be altered for detection of finer or coarse edges.