# Binary Image Analysis
An Image consisting 0s and 1s: 0 is the background and 1 is the foreground.

## Operations on Binary Images
<ol>
	<li>Aggregate pixels of the same object</li>
	<li>Distinguish foreground form background</li>
	<li>Identify the features of the object.</li>
	<li>Developing a threshold to convert greyscale into binary.</li>
</ol>


### Thresholding
Assuming that the histogram of pixel intensities is bimodal, the threshold shall be such that the in-group variances are minimized.

### Connected Component Analysis
Row-by-Row Operations:

We traverse each row in the image and label all the unconnected components as 1,2,3..

If in future iterations we see that the components 1 and 2 are connected then it is a conflict. We remember this conflict by saving 1=2 and so on further.

At the end we make a list of all the distinct components.

### Dilation
Dilation: Used for growing elements and removing holes and gaps.

For Dilation we need a structuring element S. S has an origin 1 and some 1s around the origin. This S is slid around the image like a convolution

If any one of S coincides with 1 of image then the element coinciding with origin is made 1.

S can be of any shape.

## Erosion
Erosion: Used for shrinking elements and removing bridges and protrusions.

We have structuring element S similar to that in erosion consisting of all 1s one of which is the origin.

We fit S in all places where 1s coincide with 1s only. In these places, the 1s at origin are preserved and rest everything is deleted.


## Opening
Erosion followed by dilation.
 

