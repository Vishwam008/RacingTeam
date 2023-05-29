# Digital image
Numeric representation of an image: can be a continuous function I(x, y) or discrete I(i, j).

A pixel is a picture element that contains the light intensity at some location (i,j). (A pixel can only have a particular intensity/colour) and thus will represent a certain numeric value I(i,j).

Thus an image can be represented as a matrix with its (i,j)<sup>th</sup> entry having value I(i,j).

One other way of representing an image is a plot on the x-y plane. The height of the function at any point (i,j) is I(i,j).

<b>Image histogram: </b>A plot representing the number of pixels at a particular intensity value. (Can be for the entire image, a specific part or just a channel)

We can also apply mean, mode, median to the obtained dataset.

 ### Channels
 Red Green or Blue: A channel is a dataset that maps the intensity of that particular colour to the pixel location.

 Now our image matrix is 3-d: Height ✖️ Width ✖️ Channel(3)

 The image format discussed above is the 'Raster image format' which stores images as a matrix of different channel intensities at a pixel. 

 GIF, JPG, PPM, TIF, BMP are common raster image formats.


 ### OpenCV
 Reading and Writing images:
 <code>
    import cv2
    import numpy as np
    img = cv2.imread('input.png')
    print(img.shape)            # returns height width and number of channels
    print(np.max(img))          # maximum value of intensity in the entire matrix: get an idea of bits per pixel
    cv2.imwrite('output.png', img) # making a file output.png and storing the img image
</code>

### Point Process- Pixel Arithmetic
Adding 2 images implies adding the intensities at a particular to pixel to the intensity of the corresponding pixel and likewise with subtraction

Intensities greater than 255 are given value 255 and those lower than 0 are given 0.

<b>Alpha Blending: </b>When we add 3 general images we are very likely to get an image that is pretty much white. This is because the probability of intensities adding up to >255 is very high and thus all of the a lot of pixels will be white.

This can be avoided by 'toning down' the 3 images by first multiplying the every image by an ⍺ such that ⍺ ∈ (0,1). This will result into a more visible picture with more pixels in the (0,255) range.

## Smoothening an Image
We change the value of each pixel to the average of that pixel and those around it. 

<b>Moving Average: </b>Average of all neighbouring and current pixel values
<b>Weighted moving average: </b>Average with more emphasis on the current pixel and lesser on the ones farther.

The value not on the edges can be easily smoothened by the above method.

For applying on edges we mirror the edges on onto a border covering the original image. This will allow us to apply the smoothening 'filter' onto edges as well.


## Kernels
Kernels are square matrices smaller than the image with size 2K+1 where K is the K-value of the filter.

A filter with K=1 will have a window size of 3.

<b>Generalization: </b>If h[i,j] is the filter and F[i,j] is the image then output pixel G[i,j] =  <sup>k</sup>∑<sub>u=-k</sub><sup>k</sup>∑<sub>v=-k</sub> h[u,v]F[i+u,j+v]


#### Kernals for smoothing:
We have a 3x3 matrix with all entries as 1/9(for normalization). 

The image becomes smooth.

#### Median Filtering: 
Replace the value of a pixel by the median of all the values in its 3x3 neighbourhood.

This eliminates noise but preserves sharp edges.

Can be used to reduce graininess of an image.


## Mathematical representation
We are taking a sliding inner product of the filter and the image.

<math>G[i,j] =  <sup>k</sup>∑<sub>u=-k</sub><sup>k</sup>∑<sub>v=-k</sub> h[u,v]F[i+u,j+v]</math> 

is represented as <math>h⊗F</math>

#### Gaussian Filter
Gives an image similar to the average filter but the edges are intermediate between the median and mean filters. It is basically an average with emphasis decreases away from centre. Fall is more in the corners.

## Convolution Method
The convolution method is represented as G[i,j] = 

<math>G[i,j] =  <sup>k</sup>∑<sub>u=-k</sub><sup>k</sup>∑<sub>v=-k</sub> h[u,v]F[i-u,j-v]</math> 

and it is denoted by <math>G = h * F</math>

It is given by reflecting the the cross-correlation filter about both the axes.

Properties:
<ol>
    <li>Depends on the pattern in the image rather than the position at which the filter is applied</li>
    <li>Applying filter J and K is the same as applying J*K</li>
    <li>There exists a unit filter E=[..0,0,1,0,0..] such that J*E=J</li>
</ol>

We can create a simple sharpening filter by increasing the emphasis on the pixel itself and having a negative emphasis on the neighbourhood which is the exact opposite of the average filter.

Similarly many such filters can be created by simple intuition.