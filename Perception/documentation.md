# Digital image
Numeric representation of an image: can be a continuous function I(x, y) or discrete I(i, j).

A pixel is a picture element that contains the light intensity at some location (i,j). (A pixel can only have a particular intensity/colour) and thus will represent a certain numeric value I(i,j).

Thus an image can be represented as a matrix with its (i,j)<sup>th</sup> entry having value I(i,j).

One other way of representing an image is a plot on the x-y plane. The height of the function at any point (i,j) is I(i,j).

<b>Image histogram: </b>A plot representing the number of pixels at a particular intensity value. (Can be for the entire image, a specific part or just a channel)

We can also apply mean, mode, median to the obtained dataset.

 #### Channels
 Red Green or Blue: A channel is a dataset that maps the intensity of that particular colour to the pixel location.

 Now our image matrix is 3-d: Height ✖️ Width ✖️ Channel(3)

 The image format discussed above is the 'Raster image format' which stores images as a matrix of different channel intensities at a pixel. 

 GIF, JPG, PPM, TIF, BMP are common raster image formats.


 ## OpenCV
 Reading and Writing images:
 <code>
    
    import cv2
    import numpy as np
    img = cv2.imread('input.png')
    print(img.shape)            # returns height width and number of channels
    print(np.max(img))          # maximum value of intensity in the entire matrix: get an idea of bits per pixel
    cv2.imwrite('output.png', img) # making a file output.png and storing the img image
</code>

