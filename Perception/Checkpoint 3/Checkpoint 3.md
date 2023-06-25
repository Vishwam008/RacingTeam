# Checkpoint 3
## Difference between linear and logistic regression.
linear regression is used when a continuous set of outputs is needed.
Logistic regression is when discrete labelling of predictions is needed.

Linear is used for regression.
Logistic is used for classification.

## Why is one hot encoding used.
Some of the data we are working with would be categorical or non numerical. The computer is not able to process such data and thus encoding is needed to convert all such data to numbers to which a model can be applied.


## Overfitting
Overfitting is when the model has accurate predictions for the training dataset but not for the test data.


## Find the centre of the pink doughnut starting at any point on the doughnut
First is we can traverse all of the vicinity of the start point. The first white point that we will encounter will have a normal to the circle that passes through the start point.

We can traverse the normal and we will have four boundaries through which we can get the centre.


Second is we can apply edge detection and find all normals passing through the vicinity of the point. One normal would be very close to the starting point. We can find the centre using this normal.