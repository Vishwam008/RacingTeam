# Machine Learning

## Introduction and Types
### Supervised
A set of inputs and their respective labels which are then used to create a model according towhich the labels of unseen inputs can be predicted

Usually used for regression and classification problems.

Maybe used for weather prediction, stock price pred. and sales analysis.

Techniques generally used are linear regression, logistic regression, k-nearest members and decision tree.

### Unsupervised
Only a set of inputs are given while the labels are absent. This method is mainly aimed at classifying the given inputs into groups.

K Means Clustering, Heiarchial Clustering, DBSCAN, principal component analysis.

Usually used for clustering and association problems

Used for customer segmentation.

### Reinforced
The computer is allowed to predict the label/output for inputs given one-by-one and in the process it is told if the predicted output is correct or not.

More frequently used in games.

### Classification based on required Output
<ol>
	<li>
		<h4>Classification</h4>
		A supervised problem wherein the inputs are divided into groups and fed to the learner and the learner is supposed to create a model based on which it can classify the unseen inputs into the already mentioned groups.
	</li>
	<li>
		<h4>Regression</h4>
		Supervised problem wherein the output is continuous rather than discrete. The learner makes a model based on given inputs which predicts a function for a range of unseen inputs rather than discrete labels for each.
	</li>
	<li>
		<h4>Clustering</h4>
		Unsupervised problem similar to Classification, but the groups to be classified into are not known and the inputs are given in a single bunch.
	</li>
</ol>


# Linear Regression
#### Independent and dependent variables:
Independent are the ones whose values do not depend on any other variables. Dependent are opposite.

#### Numerical and categorical variables.

#### Simple and Multiple Linear Regression
Simple is in one variable.

#### 2 important Factors:
<ol>
	<li>Which variables are significant predictors of the outcome variables?</li>
	<li>How significant is the Regression line to make predictions with highest possible accuracy</li>
</ol>

#### Regression line:
y = mx + c where y is the dependent variable and x is the independent variable.

and m is given by:
<img src="https://lh3.googleusercontent.com/drive-viewer/AFGJ81roEW_cLRcgoka_hgESD-4ZJupz_iMBh01LfWrO2OJ-fp1GuawBQBhDGn1ur3BW-F3HHPaSekbA185fPnKbdsITmpauyA=s2560">

The method used in this formula is least squares method where sumof squares of errors are minimized.



# Logistic Regression
This is a form of supervised learning and is used for classification.

Logistic regression predicts the probability of a binary event.

Odds: P(E) of event happening/P(E) of event not happening

We approximate the odds to an exponential function: p(x)/(1-p(x)) = e<sup>β<sub>o</sub>s+β<sub>1</sub>x</sup>

Similarly, the function p(x) can also be calculated. This results in a sigmoid curve. 


