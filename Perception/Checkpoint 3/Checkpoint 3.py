
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from datetime import datetime

df = pd.read_csv('forecast.csv')
df.head()
df['Dates'] = pd.to_datetime(df['Dates'])
from pmdarima import auto_arima
  
import warnings
warnings.filterwarnings("ignore")
  
train = df.iloc[:-480]
test = df.iloc[-480:] # set one year(12 months) for testing
  
# Fit a SARIMAX(0, 1, 1)x(2, 1, 1, 12) on the training set
from statsmodels.tsa.statespace.sarimax import SARIMAX
  
model = SARIMAX(train['Occupancy'], 
                order = (1, 1, 0), 
                seasonal_order =(2, 1, 1, 365))
  
result = model.fit()
result.summary()


start = len(train)
end = len(train) + len(test) - 1
print(start, end)
# Predictions for one-year against the test set
predictions = result.predict(start, end,
                             typ = 'levels').rename("Predictions")
  
# plot predictions and actual values
predictions.plot(legend = True)
test['Occupancy'].plot(legend = True)
print(predictions)
from sklearn.metrics import mean_squared_error
from statsmodels.tools.eval_measures import rmse
  

print(rmse(test["Occupancy"], predictions))
  

print(mean_squared_error(test["Occupancy"], predictions))

