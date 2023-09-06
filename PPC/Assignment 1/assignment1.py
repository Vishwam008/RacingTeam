import matplotlib.pyplot as plt
import numpy as np

Ind = np.array([21.44, 20.94, 20.42, 19.93, 19.05, 18.76, 18.51, 17.91, 17.65, 17.05, 16.57])
years = np.array(range(2010,2021))
Nep = np.array([22.59, 22.37, 22.34, 22.25, 22.22, 22.05, 21.76, 21.38, 21.03, 20.87, 20.64])
Bhu = np.array([19.54, 18.99, 18.54, 18.22, 17.74, 17.15, 16.08, 14.91, 13.79, 12.87, 12.65])

plt.plot(years, Ind, "*:r", label="India")
plt.plot(years, Nep, "*:b", label="Nepal")
plt.plot(years, Bhu, "*:g", label="Bhutan")
plt.xlabel("Years")
plt.ylabel("Birth Rate")
plt.legend()
plt.show()