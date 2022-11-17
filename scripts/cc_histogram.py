import warnings
warnings.filterwarnings( "ignore", module = "matplotlib\..*" )
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import seaborn as sns
import general_purpose as gp
from scipy import stats
import math
from statistics import NormalDist


def confidence_interval(data, confidence=0.95):
  dist = NormalDist.from_samples(data)
  z = NormalDist().inv_cdf((1 + confidence) / 2.)
  h = dist.stdev * z / ((len(data) - 1) ** .5)
  return dist.mean - h, dist.mean + h

if __name__ == '__main__':
    labels = []

    # df = pd.read_csv("./data/Histogram_point_to_point.csv", delimiter=';')
    # df = pd.read_csv("./data/Histogram_point_to_mesh.csv", delimiter=';')
    # df = pd.read_csv("./data/Histogram_point_to_mesh_hannover.csv", delimiter=';')
    df = pd.read_csv("./data/Histogram_point_to_point_hannover.csv", delimiter=';')
    df = df.dropna(axis=1, how='all')
    #df = df.transpose()

    # df[" Class start"] = df[" Class start"].div(1000)
    # df[" Class end"] = df[" Class end"].div(1000)
    #print(df[" Value"].size)
    x =  []
    for i in range(0, df[" Value"].size):
        x = np.concatenate([x, np.repeat((df[" Class start"][i] + (df[" Class end"][i] - df[" Class start"][i]) / 2), df[" Value"][i])])

    print(x)
    #ax = sns.histplot(x, kde=False, stat="density")
    plt.xlabel("absolute Distanz [mm]")
    plt.ylabel("Dichte [%]")

    mean = np.sum(x) / np.size(x)

    print("Mean: " + str(mean))
    
    # mu = 35.89
    # sigma = 54.698
    # values = np.linspace(mu - 3*sigma, mu + 3*sigma, 100)
    # plt.plot(values, stats.norm.pdf(values, mu, sigma), color="red", label="Gauss-Verteilung")
    mu, sigma = stats.norm.fit(x)
    print("sigma: " + str(sigma))
    print("mu: " + str(mu))

    plt.hist(x, bins=500, density=True, alpha=0.6, color='#1071e5')

    xmin, xmax = plt.xlim()
    values = np.linspace(xmin, xmax, 560)
    normal_pdf = stats.norm.pdf(values, mu, sigma)
    # plt.plot(values, stats.norm.pdf(values, mu, sigma), color="#e81313", label="Gauss-Verteilung")
    # plt.axvline(x = 15, color = 'green', label = '< 50%')
    # plt.axvline(x = 67, color = 'orange', label = '< 80%')
    # plt.axvline(x = 126, color = 'purple', label = '< 95%')
    plt.axvline(x = mu, color = '#ba23f6', label = 'Mittelwert')

    # cl, ch = stats.t.interval(0.95, len(normal_pdf)-1, loc=np.mean(normal_pdf), scale=stats.sem(normal_pdf))

    # print("95% low: " + str(cl)) 
    # print("95% high: " + str(ch)) 
    # plt.axvline(x = cl, color = '#fc9432', label = '95% Konfidenzintervall')
    # plt.axvline(x = ch, color = '#fc9432', label = '95% Konfidenzintervall')


    # calculate the pdf
    # x0, x1 = ax.get_xlim()  # extract the endpoints for the x-axis
    # x0 = 0
    # x_pdf = np.linspace(x0, x1, 100)
    # y_pdf = stats.norm.pdf(x_pdf)

    # ax.plot(x_pdf, y_pdf, 'r', lw=2, label='pdf')                                                   
    # ax.legend()
    plt.legend()
    plt.show()