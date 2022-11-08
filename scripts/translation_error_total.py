import warnings
warnings.filterwarnings( "ignore", module = "matplotlib\..*" )
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import seaborn as sns
import general_purpose as gp

if __name__ == '__main__':
    labels = []

    df = pd.read_csv("./data/translation_error_total.csv")
    df = df.dropna(axis=1, how='all')
    df = gp.transpose_by_iteration(df, ["odometry", "gicp + loop-closure", "gicp"])
    #plot_average_error(df, label="lm", n_measurements=5, random=False)

    # plot individual plots
    gp.plot_error(df, True, 20)
    #plot_average_iterations(df)

    plt.xlabel("Pose-Index")
    plt.ylabel("Durschnittliche Distanz [m]")

    sns.despine()
    plt.show()