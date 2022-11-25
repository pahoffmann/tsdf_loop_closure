import warnings
warnings.filterwarnings( "ignore", module = "matplotlib\..*" )
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import seaborn as sns
import general_purpose as gp

if __name__ == '__main__':
    labels = []

    df = pd.read_csv("./data/translation_error_test.csv")
    df = df.dropna(axis=1, how='all')
    df = gp.transpose_by_iteration(df, ["relative", "absolute", "index"])
    #plot_average_error(df, label="lm", n_measurements=5, random=False)
    lc_indices = df["index"]
    absolute_error = df["absolute"]

    df = df.drop("index", axis=1)
    x = []

    cnt = 0
    previous_lc_index = -1
    for i in lc_indices:
        if i == 1:
            # check previous
            if cnt == 0 or absolute_error[cnt] / absolute_error[cnt - 1] < 0.9:
                x.append(cnt)
        cnt += 1
            


    # plot individual plots
    gp.plot_error(df, True, 20)
    #plot_average_iterations(df)

    plt.xlabel("Pose-Index")
    plt.ylabel("Durschnittliche Distanz [m]")

    plt.vlines(x = x, ymin = -1, ymax = 8,
           colors = 'red',
           label = 'vline_multiple - full height')

    sns.despine()
    plt.show()