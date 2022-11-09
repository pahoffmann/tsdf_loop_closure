import warnings
warnings.filterwarnings( "ignore", module = "matplotlib\..*" )
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import seaborn as sns
import general_purpose as gp

if __name__ == '__main__':
    labels = []

    df = pd.read_csv("./data/fitness_scores_full.csv")
    df = df.dropna(axis=1, how='all')
    df = gp.transpose_by_iteration(df, ["x = 0", "x = 1", "x = 2","x = 3","x = 4","x = 5","x = 6",])
    #plot_average_error(df, label="lm", n_measurements=5, random=False)
            
    # plot individual plots
    gp.plot_error(df, True,10)
    #plot_average_iterations(df)

    plt.xlabel("Pose-Index")
    plt.ylabel("Durchschnittlicher Fitness-Score")

    plt.vlines(x = [6,7,8], ymin = -1, ymax = 8,
        colors = 'red',
        label = 'vline_multiple - full height')

    plt.hlines(y = [0.3], xmin = 0, xmax = 12,
        colors = 'green',
        label = 'hline_multiple - full height')

    sns.despine()
    plt.show()