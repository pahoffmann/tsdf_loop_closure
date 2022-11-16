import warnings
warnings.filterwarnings( "ignore", module = "matplotlib\..*" )
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import seaborn as sns
import general_purpose as gp

if __name__ == '__main__':
    labels = []

    df = pd.read_csv("./data/global_update_time.csv")
    df = df.dropna(axis=1, how='all')
    df = gp.transpose_by_iteration(df, ["update", "shift", "removal", "vis"])
    #plot_average_error(df, label="lm", n_measurements=5, random=False)

    # convert to seconds?
    df["update"] /= 1000
    df["shift"] /= 1000
    df["removal"] /= 1000
    df["vis"] /= 1000

    df["total"] = df["update"] + df["shift"] + df["removal"] + df["vis"]

    total_time = np.sum(df["total"])
    print("Total time: " + str(total_time))

    # plot individual plots
    sns.barplot(data=df)

    plt.xlabel("Pose-Index")
    plt.ylabel("Zeit [s]")

    sns.despine()
    plt.show()