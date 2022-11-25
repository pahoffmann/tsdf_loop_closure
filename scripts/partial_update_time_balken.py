import warnings
warnings.filterwarnings( "ignore", module = "matplotlib\..*" )
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import seaborn as sns
import general_purpose as gp

if __name__ == '__main__':
    labels = []

    df = pd.read_csv("./data/partial_update_time.csv")
    df = df.dropna(axis=1, how='all')
    df = gp.transpose_by_iteration(df, ["total", "update", "removal", "shift", "vis"])
    #plot_average_error(df, label="lm", n_measurements=5, random=False)

    # convert to seconds?
    df["update"] /= 1000
    df["shift"] /= 1000
    df["removal"] /= 1000
    df["vis"] /= 1000
    df["total"] /= 1000

    total_time = np.sum(df["total"] + df["shift"])

    print("Total time: " + str(total_time))

    column_titles = ["update", "shift", "removal", "vis", "total"]
    df = df.reindex(columns=column_titles)            

    # plot individual plots
    #sns.barplot(data=df, errorbar="sd")
    sns.barplot(data=df, errorbar=('ci', 95))
    #plot_average_iterations(df)

    plt.xlabel("Pose-Index")
    plt.ylabel("Zeit [s]")

    sns.despine()
    plt.show()