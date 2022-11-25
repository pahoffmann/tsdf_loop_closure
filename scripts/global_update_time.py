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
    df = gp.transpose_by_iteration(df, ["Updates", "Shift", "Löschung", "Visualisierung"])
    #plot_average_error(df, label="lm", n_measurements=5, random=False)

    # convert to seconds?
    df["Updates"] /= 1000
    df["Shift"] /= 1000
    df["Löschung"] /= 1000
    df["Visualisierung"] /= 1000

    df["Gesamt"] = df["Updates"] + df["Shift"] + df["Löschung"] + df["Visualisierung"]

    total_time = np.sum(df["Gesamt"]) / 3600
    print("Total time: " + str(total_time))

    avg_time_per_pose = np.sum(df["Gesamt"] / df.index.astype(float)) / df["Gesamt"].size
    #df["PerPose"] = (df["Gesamt"] / df.index.astype(float)) * 1000

    print(avg_time_per_pose)

    # plot individual plots
    gp.plot_error(df, True, 10)
    plt.yticks([0, 50, 100, 150, 200, 250, 300, 350, 400, 450])
    #plot_average_iterations(df)

    plt.xlabel("Pose-Index")
    plt.ylabel("Zeit [s]")

    sns.despine()
    plt.show()