import warnings
warnings.filterwarnings( "ignore", module = "matplotlib\..*" )
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
import numpy as np
import seaborn as sns
import general_purpose as gp

if __name__ == '__main__':
    labels = []

    df = pd.read_csv("./data/ground_truth.csv")
    df_initial = pd.read_csv("./data/initial_path.csv")

    df = df.dropna(axis=1, how='all')
    df = gp.transpose_by_iteration(df, ["x", "y", "z"])

    df_initial = df_initial.dropna(axis=1, how='all')
    df_initial = gp.transpose_by_iteration(df_initial, ["x", "y", "z"])
            
    mpl.rcParams['lines.linewidth'] = 2
    plt.style.use('seaborn-white')

    # plot individual plots
    # gp.plot_path(df, True, 20)
    #plot_average_iterations(df)
    # plt.scatter(-1 * df["y"], df["x"])
    fig, ax = plt.subplots()
    print(ax.spines.keys())
    ax.spines['top'].set_visible(True)
    ax.spines['right'].set_visible(True)
    gt, = ax.plot(-1 * df["y"], df["x"], label="Ground-Truth")
    odom, = ax.plot(-1 * df_initial["y"], df_initial["x"], label="Odometrie")
    #plt.set_style()

    plt.xlabel("Meter")
    plt.ylabel("Meter")
    
    ax.legend([gt, odom], ['Ground Truth', 'Odometry'])

    print(plt.style.available)



    #sns.despine()
    plt.show()
