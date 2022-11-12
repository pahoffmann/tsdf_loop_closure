import warnings
warnings.filterwarnings( "ignore", module = "matplotlib\..*" )
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
import numpy as np
import seaborn as sns
import general_purpose as gp
from mpl_toolkits import mplot3d

if __name__ == '__main__':
    labels = []

    df = pd.read_csv("./data/ground_truth.csv")
    #df_initial = pd.read_csv("./data/initial_path.csv")
    df_initial = pd.read_csv("./data/final_path.csv")
    #df_initial = pd.read_csv("./data/final_path_gicp.csv")

    df = df.dropna(axis=1, how='all')
    df = gp.transpose_by_iteration(df, ["x", "y", "z"])

    df_initial = df_initial.dropna(axis=1, how='all')
    df_initial = gp.transpose_by_iteration(df_initial, ["x", "y", "z"])
            
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    # ax.scatter(-1 * df["y"], df["x"])
    # ax.scatter(-1 * df_initial["y"], df_initial["x"])
    gt, = ax.plot(-1 * df["y"], df["x"], df["z"], label="Ground-Truth")
    odom, = ax.plot(-1 * df_initial["y"], df_initial["x"], df_initial["z"], label="Dieser Ansatz")
    #plt.set_style()

    ax.set_xlabel("Meter")
    ax.set_ylabel("Meter")
    ax.set_zlabel("Meter")
    
    ax.legend([gt, odom], ['Ground-Truth', 'GICP'])



    #sns.despine()
    plt.show()
