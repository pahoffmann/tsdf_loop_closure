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

    df = pd.read_csv("./data/vorplatz_path_corrected_3d.csv")
    df_initial = pd.read_csv("./data/vorplatz_initial_path.csv")

    df = pd.read_csv("./data/physik_final_path.csv")
    df_initial = pd.read_csv("./data/physik_initial_path.csv")

    df = df.dropna(axis=1, how='all')
    df = gp.transpose_by_iteration(df, ["x", "y", "z"])

    df_initial = df_initial.dropna(axis=1, how='all')
    df_initial = gp.transpose_by_iteration(df_initial, ["x", "y", "z"])
            
    fig = plt.figure(figsize=(16,7))
    ax = fig.add_subplot(projection='3d')

    # ax.scatter(-1 * df["y"], df["x"])
    # ax.scatter(-1 * df_initial["y"], df_initial["x"])
    gt, = ax.plot(df["x"], df["y"], df["z"], label="Korrigiert")
    odom, = ax.plot(df_initial["x"], df_initial["y"], df_initial["z"], label="Initialschätzung")
    #plt.set_style()

    ax.set_xlabel("Meter")
    ax.set_ylabel("Meter")
    ax.set_zlabel("Meter")
    
    ax.legend([gt, odom], ['Korrigiert', 'Initialschätzung'])



    #sns.despine()
    plt.show()
