import warnings
warnings.filterwarnings( "ignore", module = "matplotlib\..*" )
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import seaborn as sns
import general_purpose as gp

if __name__ == '__main__':
    labels = []

    df = pd.read_csv("./data/translation_error_gicp.csv")
    df = df.dropna(axis=1, how='all')
    #plot_average_error(df, label="lm", n_measurements=5, random=False)

    # plot individual plots
    gp.plot_error(df, ["relative", "absolute"], True)
    #plot_average_iterations(df)

    sns.despine()
    plt.show()