import general_purpose as gp
import warnings
warnings.filterwarnings( "ignore", module = "matplotlib\..*" )
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import seaborn as sns

if __name__ == '__main__':
    labels = []

    #df = pd.read_csv("./data/translation_error.csv")
    df = pd.read_csv("./data/graph_error.csv")
    df = df.dropna(axis=1, how='all')
    #plot_average_error(df, label="lm", n_measurements=5, random=False)


    #plot_error(dfs[0], n_measurements=57)
    #plot_error(dfs[1], n_measurements=5)

    # plot individual plots
    #plot_error(df, ["relative", "absolute"])
    gp.plot_error(df, ["error"], True)
    #plot_average_iterations(df)

    data = pd.DataFrame({'iteration': [1, 10, 2, 11], 'error': [0.5, 0.7, 1.0, 1.2], 'label': ['1','2','1','2']})
    #sns.lineplot(data=data, x="iteration", y="error", hue="label")

    sns.despine()
    plt.show()