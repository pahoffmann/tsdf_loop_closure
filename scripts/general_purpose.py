import warnings
warnings.filterwarnings( "ignore", module = "matplotlib\..*" )
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import seaborn as sns
#sns.set_style('ggplot')
#sns.set_palette('Set2')


def transpose_by_iteration(data: pd.DataFrame, columns: [str]):
    """
    transpose each line of csv containing iteration and error for easier (non-average) plotting
    this is the result
         0  ...     2
    0  10.0  ...  10.0
    1   8.0  ...   8.0
    2   6.0  ...   6.0
    3   4.0  ...   5.0
    4   2.0  ...   4.0
    :param data original data
    :return: transposed data
    """
    df = data.transpose()
    df.columns = columns

    return df

def stack_by_iteration(data: pd.DataFrame):
    """
    build dataframe for average plot by stacking by iteration

    result after stacking
    "old_index iteration values"
    0     0    10.0
          1    10.0
          2    10.0
    1     0     8.0
          1     8.0

    :param data: original dataframe
    :return: data stacked dataframe, iteration as index
    """

    stacked = data.stack()
    # reset index and replace old labels
    stacked = stacked.reset_index()
    # level_1 includes all old labels, drop them
    stacked.drop(columns='level_1', inplace=True)
    stacked = stacked.rename({'level_0': 'iteration', 0: 'error'}, axis='columns')
    return stacked


def plot_error(df: pd.DataFrame, columns: [str]):
    data = transpose_by_iteration(df, columns=columns)
    sns.lineplot(data=data)
    plt.xticks(rotation="90")
    plt.xticks(np.arange(0, len(data.index), 1)[::10])


def plot_average_error(df: pd.DataFrame, label:str, n_measurements: int = None, random: bool = False):
    data = transpose_by_iteration(df, n_measurements=n_measurements, random=random)
    stacked = stack_by_iteration(data)
    stacked['label'] = [label]*stacked.shape[0]
    sns.lineplot(data=stacked, x="iteration", y="error", hue="label")


def plot_average_error_multi(dfs: [pd.DataFrame], labels: [str], n_measurements: int = None, random: bool = False):
    dfs_stacked = []
    for df, label in zip(dfs, labels):
        df = transpose_by_iteration(df, n_measurements=n_measurements, random=random)
        stacked = stack_by_iteration(df)
        stacked['label'] = [label]*stacked.shape[0]
        dfs_stacked.append(stacked)

    data = pd.concat(dfs_stacked, ignore_index=True)
    sns.lineplot(data=data, x="iteration", y="error", hue="label")


def plot_average_iterations(df):
    Label = ['Label']
    Mean = [ df.iterations.mean() ]
    _min = [ df.iterations.min() ]
    _max = [ df.iterations.max() ]
    df = pd.DataFrame({'Label':Label,'Mean':Mean, 'Min': _min, 'Max': _max})

    # create ymin and ymax
    df['ymin'] = df.Mean - df.Min
    df['ymax'] = df.Max - df.Mean

    # extract ymin and ymax into a (2, N) array as required by the yerr parameter
    yerr = df[['ymin', 'ymax']].T.to_numpy()

    # plot with error bars
    sns.barplot(x='Label', y='Mean', data=df, yerr=yerr)

