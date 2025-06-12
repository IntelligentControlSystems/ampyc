'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2025, Intelligent Control Systems Group, ETH Zurich
%
% This code is made available under an MIT License (see LICENSE file).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

import numpy as np
import matplotlib.pyplot as plt

from ampyc.typing import Params
from ampyc.utils import Polytope

def plot_x_state_time(
        fig_number: int,
        x: np.ndarray,
        X: Polytope | None,
        params: Params,
        label: str | None = None,
        legend_loc: str = 'upper right',
        title: str | None = None,
        axes_labels: list[str] = ['x_1', 'x_2'],
        ) -> None:
    '''
    Plots the state trajectory x over time, including the state constraint set X.
    This function assumes a 2D state space (n=2) and plots the two state variables against time.

    Args:
        fig_number (int): The figure number to use for the plot. This allows multiple plots in the same figure.
        x (np.ndarray): The state trajectory, shape (N, n=2, T), where N is the number of time steps,
                        n is the state dimension, and T is the number of trajectories.
        X (Polytope | None): The state constraint set.
        params (Params): Parameters for plotting, e.g., color, alpha, and linewidth.
        label (str | None): Label for the plot line.
        legend_loc (str): Location of the legend in the plot.
        title (str | None): Title of the plot.
        axes_labels (list[str]): Labels for the x and y axes.
    '''

    # check if the figure number is already open
    if plt.fignum_exists(fig_number):
        fig = plt.figure(fig_number)
        ax1 = fig.axes[0]
        ax2 = fig.axes[1]
    else:
        fig, (ax1, ax2) = plt.subplots(2,1, num=fig_number, sharex=True)

    num_steps = x.shape[0]

    ax1.plot(x[:,0], color=params.color, alpha=params.alpha, linewidth=params.linewidth, label=label)
    if X is not None:
        ax1.axline((-1, X.vertices[:,0].max()), slope=0, color='k', linewidth=2)
        ax1.axline((-1, X.vertices[:,0].min()), slope=0, color='k', linewidth=2)
    ax1.set_ylabel(axes_labels[0])
    ax1.set_xlim([0, num_steps])
    ax1.grid(visible=True)
    if title is not None:
        ax1.set_title(title)

    if label is not None:
        # remove duplicate legend entries
        handles, labels = ax1.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax1.legend(by_label.values(), by_label.keys(), loc=legend_loc)
    
    ax2.plot(x[:,1], color=params.color, alpha=params.alpha, linewidth=params.linewidth)
    if X is not None:
        ax2.axline((-1, X.vertices[:,1].max()), slope=0, color='k', linewidth=2)
        ax2.axline((-1, X.vertices[:,1].min()), slope=0, color='k', linewidth=2)
    ax2.set_xlabel('time')
    ax2.set_ylabel(axes_labels[1])
    ax2.set_xlim([0, num_steps])
    ax2.grid(visible=True)

    fig.tight_layout()


def plot_x_state_state(fig_number: int,
                       x: np.ndarray,
                       X: Polytope | None,
                       params: Params,
                       label: str | None = None,
                       legend_loc: str = 'upper right',
                       title: str | None = None,
                       axes_labels: list[str] = ['x_1', 'x_2']
                       ) -> None:
    '''
    Plots the state trajectory x in the state space, including the state constraint set X.
    This function assumes a 2D state space (n=2) and plots the two state variables against each other.

    Args:
        fig_number (int): The figure number to use for the plot. This allows multiple plots in the same figure.
        x (np.ndarray): The state trajectory, shape (N, n=2, T), where N is the number of time steps,
                        n is the state dimension, and T is the number of trajectories.
        X (Polytope): The state constraint set.
        params (Params): Parameters for plotting, e.g., color, alpha, and linewidth.
        label (str | None): Label for the plot line.
        legend_loc (str): Location of the legend in the plot.
        title (str | None): Title of the plot.
        axes_labels (list[str]): Labels for the x and y axes.
    '''
    # check if the figure number is already open
    if plt.fignum_exists(fig_number):
        fig = plt.figure(fig_number)
        ax = fig.axes[0]
    else:
        fig = plt.figure(fig_number)
        ax = plt.gca()

    ax.scatter(x[0,0], x[0,1], marker='o', facecolors='none', color='k', label='initial state')
    ax.plot(x[:,0], x[:,1], color=params.color, alpha=params.alpha, linewidth=params.linewidth, label=label)
    if X is not None:
        X.plot(ax=ax, fill=False, edgecolor="k", alpha=1, linewidth=2, linestyle='-') 
    ax.set_xlabel(axes_labels[0])
    ax.set_ylabel(axes_labels[1])
    ax.grid(visible=True)

    if hasattr(params, 'zoom_out'):
        ax.set_xlim([i * params.zoom_out for i in X.xlim])
        ax.set_ylim([i * params.zoom_out for i in X.ylim])
    else:
        ax.set_xlim(X.xlim)
        ax.set_ylim(X.ylim)

    if title is not None:
        ax.set_title(title)

    # remove duplicate legend entries
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), loc=legend_loc)

    fig.tight_layout()
