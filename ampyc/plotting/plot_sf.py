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
from .plot_quad_set import plot_quad_set

from ampyc.typing import Params
from ampyc.utils import Polytope

def plot_sf(fig_number: int,
            x: np.ndarray,
            X: Polytope,
            P: np.ndarray,
            params: Params,
            label: str | None = None,
            legend_loc: str = 'upper right',
            title: str | None = None,
            axes_labels: list[str] = ['x_1', 'x_2']
            ) -> None:
    '''
    Plots the safety filter ellipsoidal invariant set as well as constraints and state trajectory.
    This function assumes a 2D state space (n=2) and a 1D input space (m=1).

    Args:
        fig_number (int): The figure number to use for the plot. This allows multiple plots in the same figure.
        x (np.ndarray): The state trajectory, shape (N, n=2, T), where N is the number of time steps,
                        n is the state dimension, and T is the number of trajectories.
        X (Polytope): The state constraint set.
        P (np.ndarray): The shape matrix of the ellipsoidal invariant set.
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

    ax.scatter(x[0,0,:], x[0,1,:], marker='o', facecolors='none', color='k', label='initial state')
    ax.plot(x[:,0,:], x[:,1,:], color=params.color, alpha=params.alpha, linewidth=params.linewidth, label=label)

    X.plot(ax=ax, fill=False, edgecolor="k", alpha=1, linewidth=2, linestyle='-') 
    ax.set_xlabel(axes_labels[0])
    ax.set_ylabel(axes_labels[1])
    ax.grid(visible=True)
    

    plot_quad_set(ax, rho=1, P=P , xy=(0,0), label=None, alpha=0.4, facecolor='b', edgecolor='black', linewidth=1)

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