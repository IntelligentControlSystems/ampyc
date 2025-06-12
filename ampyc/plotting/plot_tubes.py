'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2025, Intelligent Control Systems Group, ETH Zurich
%
% This code is made available under an MIT License (see LICENSE file).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

import numpy as np
from scipy.linalg import sqrtm
import matplotlib.pyplot as plt
from matplotlib import gridspec

from ampyc.typing import Params
from ampyc.utils import Polytope
from ampyc.plotting import plot_quad_set

def plot_tubes(fig_number: int,
               F: list[Polytope | np.ndarray],
               K: np.ndarray,
               X: Polytope | None,
               U: Polytope | None,
               params: Params,
               state_axes: list[str] = ['x_1', 'x_2'],
               input_axes: list[str] = ['u'],
               )-> None:
    '''
    Plots the state tubes and the corresponding state & input tightenings for a given sequence of polytopic tubes F.
    This function assumes a 2D state space (n=2) and a 1D input space (m=1).

    Args:
        fig_number (int): The figure number to use for the plot. This allows multiple plots in the same figure.
        F (list[Polytope | np.ndarray]): List of polytopic or ellipsoidal tubes.
        K (np.ndarray): The feedback gain matrix. This is needed to compute the input tightening.
        X (Polytope | None): The state constraint set. If None, no state constraint tightenings are plotted.
        U (Polytope | None): The input constraint set. If None, no input constraint tightenings are plotted.
        params (Params): Parameters for plotting, e.g., color, alpha, and linewidth.
        state_axes (list[str]): Labels for the state plot (x and y axes).
        input_axes (list[str]): Labels for the input plot (y axis).
    '''
    fig = plt.figure(num=fig_number, figsize=(11,6)) 
    gs = gridspec.GridSpec(1, 2, width_ratios=[1, 0.8])

    # Plot state tubes & tightening
    ax = plt.subplot(gs[0])
    if X is not None:
        X.plot(ax=ax, fill=False, edgecolor='k', alpha=1, linewidth=2, linestyle='-', label='original constraints')

    if type(F) is list:
        if type(F[0]) is Polytope:
            for i,F_i in enumerate(F):
                label = 'Tubes' if i == 0 else None
                if F_i.dim > 0:
                    F_i.plot(ax=ax, alpha=params.alpha(i), color=params.color, linewidth=0.5, linestyle='-', label=label)
                    if X is not None:
                        label = 'tightened constraints' if i == 0 else None
                        (X - F_i).plot(ax=ax, fill=False, edgecolor='k', alpha=1, linewidth=0.5, linestyle='--', label=label)
        elif type(F[0]) is np.ndarray:
            for i,F_i in enumerate(F):
                label = 'Tubes' if i == 0 else None
                plot_quad_set(ax=ax, P=F_i, rho=1, label=label, alpha=params.alpha(i), facecolor=params.color, linewidth=0.5)
                if X is not None:
                    label = 'tightened constraints' if i == 0 else None
                    x_tight = np.zeros((X.A.shape[0],))
                    inv_sqrt_F_i = np.linalg.inv(sqrtm(F_i))
                    for j in range(X.A.shape[0]):
                        x_tight[j] = np.linalg.norm(inv_sqrt_F_i @ X.A[j,:].reshape(-1,1), ord=2)
                    X_t = Polytope(A=X.A, b=X.b - x_tight)
                    X_t.plot(ax=ax, fill=False, edgecolor='k', alpha=1, linewidth=0.5, linestyle='--', label=label)
    else:
        if type(F) is Polytope:
            label = 'Tube'
            if F.dim > 0:
                F.plot(ax=ax, alpha=params.alpha, color=params.color, linewidth=0.8, linestyle='-', label=label)
                if X is not None:
                    label = 'tightened constraints'
                    (X - F).plot(ax=ax, fill=False, edgecolor='k', alpha=1, linewidth=0.8, linestyle='--', label=label)
        elif type(F) is np.ndarray:
            label = 'Tube'
            plot_quad_set(ax=ax, P=F, rho=1, label=label, alpha=params.alpha, facecolor=params.color, linewidth=0.8)
            if X is not None:
                label = 'tightened constraints'
                x_tight = np.zeros((X.A.shape[0],))
                inv_sqrt_F = np.linalg.inv(sqrtm(F))
                for j in range(X.A.shape[0]):
                    x_tight[j] = np.linalg.norm(inv_sqrt_F @ X.A[j,:].reshape(-1,1), ord=2)
                X_t = Polytope(A=X.A, b=X.b - x_tight)
                X_t.plot(ax=ax, fill=False, edgecolor='k', alpha=1, linewidth=0.8, linestyle='--', label=label)

    if X is not None:
        ax.set_xlim(X.xlim)
        ax.set_ylim(X.ylim)
    ax.set_xlabel(state_axes[0])
    ax.set_ylabel(state_axes[1])
    ax.grid(visible=True)

    # Plot input tubes & tightening
    ax = plt.subplot(gs[1])
    if U is not None:
        ax.axline((-1, U.V.max()), slope=0, color='k', linewidth=2, linestyle='-')
        ax.axline((-1, U.V.min()), slope=0, color='k', linewidth=2, linestyle='-')
    
    if type(F) is list:
        if type(F[0]) is Polytope:
            for i,F_i in enumerate(F):
                if F_i.dim > 0:
                    F_u = K @ F_i
                    ax.fill_between([-1, 25], F_u.V.min(), F_u.V.max(), color=params.color, alpha=params.alpha(i), linewidth=0.5)
                    if U is not None:
                        U_t = U - F_u
                        ax.axline((-1, U_t.V.max()), slope=0, color='k', linewidth=0.5, linestyle='--')
                        ax.axline((-1, U_t.V.min()), slope=0, color='k', linewidth=0.5, linestyle='--')
        elif type(F[0]) is np.ndarray:
            for i,F_i in enumerate(F):
                if U is not None:
                    u_tight = np.zeros((U.A.shape[0],))
                    inv_sqrt_F_i = np.linalg.inv(sqrtm(F_i))
                    for j in range(U.A.shape[0]):
                        u_tight[j] = np.linalg.norm(inv_sqrt_F_i @ K.T @ U.A[j,:].reshape(-1,1), ord=2)
                    F_u = Polytope(A=U.A, b=u_tight)
                    ax.fill_between([-1, 25], F_u.V.min(), F_u.V.max(), color=params.color, alpha=params.alpha(i), linewidth=0.5)
                    U_t = Polytope(A=U.A, b=U.b - u_tight)
                    ax.axline((-1, U_t.V.max()), slope=0, color='k', linewidth=0.5, linestyle='--')
                    ax.axline((-1, U_t.V.min()), slope=0, color='k', linewidth=0.5, linestyle='--')
    else:
        if type(F) is Polytope:
            F_u = K @ F
            ax.fill_between([-1, 25], F_u.V.min(), F_u.V.max(), color=params.color, alpha=params.alpha, linewidth=0.8)
            if U is not None:
                U_t = U - F_u
                ax.axline((-1, U_t.V.max()), slope=0, color='k', linewidth=0.8, linestyle='--')
                ax.axline((-1, U_t.V.min()), slope=0, color='k', linewidth=0.8, linestyle='--')
        elif type(F) is np.ndarray:
            if U is not None:
                u_tight = np.zeros((U.A.shape[0],))
                inv_sqrt_F = np.linalg.inv(sqrtm(F))
                for j in range(U.A.shape[0]):
                    u_tight[j] = np.linalg.norm(inv_sqrt_F @ K.T @ U.A[j,:].reshape(-1,1), ord=2)
                F_u = Polytope(A=U.A, b=u_tight)
                ax.fill_between([-1, 25], F_u.V.min(), F_u.V.max(), color=params.color, alpha=params.alpha, linewidth=0.8)
                U_t = Polytope(A=U.A, b=U.b - u_tight)
                ax.axline((-1, U_t.V.max()), slope=0, color='k', linewidth=0.8, linestyle='--')
                ax.axline((-1, U_t.V.min()), slope=0, color='k', linewidth=0.8, linestyle='--')
    
    ax.set_xlabel('time')
    ax.set_ylabel(input_axes[0])
    ax.set_xlim([0, 25])
    ax.set_ylim(U.xlim)
    ax.grid(visible=True)

    # Collect the labels and handles from the subplots
    all_handles = []
    all_labels = []
    for ax in fig.get_axes():
        handles, labels = ax.get_legend_handles_labels()
        all_handles.extend(handles)
        all_labels.extend(labels)
    ax.legend(all_handles, all_labels)

    fig.tight_layout
