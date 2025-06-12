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
from matplotlib.patches import Ellipse

def plot_quad_set(ax: plt.Axes,
                  rho: float,
                  P: np.ndarray,
                  xy: tuple[float] = (0.,0.),
                  label: str | None = None,
                  alpha: float = 0.4,
                  facecolor: str = 'blue',
                  edgecolor: str = 'black',
                  linewidth: float = 1.,
                  ) -> None:
    '''
    Plot the 2D quadratic set of the form :math: `x'Px <= \rho` on the provided plt.Axes object.

    Args:
        ax (plt.Axes): The axes on which to plot the ellipse.
        rho (float): The sublevel set of the quadratic form.
        P (np.ndarray): The 2x2 positive definite matrix defining the quadratic form.
        xy (tuple[float]): The center of the ellipse.
        label (str | None): Label for the ellipse.
        alpha (float): Transparency of the ellipse.
        facecolor (str): Color of the ellipse face.
        edgecolor (str): Color of the ellipse edge.
        linewidth (float): Width of the ellipse edge.
    '''
    # Compute the eigenvalues and eigenvectors of P
    eigvals, eigvecs = np.linalg.eig(P)

    # Compute the semi-axes of the ellipse
    a = np.sqrt(rho / eigvals[0])
    b = np.sqrt(rho / eigvals[1])

    # Compute the angle of rotation of the ellipse
    theta = np.degrees(np.arctan2(*eigvecs[:,0][::-1]))

    # Create an Ellipse object
    ellipse = Ellipse(xy=xy, width=2*a, height=2*b, angle=theta, alpha=alpha, edgecolor=edgecolor, facecolor=facecolor,lw=2, label=label, linewidth=linewidth)
    ax.add_patch(ellipse)
    