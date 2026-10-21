"""Oriented-box overlap in a common world frame (15 separating axes)."""
import numpy as np


def boxes_overlap(first, size_first, second, size_second, margin=0.002):
    half_first, half_second = np.asarray(size_first)/2, np.asarray(size_second)/2
    delta = second.t-first.t
    if np.any(np.abs(delta) > np.abs(first.R)@half_first + np.abs(second.R)@half_second + margin):
        return False
    axes = np.vstack((first.R.T, second.R.T,
        np.cross(first.R.T[:, None, :], second.R.T[None, :, :]).reshape(-1, 3)))
    lengths = np.linalg.norm(axes, axis=1)
    axes = axes[lengths > 1e-8] / lengths[lengths > 1e-8, None]
    radii = np.abs(axes @ first.R) @ half_first + np.abs(axes @ second.R) @ half_second
    return not np.any(np.abs(axes @ delta) > radii + margin)
