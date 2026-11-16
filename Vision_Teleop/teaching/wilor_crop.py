"""WiLoR's inference-only Gaussian blur using the same finite separable kernel.

Matches skimage gaussian's uint8, preserve_range, channel_axis=2 defaults:
float64 arithmetic, nearest border, truncate=4. No resizing or crop changes.
The pinned upstream module remains untouched.
"""
import cv2
import numpy as np


def gaussian(image, sigma, *, channel_axis, preserve_range):
    if (image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3
            or channel_axis != 2 or preserve_range is not True
            or not np.isscalar(sigma) or not np.isfinite(sigma) or sigma <= 0):
        raise ValueError('Only the pinned WiLoR inference blur contract is supported')
    radius = int(4.0 * sigma + 0.5)
    x = np.arange(-radius, radius + 1, dtype=np.float64)
    kernel = np.exp(-0.5 / sigma**2 * x**2)
    kernel /= kernel.sum()
    # scipy applies vertical then horizontal passes; preserve that order.
    source = image.astype(np.float64)
    vertical = cv2.sepFilter2D(source, -1, np.ones(1), kernel, borderType=cv2.BORDER_REPLICATE)
    return cv2.sepFilter2D(vertical, -1, kernel, np.ones(1), borderType=cv2.BORDER_REPLICATE)
