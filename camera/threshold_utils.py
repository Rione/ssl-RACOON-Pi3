"""HSV threshold helpers for live tuning."""

import numpy as np

from camera.settings import parse_threshold, threshold_to_string


def relax_arrays(min_arr, max_arr, h=3, s=15, v=15):
    """Expands min/max HSV ranges slightly (more permissive detection)."""
    min_out = np.array(
        [
            max(0, int(min_arr[0]) - h),
            max(0, int(min_arr[1]) - s),
            max(0, int(min_arr[2]) - v),
        ],
        dtype=np.uint8,
    )
    max_out = np.array(
        [
            min(179, int(max_arr[0]) + h),
            min(255, int(max_arr[1]) + s),
            min(255, int(max_arr[2]) + v),
        ],
        dtype=np.uint8,
    )
    return min_out, max_out


def arrays_to_strings(min_arr, max_arr):
    return threshold_to_string(min_arr), threshold_to_string(max_arr)


def strings_to_arrays(min_str, max_str, settings=None):
    settings = settings or {}
    default_min = np.array([1, 120, 100], dtype=np.uint8)
    default_max = np.array([15, 255, 255], dtype=np.uint8)
    return (
        parse_threshold(min_str, default_min),
        parse_threshold(max_str, default_max),
    )
