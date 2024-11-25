# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import numpy as np


def is_brick(mask: np.ndarray) -> bool:
    """Determine if given mask belongs to a brick using mean pixel thresholds."""
    b, g, r = np.nanmean(np.where(mask == 0.0, np.nan, mask), axis=(0, 1))
    return (b > (r + 100.0)) and (b > (g + 10.0))


def is_blue(mask: np.ndarray) -> bool:
    """Determine if given mask belongs to a blue object using mean pixel thresholds."""
    b, g, r = np.nanmean(np.where(mask == 0.0, np.nan, mask), axis=(0, 1))
    return (b > (r - 5.0)) and (b > (g - 5.0))
