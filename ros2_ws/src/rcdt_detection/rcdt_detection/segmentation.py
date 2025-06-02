# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path
from typing import Literal

import numpy as np
from ultralytics import SAM, FastSAM
from ultralytics.engine.model import Model
from ultralytics.engine.results import Results

PATH_FASTSAM: str = str(Path.home() / "models" / "FastSAM-x.pt")
PATH_SAM2: str = str(Path.home() / "models" / "sam2.1_b.pt")


def load_segmentation_model(model: Literal["SAM2", "FastSAM"] = "FastSAM") -> Model:
    """Load segmentation model from given path.

    Args:
        model (Literal["SAM2", "FastSAM"]): The model to load, either "SAM2" or "FastSAM".

    Returns:
        Model: The loaded segmentation model.
    """
    match model:
        case "FastSAM":
            return FastSAM(PATH_FASTSAM)
        case "SAM2":
            return SAM(PATH_SAM2)


def segment_image(model: Model, image: np.ndarray) -> Results:
    """Segment given image using given model.

    Args:
        model (Model): The segmentation model to use.
        image (np.ndarray): The input image to segment.

    Returns:
        Results: The segmentation results containing masks and other information.
    """
    if isinstance(model, FastSAM):
        height, width, _ = image.shape
        return model(image, imgsz=(height, width))[0]
    if isinstance(model, SAM):
        return model(image)[0]
