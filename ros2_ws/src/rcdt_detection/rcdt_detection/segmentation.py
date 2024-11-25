# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path
from typing import Literal

from ultralytics.engine.model import Model
from ultralytics.engine.results import Results
from ultralytics import FastSAM, SAM
import numpy as np

PATH_FASTSAM: str = str(Path.home() / "models" / "FastSAM-x.pt")
PATH_SAM2: str = str(Path.home() / "models" / "sam2.1_b.pt")


def load_segmentation_model(model: Literal["SAM2", "FastSAM"] = "FastSAM") -> Model:
    """Load segmentation model from given path."""
    match model:
        case "FastSAM":
            return FastSAM(PATH_FASTSAM)
        case "SAM2":
            return SAM(PATH_SAM2)


def segment_image(model: Model, image: np.ndarray) -> Results:
    """Segment given image using given model."""
    if isinstance(model, FastSAM):
        height, width, _ = image.shape
        return model(image, imgsz=(height, width))[0]
    if isinstance(model, SAM):
        return model(image)[0]
