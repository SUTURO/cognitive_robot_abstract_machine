"""
This is the camera config for a openCV camera without depth information.
"""

from __future__ import annotations

# %% Imports
from krrood.patterns.classproperty import classproperty
from dataclasses import dataclass, field

import cv2
from sensor_msgs.msg import CameraInfo
from typing_extensions import TYPE_CHECKING

from robokudo.descriptors.camera_configs.base_camera_config import BaseCameraConfig
from robokudo.types.cv import ColorToDepthRatio

if TYPE_CHECKING:
    import numpy.typing as npt


@dataclass(slots=True)
class OpenCVCameraConfig(BaseCameraConfig):
    """
    Configuration of a color source read through OpenCV's video capture: a physical
    camera, a video file, an image sequence or a network stream.

    .. note::
        When using image sequences as input, the first image must have a number
        between 0 and 4, and there cannot be any gaps in the sequence numbering.
    """

    @classproperty
    def registry_name(cls) -> str:
        """
        Identifier selecting this camera configuration.
        """
        return "opencv"

    interface_type: str = "OpenCV"
    """
    Camera interface reading this source.
    """
    device: int | str = 0
    """
    Input source identifier (camera index, file path, or URL).

    device (integer for I/O device or path to image/video file) and flag
    integer:  id of the video capturing device to open
              Use 0 to open default camera using default backend
    string:   path to a video file
              path to image sequence (e.g. 'my/path/img_%02d.jpg').
                  Note: First image needs number between 0 and 4 and the following numbers cannot contain any gaps.
              URL of video/camera stream or image
    """
    api_preference: int = cv2.CAP_ANY
    """
    Preferred OpenCV capture API backend.
    """
    device_driver_flag: int = 0
    """
    Flag argument to use when retrieving/reading the frames.
    """
    buffered_frame_grab_seconds: float = 0.005
    """
    Duration below which grabbing a live camera frame counts as taking one the driver
    buffered since the last read, so a newer frame is grabbed instead.
    """
    maximum_buffered_frames: int = 10
    """
    Most buffered live camera frames skipped before reading one.
    """
    normalize_rgb: bool = True
    """
    Normalize/stabilise rgb image contrast/brightness.
    """
    loop_mode: int = -1
    """
    Repetitions after the last frame of a file source: positive for that many, zero for
    none, negative for endless.

    .. note::
        Only works with a file as source
    """
    depth: npt.NDArray | None = None
    """
    Static depth image, which is used as depth image for all rgb images.
    """
    update_global_with_depth_parameter: bool = True
    """
    Flag to update global depth parameters.
    """
    camera_info: CameraInfo | None = None
    """
    Calibration matching the configured capture resolution.
    """
    camera_frame: str = "opencv_optical_frame"
    """
    Optical frame used when :attr:`camera_info` declares no frame.
    """
    camera_intrinsic = None
    """
    Camera intrinsic parameters.
    """
    color_to_depth_ratio: ColorToDepthRatio = field(default_factory=ColorToDepthRatio)
    """
    Size of the depth image relative to the color image.
    """
    viewpoint_camera_to_world = None
    """
    Camera to world transformation.
    """
