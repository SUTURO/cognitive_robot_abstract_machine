# %% imports
from __future__ import annotations

import logging
from dataclasses import dataclass, field

import pytest

from robokudo.descriptors.camera_configs.config_cv_camera_without_depth import (
    OpenCVCameraConfig,
)
from robokudo.io.camera_without_depth_interface import (
    TYPE_CAMERA,
    TYPE_VIDEO,
    OpenCVCameraWithoutDepthInterface,
)


# %% frame source mimic
@dataclass
class Clock:
    """
    Monotonic time the frame source advances while it grabs.
    """

    now: float = 0.0
    """
    Current time in seconds.
    """


@dataclass
class TimedFrameSource:
    """
    A capture whose grabs take the given durations, as buffered and freshly captured
    frames do.
    """

    clock: Clock
    """
    Clock advanced by every grab.
    """

    grab_seconds: list[float]
    """
    Duration of each successive grab.
    """

    grab_count: int = 0
    """
    Number of grabs so far.
    """

    def grab(self) -> bool:
        """
        :return: Whether a frame was grabbed, which is always the case.
        """
        self.clock.now += self.grab_seconds[self.grab_count]
        self.grab_count += 1
        return True


@dataclass
class ReaderWithSource:
    """
    An OpenCV reader together with the frame source it reads from.
    """

    reader: OpenCVCameraWithoutDepthInterface
    """
    The reader under test.
    """

    source: TimedFrameSource
    """
    The frame source the reader grabs from.
    """

    camera_config: OpenCVCameraConfig = field(default_factory=OpenCVCameraConfig)
    """
    Configuration of the reader.
    """


# %% helpers
def _reader(
    monkeypatch: pytest.MonkeyPatch, stream_type: int, grab_seconds: list[float]
) -> ReaderWithSource:
    clock = Clock()
    monkeypatch.setattr(
        "robokudo.io.camera_without_depth_interface.time.monotonic",
        lambda: clock.now,
    )
    source = TimedFrameSource(clock=clock, grab_seconds=grab_seconds)
    camera_config = OpenCVCameraConfig()
    reader = object.__new__(OpenCVCameraWithoutDepthInterface)
    reader.camera_config = camera_config
    reader.video_capture = source
    reader.stream_type = stream_type
    reader._has_new_data = False
    reader._backup_color = None
    reader.rk_logger = logging.getLogger("robokudo-opencv-buffer-test")
    return ReaderWithSource(reader=reader, source=source, camera_config=camera_config)


# %% buffered frames
def test_live_camera_skips_frames_buffered_since_the_last_read(monkeypatch):
    buffered = 0.0
    fresh = 0.033
    reader_with_source = _reader(
        monkeypatch, TYPE_CAMERA, [buffered, buffered, buffered, buffered, fresh]
    )

    assert reader_with_source.reader.has_new_data()
    assert reader_with_source.source.grab_count == 5


def test_live_camera_read_without_buffered_frames_grabs_once(monkeypatch):
    fresh = 0.033
    reader_with_source = _reader(monkeypatch, TYPE_CAMERA, [fresh, fresh])

    assert reader_with_source.reader.has_new_data()
    assert reader_with_source.source.grab_count == 1


def test_live_camera_stops_skipping_after_the_maximum_buffered_frames(monkeypatch):
    camera_config = OpenCVCameraConfig()
    instant_grabs = [0.0] * (camera_config.maximum_buffered_frames + 5)
    reader_with_source = _reader(monkeypatch, TYPE_CAMERA, instant_grabs)

    assert reader_with_source.reader.has_new_data()
    assert (
        reader_with_source.source.grab_count
        == reader_with_source.camera_config.maximum_buffered_frames + 1
    )


def test_video_file_never_skips_frames(monkeypatch):
    reader_with_source = _reader(monkeypatch, TYPE_VIDEO, [0.0, 0.0, 0.0])

    assert reader_with_source.reader.has_new_data()
    assert reader_with_source.source.grab_count == 1
