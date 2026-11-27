"""
An analysis engine running in a process of its own, beside the program that queries it.
"""

from __future__ import annotations

# %% Imports

import signal
import subprocess
import sys
import threading
from dataclasses import dataclass, field
from datetime import timedelta
from enum import StrEnum

from krrood.patterns.classproperty import classproperty

from robokudo.exceptions import AnalysisEngineDidNotStart

# %% Analysis engines to choose from


class AnalysisEngineModule(StrEnum):
    """
    The modules of the analysis engines in RoboKudo that answer queries about what the
    head camera of a robot sees.
    """

    HUMAN_DETECTION = "human_detection"
    """
    Detects humans and tells their posture.
    """

    GPSR = "gpsr.gpsr_implementation"
    """
    Answers the queries of the General Purpose Service Robot task.
    """


# %% The process


class InterpreterOption(StrEnum):
    """
    The options of the Python interpreter an analysis engine is started with.
    """

    UNBUFFERED = "-u"
    """
    Writes the output as it is produced.
    """

    MODULE = "-m"
    """
    Runs a module as the main script.
    """


class MainScriptOption(StrEnum):
    """
    The options of :mod:`robokudo.scripts.main` an analysis engine is started with.
    """

    ANALYSIS_ENGINE = "_ae"
    """
    Names the module of the analysis engine.
    """

    ROS_PACKAGE = "_ros_pkg"
    """
    Names the ROS package holding the analysis engine.
    """

    HEADLESS = "_headless"
    """
    Runs the analysis engine without opening windows.
    """


@dataclass(kw_only=True)
class AnalysisEngineProcess:
    """
    An analysis engine in a separate process whose output is forwarded to this process.
    """

    analysis_engine: str
    """
    The module of the analysis engine below ``descriptors/analysis_engines`` of
    :attr:`ros_package`, such as ``gpsr.gpsr_implementation``.
    """

    ros_package: str = "robokudo"
    """
    The ROS package holding the analysis engine.
    """

    shows_views: bool = True
    """
    Whether RoboKudo opens its windows showing what the analysis engine sees and found.
    """

    startup_timeout: timedelta = timedelta(minutes=2)
    """
    How long :meth:`start` waits for the analysis engine to run. An analysis engine that
    downloads its models on its first start needs longer.
    """

    _process: subprocess.Popen | None = field(init=False, default=None)
    """
    The running process, None before :meth:`start`.
    """

    _running: threading.Event = field(init=False, default_factory=threading.Event)
    """
    Set once the process announced that it runs the analysis engine.
    """

    @classproperty
    def main_script(cls) -> str:
        """
        The module of :mod:`robokudo.scripts.main`, which runs an analysis engine.

        :return: The module name passed to the Python interpreter.
        """
        return "robokudo.scripts.main"

    @classproperty
    def running_announcement(cls) -> str:
        """
        The start of the line :mod:`robokudo.scripts.main` logs right before it ticks the
        analysis engine for the first time.

        :return: The text identifying the startup announcement.
        """
        return "Running AE named"

    def start(self) -> None:
        """
        Start the analysis engine and wait until it runs, at most
        :attr:`startup_timeout`.

        A query sent before then is never answered, since RoboKudo accepts it before the
        analysis engine can take it.

        :raises AnalysisEngineDidNotStart: If it does not run in time.
        """
        self._process = subprocess.Popen(
            [
                sys.executable,
                InterpreterOption.UNBUFFERED,
                InterpreterOption.MODULE,
                self.main_script,
                MainScriptOption.ANALYSIS_ENGINE,
                self.analysis_engine,
                MainScriptOption.ROS_PACKAGE,
                self.ros_package,
                *([] if self.shows_views else [MainScriptOption.HEADLESS]),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        threading.Thread(target=self._forward_output, daemon=True).start()
        if not self._running.wait(self.startup_timeout.total_seconds()):
            self.stop()
            raise AnalysisEngineDidNotStart(
                analysis_engine=self.analysis_engine,
                timeout_seconds=self.startup_timeout.total_seconds(),
            )

    def _forward_output(self) -> None:
        """
        Write every line the process outputs to this process's output, and set
        :attr:`_running` once the process announced that it runs the analysis engine.
        """
        for line in self._process.stdout:
            sys.stdout.write(line)
            if self.running_announcement in line:
                self._running.set()

    def stop(self) -> None:
        """
        Stop the analysis engine the way Ctrl+C does, and wait until its process ended.
        Does nothing if the process did not start or has already ended.
        """
        if self._process is None or self._process.poll() is not None:
            return
        self._process.send_signal(signal.SIGINT)
        self._process.wait()
