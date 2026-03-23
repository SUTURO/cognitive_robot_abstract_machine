#!/usr/bin/env python

from __future__ import annotations

import os
import signal
import sys
from dataclasses import dataclass, field
from typing import List, Optional

import rclpy
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QFileDialog,
    QMessageBox,
    QProgressBar,
    QScrollArea,
    QGridLayout,
    QGroupBox,
    QCheckBox,
    QLabel,
    QDialog,
    QLineEdit,
    QDoubleSpinBox,
)

from giskardpy.middleware.ros2 import rospy
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
    ShapeSource,
)
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Color
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.robots.abstract_robot import (
    Neck,
    Arm,
    KinematicChain,
    HumanoidGripper,
    ParallelGripper,
    Finger,
    Camera,
    FieldOfView,
    Manipulator,
    Sensor,
)
from semantic_digital_twin.robots.robot_mixins import (
    HasNeck,
    HasArms,
    SpecifiesLeftRightArm,
)


@dataclass
class RobotSemanticAnnotationBuilderInterface:
    """
    Interface for managing robot semantic annotations.
    """

    world: World = field(init=False)
    """
    World instance for managing bodies.
    """

    def __post_init__(self):
        self.world = World()
        with self.world.modify_world():
            self.world.add_body(Body(name=PrefixedName("map")))
        VizMarkerPublisher(
            _world=self.world, node=rospy.node, shape_source=ShapeSource.COLLISION_ONLY
        ).with_tf_publisher()

    def load_urdf(self, urdf_path: str):
        """
        Loads a URDF file and merges it into the world.
        """
        robot_world = URDFParser.from_file(urdf_path).parse()
        with self.world.modify_world():
            self.world.clear()
            self.world.add_body(map_body := Body(name=PrefixedName("map")))
            self.world.merge_world(
                robot_world, FixedConnection(parent=map_body, child=robot_world.root)
            )

    def reset_body_colors(self):
        """
        Sets all body colors to white transparent.
        """
        with self.world.modify_world():
            for body in self.world.bodies_with_collision:
                for shape in body.collision.shapes:
                    shape.color = Color(1.0, 1.0, 1.0, 0.5)

    def highlight_body(self, body: Body):
        """
        Highlights the given body by changing its color.
        """
        highlight_color = Color(1.0, 0.0, 0.0, 1.0)
        self.reset_body_colors()
        with self.world.modify_world():
            body.collision.dye_shapes(highlight_color)

    @property
    def bodies(self) -> List[Body]:
        """
        Returns a sorted list of all bodies with collision.
        """
        return list(sorted(self.world.bodies_with_collision, key=lambda x: x.name.name))


@dataclass
class BodyButton(QPushButton):
    """
    A button representing a robot body.
    """

    body: Body
    application: Application

    def __post_init__(self):
        super().__init__(self.body.name.name)
        self.clicked.connect(self.on_click)
        self.setMinimumHeight(40)

    def on_click(self):
        """
        Callback for button click.
        """
        self.application.interface.highlight_body(self.body)
        self.application.last_highlighted_body = self.body
        if self.application.selection_mode:
            self.application.handle_body_selection(self.body)


class ProgressBarWithText(QProgressBar):
    """
    A progress bar that displays text.
    """

    def set_progress(self, value: int, text: Optional[str] = None):
        """
        Sets the progress value and optional text.
        """
        value = int(min(max(value, 0), 100))
        self.setValue(value)
        if text is not None:
            self.setFormat(f"{text}: %p%")
        self.parent().repaint()


@dataclass
class ComponentSelectionDialog(QDialog):
    """
    Dialog for choosing robot components.
    """

    has_neck: bool = False
    has_arms: bool = False
    specifies_left_right_arm: bool = False

    def __post_init__(self):
        super().__init__()
        self.setWindowTitle("Choose robot components")
        self.init_ui()

    def init_ui(self):
        """
        Initialize the dialog UI.
        """
        layout = QVBoxLayout()

        self.has_neck_checkbox = QCheckBox("HasNeck")
        self.has_neck_checkbox.setChecked(self.has_neck)
        self.has_arms_checkbox = QCheckBox("HasArms")
        self.has_arms_checkbox.setChecked(self.has_arms)
        self.specifies_left_right_arm_checkbox = QCheckBox("SpecifiesLeftRightArm")
        self.specifies_left_right_arm_checkbox.setChecked(self.specifies_left_right_arm)

        self.has_arms_checkbox.toggled.connect(self._handle_has_arms_toggled)
        self.specifies_left_right_arm_checkbox.toggled.connect(
            self._handle_specifies_left_right_arm_toggled
        )

        layout.addWidget(self.has_neck_checkbox)
        layout.addWidget(self.has_arms_checkbox)
        layout.addWidget(self.specifies_left_right_arm_checkbox)

        self.ok_button = QPushButton("OK")
        self.ok_button.clicked.connect(self.accept)
        layout.addWidget(self.ok_button)

        self.setLayout(layout)

    def _handle_has_arms_toggled(self, checked: bool):
        """
        Handles HasArms checkbox toggle, ensuring mutual exclusivity.
        """
        if checked:
            self.specifies_left_right_arm_checkbox.setChecked(False)

    def _handle_specifies_left_right_arm_toggled(self, checked: bool):
        """
        Handles SpecifiesLeftRightArm checkbox toggle, ensuring mutual exclusivity.
        """
        if checked:
            self.has_arms_checkbox.setChecked(False)

    def get_selection(self) -> dict:
        """
        Returns the current selection.
        """
        return {
            "HasNeck": self.has_neck_checkbox.isChecked(),
            "HasArms": self.has_arms_checkbox.isChecked(),
            "SpecifiesLeftRightArm": self.specifies_left_right_arm_checkbox.isChecked(),
        }


@dataclass
class TypeSelectionDialog(QDialog):
    """
    Dialog for choosing between multiple types.
    """

    title: str
    options: List[str]
    selected_type: Optional[str] = field(init=False, default=None)

    def __post_init__(self):
        super().__init__()
        self.setWindowTitle(self.title)
        self.init_ui()

    def init_ui(self):
        """
        Initialize the UI.
        """
        layout = QVBoxLayout()
        for option in self.options:
            button = QPushButton(option)
            button.clicked.connect(
                lambda checked, opt=option: self.on_option_selected(opt)
            )
            layout.addWidget(button)
        self.setLayout(layout)

    def on_option_selected(self, option: str):
        """
        Callback for when an option is selected.
        """
        self.selected_type = option
        self.accept()


@dataclass
class FingerWidget(QGroupBox):
    """
    Widget for managing finger properties.
    """

    application: Application
    finger: Finger
    parent_widget: QWidget

    def __post_init__(self):
        super().__init__(self.finger.name.name)
        self.init_ui()

    def init_ui(self):
        """
        Initialize the UI.
        """
        layout = QVBoxLayout()
        remove_button = QPushButton("Remove Finger")
        remove_button.clicked.connect(
            lambda: self.application.remove_finger(self.finger)
        )
        layout.addWidget(remove_button)

        root_name = self.finger.root.name.name if self.finger.root else "None"
        root_button = QPushButton(f"Root Body: {root_name}")
        root_button.clicked.connect(
            lambda: self.application.assign_body_to_finger_root(self.finger)
        )
        layout.addWidget(root_button)

        tip_name = self.finger.tip.name.name if self.finger.tip else "None"
        tip_button = QPushButton(f"Tip Body: {tip_name}")
        tip_button.clicked.connect(
            lambda: self.application.assign_body_to_finger_tip(self.finger)
        )
        layout.addWidget(tip_button)

        self.setLayout(layout)


@dataclass
class ManipulatorWidget(QGroupBox):
    """
    Widget for managing manipulator properties.
    """

    application: Application
    arm: Arm
    manipulator: Manipulator

    def __post_init__(self):
        super().__init__(f"Manipulator: {self.manipulator.__class__.__name__}")
        self.init_ui()

    def init_ui(self):
        """
        Initialize the UI.
        """
        layout = QVBoxLayout()
        remove_button = QPushButton("Remove Manipulator")
        remove_button.clicked.connect(
            lambda: self.application.remove_manipulator(self.arm)
        )
        layout.addWidget(remove_button)

        root_name = self.manipulator.root.name.name if self.manipulator.root else "None"
        root_button = QPushButton(f"Root Body: {root_name}")
        root_button.clicked.connect(
            lambda: self.application.assign_body_to_manipulator_root(self.arm)
        )
        layout.addWidget(root_button)

        tool_name = (
            self.manipulator.tool_frame.name.name
            if self.manipulator.tool_frame
            else "None"
        )
        tool_button = QPushButton(f"Tool Frame: {tool_name}")
        tool_button.clicked.connect(
            lambda: self.application.assign_body_to_manipulator_tool_frame(self.arm)
        )
        layout.addWidget(tool_button)

        add_finger_button = QPushButton("Add Finger")
        add_finger_button.clicked.connect(
            lambda: self.application.add_finger_to_manipulator(self.arm)
        )
        layout.addWidget(add_finger_button)

        if isinstance(self.manipulator, ParallelGripper):
            if self.manipulator.finger:
                add_finger_button.setEnabled(False)
                layout.addWidget(
                    FingerWidget(self.application, self.manipulator.finger, self)
                )
            if self.manipulator.thumb:
                layout.addWidget(
                    FingerWidget(self.application, self.manipulator.thumb, self)
                )
        elif isinstance(self.manipulator, HumanoidGripper):
            for finger in self.manipulator.fingers:
                layout.addWidget(FingerWidget(self.application, finger, self))
            if self.manipulator.thumb:
                layout.addWidget(
                    FingerWidget(self.application, self.manipulator.thumb, self)
                )

        self.setLayout(layout)


@dataclass
class SensorWidget(QGroupBox):
    """
    Widget for managing sensor properties.
    """

    application: Application
    arm: Arm
    sensor: Sensor

    def __post_init__(self):
        super().__init__(f"Sensor: {self.sensor.name.name}")
        self.init_ui()

    def init_ui(self):
        """
        Initialize the UI.
        """
        layout = QVBoxLayout()
        remove_button = QPushButton("Remove Sensor")
        remove_button.clicked.connect(
            lambda: self.application.remove_sensor(self.arm, self.sensor)
        )
        layout.addWidget(remove_button)

        root_name = self.sensor.root.name.name if self.sensor.root else "None"
        root_button = QPushButton(f"Root Body: {root_name}")
        root_button.clicked.connect(
            lambda: self.application.assign_body_to_sensor_root(self.sensor)
        )
        layout.addWidget(root_button)

        if isinstance(self.sensor, Camera):
            min_height_layout = QHBoxLayout()
            min_height_layout.addWidget(QLabel("Min Height:"))
            min_height_spin = QDoubleSpinBox()
            min_height_spin.setValue(self.sensor.minimal_height)
            min_height_spin.valueChanged.connect(
                lambda val: setattr(self.sensor, "minimal_height", val)
            )
            min_height_layout.addWidget(min_height_spin)
            layout.addLayout(min_height_layout)

            max_height_layout = QHBoxLayout()
            max_height_layout.addWidget(QLabel("Max Height:"))
            max_height_spin = QDoubleSpinBox()
            max_height_spin.setValue(self.sensor.maximal_height)
            max_height_spin.valueChanged.connect(
                lambda val: setattr(self.sensor, "maximal_height", val)
            )
            max_height_layout.addWidget(max_height_spin)
            layout.addLayout(max_height_layout)

            default_checkbox = QCheckBox("Default Camera")
            default_checkbox.setChecked(self.sensor.default_camera)
            default_checkbox.toggled.connect(
                lambda checked: setattr(self.sensor, "default_camera", checked)
            )
            layout.addWidget(default_checkbox)

            if not self.sensor.field_of_view:
                self.sensor.field_of_view = FieldOfView(0.0, 0.0)

            fov_v_layout = QHBoxLayout()
            fov_v_layout.addWidget(QLabel("FOV Vertical:"))
            fov_v_spin = QDoubleSpinBox()
            fov_v_spin.setValue(self.sensor.field_of_view.vertical_angle)
            fov_v_spin.valueChanged.connect(
                lambda val: setattr(self.sensor.field_of_view, "vertical_angle", val)
            )
            fov_v_layout.addWidget(fov_v_spin)
            layout.addLayout(fov_v_layout)

            fov_h_layout = QHBoxLayout()
            fov_h_layout.addWidget(QLabel("FOV Horizontal:"))
            fov_h_spin = QDoubleSpinBox()
            fov_h_spin.setValue(self.sensor.field_of_view.horizontal_angle)
            fov_h_spin.valueChanged.connect(
                lambda val: setattr(self.sensor.field_of_view, "horizontal_angle", val)
            )
            fov_h_layout.addWidget(fov_h_spin)
            layout.addLayout(fov_h_layout)

        self.setLayout(layout)


@dataclass
class NeckWidget(QGroupBox):
    """
    Widget for managing neck properties.
    """

    application: Application

    def __post_init__(self):
        super().__init__("Neck")
        self.init_ui()

    def init_ui(self):
        """
        Initializes the UI.
        """
        layout = QVBoxLayout()
        remove_button = QPushButton("Remove Neck")
        remove_button.clicked.connect(self.application.remove_neck)
        layout.addWidget(remove_button)

        pitch_body_name = (
            self.application.neck.pitch_body.name.name
            if self.application.neck.pitch_body
            else "None"
        )
        pitch_button = QPushButton(f"pitch body: {pitch_body_name}")
        pitch_button.clicked.connect(self.application.assign_body_to_neck_pitch)
        layout.addWidget(pitch_button)

        yaw_body_name = (
            self.application.neck.yaw_body.name.name
            if self.application.neck.yaw_body
            else "None"
        )
        yaw_button = QPushButton(f"yaw body: {yaw_body_name}")
        yaw_button.clicked.connect(self.application.assign_body_to_neck_yaw)
        layout.addWidget(yaw_button)

        root_body_name = (
            self.application.neck.root.name.name
            if self.application.neck.root
            else "None"
        )
        root_button = QPushButton(f"Root Body: {root_body_name}")
        root_button.clicked.connect(self.application.assign_body_to_neck_root)
        layout.addWidget(root_button)

        tip_body_name = (
            self.application.neck.tip.name.name if self.application.neck.tip else "None"
        )
        tip_button = QPushButton(f"Tip Body: {tip_body_name}")
        tip_button.clicked.connect(self.application.assign_body_to_neck_tip)
        layout.addWidget(tip_button)

        visualize_button = QPushButton("Visualize")
        visualize_button.setEnabled(
            self.application.neck.root is not None
            and self.application.neck.tip is not None
        )
        visualize_button.clicked.connect(
            lambda: self.application.visualize_chain(self.application.neck)
        )
        layout.addWidget(visualize_button)

        self.setLayout(layout)


@dataclass
class ArmWidget(QGroupBox):
    """
    Widget for managing arm properties.
    """

    application: Application
    arm: Arm

    def __post_init__(self):
        super().__init__("Arm")
        self.init_ui()

    def init_ui(self):
        """
        Initializes the UI.
        """
        layout = QVBoxLayout()
        remove_button = QPushButton("Remove Arm")
        remove_button.clicked.connect(lambda: self.application.remove_arm(self.arm))
        layout.addWidget(remove_button)

        root_body_name = self.arm.root.name.name if self.arm.root else "None"
        root_button = QPushButton(f"Root Body: {root_body_name}")
        root_button.clicked.connect(
            lambda: self.application.assign_body_to_arm_root(self.arm)
        )
        layout.addWidget(root_button)

        tip_body_name = self.arm.tip.name.name if self.arm.tip else "None"
        tip_button = QPushButton(f"Tip Body: {tip_body_name}")
        tip_button.clicked.connect(
            lambda: self.application.assign_body_to_arm_tip(self.arm)
        )
        layout.addWidget(tip_button)

        visualize_button = QPushButton("Visualize")
        visualize_button.setEnabled(
            self.arm.root is not None and self.arm.tip is not None
        )
        visualize_button.clicked.connect(
            lambda: self.application.visualize_chain(self.arm)
        )
        layout.addWidget(visualize_button)

        add_manipulator_button = QPushButton("Add Manipulator")
        add_manipulator_button.setEnabled(self.arm.manipulator is None)
        add_manipulator_button.clicked.connect(
            lambda: self.application.add_manipulator_callback(self.arm)
        )
        layout.addWidget(add_manipulator_button)

        add_sensor_button = QPushButton("Add Sensor")
        add_sensor_button.clicked.connect(
            lambda: self.application.add_sensor_callback(self.arm)
        )
        layout.addWidget(add_sensor_button)

        self.setLayout(layout)


@dataclass
class Application(QMainWindow):
    """
    The main application for the robot semantic annotation builder tool.
    """

    interface: RobotSemanticAnnotationBuilderInterface = field(
        init=False, default_factory=RobotSemanticAnnotationBuilderInterface
    )
    """
    Reference to a RobotSemanticAnnotationBuilderInterface instance.
    """
    timer: QTimer = field(init=False, default_factory=QTimer)
    """
    Timer used to update the ui periodically.
    """
    chosen_components: dict = field(
        default_factory=lambda: {
            "HasNeck": False,
            "HasArms": False,
            "SpecifiesLeftRightArm": False,
        }
    )
    """
    Dictionary storing the state of chosen robot components.
    """
    selection_mode: Optional[str] = None
    """
    The current selection mode for body assignment.
    """
    neck: Optional[Neck] = None
    """
    The neck of the robot.
    """
    arms: List[Arm] = field(default_factory=list)
    """
    The arms of the robot.
    """

    def __post_init__(self):
        super().__init__()
        self.timer.start(1000)
        self.timer.timeout.connect(lambda: None)
        self.init_ui_components()

    def init_ui_components(self):
        """
        Initialize all ui components.
        """
        self.setWindowTitle("Robot Semantic Annotation Builder Tool")
        self.setMinimumSize(1000, 600)

        self.urdf_progress = ProgressBarWithText(self)
        self.urdf_progress.set_progress(0, "No urdf loaded")

        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.body_buttons_widget = QWidget()
        self.body_buttons_layout = QGridLayout(self.body_buttons_widget)
        self.body_buttons_layout.setSpacing(0)
        self.body_buttons_layout.setContentsMargins(0, 0, 0, 0)
        self.body_buttons_layout.setAlignment(Qt.AlignTop)
        self.scroll_area.setWidget(self.body_buttons_widget)

        left_layout = QVBoxLayout()
        left_layout.addLayout(self._create_urdf_box_layout())
        self.selection_label = QLabel("")
        self.selection_label.setStyleSheet("font-weight: bold; color: blue;")
        left_layout.addWidget(self.selection_label)
        left_layout.addWidget(self.scroll_area)

        main_layout = QHBoxLayout()
        main_layout.addLayout(left_layout, stretch=4)

        self.right_scroll_area = QScrollArea()
        self.right_scroll_area.setWidgetResizable(True)
        self.right_widget = QWidget()
        self.right_layout = self._create_chosen_components_layout()
        self.right_widget.setLayout(self.right_layout)
        self.right_scroll_area.setWidget(self.right_widget)

        main_layout.addWidget(self.right_scroll_area, stretch=1)

        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

    def _create_chosen_components_layout(self) -> QVBoxLayout:
        """
        Creates the layout for chosen components on the right.
        """
        layout = QVBoxLayout()
        self.choose_components_button = QPushButton("Choose Robot Components")
        self.choose_components_button.setEnabled(False)
        self.choose_components_button.clicked.connect(
            self._open_component_selection_dialog
        )
        layout.addWidget(self.choose_components_button)

        self.chosen_components_label = QLabel("Chosen Components:")
        layout.addWidget(self.chosen_components_label)

        self.components_list_layout = QVBoxLayout()
        layout.addLayout(self.components_list_layout)
        layout.addStretch()
        return layout

    def _open_component_selection_dialog(self):
        """
        Opens the component selection dialog.
        """
        dialog = ComponentSelectionDialog(
            has_neck=self.chosen_components["HasNeck"],
            has_arms=self.chosen_components["HasArms"],
            specifies_left_right_arm=self.chosen_components["SpecifiesLeftRightArm"],
        )
        if dialog.exec_():
            self.chosen_components = dialog.get_selection()
            self._update_chosen_components_list()

    def _update_chosen_components_list(self):
        """
        Updates the list of chosen components on the right.
        """
        while self.components_list_layout.count():
            item = self.components_list_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()

        for component, chosen in self.chosen_components.items():
            if chosen:
                self.components_list_layout.addWidget(QLabel(f"- {component}"))
                if component == "HasNeck":
                    add_neck_button = QPushButton("Add Neck")
                    add_neck_button.setEnabled(self.neck is None)
                    add_neck_button.clicked.connect(self._add_neck_callback)
                    self.components_list_layout.addWidget(add_neck_button)

                if component == "HasArms":
                    add_arm_button = QPushButton("Add Arm")
                    add_arm_button.clicked.connect(self._add_arm_callback)
                    self.components_list_layout.addWidget(add_arm_button)

                if component == "SpecifiesLeftRightArm":
                    add_left_arm_button = QPushButton("Add Left Arm")
                    add_left_arm_button.setEnabled(
                        not any(a.name.name == "left_arm" for a in self.arms)
                    )
                    add_left_arm_button.clicked.connect(
                        lambda: self._add_arm_callback("left_arm")
                    )
                    self.components_list_layout.addWidget(add_left_arm_button)

                    add_right_arm_button = QPushButton("Add Right Arm")
                    add_right_arm_button.setEnabled(
                        not any(a.name.name == "right_arm" for a in self.arms)
                    )
                    add_right_arm_button.clicked.connect(
                        lambda: self._add_arm_callback("right_arm")
                    )
                    self.components_list_layout.addWidget(add_right_arm_button)

        if self.neck:
            self.components_list_layout.addWidget(NeckWidget(self))

        for arm in self.arms:
            self.components_list_layout.addWidget(ArmWidget(self, arm))
            if arm.manipulator:
                self.components_list_layout.addWidget(
                    ManipulatorWidget(self, arm, arm.manipulator)
                )
            for sensor in arm.sensors:
                self.components_list_layout.addWidget(SensorWidget(self, arm, sensor))

    def _add_neck_callback(self):
        """
        Callback for adding a neck.
        """
        self.neck = Neck(name=PrefixedName("neck"))
        self._update_chosen_components_list()

    def _add_arm_callback(self, name: Optional[str] = None):
        """
        Callback for adding an arm.
        """
        arm_name = name if name else f"arm_{len(self.arms)}"
        arm = Arm(name=PrefixedName(arm_name))
        self.arms.append(arm)
        self._update_chosen_components_list()

    def remove_neck(self):
        """
        Removes the neck.
        """
        self.neck = None
        self._update_chosen_components_list()

    def remove_arm(self, arm: Arm):
        """
        Removes the given arm.
        """
        self.arms.remove(arm)
        self._update_chosen_components_list()

    def add_manipulator_callback(self, arm: Arm):
        """
        Callback for adding a manipulator to an arm.
        """
        dialog = TypeSelectionDialog(
            "Choose Manipulator Type", ["HumanoidGripper", "ParallelGripper"]
        )
        if dialog.exec_():
            if dialog.selected_type == "HumanoidGripper":
                arm.manipulator = HumanoidGripper(
                    name=PrefixedName(f"{arm.name.name}_gripper"),
                    tool_frame=None,
                    front_facing_orientation=None,
                    front_facing_axis=None,
                    thumb=Finger(name=PrefixedName(f"{arm.name.name}_thumb")),
                )
            elif dialog.selected_type == "ParallelGripper":
                arm.manipulator = ParallelGripper(
                    name=PrefixedName(f"{arm.name.name}_gripper"),
                    tool_frame=None,
                    front_facing_orientation=None,
                    front_facing_axis=None,
                    finger=None,
                    thumb=Finger(name=PrefixedName(f"{arm.name.name}_thumb")),
                )
            self._update_chosen_components_list()

    def add_finger_to_manipulator(self, arm: Arm):
        """
        Adds a finger to a manipulator.
        """
        if isinstance(arm.manipulator, HumanoidGripper):
            finger_name = (
                f"{arm.manipulator.name.name}_finger_{len(arm.manipulator.fingers)}"
            )
            arm.manipulator.fingers.append(Finger(name=PrefixedName(finger_name)))
            self._update_chosen_components_list()
        elif isinstance(arm.manipulator, ParallelGripper):
            if arm.manipulator.finger is None:
                finger_name = f"{arm.manipulator.name.name}_finger"
                arm.manipulator.finger = Finger(name=PrefixedName(finger_name))
                self._update_chosen_components_list()

    def remove_finger(self, finger: Finger):
        """
        Removes a finger from its manipulator.
        """
        for arm in self.interface.robot.arms:
            if arm.manipulator:
                if isinstance(arm.manipulator, HumanoidGripper):
                    if finger in arm.manipulator.fingers:
                        arm.manipulator.fingers.remove(finger)
                        self._update_chosen_components_list()
                        return
                elif isinstance(arm.manipulator, ParallelGripper):
                    if arm.manipulator.finger == finger:
                        arm.manipulator.finger = None
                        self._update_chosen_components_list()
                        return

    def remove_manipulator(self, arm: Arm):
        """
        Removes the manipulator from an arm.
        """
        arm.manipulator = None
        self._update_chosen_components_list()

    def add_sensor_callback(self, arm: Arm):
        """
        Callback for adding a sensor to an arm.
        """
        dialog = TypeSelectionDialog("Choose Sensor Type", ["Camera"])
        if dialog.exec_():
            if dialog.selected_type == "Camera":
                sensor_name = f"{arm.name.name}_camera_{len(arm.sensors)}"
                camera = Camera(name=PrefixedName(sensor_name))
                arm.sensors.append(camera)
            self._update_chosen_components_list()

    def remove_sensor(self, arm: Arm, sensor: Sensor):
        """
        Removes a sensor from an arm.
        """
        arm.sensors.remove(sensor)
        self._update_chosen_components_list()

    def assign_body_to_manipulator_root(self, arm: Arm):
        """
        Starts selection mode for manipulator root body.
        """
        self.selection_mode = f"manipulator_root_{arm.name.name}"
        self.selection_label.setText(
            f"Select Root Body for {arm.manipulator.name.name}"
        )

    def assign_body_to_manipulator_tool_frame(self, arm: Arm):
        """
        Starts selection mode for manipulator tool frame.
        """
        self.selection_mode = f"manipulator_tool_{arm.name.name}"
        self.selection_label.setText(
            f"Select Tool Frame for {arm.manipulator.name.name}"
        )

    def assign_body_to_finger_root(self, finger: Finger):
        """
        Starts selection mode for finger root body.
        """
        self.selection_mode = f"finger_root_{finger.name.name}"
        self.selection_label.setText(f"Select Root Body for {finger.name.name}")

    def assign_body_to_finger_tip(self, finger: Finger):
        """
        Starts selection mode for finger tip body.
        """
        self.selection_mode = f"finger_tip_{finger.name.name}"
        self.selection_label.setText(f"Select Tip Body for {finger.name.name}")

    def assign_body_to_sensor_root(self, sensor: Sensor):
        """
        Starts selection mode for sensor root body.
        """
        self.selection_mode = f"sensor_root_{sensor.name.name}"
        self.selection_label.setText(f"Select Root Body for {sensor.name.name}")

    def assign_body_to_neck_pitch(self):
        """
        Starts selection mode for neck pitch body.
        """
        self.selection_mode = "neck_pitch"
        self.selection_label.setText("Select Pitch Body for Neck")

    def assign_body_to_neck_yaw(self):
        """
        Starts selection mode for neck yaw body.
        """
        self.selection_mode = "neck_yaw"
        self.selection_label.setText("Select Yaw Body for Neck")

    def assign_body_to_neck_root(self):
        """
        Starts selection mode for neck root body.
        """
        self.selection_mode = "neck_root"
        self.selection_label.setText("Select Root Body for Neck")

    def assign_body_to_neck_tip(self):
        """
        Starts selection mode for neck tip body.
        """
        self.selection_mode = "neck_tip"
        self.selection_label.setText("Select Tip Body for Neck")

    def assign_body_to_arm_root(self, arm: Arm):
        """
        Starts selection mode for arm root body.
        """
        self.selection_mode = f"arm_root_{arm.name.name}"
        self.selection_label.setText(f"Select Root Body for {arm.name.name}")

    def assign_body_to_arm_tip(self, arm: Arm):
        """
        Starts selection mode for arm tip body.
        """
        self.selection_mode = f"arm_tip_{arm.name.name}"
        self.selection_label.setText(f"Select Tip Body for {arm.name.name}")

    def handle_body_selection(self, body: Body):
        """
        Handles body selection based on the current mode.
        """
        if not self.selection_mode:
            return

        if self.selection_mode == "neck_pitch" and self.neck:
            self.neck.pitch_body = body
        elif self.selection_mode == "neck_yaw" and self.neck:
            self.neck.yaw_body = body
        elif self.selection_mode == "neck_root" and self.neck:
            self.neck.root = body
        elif self.selection_mode == "neck_tip" and self.neck:
            self.neck.tip = body
        elif self.selection_mode.startswith("arm_root_"):
            arm_name = self.selection_mode.replace("arm_root_", "")
            arm = next((a for a in self.arms if a.name.name == arm_name), None)
            if arm:
                arm.root = body
        elif self.selection_mode.startswith("arm_tip_"):
            arm_name = self.selection_mode.replace("arm_tip_", "")
            arm = next((a for a in self.arms if a.name.name == arm_name), None)
            if arm:
                arm.tip = body

        elif self.selection_mode.startswith("manipulator_root_"):
            arm_name = self.selection_mode.replace("manipulator_root_", "")
            arm = next((a for a in self.arms if a.name.name == arm_name), None)
            if arm and arm.manipulator:
                arm.manipulator.root = body
        elif self.selection_mode.startswith("manipulator_tool_"):
            arm_name = self.selection_mode.replace("manipulator_tool_", "")
            arm = next((a for a in self.arms if a.name.name == arm_name), None)
            if arm and arm.manipulator:
                arm.manipulator.tool_frame = body
        elif self.selection_mode.startswith("finger_root_"):
            finger_name = self.selection_mode.replace("finger_root_", "")
            # Look for finger in all arms/manipulators
            for arm in self.arms:
                if arm.manipulator:
                    if isinstance(arm.manipulator, ParallelGripper):
                        if (
                            arm.manipulator.finger
                            and arm.manipulator.finger.name.name == finger_name
                        ):
                            arm.manipulator.finger.root = body
                        if (
                            arm.manipulator.thumb
                            and arm.manipulator.thumb.name.name == finger_name
                        ):
                            arm.manipulator.thumb.root = body
                    elif isinstance(arm.manipulator, HumanoidGripper):
                        for f in arm.manipulator.fingers:
                            if f.name.name == finger_name:
                                f.root = body
                        if (
                            arm.manipulator.thumb
                            and arm.manipulator.thumb.name.name == finger_name
                        ):
                            arm.manipulator.thumb.root = body
        elif self.selection_mode.startswith("finger_tip_"):
            finger_name = self.selection_mode.replace("finger_tip_", "")
            for arm in self.arms:
                if arm.manipulator:
                    if isinstance(arm.manipulator, ParallelGripper):
                        if (
                            arm.manipulator.finger
                            and arm.manipulator.finger.name.name == finger_name
                        ):
                            arm.manipulator.finger.tip = body
                        if (
                            arm.manipulator.thumb
                            and arm.manipulator.thumb.name.name == finger_name
                        ):
                            arm.manipulator.thumb.tip = body
                    elif isinstance(arm.manipulator, HumanoidGripper):
                        for f in arm.manipulator.fingers:
                            if f.name.name == finger_name:
                                f.tip = body
                        if (
                            arm.manipulator.thumb
                            and arm.manipulator.thumb.name.name == finger_name
                        ):
                            arm.manipulator.thumb.tip = body
        elif self.selection_mode.startswith("sensor_root_"):
            sensor_name = self.selection_mode.replace("sensor_root_", "")
            for arm in self.arms:
                for sensor in arm.sensors:
                    if sensor.name.name == sensor_name:
                        sensor.root = body

        self.selection_mode = None
        self.selection_label.setText("")
        self._update_chosen_components_list()

    def visualize_chain(self, chain: KinematicChain):
        """
        Visualizes the kinematic chain.
        """
        self.interface.reset_body_colors()
        highlight_color = Color(1.0, 0.0, 0.0, 1.0)
        left_chain, root, right_chain = (
            self.interface.world.compute_split_chain_of_kinematic_structure_entities(
                chain.root, chain.tip
            )
        )

        full_chain: List[Body] = left_chain + root + right_chain
        with self.interface.world.modify_world():
            for body in full_chain:
                body.collision.dye_shapes(highlight_color)

    def _create_urdf_box_layout(self) -> QHBoxLayout:
        """
        Creates the layout for URDF loading.
        """
        self.load_urdf_file_button = QPushButton("Load urdf from file")
        self.load_urdf_file_button.clicked.connect(self._load_urdf_file_button_callback)
        urdf_section = QHBoxLayout()
        urdf_section.addWidget(self.load_urdf_file_button)
        urdf_section.addWidget(self.urdf_progress)
        return urdf_section

    def _load_urdf_file_button_callback(self):
        """
        Callback for the load URDF button.
        """
        options = QFileDialog.Options()
        options |= QFileDialog.ReadOnly
        urdf_file, _ = QFileDialog.getOpenFileName(
            self,
            "Select URDF File",
            "",
            "urdf files (*.urdf);;All files (*)",
            options=options,
        )
        if urdf_file:
            if not os.path.isfile(urdf_file):
                QMessageBox.critical(
                    self, "Error", f"File does not exist: \n{urdf_file}"
                )
                return

            self.interface.load_urdf(urdf_file)
            self.urdf_progress.set_progress(100, f"Loaded {urdf_file}")
            self.refresh_body_buttons()
            if self.interface.bodies:
                self.choose_components_button.setEnabled(True)

    def refresh_body_buttons(self):
        """
        Refreshes the list of body buttons in a grid layout.
        """
        while self.body_buttons_layout.count():
            item = self.body_buttons_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()

        columns = 5
        for index, body in enumerate(self.interface.bodies):
            button = BodyButton(body=body, application=self)
            row = index // columns
            column = index % columns
            self.body_buttons_layout.addWidget(button, row, column)


def handle_sigint(sig, frame):
    """Handler for the SIGINT signal."""
    rospy.shutdown()
    QApplication.quit()


if __name__ == "__main__":
    rospy.init_node("robot_semantic_annotation_builder")
    signal.signal(signal.SIGINT, handle_sigint)

    app = QApplication(sys.argv)
    window = Application()
    window.show()
    exit_code = app.exec_()
    rospy.shutdown()
    sys.exit(exit_code)
