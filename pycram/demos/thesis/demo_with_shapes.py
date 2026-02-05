from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt

from pycram.demos.thesis.binding.mixmap import MixmapParams, solve_mixmap
from pycram.demos.thesis.geometry.volume_models import (
    CylinderVolumeModel,
    volume_from_body_collision,
)
from pycram.demos.thesis.primitives.volume_agitation import (
    VolumeAnchor,
    AgitationSpec,
    compile_volume_agitation,
)
from pycram.demos.thesis.tools.visualize_mixmap import clearance_grid

from pycram.demos.thesis.geometry.surface_models import SurfacePlane
from pycram.demos.thesis.primitives.surface_interaction import (
    bind_surface_anchor,
    SweepSpec,
    ScrubSpec,
    compile_wipe_raster_scrub,
)
from pycram.demos.thesis.geometry.volume_models import volume_from_body_collision
from pycram.demos.thesis.primitives.seperation_devision import (
    SliceSpec,
    bind_slice_anchors_along_x,
    compile_slice_down_through_up,
    compile_saw_slice_sequence,
    SawSliceSpec,
    CutSide,
)

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.geometry import Box, Scale


def demo_cylinder_mixmap() -> None:
    volume = CylinderVolumeModel(radius=0.12, half_height=0.08)
    params = MixmapParams(z_ratio=0.35, epsilon=0.01, grid_step=0.005)
    result = solve_mixmap(volume, params)
    if result is None:
        raise RuntimeError("no feasible stance in cylinder")

    xs, ys, grid = clearance_grid(volume, result.z_mix, params.grid_step)

    print("CYLINDER MIXMAP")
    print("z_mix:", result.z_mix)
    print("p_in_container:", result.p_in_container)
    print("clearance:", result.clearance)

    plt.figure()
    plt.imshow(
        grid, origin="lower", extent=[xs[0], xs[-1], ys[0], ys[-1]], aspect="equal"
    )
    plt.title("Cylinder clearance at z_mix")
    plt.xlabel("x in container frame")
    plt.ylabel("y in container frame")
    plt.colorbar()
    plt.scatter([result.p_in_container[0]], [result.p_in_container[1]], marker="x")
    plt.show()


def demo_body_box_to_agitation() -> None:
    box = Body(
        name=PrefixedName("muh"),
        collision=ShapeCollection([Box(scale=Scale(0.1, 0.1, 0.1))]),
    )

    volume = volume_from_body_collision(box, padding=0.005)
    if volume is None:
        raise RuntimeError("no volume model from body collision")

    params = MixmapParams(z_ratio=0.35, epsilon=0.01, grid_step=0.005)
    result = solve_mixmap(volume, params)
    if result is None:
        raise RuntimeError("no feasible stance in box")

    anchor = VolumeAnchor(
        frame_id=getattr(box, "tf_frame", "box"), p=result.p_in_container
    )
    spec = AgitationSpec(
        turns=3, angle_step_deg=15.0, radius_step=0.0015, z_step=0.0002
    )

    wps = list(compile_volume_agitation(anchor, spec, volume=volume, epsilon=0.01))

    print("BODY BOX -> VOLUME AGITATION")
    print(
        "bounds_xy:", volume.inner_bounds_xy(), "inner_height:", volume.inner_height()
    )
    print("anchor:", result.p_in_container, "clearance:", result.clearance)
    print("waypoints:", len(wps))

    xy = np.array([[wp.pose.position.x, wp.pose.position.y] for wp in wps], dtype=float)
    zz = np.array([wp.pose.position.z for wp in wps], dtype=float)

    plt.figure()
    plt.plot(xy[:, 0], xy[:, 1])
    plt.scatter([result.p_in_container[0]], [result.p_in_container[1]], marker="x")
    plt.title("Volume agitation path in object frame (x,y)")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.axis("equal")
    plt.show()

    plt.figure()
    plt.plot(zz)
    plt.title("Volume agitation depth over steps (z)")
    plt.xlabel("step")
    plt.ylabel("z")
    plt.show()


def demo_surface_wipe() -> None:
    surface = SurfacePlane(
        frame_id="table",
        half_extents_xy=np.array([0.30, 0.20], dtype=float),
        z_contact=0.0,
    )

    anchor = bind_surface_anchor(surface, margin=0.02)
    if anchor is None:
        raise RuntimeError("no feasible surface anchor")

    sweep = SweepSpec(spacing=0.05, margin=0.02, z_offset=0.0)
    scrub = ScrubSpec(radius=0.012, points_per_cycle=40, cycles=2, z_offset=0.0)

    wps = list(compile_wipe_raster_scrub(surface, sweep, scrub))

    xy = np.array([[wp.pose.position.x, wp.pose.position.y] for wp in wps], dtype=float)

    plt.figure()
    plt.plot(xy[:, 0], xy[:, 1])
    plt.scatter([0.0], [0.0], marker="x")
    plt.title("Wipe primitive: raster sweep + local scrub (x,y)")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.axis("equal")
    plt.show()


def demo_separation_slicing_box() -> None:

    box = Body(
        name=PrefixedName("cut_box"),
        collision=ShapeCollection([Box(scale=Scale(0.18, 0.10, 0.06))]),
    )

    volume = volume_from_body_collision(box, padding=0.0)
    if volume is None:
        raise RuntimeError("no volume model from body collision")

    half_extents = volume.half_extents
    spec = SliceSpec(
        slice_thickness=0.03,
        z_clearance=0.03,
        z_cut=0.0,
        margin_xy=0.005,
    )

    anchors = bind_slice_anchors_along_x(
        getattr(box, "tf_frame", "cut_box"),
        half_extents,
        spec,
    )

    wps = []
    for a in anchors:
        wps.extend(list(compile_slice_down_through_up(a, half_extents, spec)))

    print("SEPARATION & DIVISION -> SLICING (BOX)")
    print("half_extents:", half_extents)
    print("slice_thickness:", spec.slice_thickness)
    print("num_slices:", len(anchors))
    print("num_waypoints:", len(wps))

    xy = np.array([[wp.pose.position.x, wp.pose.position.y] for wp in wps], dtype=float)
    zz = np.array([wp.pose.position.z for wp in wps], dtype=float)

    plt.figure()
    plt.scatter(xy[:, 0], xy[:, 1], s=6)
    plt.title("Slice anchors projected to (x,y) in object frame")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.axis("equal")
    plt.show()

    plt.figure()
    plt.plot(zz)
    plt.title("Slice motion profile over waypoints (z)")
    plt.xlabel("waypoint index")
    plt.ylabel("z")
    plt.show()


def demo_specialized_cut_sequence() -> None:
    from pycram.demos.thesis.geometry.volume_models import volume_from_body_collision

    box = Body(
        name=PrefixedName("cut_box"),
        collision=ShapeCollection([Box(scale=Scale(0.18, 0.10, 0.06))]),
    )

    volume = volume_from_body_collision(box, padding=0.0)
    if volume is None:
        raise RuntimeError("no volume model from body collision")

    half_extents = volume.half_extents
    anchors = bind_slice_anchors_along_x(
        obj_frame_id=getattr(box, "tf_frame", "cut_box"),
        obj_half_extents=half_extents,
        slice_thickness=0.03,
    )

    spec = SawSliceSpec(
        slice_thickness=0.03,
        tool_half_length=0.10 / 2.0,
        prelift_z=float(half_extents[2]) + 0.06,
        insert_z=float(half_extents[2]) - 0.01,
        y_standoff=0.02,
        tilt_y_deg=15.0,
        pitch_x_deg=90.0,
        stroke_x=0.08,
        rotate_y_shift=0.005,
        rotate_x_shift=0.0,
        return_y_shift=0.02,
        return_x_shift=0.08,
        final_lift_z=float(half_extents[2]) + 0.06,
    )

    wps = []
    for a in anchors[: max(1, len(anchors) // 2)]:
        side = CutSide.POS_Y
        wps.extend(list(compile_saw_slice_sequence(a, side, spec)))

    xy = np.array([[wp.pose.position.x, wp.pose.position.y] for wp in wps], dtype=float)
    zz = np.array([wp.pose.position.z for wp in wps], dtype=float)

    plt.figure()
    plt.plot(xy[:, 0], xy[:, 1])
    plt.title("Specialized cut sequence projected to (x,y)")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.axis("equal")
    plt.show()

    plt.figure()
    plt.plot(zz)
    plt.title("Specialized cut sequence depth (z)")
    plt.xlabel("waypoint index")
    plt.ylabel("z")
    plt.show()


def main() -> None:
    demo_cylinder_mixmap()
    demo_body_box_to_agitation()
    demo_surface_wipe()
    demo_separation_slicing_box()
    import inspect

    print("bind_slice_anchors_along_x:", bind_slice_anchors_along_x)
    print("module:", bind_slice_anchors_along_x.__module__)
    print("file:", inspect.getsourcefile(bind_slice_anchors_along_x))
    print("sig:", inspect.signature(bind_slice_anchors_along_x))

    demo_specialized_cut_sequence()


if __name__ == "__main__":
    main()
