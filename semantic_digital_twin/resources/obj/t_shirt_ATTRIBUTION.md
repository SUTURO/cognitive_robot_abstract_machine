# `t_shirt.obj`

`t_shirt.obj` is made from the file `meshes/Green_Tshirt/green_tshirt_5k.obj` of the
[RGBench Cloth Sim2Real dataset](https://huggingface.co/datasets/RGBench/RGBench-Cloth-Sim2Real-v1)
(revision `136c00dc5f96b6b3d20427e93875a1c00d7a7cc9`).

It is part of *Real Garment Benchmark (RGBench): A Comprehensive Benchmark for Robotic
Garment Manipulation featuring a High-Fidelity Scalable Simulator* by Wenkang Hu, Xincheng
Tang, Yanzhi E, Yitong Li, Zhengjie Shu, Wei Li, Huamin Wang and Ruigang Yang
(<https://rgbench.github.io/>), and is licensed under the
[Creative Commons Attribution 4.0 International License](https://creativecommons.org/licenses/by/4.0/).
The RGBench garment asset library, which carries a non-commercial license, is not the
source of this file.

## Changes

The original's 4,867 vertices were reduced to 500, keeping the shape, the two layers and the
four openings:

1. The vertices the panels share at the seams were merged, dropping texture coordinates and
   normals (`trimesh.Trimesh.merge_vertices(merge_tex=True, merge_norm=True)`).
2. The surface was remeshed with MeshLab's isotropic explicit remeshing (`pymeshlab`,
   `meshing_isotropic_explicit_remeshing` with 10 iterations, a target edge length of
   0.0582 m, a feature angle of 60°, and a maximum distance from the surface of 0.005 m),
   which left 504 vertices.
3. Four edges were collapsed with MeshLab's quadric edge collapse decimation, keeping the
   boundary and the topology.
