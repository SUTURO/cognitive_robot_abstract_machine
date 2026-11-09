# Human

A rigid human standing in a T-pose, described by `human.urdf`, about 3100 triangles in all.

The meshes and textures were made with MPFB 2.0.18 (MakeHuman for Blender) in Blender 5.2.2
from the MakeHuman system assets, which are licensed
[CC0](https://static.makehumancommunity.org/assets/assetpacks/makehuman_system_assets.html):

| File | MakeHuman asset |
|---|---|
| `body` | base mesh with the skin `young_caucasian_male` |
| `clothes` | `male_casualsuit01` |
| `shoes` | `shoes01` |
| `hair` | `short02` |

The human wears the T-pose MPFB ships for its default rig (`poses/default_fk/t-pose.json`).
Every mesh was decimated to 8 % of its faces, and the textures were shrunk to 512 pixels
for the body, 256 for the clothes and the hair and 128 for the shoes.
