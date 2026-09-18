# Source provenance and conversion notes

The MuJoCo model in `models/flexiv_rizon4s_scene.xml` was reconstructed from:

- Simulink/Simscape source: `E:\matlab2023b\workfile\matlab23b_workfiles\flexiv_rizon4s\simscape_of_flexiv\flexiv_rizon4s_kinematics_2025_5_14.slx`
- URDF next to the source project: `flexiv_rdk/urdf/new_of_flexiv_rizon4s_kinematics.urdf`
- Rizon 4s collision meshes: `flexiv_rdk/urdf/meshes/rizon4s/collision/link0.stl` through `link7.stl`

`diagnostics/source_model_inventory.json` is the generated block and MATLAB Function inventory of the `.slx`. Direct inspection of the Simscape Inertia blocks established a 500 kg free-floating cubic base with diagonal inertia `[20.833333333, 20.833333333, 20.833333333] kg m^2`, a 0.25 m arm mount, and the link masses used by the bundled URDF.

The source Simscape model permits zero principal inertia values for several distal links. MuJoCo requires positive-definite inertia tensors, so only those zero entries were regularized using the positive approximate Rizon 4s inertias shipped in the adjacent ROS description. Joint origins, axes, limits, link masses, COM offsets, the 500 kg base, zero gravity, and the source home joint angles are otherwise preserved.
