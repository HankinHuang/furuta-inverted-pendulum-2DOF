# Mechanical CAD & Laser-Cutting Profiles

SolidWorks part files (`.SLDPRT`) and 2D laser-cutting profiles (`.DXF`) for the 2-DOF Furuta Rotary Inverted Pendulum mechanical rig.

## Files

| File | Format | Description |
| :--- | :--- | :--- |
| `底板.SLDPRT` / `底板.DXF` | SolidWorks / DXF | Base mounting plate supporting the bottom-center motor & frame |
| `第一板.SLDPRT`, `第二板.SLDPRT`, `第三板.SLDPRT` | SolidWorks | Multi-tier structural support plates enclosing motor, coupling, and slip ring |
| `上板.SLDPRT` / `上板.DXF` | SolidWorks / DXF | Upper structural plate |
| `上板稳定.SLDPRT` / `上板稳定.DXF` | SolidWorks / DXF | Reinforced upper stabilization plate |
| `平衡板.SLDPRT` | SolidWorks | Rotary arm / balancing support plate |
| `newlever.dxf` | DXF | Rotary lever / arm profile |

## Design Highlights

- **Low Center of Mass**: The 12 V DC geared motor is mounted at the bottom center of the cylinder-shaped supporting frame to minimize overturning moment and simplify motor-encoder wiring.
- **Continuous Rotation**: Main shaft torque passes through a 4 mm-to-5 mm coupling and hollow-shaft conductive slip ring to prevent wire entanglement during multi-turn swing-up and disturbance tests.
- **Measured Inertias & Dimensions**: Full rigid-body mass, CoM, and moment-of-inertia derivations used in simulation and firmware are documented in [`docs/system_parameters.md`](../../docs/system_parameters.md).
