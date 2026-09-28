# Denavit-Hartenberg & Kinematics Engine Skill

High-performance forward and inverse kinematics solver for robotic manipulators using standard Denavit-Hartenberg (DH) parameters and analytical geometric kinematic solvers.

```mermaid
flowchart LR
    DH["DH Parameters (θ, d, a, α)"] --> Trans["4x4 Homogeneous Transform"]
    Trans --> FK["End-Effector Pose (x, y, z)"]
    Target["Target Position (x, y)"] --> IK["Analytical 2-Link IK"]
    IK --> JointAngles["Joint Angles (θ1, θ2)"]
```

## Features
- **100% Python Standard Library**: Zero external dependencies.
- **Denavit-Hartenberg Parameters**: Arbitrary N-DOF serial chain forward kinematics.
- **Analytical Planar IK**: Closed-form 2-DOF elbow-up/down inverse kinematics.
- **MCP Server**: Stdio JSON-RPC 2.0 integration for AI agents.
