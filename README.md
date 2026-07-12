# Introduction

Riemann is a 3D Geometry Processing / Collision Detection / Physics Engine with zero dependencies.
This is a personal learning project, I want to write my own physics engine to understand some of the key algorithm that makes a physics engines works.

This project is named after the 19th-century German mathematician Bernhard Riemann, who invented Riemannian geometry and the concept of manifold.

## Features

Rigid Body Dynamics Simulation

* Simulation and Collision Detection for various shapes
    * Sphere
    * Oriented Box
    * Capsule
    * Cylinder
    * Convex Mesh
    * Triangle Mesh
    * Height Field

* Collision detection
    * Raycast Test
    * Overlap Test
    * Sweep Test
    * Bounding Volume Hierarchy
    * Sweep and Prune
    * Gilbert–Johnson–Keerthi algorithm
    * Expanding Polytope algorithm
    
* Numerical Method
    * Symplectic Euler / Implicit Euler / Verlet Integrator
    * Jacobian Matrix
    * Sequential Impulse Solver
    * Global LCP Solver using Projected Gauss-Seidel

Position Based Dynamics

Fluid Dynamics Solver


## Examples

#### Dominoes

![domino](https://user-images.githubusercontent.com/29682318/180676459-119abeea-6d8a-4b9e-8fc4-ad87207eb6ed.gif)

#### Stable boxes stacking

![stack](https://user-images.githubusercontent.com/29682318/180676472-73c80918-6daa-44b0-a34f-e3fcffebdd1e.gif)

#### Ragdoll Simulation

![stack](https://github.com/user-attachments/assets/e8d3a438-5ed9-438d-ad14-cdd8941a8b34)

#### Glass Break Simulation

![stack](https://github.com/user-attachments/assets/3a5ff5ff-1315-43bf-b96f-f2a04574140e)

#### Cloth Simulation

![stack](https://github.com/user-attachments/assets/2b02a8ab-46af-479f-93c5-830e9926144f)

#### Mesh QEM Optimizer

![qem](https://github.com/atlantis13579/Riemann/assets/29682318/ffdb88ea-a114-43a5-8864-8b5ab22fd282)
