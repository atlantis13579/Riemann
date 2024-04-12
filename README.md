# Introduction

Riemann is a 3D Geometry / Collision Detection / Physics Engine with zero dependencies.
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

Vehicle Dynamics / Autonomous Driving Simulation


## Examples

#### Rigid Body Dynamics Simulation for multiple boxes column corpsing

![boxes](https://user-images.githubusercontent.com/29682318/180112516-54e574d6-8462-43bb-a6a7-230b2a58c7f6.gif)

#### Dominoes

![domino](https://user-images.githubusercontent.com/29682318/180676459-119abeea-6d8a-4b9e-8fc4-ad87207eb6ed.gif)

#### Stable boxes stacking

![stack](https://user-images.githubusercontent.com/29682318/180676472-73c80918-6daa-44b0-a34f-e3fcffebdd1e.gif)

#### Mesh QEM Optimizer

![qem](https://github.com/atlantis13579/Riemann/assets/29682318/ffdb88ea-a114-43a5-8864-8b5ab22fd282)

