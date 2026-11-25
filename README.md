# Introduction

Riemann is a 3D Geometry Processing / Collision Detection / Physics Engine with zero dependencies.
This is a personal learning project, I want to write my own physics engine to understand some of the key algorithm that makes a physics engines works.

This project is named after the 19th-century German mathematician Bernhard Riemann, who invented Riemannian geometry and the concept of manifold.

## Features

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
    * Position Based Dynamics

* Rigid Body Dynamics Simulation
    * Continuous collision detection
    * Island sleeping
    * Reduced-coordinate articulation with Featherstone Solver, transmissions, and tendon constraints
    * Query-wheel vehicles with suspension
    * Multi joint Robotics

* Destruction Simulation
    * Graph Based Hierarchy Destruction
    * Graph Partition Algorithm
    * Mesh Cut Algorithm
    * Voronoi Tessellation
    * Authored fracture assets, rigid-cluster replay, and bounded destruction events

* Deformation Body Simulation
    * Rope Simulation
    * Cloth Simulation
    * Elastic Simulation
    * Elastoplastic tetrahedral vehicle cages with crumple zones, edge and volume plasticity, and inversion barriers
    * Mesh Morphing
    * Triangle cloth with bending, wind, tethers, and continuous surface contacts
    * Projective dynamics and tetrahedral materials with rigid-body coupling
    * Cosserat hair guides, strand collision, and groom interpolation

* Fluid Dynamics Simulation
    * Eulerian Approach
    * Lagrangian Approach
    * Smoothed Particle Hydrodynamics

* Aero Dynamics Simulation    

* Continuum Media Simulation
    * Snow and Sand Simulation
    * Three-dimensional quadratic B-spline MPM with PIC/FLIP, APIC, and MLS-MPM transfers
    * Fixed-corotated snow elasticity, compression hardening, and Drucker-Prager sand plasticity
    * CFL-controlled integration and spatially binned GPU grid gathering without floating-point atomics
 
* 2D Physics Simulation
    * The 2D Physics Simulation is implemented in a separate project [Riemann2d](https://github.com/atlantis13579/Riemann2d)

## Examples

#### Dominoes Simulation

https://github.com/user-attachments/assets/00452f08-55f5-4078-b492-08461270b7c8

#### Ragdoll Simulation

https://github.com/user-attachments/assets/a835a93f-ac16-4bb1-9c1c-c3aa92feb4a5

#### Rope Simulation

https://github.com/user-attachments/assets/3b1fc795-60ff-47c3-871d-e2d8c96425d1

#### Robotics Simulation

The Action is trained using PPO (Proximal Policy Optimization) in [ElmanLLM](https://github.com/atlantis13579/Elman) Reinforcement Learning Framework.

https://github.com/user-attachments/assets/51c44659-9074-41c7-b416-be62b5a04130

#### Machices Simulation

https://github.com/user-attachments/assets/574b2c11-2859-455a-9c97-42bc8fe91dd5

#### Glass Break Simulation

https://github.com/user-attachments/assets/399abd7d-f47e-440f-8e50-3c1b3fadf772

#### Destruction Simulation

https://github.com/user-attachments/assets/f259ba6b-b976-4cac-a987-e496b8c630e4

#### Fluid Simulation

https://github.com/user-attachments/assets/85664357-83f4-407d-aa57-2d0e3936d5f2

#### AeroDynamics Simulation

https://github.com/user-attachments/assets/9e89b90d-63aa-42cb-9b36-94b8f594c940

#### Snow Simulation

https://github.com/user-attachments/assets/54ed8ea1-1052-4af5-8e45-0bdca680a674

## 3D Geometry Processing

#### Mesh QEM Optimizer

<img width="1280" height="720" alt="Mesh_QEM_Optimizer" src="https://github.com/user-attachments/assets/f376327e-52ff-4794-b5ce-bd1ee9033d35" />

#### Mesh Cutting

<img width="1280" height="720" alt="Mesh_Cutting" src="https://github.com/user-attachments/assets/0af395c3-3b72-439c-82cc-3092dcfdcc08" />

#### Voronoi Decomposition

<img width="1280" height="720" alt="3D_Voronoi_Decomposition" src="https://github.com/user-attachments/assets/e2e03638-a537-479e-bba0-d0ace199acf8" />



