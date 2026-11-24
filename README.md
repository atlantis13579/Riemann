# Introduction

Riemann is a 3D Geometry Processing / Collision Detection / Physics Engine with zero dependencies.
This is a personal learning project, I want to write my own physics engine to understand some of the key algorithm that makes a physics engines works.

This project is named after the 19th-century German mathematician Bernhard Riemann, who invented Riemannian geometry and the concept of manifold.

## Features

* Rigid Body Dynamics Simulation
    * Continuous collision detection, temporal substeps, island sleeping, and persistent friction patches
    * Reduced-coordinate articulation with Featherstone dynamics, transmissions, and tendon constraints
    * Physical animation, pose drives, and ragdoll recovery
    * Capsule and box character controllers with swept walking and ballistic jumping
    * Query-wheel vehicles with suspension, combined tire slip, drivetrain, and chassis aerodynamics
    * Articulated robot walking with joint motors and terrain contacts

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

## Examples

#### Dominoes

https://github.com/user-attachments/assets/fade84c5-58d8-46ec-8d3e-f972f670b7fb

#### Seesaw

https://github.com/user-attachments/assets/0463e259-d197-4550-8242-817025141715

#### Ragdoll Simulation

https://github.com/user-attachments/assets/a835a93f-ac16-4bb1-9c1c-c3aa92feb4a5

#### Rope Simulation

https://github.com/user-attachments/assets/3b1fc795-60ff-47c3-871d-e2d8c96425d1

#### Robotics Simulation (The Policy Weights are trained using [ElmanLLM](https://github.com/atlantis13579/Elman))

https://github.com/user-attachments/assets/51c44659-9074-41c7-b416-be62b5a04130

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



