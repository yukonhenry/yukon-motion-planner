# Yukon Motion Planner

![img_1.png](img_1.png)

## Overview

Yukon Motion Planner is a platform to build and evaluate motion planning algorithms. There is a frontend to visualize
and edit the environment, and to visually validate the planning performance. The backend implements the planning engine,
stores the environment and planning results, and exposes an API for the frontend to interact with.

There will be a particular focus on researching the edges of the consequences of dynamic constraints - including both
kinodynamic constraints and actuator dynamics for the robot, as well as moving obstacles and temporal planning. We will
also explore the mathematical optimization underpinnings of the planning algorithms, including exploring topological and
geometric properties of the configuration space, and how they relate to the performance of the planning algorithms.

Backend is built in Rust, using Axum/SeaORM/Postgres. Frontend built in React/TypeScript, using Vite.

## Instructions

* Install Rust and Node.js (with pnpm) if you don't have them already.
* Run `pnpm install` in the `frontend` directory to install frontend dependencies.
* Run `cargo build` in the root directory to build the backend.
* Run `docker compose up` in the root directory to start a Postgres database.'
* Run `cargo run` in the root directory to start the backend server.
* Run `pnpm run dev` in the `frontend` directory to start the frontend server.

# In Progress

* More planning algorithms (only A*, D*-Lite, and RRT*/SST* are implemented)
* Visualization of not only the planning result, but also the planning process and topological/geometric properties of
  the configuration space
* Moving/Varying obstacles and Temporal planning
* Actuator Dynamics and Kinodynamic constraints
* Uncertainty modeling
* 3D/Multi-dimensional planning
* Multi-robot planning
* Host the backend