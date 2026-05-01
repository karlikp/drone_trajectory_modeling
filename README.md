# Drone Trajectory Modeling (3DOF)

This project shows a simple model of drone movement in 3D space (3 DOF).

It is part of the project:  
**“Mechatronic module for water sampling with drone”**

## Table of Contents
- [Project Description](#project-description)
- [Project Goal](#project-goal)
- [Model](#model)
- [Project Structure](#project-structure)
- [Requirements](#requirements)
- [Installation](#installation)
- [Run Simulation](#run-simulation)
- [Output](#output)
- [Simulation Scenario](#simulation-scenario)
- [Notes](#notes)

## Project Description

This project simulates how a drone moves in space.

The model includes:
- movement in X, Y, Z directions
- drone thrust (force from motors)
- gravity
- air resistance
- change of mass (when drone takes water sample)
- disturbances (for example: water tube)


## Project Goal

- create a simple drone model
- simulate drone flight
- analyze:
  - hovering
  - movement in space
  - water sampling process

## Model

State of the system:

x = [x, y, z, vx, vy, vz]

Meaning:
- x, y, z → position
- vx, vy, vz → velocity

## Project Structure

drone_trajectory_modeling/
│  
├── drone_model.py      # drone model (equations)  
├── simulation.py       # simulation script  
├── README.md  

## Requirements

- Python 3.8+
- numpy
- scipy
- matplotlib

## Virtual Environment (recommended)

It is recommended to use a virtual environment.

## Installation

### 1. (Optional) Clone repository

git clone <your-repo-url>  
cd drone_trajectory_modeling  

### 2. Create virtual environment

python3 -m venv venv  

### 3. Activate environment

Linux / macOS:

source venv/bin/activate  

Windows:

venv\Scripts\activate  

### 4. Install dependencies

pip install numpy scipy matplotlib  

## Run Simulation

python3 simulation.py  

## Output

The program generates plots:

- trajektoria_3d.png → 3D flight path  
- polozenie_czas.png → position over time  
- predkosci_czas.png → velocity over time  
- portret_fazowy_z.png → phase plot (z vs vz)  

## Simulation Scenario

The simulation shows:

1. Hover (drone stays in place)  
2. Move forward (X axis)  
3. Move sideways (Y axis)  
4. Take water sample (mass increases)  
5. Disturbance (tube interaction)  

## Notes

This is a simplified model for learning and simulation purposes only.
