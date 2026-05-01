# Drone Trajectory Modeling (3DOF)

This project shows a simple model of drone movement in 3D space (3 DOF).

It is part of the project:  
**“Mechatronic module for water sampling with drone”**

---

## Project Description

This project simulates how a drone moves in space.

The model includes:
- movement in X, Y, Z directions
- drone thrust (force from motors)
- gravity
- air resistance
- change of mass (when drone takes water sample)
- disturbances (for example: water tube)


---

## Project Goal

- create a simple drone model
- simulate drone flight
- analyze:
  - hovering
  - movement in space
  - water sampling process

---

## Model

State of the system:

x = [x, y, z, vx, vy, vz]

Meaning:
- x, y, z → position
- vx, vy, vz → velocity

---

## Project Structure

drone_trajectory_modeling/
│  
├── drone_model.py      # drone model (equations)  
├── simulation.py       # simulation script  
├── README.md  

---

## Requirements

- Python 3.8+
- numpy
- scipy
- matplotlib

---

## Virtual Environment (recommended)

It is recommended to use a virtual environment.

### Create environment

```bash
python3 -m venv venv