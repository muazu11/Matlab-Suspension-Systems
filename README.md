# Active Suspension System Simulation: H-Infinity and LQR Controllers
This repository contains MATLAB scripts and Simulink models to design and evaluate H-Infinity (𝐻∞) and LQR (Linear Quadratic Regulator) controllers for a quarter-car active suspension system. The goal is to improve ride comfort and road handling under various road disturbance conditions.

## Repository Structure

The repository follows a structured directory layout to ensure clarity and modularity:

```plaintext
Matlab-Suspension-Systems/
├── H-Infinity
│   └── HInfinity.m               # MATLAB script for H-Infinity controller
├── LQR
│   ├── LQR.m                     # MATLAB script for LQR controller
│   ├── QuarterCar_LQR_Control.slx # Simulink model for LQR controller
│   ├── QuarterCart_LQR_Control.slxc # Simulink cache file
│   └── slprj/                    # Simulink project cache files
├── README.md                     # Documentation (this file)
```
---
## Software Requirements
To run the simulations, you need:

- MATLAB (R2023a or later)
- Simulink (R2023a or later)
- Control System Toolbox

## Description of Files

H-Infinity Controller:
- HInfinity.m: Implements the H-Infinity control design for the quarter-car model using MATLAB’s hinfsyn function.
- Researchers can modify the weighting functions Ws_d,Wa_b,W_road,W_act in the HInfinity.m script to prioritize robustness or control effort based on their requirements. These functions determine the trade-off between disturbance rejection, control effort, and system output performance.
Key Features:
- Minimizes the worst-case disturbance amplification.
- Simulates body travel, suspension deflection, and control force.

LQR Controller:
- LQR.m: MATLAB script for the LQR controller design using the state-space model.
- QuarterCar_LQR_Control.slx: Simulink model for the LQR-controlled suspension system.
- Researchers can customize the Q matrix (state weighting) and R matrix (control effort weighting) in the LQR.m script to fit specific performance requirements. 
Key Features:
- Minimizes the quadratic cost function 
- Simulates open-loop and closed-loop responses for comparison.
Simulink Cache:
- Contains cache files to ensure compatibility when opening the Simulink model.

## How to Run the Simulations
1. Running the LQR Controller Simulation
Steps:
- Open MATLAB and set the current working directory to the LQR folder.
- Open the Simulink model QuarterCar_LQR_Control.slx.
- Run the simulation to observe open-loop and closed-loop responses.
The results include:
- Car body travel,
- Suspension deflection,
- Control force,
- Body acceleration.
This script initializes the system, computes the LQR controller, and simulates both open-loop and closed-loop responses.

2. Running the H-Infinity Controller Simulation
Steps:
- Open MATLAB and set the current working directory to the H-Infinity folder.
- Run the simulation.
The script:
- Designs the H-Infinity controller using MATLAB’s hinfsyn function.
- Simulates system responses under road disturbances.
- Plots performance metrics, such as body travel, suspension deflection, body acceleration and control force.

## Simulation Outputs
Both controllers generate the following plots for comparison:
- Body Travel: Vertical displacement of the car body.
- Suspension Deflection: Relative motion between car body and wheel.
- Control Force: Actuator force applied to the suspension system.
- Body Acceleration: Rate of change of body displacement (important for passenger comfort)

## How to Compare the Controllers
Performance Metrics:
- Body acceleration (𝑎𝑏): Passenger comfort.
- Suspension deflection (𝑠𝑑): Road handling.
- Control force (𝑓𝑠): Actuator effort.

Run both simulations under the same road disturbance:
- Compare the H-Infinity results (HInfinity.m) with the LQR results (LQR.m).
- Evaluate the trade-off between robustness (H-Infinity) and optimal performance (LQR).

## Acknowledgments
The repository was created to address reviewers' comments and provide transparency for our study. The provided models and scripts are useful for students, researchers, and practitioners working on control systems for active suspension systems.
