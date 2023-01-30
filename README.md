# Manipulation, Estimation, & Control: 

**Instructor:** George Kantor <br>
**Term:** Fall 2022

### Learning Objectives:
> *This course provides an overview of the current techniques that allow robots to move 
around, interact with the world, and keep track of where they are. The kinematics and dynamics of 
electromechanical systems will be covered with a particular focus on their application to robotic 
arms. Some basic principles of robot control will be discussed, ranging from independent-joint PID 
tracking to coupled computed torque approaches. State estimation techniques including the 
extended Kalman filter will be covered, especially as they are used in solving common problems 
faced in robotics applications.*

### Course Overview:
> *By the end of this course, students are expected to be able to do the following:* <br>
> - *Simulate and analyze dynamic behavior of physical systems through ordinary differential* <br>
equations and difference equations. <br>
> - *Derive transfer function representation of linear systems from ordinary differential 
equations and difference equations using Laplace and z transforms.* 
> - *Predict qualitative and quantitative behavior of continuous and discrete time linear systems <br>
by examining transfer function poles.* <br>
> - *Derive single transfer function of systems composed of multiple component linear systems 
combined in series, parallel, and feedback configurations.* <br>
> - *Synthesize PID feedback controllers to achieve desired step-response characteristics.* <br>
> - *Analyze transient behavior of linear and nonlinear state-space systems via eigenvalues and 
Lyapunov theory.* <br>
> - *Construct state feedback controllers and state observers for linear state space systems via 
pole placement and linear quadratic regulator.* <br>
> - *Understand the Kalman filter equations, and implement extended Kalman filter for robotic 
state estimation and SLAM.* <br>
> - *Understand pose graph estimation techniques, and implement them for problems such as 
SLAM and vision-based odometry.* 

<br><hr>

# Assignment 1: 

![pendulum-cart]()<br>
**Description:** **


## Topics Covered:
- Stability & controllability conditions for LTI systems
- Pole placement for full-state feedback
- Equilibrium points for continuous systems
- Linearizing nonlinear systems
- LQR control on linearized systems


<hr>

# Assignment 2:

![observer]()<br>
**Description:** **

## Topics Covered:
- Closed-Loop feedback with unity negative feedback
- Final Value Theorem
- PID Control
- Tuning Gains in PID control
- Linear-Observer for State Estimation

<hr>

# Assignment 3:

![ekf]()<br>
![pf]()<br>
**Description:** **


## Topics Covered:
- Extended Kalman Filter
- Dead-Reckoners 
- Particle Filter

<hr>

# Assignment 4:

![htm]()<br>
**Description:** **


## Topics Covered:
- Homogeneous Transformation Matrices (HTM) in SE(2)
- Absolute versus relative HTM
- HTM abstraction for robotic manipulation applications
- Denavit-Hartenburg (DH) parameters & DH tables
- Jacobians for robotic manipulators, via direct-differention & column-column methods
- Singularities of robotic manipulators