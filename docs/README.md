# Line Tracking Problem
Hi Students! Some docs for you to start the project.
## Table of Contents
1. [Kinematic Model of Unicycle](#1-kinematic-model-of-unicycle)
    1. [Differential Drive Robot Model](#11-differential-drive-robot-model)
    2. [Discretized Model](#12-discretized-model)
2. [Control Tasks for a Robotic Vehicle](#2-control-tasks-for-a-robotic-vehicle)

## 1) Kinematic Model of Unicycle
The Kinematic Model of Unicycle can be derived from the null basis of Pfaffian Matrix $A(q)$:

$$ 
\dot{q}(t) = S(q) \ \nu(t) \rightarrow 
\begin{bmatrix} 
\dot{x} \\
\dot{y} \\
\dot{\theta} \end{bmatrix} = \begin{bmatrix} 
                                \cos(\theta) & 0 \\
                                \sin(\theta) & 0 \\
                                0 & 1 
                            \end{bmatrix}
                                        \begin{bmatrix}
                                            v \\
                                            \omega
                                        \end{bmatrix}
$$

### 1.1) Differential Drive Robot Model
Let be $R$ the radius of a single wheel and $L$ the distance between the two wheels. We can define the angular velocity $\omega_r$ and $\omega_l$ as the two angular velocities of the wheels. From simple geometrical consideration, we can write:

$$
\begin{cases}
v       = \frac{R}{2} \left( \omega_l + \omega_r \right) \\
\omega  = \frac{R}{L} \left( \omega_r - \omega_l \right)
\end{cases} \rightarrow \begin{bmatrix} v \\ 
                                        \omega \end{bmatrix} = \begin{bmatrix} \frac{R}{2} & \frac{R}{2} \\  
                                                                    \frac{R}{L} & -\frac{R}{L} \end{bmatrix} \begin{bmatrix} \omega_r \\ 
                                                                                                                                \omega_l \end{bmatrix}
$$

### 1.2) Discretized model
Using Forward Euler, we can write:
$$\dot{q} \approx \frac{q(k+1) - q(k)}{T_s}$$

where $T_s$ is the sampling period. Finally, we can discretize the Kinematic Model of Unicycle:

$$q(k + 1) = q(k) + T_s \ S(q) \ \nu(k)$$

Expanding the previous equation:

$$\begin{cases}
x_{k+1} = x_k + T_s \ \cos(\theta_k) v_k \\
y_{k+1} = y_k + T_s \ \sin(\theta_k) v_k \\
\theta_{k+1} = \theta_{k} + T_s \ \omega_k
\end{cases}$$

## 2) Control Tasks for a Robotic Vehicle
In robotics, there are 3 classic control tasks, such as:
1. **Point-to-Point**: The robot reaches a desired constant pose $\bar{q}_d = cost$.
2. **Trajectory Tracking**: The robot follows a desired pose trajectory $q_{d}(t)$.
3. **Path Following**: In this case, the robotic vehicle follows an assigned curve $C(q) = 0$ (e.g. a circumference).
For Line Following, only the last two tasks are useful.

### 2.1) Path Following
Let be $p_b = [x_b, y_b]^T$ the position of the vehicle. From the fixed frame, we can define the body frame $\{ S_{b} \}=[ p_b; \hat{i}_b, \hat{j}_b ]$ where:

$$
\hat{i}_b = \begin{bmatrix} \cos(\theta) \\ 
                            \sin(\theta) \end{bmatrix} \qquad \hat{j}_b = \begin{bmatrix} -\sin(\theta) \\ 
                                                                                            \cos(\theta) \end{bmatrix} 
$$

Let be $p_c$ the projection of this point on the curve $C(q)$, at a specific value of the curvilinear abscissa $\sigma$. We can define the Frenet-Serret frame $\{S_{c}\} = \{p_c; \hat{i}_c, \hat{j}_c\}$, where:

$$
\hat{i}_c = \begin{bmatrix} \cos(\theta_c) \\ 
                            \sin(\theta_c) \end{bmatrix} \qquad \hat{j}_c = \begin{bmatrix} -\sin(\theta_c) \\ 
                                                                                            \cos(\theta_c) \end{bmatrix} 
$$

with $\theta_c$ is the angle between the tangent of the curve and the fixed $x$ axis.

We can do a change of variables of the kinematic model of unicycle, defining

$$
\begin{cases}
\psi = \theta - \theta_c & \textnormal{Orientation Error} \\
d = y_b - y_c & \textnormal{Curvilinear Ordinate} \\
\end{cases}
$$

and obtaining:

$$
\begin{bmatrix} \dot{\sigma} \\ 
                \dot{d} \\ 
                \dot{\psi}  \end{bmatrix} = \begin{bmatrix} \frac{\cos(\psi)}{1 - d \gamma(\sigma)} & 0 \\ 
                                                            \sin(\psi) & 0 \\ 
                                                            - \frac{\cos(\psi) \gamma(\sigma)}{1 - d \gamma(\sigma)} & 1 \end{bmatrix} \begin{bmatrix} v \\ 
                                                                                                                                                                w \end{bmatrix}
$$

where $\gamma(\sigma) = \frac{\partial C(\sigma)}{\partial \sigma}$. 

#### Control Algorithms
To solve this problem, you can use a PID controller applied to the error in orientation $\psi$ and/or in position $d$. A more complex controller, based on the nonlinear dynamics of the system, is:

$$
\omega(t) = - k_{\psi} \psi - d \bar{v} \textnormal{sinc}(\psi) + \bar{v} \left(\frac{\cos(\psi) \gamma(\sigma)}{1 - d \gamma(\sigma)}\right)
$$

As you can see, in this control law there is:
- A proportional term $-k_{\psi} \psi$ on the orientation error.
- A compensation term $\left(- d \bar{v} \textnormal{sinc}(\psi) + \bar{v} \frac{\cos(\psi) \gamma(\sigma)}{1 - d \gamma(\sigma)}\right)$ that eliminates the nonlinearities in the differential equation in $\dot{\psi}$.

### 2.2) Trajectory Tracking
Given a desired trajectory $q_d$, we can consider it as generated from:

$$
\begin{bmatrix} \dot{x}_d \\ 
                \dot{y}_d \\ 
                \dot{\theta}_d \end{bmatrix} = \begin{bmatrix} 
                                \cos(\theta_d) & 0 \\
                                \sin(\theta_d) & 0 \\
                                0 & 1 
                            \end{bmatrix}
                                        \begin{bmatrix}
                                            \hat{v} \\
                                            \hat{\omega}
                                        \end{bmatrix}
$$

Let be the error state $e = q - q_d = [e_x , e_y , e_{\theta}]^T$. To uniform the notation, it is possible to point out that:
- The cartesian error ($e_x$, $e_y$) expressed in the body frame ($e_x^b$, $e_y^b$) is equivalent of $\sigma$ and $d$ in the case of *Path Following* task.
$$
\begin{bmatrix}
\dot{\sigma} \\ 
\dot{d} \\ 
\dot{\psi}
\end{bmatrix} = \begin{bmatrix} v - \hat{v} \cos(\psi) + d \omega \\ 
                \hat{v} \sin(\psi) - \sigma \, \omega \\ 
                \omega - \hat{\omega} \end{bmatrix}
$$

#### Control Algorithms
From this model, you can simply apply a PID controller on the set (or subset) of error variables $e(t)$. A more complex controller algorithm, based on Lyapunov theory, can be:
$$
\omega(t) = \hat{\omega} - k_{\psi} \psi - d \hat{v} \textnormal{sinc}(\psi) 
$$
The control law is similar to the *Path Following* case, in which there is also a *feedforward* action, in which we use the information of the linear and angular velocity of the target.

### Difference between Trajectory Tracking and Path Following Tasks
In the *Path Following* task, the robot follows an assigned curve, without any specifications on the time. On the contrary, in the *Trajectory Tracking* task, the robotic vehicle tracks a trajectory defined in *time*, such as the robot has to follow a moving target.
 To understand better the real difference between the tasks, let's focus on a simple example. Imagine that we want to control our robot to follows a circular trajectory. We can use both approaches:
*Trajectory Tracking*: 

$$
q_d(t) = \begin{bmatrix} x_d(t) \\ 
                               y_d(t) \end{bmatrix} = R_{c} \begin{bmatrix} \cos(\omega_{c} t) \\ 
                                                                            \sin(\omega_{c} t) \end{bmatrix}
$$

*Path Following*:

$$
C(q) = x^2 + y^2 - R_{c}^2
$$

At the instant $t = \bar{t}$ the robot stop for some reasons (e.g. obstacles) and restart after $\Delta t$. In the case of *Path Following*, the vehicle continue to follow the desired path. On the contrary, in the case of *Trajectory Tracking*, the vehicle is controlled to follow not the path, but the desired trajectory at the instant $\bm{q}_d(t + \Delta t)$, going out from the desired curve and reaching the "moving target". Form this example, we can deduce that:
- In the case of *Path Following*, the vehicle is forced to follow a **specific curve in space** $C(q)$.
- In the case of *Trajectory Tracking*, the vehicle follows a moving target $q_d(t)$.
