# Line Tracking Problem
**[Docs under development]**
Some information about the kinematic of the system

## Kinematic Model of Unicycle
The Kinematic Model of Unicycle can be derived from the null basis of Pfaffian Matrix $A(q)$:

$$ 
\dot{q}(t) = S(q) \ \nu(t) \rightarrow 
\begin{pmatrix} 
\dot{x} \\
\dot{y} \\
\dot{\theta} \end{pmatrix} = \begin{pmatrix} 
                                \cos(\theta) & 0 \\
                                \sin(\theta) & 0 \\
                                0 & 1 
                            \end{pmatrix}
                                        \begin{pmatrix} 
                                            v \\
                                            \omega
                                        \end{pmatrix}
$$

### Diff Drive Robot Model
Let be $R$ the radius of a single wheel and $L$ the distance between the two wheels. We can define the angular velocity $\omega_r$ and $\omega_l$ as the two angular velocities of the wheels. From simple geometrical consideration, we can write:
$$
\begin{cases}
v       = \frac{R}{2} \left( \omega_l + \omega_r \right) \\
\omega  = \frac{R}{L} \left( \omega_r - \omega_l \right)
\end{cases}
\rightarrow
\begin{bmatrix}
  v \\
  \omega \end{bmatrix} = \begin{bmatrix} \frac{R}{2} & \frac{R}{2} \\
  \frac{R}{L} & -\frac{R}{L}
\end{bmatrix}
\begin{bmatrix}
    \omega_r \\
    \omega_l
\end{bmatrix}
$$

### Discretized model
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
