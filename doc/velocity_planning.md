# Velocity Planning

> refs: Time-Optimal Feedrate Planning for Freeform Toolpaths for Manufacturing Applications, Christina Qing Ge Chen

## 1. Introduction and Problem Formulation

### 1.1 Geometric Path

path $r(s)$, geometric derivatives $(r', r'', r''')$ :

$$
\begin{align}
r(s) &= [x(s), y(s), z(s)]\\
 r'&=\frac{dr(s)}{ds},\\
 r''&=\frac{d^2 r(s)}{ds^2},\\
 r'''&=\frac{d^3 r(s)}{ds^3} \\
s &\in [0, L]
\end{align}
$$

where $r$ is the place holder for the three Cartesian coordinates $x,\ y,\ z$; &emsp; $s$ is a scalar parameter that spans the total arc displacement of the path; &emsp; $L$ is the total length of the path.

velocity, acceleration, jerk:

$$
\begin{align}
    v_r &= r' \dot{s} \\
    a_r &= r' \ddot{s} + r'' \dot{s}^2 \\
    j_r &= r' \overset{\dotsb}{s} + 3 r'' \dot{s}\ddot{s}+ r''' \dot{s}^3
\end{align}
$$

### 1.2 KeyPoints for Path

For velocity planning, the path must be continuous to the second geometric derivative $(C^2)$ in order for jerk to be bounded.

- Cubic Spline
    - Difference between **chord displacement** $(du)$ and **arc length displacement** $(ds)$ cause by non-zero curvature;
    - cubic spline is parameterized by chord displacement;
    - velocity is associated to arc length displacement;

#### 1.2.1 chord length $u$ and arc displacement $s$ in 2D

$$
\begin{align}
    \Delta s &= \sqrt{\Delta x^2 + \Delta y^2} \\
   &\ \Delta x = \frac{dx}{du} \Delta u \\
   &\ \Delta y = \frac{dy}{du} \Delta u \\
   so, \ ds &= \sqrt{x'^2 + y'^2} du
\end{align}
$$

### 1.3 Derivative

Using the chain rule,&emsp; **Primes** $(r')$ denote a derivative with respect to the arc parameter $s$, &emsp; **overhead dots** $(\dot{s})$ denote a derivative with respect to time $t$;

$$
\begin{align}
    v &= \frac{dr}{dt} = \frac{dr}{ds} \frac{ds}{dt} = r' \dot{s} \\
    a &= \frac{d^2 r}{dt^2} = \frac{d}{dt}(r' \dot{s}) = \frac{d\dot{s}}{dt} r' + \frac{dr'}{dt} \dot{s} = \frac{d\dot{s}}{dt} r' + \frac{dr'}{ds} \frac{ds}{dt} \dot{s} \\
    &= r' \ddot{s} + r'' \dot{s}^2 \\
    j &= \frac{d^3 r}{dt^3} = \frac{d}{dt}(r' \ddot{s} + r'' \dot{s}^2) = \frac{d \ddot{s}}{dt} r' + \frac{dr'}{dt} \ddot{s} + \frac{d \dot{s}^2}{dt} r'' + \frac{dr''}{dt}\dot{s}^2 \\
    &= r' \overset{\dotsb}{s} + 3 r'' \dot{s} \ddot{s} + r''' \dot{s}^3
\end{align}
$$


## 2. Time optimal trajectory

The goal of Time optimal trajectory is to reduce the total cycle time. Intuitively, the average velocity should be maximized.

so, an optimization problem that seek to maximize average feedrate $\dot{s}$ along the path can be formed:

$$
\begin{align}
    \max &\int^L_0 \dot{s}\ ds \\
    s.t. & \\
    & |v_{max}| \geq |r' \dot{s}| \\
    & |a_{max}| \geq |r' \ddot{s} + r'' \dot{s}^2| \\
    & |j_{max}| \geq |r' \overset{\dotsb}{s} + 3 r'' \dot{s} \ddot{s} + r''' \dot{s}^3| \\
    & s \in [0, L] \\
\end{align}
$$

**It's hard to solve.** Since $t$ is unknown, $\dot{s},\ \ddot{s},\ \overset{\dotsb}{s}$ can't be computed explicitly.

### 2.1 Non-linear problem
Therefore, a new parameter $q$ is defined as **the square of velocity** :

$$
\begin{align}
    q   &= \dot{s}^2 \\
    q'  &= \frac{d\dot{s}^2}{ds} = 2 \dot{s} \frac{d\dot{s}}{ds} = 2 \dot{s}\frac{d\dot{s}}{dt} \frac{dt}{ds} = \frac{2 \dot{s}\ddot{s}}{\dot{s}} = 2 \ddot{s} \\
    q'' &= 2 \frac{d\ddot{s}}{ds} = 2 \frac{d\ddot{s}}{dt} \frac{dt}{ds} = \frac{2 \overset{\dotsb}{s}}{\dot{s}}
\end{align}
$$

the derivatives with respect to time:

$$
\begin{align}
    \dot{s}  &= \sqrt{q} \\
    \ddot{s} &= \frac{1}{2}q' \\
    \overset{\dotsb}{s} &= \frac{1}{2}q'' \dot{s} = \frac{1}{2}q'' \sqrt{q} \\
\end{align}
$$

Now, we get the equivalent optimization problem:

$$
\begin{align}
    \max &\int^L_0 q\ ds \\
    s.t. & \\
    & |v_{max}| \geq |r'| \sqrt{q} \\
    & |a_{max}| \geq |r'' q + \frac{1}{2}r' q'| \\
    & |j_{max}| \geq |r'''q + \frac{3}{2}r'' q' + \frac{1}{2}r'q''| \sqrt{q} \\
    & s \in [0, L] \\
\end{align}
$$

the terms $q', q'', \sqrt{q}$ introduce **non-linear** for the problem.

#### 2.1.1 B-spline

Constructe the profile for $q$ in the form of a **second order B-spline**.

- Basis function $N_i(s)$
- Control points $p$
- Knots vector $S_n = [0, 0, 0, s_1, s_2,\dotsb, s_{k-3},L,L,L]$

$$
\begin{align}
    q(s) &= \sum^K_{i=1}N_{i,2}(s) \cdot p_i \\
    q'(s) &= \sum^K_{i=1}N'_{i,2}(s) \cdot p_i \\
    q''(s) &= \sum^K_{i=1}N''_{i,2}(s) \cdot p_i \\
\end{align}
$$


### 2.2 Linear Programing (LP)

#### 2.2.1 Linearization using Pseudo-Jerk

Jerk constraint:
$$
|j_{max}| \geq |r'''q + \frac{3}{2}r'' q' + \frac{1}{2}r'q''| \sqrt{q}
$$

pseudo-jerk $\widetilde{j} = |r'''q + \frac{3}{2}r'' q' + \frac{1}{2}r'q''| \sqrt{q^*}$

**Step 1: solve $q^*$ for pseudo-jerk**

$q^*$ denote an approximate upper bound for the $q$ profile, and keep under the $j_{max}$;


$$
\begin{align}
    \max &\sum^{k}_{1} q^* \\
    s.t. & \\
    & |v_{max}| \geq |r'| \sqrt{q^*} \\
    & |a_{max}| \geq |r'' q^* + \frac{1}{2}r' q'^*| \\
    & |j_{max}| \geq |r'''q^*| \sqrt{q^*} \\
    & s \in [0, L] \\
    optional\ constraints:& \\
    & q_{process\_limit} \geq q^* \\
    & q_{BC} = q^* \\
\end{align}
$$

- $q_{process\_limit}$ is the constraint from the **physical limitations** of the vehicle machine, such as load, curvature.
- $q_{BC}$ is the boundary conditions imposed on the velocity profile

**Step 2: solve LP**

$$
\begin{align}
    \max &\sum^{k}_{1} q_i \\
    s.t. & \\
    & |v_{max}| \geq |r'| \sqrt{q} \\
    & |a_{max}| \geq |r'' q + \frac{1}{2}r' q'| \\
    & |j_{max}| \geq |r'''q + \frac{3}{2}r'' q' + \frac{1}{2}r'q''| \sqrt{q^*} \\
    & q^* \geq q \\
    & s \in [0, L] \\
    optional\ constraints:& \\
    & q_{process\_limit} \geq q \\
    & q_{BC} = q \\
\end{align}
$$


## 3. Heuristic Window Method

For industry robot that working with people, the velocity limit is not only come from the physical limitations, but also the people feel near the vehicle.

Generally, the ground friction of the factory is not stable. Therefore, speed limit is set for each path segment.

Usually, the cost of speed optimization problem is minimum jerk and minimum time. Therefore, the results of speed planning usually introduce **speed oscillations** between deceleration and acceleration.

Select the velocity constraint of the arch shape as a window, avoid the situation that acceleration after deceleration.

### 3.1 Algorithm flow

1. Determine the window size according to the speed limit;
2. Calculate the maximum feasible speed at the end point;
   1. bisection search for a feasible feed
3. Solving speed using linear programming (§2.2);








