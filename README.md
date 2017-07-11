
# Model Predictive Control

## The Vehicle Model

The state of the vehicle is defined as a tuple of it's coordinates $\left( x, y \right)$, heading angle $\psi$ and velocity $v$. The evolution of the state in a time interval $\Delta t$ is governed by the following kinematic equations:

$$
\begin{eqnarray}
x(t + \Delta t) &=& x(t) + v(t) cos(\psi(t)) \Delta t \\
y(t + \Delta t) &=& y(t) + v(t) sin(\psi(t)) \Delta t \\
v(t + \Delta t) &=& v(t) + a(t) \Delta t \\
\psi(t + \Delta t) &=& \psi(t) + \frac{v(t) \Delta t \delta(t)}{L_f}
\end{eqnarray}
$$

where the actuator controls $\left [ a(t), \delta(t) \right ]$ consists of the acceleration (proportional to throttle)  and turning angle at time $t$. $L_f$ is a calibration constant that depends on the length of the vehicle.

### Following the Track

The vehicle needs to follow a track provided by the simulator. The information provided by the simulator are
- Vehicle coordinates
- List of coordinates of the forward looking path
- Velocity
- Steering angle


```python

```
