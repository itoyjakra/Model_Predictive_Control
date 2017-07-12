
# Model Predictive Control

### The Vehicle Model

The state of the vehicle is defined as a tuple of it's coordinates $\left( x, y \right)$, heading angle $\psi$ and velocity $v$. The evolution of the state in a time interval $\Delta t$ is governed by the following kinematic equations:

$$
\begin{eqnarray}
x(t + \Delta t) &=& x(t) + v(t) cos(\psi(t)) \Delta t \\ \nonumber
y(t + \Delta t) &=& y(t) + v(t) sin(\psi(t)) \Delta t \\ \nonumber
v(t + \Delta t) &=& v(t) + a(t) \Delta t \\ \nonumber
\psi(t + \Delta t) &=& \psi(t) + \frac{v(t) \Delta t \delta(t)}{L_f} \nonumber
\end{eqnarray}
$$

where the actuator controls $\left [ a(t), \delta(t) \right ]$ consists of the acceleration (proportional to throttle)  and turning angle at time $t$ respectively. $L_f$ is a calibration constant that depends on the length of the vehicle.

### Following the Track

The vehicle needs to follow a track provided by the simulator. The information provided by the simulator are
- Vehicle coordinates
- List of coordinates of the waypoints
- Velocity
- Steering angle

A polynomial of degree 3 is fitted through the waypoints and a list of optimized waypoints are generated based on the current vehicle state with respect to the expected state. The optimal actuator response is generated to follow the optimized waypoints.

### Error Measurement

A robust set of error measures is required to obtain to optimize the path the vehicle needs to follow. The primary and most important are
- Cross Track Error (cte): The distance between the actual and desired location (center of the track) of the vehicle 
- Orientation Error ($e\psi$): The angle between the actual and desired heading direction of the vehicle

The definition and the update equation of the cross track error are as follows:

$$
\begin{eqnarray}
\mathrm{cte}(t) &=& f(x(t)) - y(t) \nonumber \\
\mathrm{cte}(t+\Delta t) &=& \mathrm{cte}(t) + v(t) sin(e\psi(t)) \Delta t \nonumber
\end{eqnarray}
$$

where $f$ is the polynomial fit to the trajectory. The orientation error and its update equation are as follows:

$$
\begin{eqnarray}
e\psi(t) &=& \psi(t) - \tan^{-1}(f^\prime(x(t))) \nonumber \\
e\psi(t+\Delta t) &=& e\psi(t) + \frac{v(t) \delta(t) \Delta t}{L_f} \nonumber
\end{eqnarray}
$$

### Coordinate Transformation

The waypoints received from the simulator are in the global coordinate system. The error calculations become simple when done in the vehicle coordinate system. The waypoints can be transformed into the vehicle coordinate system by translation them according to the vehicle's position, followed bt a rotation by the amount of the vehicle's heading angle.

### Actuator Control

The best actuator controls are obtained by minimizing a cost function with respect to an augmented state $[x, y, v, \psi, \mathrm{cte}, e\psi]$ of the vehicle subject to certain constraints. The path is optimized for $N$ future steps  with a step length of $\Delta t$. The constraints guarantee that the states in the future follow the kinematic model equations. The cost function can have several components:
- Cost of deviating from ideal state measured by cte, $e\psi$, departure from target speed etc.
- Cost of using the actuators
- Cost of changing the actuators too fast

Typically using a large weight to the cost of deviation from ideal state provides a more stable ride. Adding a cost of changing the actuators too fast provides a smoother ride. The cost of using the actuators at all is not that significant.

Once the best actuator controls are used for the present time step, the vehicle is allowed to respond to the change and move for a duration $\Delta t$. The recommended actuations for the subsequent time steps are thrown away at this point and the path is reoptimized and the calculation is repeated.

### Latency

There is always a time lag between the time an actuator control is initiated and it is actually engaged. One way to deal with it is to use a time step $\Delta t$ that is at least as large as the latency in the system. 

The length of the future path being optimized is $L = N \Delta t$. If $L$ is too short, the actuators will be tuned to the immediate path ahead and will fail to negotiate a sharp bend for example. A larger $L$ is usually better for a smoother behavior of the vehicle but at some point the polynomial will fail to fit the waypoints properly and that may lead to a crash. Hence, the choice of $N$ is critical given that $\Delta t$ is constrained by the latency of the system.

It requires some amount of trial and error to obtain the optimal value of $N$, using $\Delta t \ge $ latency. A range of values between N=3 to N=15 were tried before choosing N=10 as the optimal value with $\Delta t = 2$ * latency.


```python

```
