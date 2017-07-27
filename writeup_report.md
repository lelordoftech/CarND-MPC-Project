## Model Predictive Control (MPC)

The goals / steps of this project are the following:

* Choose state, input(s), dynamics, constraints and implement Model Predictive Control to drive the car around the track.
* Implement Model Predictive Control that handles a 100 millisecond latency.

[//]: # (Image References)
[image1]: ./output_images/compilation.png
[image2]: ./output_images/bad_timestepLength_and_duration_1.png
[image3]: ./output_images/bad_timestepLength_and_duration_2.png
[image4]: ./output_images/bad_timestepLength_and_duration_3.png
[image5]: ./output_images/bad_timestepLength_and_duration_4.png
[image6]: ./output_images/good_timestepLength_and_duration.png
[video1]: ./output_videos/MPC_Final.mp4

## [Rubric](https://review.udacity.com/#!/rubrics/896/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---

### Compilation

#### 1. Your code should compile.

![alt text][image1]

### Implementation

#### 1. The Model

Follow lessons, I build my model look like below:

##### 1.1 State

My model have state for tracking:

* `x`: x position of the vehicle in **m**
* `y`: y position of the vehicle **m**
* `psi`: The orientation of the vehicle in **radians**
* `v`: The current velocity in **mph**
* `cte`: Cross Track Error in **m**
* `epsi`: Orientation Error in **radians**

##### 1.2 Actuators

My model have actuator for controlling:

* `delta`: Steering angle in **radians** `[-1, 1]`. It must be divide by `deg2rad(25)` to compatible with simulator.
* `a`: Acceleration (throttle) `[-1, 1]`

##### 1.3 Update equations

###### 1.3.1 Update state

```cpp
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
```

* `x[t]` : current x position of the vehicle
* `v[t]*cos(psi[t])*dt` : the change in position caused by the vehicle's movement

```cpp
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
```

* `y[t]` : current y position of the vehicle
* `y[t]*sin(psi[t])*dt` : the change in position caused by the vehicle's movement

```cpp
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
```

* `psi[t]` : current orientation of the vehicle
* `v[t]/Lf*delta[t]*dt` : the change in orientation caused by the vehicle's movement

```cpp
v_[t+1] = v[t] + a[t] * dt
```

* `v[t]` : current velocity of the vehicle
* `a[t]*dt` : the change in velocity caused by the vehicle's movement

```cpp
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
```

* `f(x[t]) - y[t]` : current cross track error (`cte[t]`)
* `v[t]*sin(epsi[t])*dt` : the change in error caused by the vehicle's movement

```cpp
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

* `psi[t] - psides[t]` : current orientation error (`epsi[t]`)
* `v[t]*delta[t]/Lf*dt` : the change in error caused by the vehicle's movement

`psides[t]` : can be calculated as `arctan(f′(x))`

When `f(x) = coeffs[0] + coeffs[1] * x + coeffs[2] * x ^ 2 + coeffs[3] * x0 ^ 3`, we have orientation destination:

* Degree 1: `arctan(coeffs[1])`
* Degree 2: `arctan(coeffs[1] + 2*coeffs[2]*x)`
* Degree 3: `arctan(coeffs[1] + 2*coeffs[1]*x + 3*coeffs[2]*x^2)`

###### 1.3.2 Update actual cost function:

* Based on the reference state

```cpp
for (t = 0; t < N; t++) {
  fg[0] += CppAD::pow(vars[cte_start + t] - ref_cte, 2);
  fg[0] += CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
  fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
}
```

* Minimize change-rate

```cpp
for (t = 0; t < N - 1; t++) {
  fg[0] += CppAD::pow(vars[delta_start + t], 2);
  fg[0] += CppAD::pow(vars[a_start + t], 2);
}
```

* Minimize the value gap between sequential actuations

```cpp
for (t = 0; t < N - 2; t++) {
  fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
  fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}
```

To tuning MPC, I multiply above part by a value > 1. It help the steering angle values to be smooth.

File `MPC.cpp`, line 47-64:

```cpp
    // The part of the cost based on the reference state.
    for (t = 0; t < N; t++) {
      fg[0] += 2000 * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
      fg[0] += 2000 * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (t = 0; t < N - 1; t++) {
      fg[0] += 5 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 5 * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (t = 0; t < N - 2; t++) {
      fg[0] += 200 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 10 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```

#### 2. Timestep Length and Elapsed Duration (N & dt)

I start with sample in the Quizz:

```cpp
size_t N = 25;
double dt = 0.05;
```

![alt text][image2]

The result is too bad.

After that I try to increase duration:

```cpp
size_t N = 25;
double dt = 0.1;
```

![alt text][image3]

And the result is still bad.

Another case, I try to decrease timestep lenght, it maybe help me speed up calculation:

```cpp
size_t N = 20;
double dt = 0.05;
```

![alt text][image4]

The result is better although vehicle still cannot keep lane.

Continue decrease timestep lenght, I get better result:

```cpp
size_t N = 10;
double dt = 0.05;
```

![alt text][image5]

That's great, vehicle can run far away but it run with sine wave and throw out of lane when get into curve lane.

The prediction horizon T in this case is 0.5s. It maybe too small. So I increase duration:

```cpp
size_t N = 10;
double dt = 0.1;
```

![alt text][image6]

That's awesome, vehicle can keep lane perfectly. The steering does change smoothly.

I also try to continue increase duration, so the prediction horizon T is higher. But vehicle run slowly (10mph).

So I choose these final value for my project.

File `MPC.cpp`, line 9-10:

```cpp
size_t N = 10;
double dt = 0.1;
```

#### 3. Polynomial Fitting and MPC Preprocessing

##### 3.1 Polynomial Fitting

To predict better with all straight and curve lane cases, I did not use Polynomial Fitting with degree is 1.

I already tested degree is 2. It works well.

But to make the car run more smoothly, reduce Cross Track Error, I used degree is 3. I thing it is enough for this project, maybe higher value will compatible with high speed.

File `main.cpp`, line 121:

```cpp
auto coeffs = polyfit(eigen_ptsx, eigen_ptsy, 3);
```

##### 3.2 MPC Preprocessing

Before using MPC, I transform waypoints data from map's coordinate system to car's coordinate system.

Because we need to use MPC to predict actuators (steering angle, throttle) for the vehicle, these value must be in car's coordinate system. So vehicle state must be in car's coordinate system.

File `main.cpp`, line 103-111:

```cpp
for (i = 0; i < ptsx.size(); i++)
{
  // translation
  double X = ptsx[i] - px;
  double Y = ptsy[i] - py;
  // rotation counter-clockwise psi
  ptsx[i] = (X*cos(psi) + Y*sin(psi));
  ptsy[i] = (-X*sin(psi) + Y*cos(psi));
}
```

#### 4. Model Predictive Control with Latency

##### 4.1 Without Latency

In this case, we just need to use the data sent back from the server after tranform coordinate step for input of MPC.

```cpp
Eigen::VectorXd state(6);
state << 0, 0, 0, v, cte, epsi;
auto vars = mpc.Solve(state, coeffs);
```

##### 4.2 With Latency

In this case, I try to predict vehicle state after latency time and use this state for input of MPC.

So we can predict the future state of vehicle after latency time.

File `main.cpp`, line 136-152:

```cpp
Eigen::VectorXd state(6);
state << 0, 0, 0, v, cte, epsi;

// predict state in 100ms
double delta = j[1]["steering_angle"];
double acceleration = j[1]["throttle"];
double latency = 0.1;

double x_pred = 0 + v*cos(0)*latency;
double y_pred = 0 + v*sin(0)*latency;
double psi_pred = 0 - v*delta/Lf*latency; // compatible to simulator
double v_pred = v + acceleration*latency;
double cte_pred = polyeval(coeffs, x_pred) - y_pred;
double epsi_pred = 0 - atan(coeffs[1] + 2*coeffs[2]*x_pred + 3*coeffs[3]*x_pred*x_pred);
state << x_pred, y_pred, psi_pred, v_pred, cte_pred, epsi_pred;

auto vars = mpc.Solve(state, coeffs);
```

### Simulation

#### 1. The vehicle must successfully drive a lap around the track.

Here's a [link to my final video result][video1]

---

### References:

* https://www.youtube.com/watch?v=bOQuhpz3YfU&index=5&list=PLAwxTw4SYaPnfR7TzRZN-uxlxGbqxhtm2
* https://discussions.udacity.com/t/how-to-incorporate-latency-into-the-model/257391

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.

##### Techniques:

* Transfomation: translation and translation
* Predict state in latency time
* Polynomial Fitting
* Model Predictive Control

##### Fail cases:

* The speed is too high (>100mph), my controller maybe cannot adaptive.

##### Improve:

* Improve MPC to work with high speed (>100mph)
* Improve MPC to work with high latency (>100ms)
* Implement PID to work with MPC.
