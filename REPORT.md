# Model description

State of the model is described by 6 variables:
* `x` and `y` coords
* angle `psi`
* velocity `v`
* position and direction errors `cte` and `epsi`

Transition from state `t` to `t+1` described with next set of formulas

    x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
    y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
    psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
    v_[t+1] = v[t] + a[t] * dt
    cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
    epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
    
where `psides` is desirable direction value for `psi` angle,
`Lf` respects to the range of vehicle center of mass and its front.

There are only 2 actuators for this simple model: `throttle` and `steering_angle`.
`throttle` might have negative values, which correspond to pushig the break.
In this project steering angle is limited by 25 degrees and acceleration is limited to keep 60mph speed.

# `N` and `dt` params

I selected 10 steps of 100ms after several trials.
In my tests i noticed that with large speed we need smaller `dt` since the road situation changes much faster,
but this also causes not-far estimations with same amount of steps.
Increasing number of steps is longer for my machine, so this set seem optimal for selected speed.
For higher speeds it is recommended to decrease `dt` and increase `N`.

# Polynom fit preprocessing

The only preprocessing I used is converting global coordinates from simulator to car local ones.
For this I used formulas from Q&A session:

    for (size_t i = 0; i < ptsx.size(); ++i) {
        double shift_x = ptsx[i] - px;
        double shift_y = ptsy[i] - py;
        ptsx[i] = shift_x * cos(-psi) - shift_y * sin(-psi);
        ptsy[i] = shift_x * sin(-psi) + shift_y * cos(-psi);
    }

# Latency

To handle 100ms latency, on every data processing step I first shift `px` and `py` positions predicting car location
in 100ms future.

    px = px + v * cos(psi) * latency;
    py = py + v * sin(psi) * latency;
    psi = psi - v * steer_value / Lf * latency;
    v += throttle_value * latency;

This is done before polynom preprocessing, so subsequent code, adjusted to have 0-values for `x`, `y` and `psi`, is the same