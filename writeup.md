## Writeup for the MPC Control project

### By: Ibrahim Almohandes, Sept. 10, 2017

In this project, I am using the Model-Predictive-Control (MPC) method to control both steering and speed of the vehicle around the simulator track.

In the lambda function (defined inside _main()_ in **main.cpp**), which gets triggered by the simulator events, I first read the simulator provided values of vehicle position (_px_, _py_), orientation (_psi_), and speed (_v_), as well as the waypoints (_ptsx_, _ptsy_). These values are given in the simulator's map (global) space.

In order to make the model work with the MPC solver, I transform the waypoints' coordinates from the map space to the vehicle (local) space, by using translation and rotation (lines 122-129). Then, I fit the transformed waypoints into a third-degree polynomial (lines 136-138).

Next, I calculate cross-track-error (_cte_) and orientation error (_epsi_), both are now in the vehicle coordinates. After that, I call the MPC solver with the initial _state_. Notice that as _px_, _py_, and _psi_ should be effectively zeros in the vehicle domain, I pass them as such to the MPC solver.

The MPC solver function (defined as _MPC::Solve()_ in **MPC.cpp**) returns a vector called _var_, where the first two values represent the actuators which I send back to the simulator as _steering_angle_ (after conversion into the [-1, 1] range) and _throttle_.

Now let's describe how the MPC solver works. The MPC model is defined and implemented in the files **MPC.h** and **MPC.cpp** respectively.

### The MPC Model:

The MPC class has a function called Solve() which takes two input vectors: an initial six-value _state_ and the fitted polynomial coefficients (_coeffs_). Then a new single-dimensional vector called _var_ is declared (which will sequentially contain all the values of state and actuation vectors). All the _vars_ values are initialized to zeros, except the start values of only state variables, which are assigned to the initial _state_ values. After that, I define lower and upper bounds on the _vars_ values (_vars_lowerbound_ and _vars_upperbound_) which are relaxed for state values, but limited for the actuator values into the ranges: *[-25\*PI/180, 25\*PI/180]* for steering (_delta_), and *[-1, 1]* for acceleration (_a_).

Then I define the constraints lower and upper bounds (_constraints_lowerbound_ and _constraints_upperbound_) for only the state variables, which are all assigned to zeros, except the start values which are assigned to the initial state values (_x_, _y_, _psi_, _v_, _cte_, and _epsi_).

In addition to the MPC class and its Solve() function, a class called FG_eval is defined with an _operator()_ function (lines 40-140 in **MPC.cpp**) which gets called by the CppAdd solver, along with the all the variables, lower/upper bounds, and constraints.

Here's how the FG_eval::operator() function works:

>It takes the _vars_ values as input, and another variable passed by reference, which the function will calculate (_fg_). The first value of _fg_ holds the cost function value (which needs to be optimized/minimized). Then, I add to this cost all the variables and changes in variables that I need to minimize. These are defined at lines 55-75 (in **MPC.cpp**). Here, first I choose to minimize _cte_, _epsi_, and _v_ in comparison to desired reference values (zero for the first two, and a reference speed for the third). I also want to minimize the use of actuators (_delta_ and _a_). Finally, I desire to minimize the sequential differences for both _delta_ and _a_. I found out - after driving the vehicle in the simulator - that I need to apply a large weight on the sequential steering change (at line 73) to protect the vehicle from wild swings in steering which will eventually crash the vehicle or drive it completely off the road. As we increase speed, we may need to increase this factor even more (as I will later explain, at the last section).

>Then the function loops through N iterations, which is the number of MPC points we need to predict. At first iteration, we assign to the constraints vector (_fg_) the start values of state variables and actuators from _vars_. Notice that we start from _fg[1]_, as the cost is stored at _fg[0]_. Then, for all the remaining iterations, we try to minimize the difference - to zero - between the current and next states, as per the equations at lines 130-137. After _N_ iterations, the function returns with a complete _fg_ vector, which will be used by the CppADD solver.

>Then I pass the _fg_eval_ object, along with all the variables, variable bounds, and constraints, to the CppAdd solver function _CppAD::ipopt::solve()_ (at lines 259-261), which calculates the new state and actuator values for us. Then, I only need to return back (from _MPC::Solve()_) the actuator start control values (_delta_ and _a_). However, I also return all the predicted points (_x_ and _y_), by adding them to the returned vector, so that they can be used for visualization of MPC points in the simulator.

Back to the lambda function inside _main()_: In addition to passing back the actuators' control values for steering and acceleration, I also send to the simulator a set of equally-spaced points from the fitted polynomial (which get drawn in yellow), as well as the predicted MPC points we got from the solver (which get drawn in green). Before passing control values - and points - back to the simulator, we need to simulate the real behavior of actuators by applying a latency delay of _10 ms_.

### Time step Length and Elapsed Duration (N & dt)

I tried multiple combinations of _N_ and _dt_ values, and found the following observations:

1. Applying ```dt > latency```, for example ```dt = 0.15``` - which is more than the actuator's latency (```10 ms```) - will cause the vehicle to crash or go off the road, as this is too much duration to predict upon. That is regardless of the _N_ value.
2. Applying ```dt = latency```, which if only combined with a good number of predicted points (_N_), the vehicle will be driven within the paved portion of the road. I found ```N = 10``` to be a good candidate to apply besides ```dt = 0.1```. This solution works without requiring a special consideration for the actuators' latency. 
3. Applying ```dt < latency``` will not work - in other words, vehicle crashes or goes off the road - unless we make a special consideration for latency before applying the _state_ input to the MPC solver. However, if we predict future state values (as I will explain in the next section), then pass these (future) values as initial state to the MPC solver, then we can get as comparably good driving behavior as (even slightly better than) the one we got in case (**2**) above. I tried the following values ```dt = 0.05, 0.075```, and they work well with a value of ```N = 10```. I also found increasing _N_ to ```15, 20, 25``` also works. However, it's not recommended to increase _N_ too much, as this will lead to a fairly large total delay ```= N * dt``` that can worsen the driving behavior.

Finally, I settled on ```dt = 0.05, and N = 10``` to autonomously drive the vehicle in the simulator, as you can see in my recorded video [mpc_video.mov](./mpc_video.mov). I also used a reference speed of 60 mph, with more fine tuning of the cost function (as will be explained in the last section).

### Model Predictive Control with Latency

- As I explained earlier, if the duration (_dt_) for calculating each predicted state in the MPC solver is less than the actuators` latency, we need to make a special arrangement for that by first predicting the future state (using current state, actuators, and latency), as can be seen in the equations on lines 103-107 in **main.cpp**. Then, this future state can be used, after coordinate transformation, as the initial state passed to the MPC solver.
- If the the duration _dt_ is equal to latency, then removing the special considerations (as described in the previous bullet point), will not have much effect on the vehicle's driving behavior if it gets combined with a suitable N value, which I explained in the previous section.
- Finally having a large _dt_ (greater than latency) is not a good idea, regardless of compensating for actuators' latency or not.

### Increasing Speed and fine tuning the cost variables

I was able to increase the vehicle's speed from 40 to 60 mph by increasing the multiplication factor of the change in steering (as per the equation on line 73 in **MPC.cpp**). First, I was able to drive the vehicle autonomously at 40 mph when this factor was set to 500. However, when I increased the speed to 60 mph, I found I have to increase this factor to at least 1000 to be able to drive the vehicle safely. Increasing this value more can lead to a better steering/driving as I can tell from the smaller visual differences between the yellow and green lines. Hence, I used a factor of 2000 for my recorded video. Increasing this value too much may increase the computational effort/time of the MPC solver without any real benefit (which might cause a much delayed response).
