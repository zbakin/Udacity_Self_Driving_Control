# Control and Trajectory Tracking for Autonomous Vehicle

# Proportional-Integral-Derivative (PID)

In this project, you will apply the skills you have acquired in this course to design a PID controller to perform vehicle trajectory tracking. Given a trajectory as an array of locations, and a simulation environment, you will design and code a PID controller and test its efficiency on the CARLA simulator used in the industry.

### Installation

Run the following commands to install the starter code in the Udacity Workspace:

Clone the <a href="https://github.com/udacity/nd013-c6-control-starter/tree/master" target="_blank">repository</a>:

`git clone https://github.com/udacity/nd013-c6-control-starter.git`

## Run Carla Simulator

Open new window

* `su - student`
// Will say permission denied, ignore and continue
* `cd /opt/carla-simulator/`
* `SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl`

## Compile and Run the Controller

Open new window

* `cd nd013-c6-control-starter/project`
* `./install-ubuntu.sh`
* `cd pid_controller/`
* `rm -rf rpclib`
* `git clone https://github.com/rpclib/rpclib.git`
* `cmake .`
* `make` (This last command compiles your c++ code, run it after every change in your code)

## Testing

To test your installation run the following commands.

* `cd nd013-c6-control-starter/project`
* `./run_main_pid.sh`
This will silently fail `ctrl + C` to stop
* `./run_main_pid.sh` (again)
Go to desktop mode to see CARLA

If error bind is already in use, or address already being used

* `ps -aux | grep carla`
* `kill id`


## Results and Discussion

### Step 1

![alt text](https://github.com/zbakin/Udacity_Self_Driving_Control/blob/master/step%201.png "Step 1 - no movement")

#### Tuning parameters:
```
pid_steer.Init(0.4, 0.001, 0.8, 1.2, -1.2);
pid_throttle.Init(0.2, 0.001, 0.06, 1.0, -1.0);
```
  

### Step 2

```
// calculate throttle error which is difference between actual and desired speeds
// v_points[closes_idx] gives closest point velocity in the planned velocity trajectory, velocity gives actual car velocity
error_throttle = v_points[closest_idx] - velocity;
```

### Step 3
```
// first get the angle between actual car and next planned waypoint using angle_between_points()
// then subtract yaw from it to get the angle between actual and desired positions -> steering error
error_steer = angle_between_points(x_position, y_position, x_points[closest_idx], y_points[closest_idx]) - yaw;
```

### Step 4

The following pictures represent the steering and throttle errors made by the car and PID controller, with respect to the given trajectory.

![alt text](https://github.com/zbakin/Udacity_Self_Driving_Control/blob/master/final_plot1.png "Step 4 - steering error")
In the given plot, you can see the amplitude of the steering error on y-axis, and the sample iteration of movement on x-axis. These iterations were fed to Carla simulator to demonstrate the movement of the car.
The PID controller makes sure the average of steering error is around 0. To calculate steering error, reference yaw needed to be found. This was done by calculating the closest point from the ego vehicle to the path. There was another method to calculate reference yaw -> the ego vehicle and the last point of the path. However, that method showed worse results. 


![alt text](https://github.com/zbakin/Udacity_Self_Driving_Control/blob/master/final_plot2.png "Step 4 - throttle error")

Here, the amplitude of the throttle error is shown on y-axis. The more iterations are complete, the more data is feeded to PID controller. That is why with time the controller stabilises and the plot smoothes. The average error approximates to 1.
In this PID controller, also closest point method to obtain reference velocity was used.


### What is the effect of the PID according to the plots, how each part of the PID affects the control command?
PID stands for Proportional, Intergral and Derivative.

1. Proportional parameter - proportionally changes the cross track error. When using this parameter only, the car tends to overshoot.
2. Integral parameter - takes into account all past error information and adjust accordingly to reduce steady state error.
3. Derivative parameter - uses rate of change of the cross track error. This helps to minimise the overshoot of the proportional part. 


### How would you design a way to automatically tune the PID parameters?
Method called Twiddle is used to tune the parameters of Kp, Ki, Kd. In brief, the algorithm iterates through variations of error parameters with the aim to minimise the average cross track error(cte). By implementing such an algorithm in a script, the parameters could be tuned automatically. 

### PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller?

#### Pros
Fast and easy to implement. There is not much knowledge needed. Parameters tuning is straightforward with the use of algorithm such as Twiddle.

#### Cons
The integral parameter takes some time before it can give an effect.
