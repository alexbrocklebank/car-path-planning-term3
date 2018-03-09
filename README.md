# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```

## Path Planner Logic

The very first thing added is two variables `int lane` and `double ref_vel` to keep track of
both the current lane of our autonomous vehicle, which I'll abbreviate often as AV, and the
reference velocity.  These are declared outside the main lamda function to persistently keep track
of these values between websocket calls.  The foundation of my code was obtained from the
walkthrough video, which helped me quickly get up to speed with controlling the simulator.

My approach was to set boolean flags for various states the AV could be in, such as:
* `bool too_close`: Being too close to the leading vehicle.
* `bool too_slow`: Traveling at a sub-optimal speed.
* `bool lane_change_left`: Attempt to lane change left when available.
* `bool lane_change_right`: Attempt to lane change right when available.
* `vector<bool> safe`: An array of 3 boolean values, one for each lane, determining if they are safe to perform a lane change into by having no vehicles inhabiting them close to the AV along the s axis of the road.

In addition to these flags, I have a few other important variables to consider:
* `vector<double> nearest_forward`: Vector of the distance to the closest vehicle to the AV in each of the lanes, with a 100m horizon set.
* `vector<double> nearest_behind`: Vector of the distance to the closest vehicle to the AV in each lane, with a 100m horizon set in the reverse direction.
* `double lead_speed`: Keeps track of the leading vehicles speed to later ensure we do not rear-end the vehicle.
* `double lead_distance`: Stores the distance to the lead vehicle by finding the difference in s-values.

My implementation of the path planner is very simple and straightforward.  With these few variables in place I proceed to break the problem down into a few easy steps.
1. Loop through sensor fusion data to analyze locations of other cars.
2. Closest vehicles to the AV have their distances stored to be aware of possible gaps in adjacent lanes or possible tailgating of a lead vehicle.
3. Determine changes to the vehicle's s-value component by speeding up or slowing down with regards to current speed (`too_slow`) or proximity to lead vehicle (`too_close`).
4. Make decisions based on the vehicle's d-value by changing lanes left (`lane_change_left`) or right (`lane_change_right`).  If there's a choice of making either lane change, I use the `nearest_forward` vector to determine which lane change has a greater distance of open space.  Finally, before making the maneuver, I check to see if those lanes are safe (`safe[lane]`).
5. Determine future path x and y waypoints.
6. Use a spline to smooth the path and evenly space the projected path waypoints.
7. Provide the smoothed path to the `msgJson["next_x"]` and `msgJson["next_y"]` to be transmitted to the simulator.

## Reflection
