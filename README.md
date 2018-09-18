# simple_global_planner
This package provides the simplest global planner for move_base.  
The planner will just bind start and goal points linearly.  
Then, make a path by following parameter named resolution.  
Whatever the goal point is on the obstacle or not, this planner DOES NOT CARE, and generates a path.

## Installation
### 1. Just put this package into your catkinized workspace by git clone.
```bash
$ cd <catkin_ws>/src
$ git clone https://github.com/RyodoTanaka/simple_global_planner.git
```

### 2. Build it
To use `catkin build` (catkin tools) is recommended.
```bash
$ cd <catkin_ws>
$ catkin build
```

### 3. Path setting
```bash
$ cd <catkin_ws>
$ source devel/setup.bash
```

## Usage
Setup as the global_planner for move_base.  
The following setting is the example for move_base configuration.
```yaml
base_global_planner: simple_global_planner/SimpleGlobalPlanner
SimpleGlobalPlanner:
  resolution: 50
```

## Parameter
#### resolution
The path resolution between start and goal points.  
default: 50
