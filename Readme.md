# AMR Class

Result of the course "autonomous mobile robots" at the university Ravensburg-Weingarten. The goal is to escape a maze with the Turtlebot. The training maze is given and known, however the contest maxze is unknown.

It is not allowed to create a map and contrary to the most maze games found online, there are more option than left, right, forth and back. Since there are no fix positions like on a chess board, the Turtlebot can be anywhere in the maze.

The coordinates of the goal are published to the topic `goal_pub` and the robot permanently listens to that topic. It also tracks the travelled distance using the given `dist_tracker`


![](https://fbe-gitlab.hs-weingarten.de/mat-iki/amr-mat/blob/master/.img/tier_z.png)

## Start simulation
```
roslaunch ll-162143_maze maze_solver.launch
```

## Solution

For info on the solution, please see the [Wiki](https://github.com/BananaKingg/AMR/wiki)
