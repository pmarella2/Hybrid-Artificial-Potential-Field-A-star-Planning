# Hybrid-Artificial-Potential-Field-A-star-Planning
This method takes best of both world. On one hand,it tries to reduce the overall path cost by using A-star and on other hand it reduces the time complexity by adapting real time reactive power from Artificial-Potential method of motion Planning.
Welcome to the Hybrid-Artificial-Potential-Field-A-star-Planning wiki!

Here we are trying to mix two known method of motion planning , A-star and Artificial-Potential Field method.

We are implementing these two methods using following two mixtures(Environment is considered to be static).

We have find the planned path using A*, which will run offline ad find a path from source to destination.
Now two variations of reactive method (Artificial Potential field ) is boosted by this already planned path. 

1) We know that A-star is offline algorithm, which calculates path to goal directly without any feedback from next frame.  Now we can divide our planned path into local goals and use Artificial-potential method to get to those. This way it will be faster and cheaper.The algorithm goes like this.

```
local_goals = divide path from A*
local_source = current source
local_goal = local_goals[0]
path = empty_list
while local_source != final_goal:
    path += Artificial_potential_path(local_source, local_goal)
    local_source = local_goal
    local_goal = next(local_goals)
print path
```
2)Here we use A-star and Potential field simultaneously depending on some parameters. Currently in this code we are considering that parameter as distance from nearest obstacle. So algorithm goes like this.

```
if distance from nearest obstacle > k(some parameter):
    next position = A-star planner
else:
    next position = Artificial-Potential-field planner
```
