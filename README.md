# pyrobosim with Lazy PRM planner

Lazy PRM is a specific variant of the PRM algorithm. In the context of motion planning, "lazy" strategies focus on postponing computations until they are actually needed. In the case of Lazy PRM, the roadmap is built incrementally as the planner explores the configuration space.

The basic idea behind Lazy PRM is to only generate and evaluate portions of the roadmap that are necessary to find a path when requested. Traditional PRM constructs the entire roadmap in advance, which can be computationally expensive, especially in high-dimensional spaces. Lazy PRM, on the other hand, postpones the expensive parts of the computation until they are needed for a specific query.

Lazy PRM is often preferred in scenarios where the configuration space is large or complex, as it can potentially save computational resources by only computing the parts of the roadmap that are relevant to the specific start and goal configurations.

## implementation

In this Lazy PRM implementation, during the sampling stage, a search graph is constructed without expensive collision checks. When sampling the configuration space, a randomly selected node is added to the graph without verifying whether the node is free. As all nodes are generated throughout the space, edges between nodes are also created without confirming their collision-free status.

During the planning stage, when a path is generated, collision checks come into play to verify whether a waypoint and the edge leading to it are collision-free. If a collision is detected, the waypoint is removed from the graph, and a new path is subsequently generated based on the updated graph.

## result

For about same amount of sampling time, PRM planner and Lazy PRM planner is compared in navigation to bedroom, bathroom and kitchen, and result is as following

### PRM planner
sampling time, nodes, and planning time to each room
```
Planner : PRMPlannerPolygon
nodes: 200
Sampling time : 1.8359971046447754
[robot] Navigating to Room: bedroom
Planning time : 0.04686617851257324
[robot] Navigating to Room: bathroom
Planning time : 0.07547473907470703
[robot] Navigating to Room: kitchen
Planning time : 0.06770682334899902
```

A figure of PRM planner search graph with 200 nodes
![PRM planner](https://github.com/guangyingjiang/pyrobosim/blob/custom_PRM/figures/PRM.png)

### Lazy PRM planner
sampling time, nodes, and planning time to each room
```
Planner : LazyPRMPlannerPolygon
nodes: 1000
Sampling time : 1.4912128448486328
[robot] Navigating to Room: bedroom
Planning time : 2.99320125579834
[robot] Navigating to Room: bathroom
Planning time : 1.2985970973968506
[robot] Navigating to Room: kitchen
Planning time : 0.0724947452545166
```

Figures of Lazy PRM planner search graph with 1000 nodes, and note that search graph is updated after each run
![Lazy PRM planner From start to bedroom](https://github.com/guangyingjiang/pyrobosim/blob/custom_PRM/figures/Lazy%20PRM%20to%20bedroom.png)
![Lazy PRM planner From bedroom to bathroom](https://github.com/guangyingjiang/pyrobosim/blob/custom_PRM/figures/Lazy%20PRM%20to%20bathroom.png)
![Lazy PRM planner From bathroom to kitchen](https://github.com/guangyingjiang/pyrobosim/blob/custom_PRM/figures/Lazy%20PRM%20to%20kitchen.png)

### conclusion

 * For about same amount of sampling time, Lazy PRM planner can sample more nodes than PRM planner due to no collision checks.
 * Lazy PRM planner uses more planning time than PRM planner for first few runs due to collision checks in planning stage.
 * Lazy PRM planner about same time than PRM planner after a few runs due to updated and simplified search graph.