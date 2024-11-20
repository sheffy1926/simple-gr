Nate Sheffield - A02268057
VLSI 5460 - Design Automation

What is the purpose of each of the 3 stages in this routing algorithm? ********************************

The Initial Routing lays out an initial path for all nets with minimal effort. The initial routing 
prioritizes simple "flat" routes that stick to a single layer. This routing is congestion aware but 
uses it as a soft constraint which allows for some overflow because it is prioritizing connecting 
routes together. 

The Rip-Up and Re-Route iteratively improves the solution by looking at the congestion and overflows 
caused by the initial routing. It identifies any nets that pass through overflow edges, removes 
their paths and reroutes them with updated costs to avoid overflows. Rip-up and re-route also uses 
cost functions that track the history of routes and penalizes edges that repeatedly overflow. 

The Greedy Routing focuses on minimizing wire length once an overflow free solution is found for all
nets. The greedy routing revisits each net, removes its current route knowing at least one overflow 
free solution is available. This routing ensures the shortest paths possible without causing any new
overflows. 

How are they different from each other? ***************************************************************

Initial Routing finds fast, simple, rough solutions while using bounding box constraints. 
Rip-Up and Re-Route focuses on eliminating overflow by iteratively rerouting nets.
Greedy Routing focuses on minimizing wire length by removing its current net to see if a shortest path
without overflow exists. 

How do they work together as a flow? ******************************************************************
The initial routing provides a basic routing solution for the nets. The rip-up and re-route refines 
the initial basic solution by iteratively improving the soltuion ensuring all nets are routable 
without overflows. The greedy solution finalizes the final solution by optimizing wire lengths 
without introducing new overflows.

How does SimpleGR build a framework to allow the 3 stages?
SimpleGR builds a framework composed of a combination of a data structure that uses cost models to 
route the nets in each stage. The data structure is made up of a 3D grid representation, that has Gcells, 
edges and their properties. SimpleGR then uses this data structure in a priority queue to initially route 
the nets. This queue also all routable nets to be routed. The initial routing prioritizes flat routes and 
is congestion aware but allows some overflow to route all nets. Next SimpleGR then make iterative 
improvements to the solution by using a DLMcost function to balance congestion and routing demand for the 
ripping-up and re-routing. This then produces a solution that if routable will have no overflows for all 
nets within the grid.Lastly SimpleGR uses a greedy algorithm to minimize the wire length of the nets 
without adding any new overflows by using the UnitCost function. This framework set in the grid 
representation of the cells, edges and their properities allows the program to efficiently route, re-route 
and optimize the connections of all nets within the grid.