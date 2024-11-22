/*
 * MazeRouter.cpp
 * Work on this file to complete your maze router
 */

#include "SimpleGR.h"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
// Implement an A* search based maze routing algorithm
// and a corresponding back-trace procedure
// The function needs to correctly deal with the following conditions:
// 1. Only search within a bounding box defined by botleft and topright points
// 2. Control if any overflow on the path is allowed or not
///////////////////////////////////////////////////////////////////////////////
CostType SimpleGR::routeMaze(Net& net, bool allowOverflow, const Point &botleft,
                             const Point &topright, const EdgeCost &func, vector<Edge*> &path) {

  // find out the ID of the source and sink gcells
  const IdType srcGCellId = getGCellId(net.gCellOne);
  const IdType snkGCellId = getGCellId(net.gCellTwo);

  // insert the source gcell into the priority queue
  priorityQueue.clear();
  priorityQueue.setGCellCost(srcGCellId, 0., 0., NULLID);

  // Instantiate the Cost function to calculate the Manhattan distance between
  // two gcells. This distance is used as the heuristic cost for A* search.
  ManhattanCost &heuristic = ManhattanCost::getFunc();

  // A* search kernel
  // Loop until all "frontiers" in the priority queue are exhausted, or when
  // the sink gcell is found.
  do {
    // Get the GCell with the lowest cost from the priority queue
    IdType currentGCellId = priorityQueue.getBestGCell();
    priorityQueue.rmBestGCell();

    // Stop if we reach the sink GCell
    if (currentGCellId == snkGCellId) break;

    const GCell &currentGCell = getGCell(currentGCellId);

    // Explore all edges connected to the current GCell
    for (const IdType edgeId : getConnectedEdges(currentGCell)) {
      if (edgeId == NULLID) continue;

      const Edge &edge = grEdgeArr[edgeId];
      const GCell *nextGCell = (edge.gcell1 == &currentGCell) ? edge.gcell2 : edge.gcell1;

      if (!nextGCell) continue;

      // Skip GCells outside the bounding box
      if (!withinBoundingBox(*nextGCell, botleft, topright)) continue;

      // Skip edges if overflow is not allowed and they are over capacity
      if (!allowOverflow && edge.usage + minWidths[edge.layer] + minSpacings[edge.layer] > edge.capacity) continue;

      // Calculate costs
      const IdType nextGCellId = getGCellId(*nextGCell);
      CostType pathCost = priorityQueue.getGCellData(currentGCellId).pathCost + func(edgeId);
      CostType totalCost = pathCost + heuristic(*nextGCell, net.gCellTwo);

      // Update priority queue if this path is better
      priorityQueue.setGCellCost(nextGCellId, totalCost, pathCost, currentGCellId);
    }
  } while (!priorityQueue.isEmpty());

  // now backtrace and build up the path, if we found one
  // back-track from sink to source, and fill up 'path' vector with all the edges that are traversed
  if (priorityQueue.isGCellVsted(snkGCellId)) {
    IdType backtrackGCellId = snkGCellId;

    while (backtrackGCellId != srcGCellId) {
      const IdType parentGCellId = priorityQueue.getGCellData(backtrackGCellId).parentGCell;
      const Edge &traversedEdge = findEdgeBetween(getGCell(parentGCellId), getGCell(backtrackGCellId));
      path.push_back(const_cast<Edge*>(&traversedEdge));
      backtrackGCellId = parentGCellId;
    }

    // Reverse the final path to ensure it goes from source to sink
    reverse(path.begin(), path.end());
  }

  // calculate the accumulated cost of the path
  const CostType finalCost =
      priorityQueue.isGCellVsted(snkGCellId) ?
          priorityQueue.getGCellData(snkGCellId).pathCost : numeric_limits<CostType>::max();

  // clean up
  priorityQueue.clear();

  return finalCost;
}

// Utility function to check if a GCell is within the bounding box
bool SimpleGR::withinBoundingBox(const GCell &gcell, const Point &botleft, const Point &topright) const {
  return (gcell.x >= botleft.x && gcell.x <= topright.x) &&
         (gcell.y >= botleft.y && gcell.y <= topright.y); 
}

// Utility function to get all connected edges for a given GCell
vector<IdType> SimpleGR::getConnectedEdges(const GCell &gcell) const {
  return {gcell.incX, gcell.decX, gcell.incY, gcell.decY, gcell.incZ, gcell.decZ};
}

// Utility function to find the edge between two connected GCells
const Edge& SimpleGR::findEdgeBetween(const GCell &cell1, const GCell &cell2) {
    const auto cell1_id = getGCellId(cell1);
    const auto cell2_id = getGCellId(cell2);

        const auto cell1_coord = gcellIdtoCoord(cell1_id);
        const auto cell2_coord = gcellIdtoCoord(cell2_id);

        IdType edgeId{};

        // check all possible directions the edge could be (as I haven't found a better
        // way to get the edge id, which we need to get the edge reference)
        if (cell1_coord.x != cell2_coord.x) {
            if (cell1_coord.x < cell2_coord.x) {
                edgeId = cell1.incX;
            } else {
                edgeId = cell1.decX;
            }
        } else if (cell1_coord.y != cell2_coord.y) {
            if (cell1_coord.y < cell2_coord.y) {
                edgeId = cell1.incY;
            } else {
                edgeId = cell1.decY;
            }
        } else if (cell1_coord.z != cell2_coord.z) {
            if (cell1_coord.z < cell2_coord.z) {
                edgeId = cell1.incZ;
            } else {
                edgeId = cell1.decZ;
            }
        }

        auto &edge = grEdgeArr[edgeId];
        return edge;
}
