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
CostType SimpleGR::routeMaze(Net &net,
    bool allowOverflow,
    const Point &botleft,
    const Point &topright,
    const EdgeCost &func,
    vector<Edge *> &path)
{

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
        const IdType currentGCellId = priorityQueue.getBestGCell();
        priorityQueue.rmBestGCell();

        // Stop if we reach the sink GCell
        if (currentGCellId == snkGCellId) break;

        const GCell &currentGCell = getGCell(currentGCellId);

        // Explore all edges connected to the current GCell
        const auto edges = getConnectedEdges(currentGCell);
        for (const IdType edgeId : edges) {
            if (edgeId == NULLID) continue;

            const Edge &edge = grEdgeArr[edgeId];
            const GCell *nextGCell = (edge.gcell1 == &currentGCell) ? edge.gcell2 : edge.gcell1;

            // Skip GCells outside the bounding box
            if (!withinBoundingBox(*nextGCell, botleft, topright)) continue;

            // Skip edges if overflow is not allowed and they are over capacity
            if (!allowOverflow && edge.usage + minWidths[edge.layer] + minSpacings[edge.layer] > edge.capacity)
                continue;

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
            path.push_back(const_cast<Edge *>(&traversedEdge));
            backtrackGCellId = parentGCellId;
        }

        // Reverse the final path to ensure it goes from source to sink
        reverse(path.begin(), path.end());
    }

    // calculate the accumulated cost of the path
    const CostType finalCost = priorityQueue.isGCellVsted(snkGCellId) ? priorityQueue.getGCellData(snkGCellId).pathCost
                                                                      : numeric_limits<CostType>::max();

    // clean up
    priorityQueue.clear();

    return finalCost;
}

// Utility function to check if a GCell is within the bounding box
bool SimpleGR::withinBoundingBox(const GCell &gcell, const Point &botleft, const Point &topright) const
{
    return (gcell.x >= botleft.x && gcell.x <= topright.x) && (gcell.y >= botleft.y && gcell.y <= topright.y);
}

// Utility function to get all connected edges for a given GCell
vector<IdType> SimpleGR::getConnectedEdges(const GCell &gcell) const
{
    return { gcell.incX, gcell.decX, gcell.incY, gcell.decY, gcell.incZ, gcell.decZ };
}

// Utility function to find the edge between two connected GCells
const Edge &SimpleGR::findEdgeBetween(const GCell &g1, const GCell &g2)
{
    for (IdType edgeId : getConnectedEdges(g1)) {
        if (edgeId == NULLID) continue;
        const Edge &edge = grEdgeArr[edgeId];
        if ((edge.gcell1 == &g1 && edge.gcell2 == &g2) || (edge.gcell1 == &g2 && edge.gcell2 == &g1)) { return edge; }
    }
    assert(false && "Edge between GCells not found!");
    return grEdgeArr[0];// Should not reach here due to the assert
}
