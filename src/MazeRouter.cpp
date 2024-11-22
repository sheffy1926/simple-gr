/*
 * MazeRouter.cpp
 * Work on this file to complete your maze router
 */

#include "SimpleGR.h"

///////////////////////////////////////////////////////////////////////////////
// Implement an A* search based maze routing algorithm
// and a corresponding back-trace procedure
// The function needs to correctly deal with the following conditions:
// 1. Only search within a bounding box defined by botleft and topright points
// 2. Control if any overflow on the path is allowed or not
///////////////////////////////////////////////////////////////////////////////
CostType SimpleGR::routeMaze(Net &net,
    bool allow_overflow,
    const Point &bot_left,
    const Point &top_right,
    const EdgeCost &edge_cost,
    std::vector<Edge *> &path)
{
    // Get the ID of the source and destination cells
    const IdType source_cell_id = getGCellId(net.gCellOne);
    const IdType dest_cell_id = getGCellId(net.gCellTwo);

    // Get a reference to the source cell (in order to get more information
    // about the source cell)
    const auto &dest_cell = getGCell(dest_cell_id);

    // the priority queue keeps track of which cells are visited
    // insert the source cell to the priority queue to indicate that it has been visited
    priorityQueue.setGCellCost(source_cell_id, 0., 0., NULLID);

    //@brief calculates the manhattan distance between two cells
    //  The `ManhattanCost` function object is defined in SimpleGR.h
    ManhattanCost &manhattanDistance = ManhattanCost::getFunc();

    //@brief checks if a cell is in the bounding box
    //  Note: this function ignores the z-boundary, as when `routeMaze` is called
    //  from SimpleGR `0` is always passed for the bot left and top right z-coords.
    //
    //@param cell The location of the cell to check
    //@return Whether or not `cell` is in the bounding box
    auto in_bounding_box = [&bot_left, &top_right](const Point &cell) {
        return (bot_left.x <= cell.x && cell.x <= top_right.x) && (bot_left.y <= cell.y && cell.y <= top_right.y);
    };

    //@brief given a GCell and an edgeId returns the connecting GCell
    //
    // For example, if GCell 1 is connected to GCell 2 via edge 10, this function
    // call would look like this:
    //
    // auto connected_cell = get_connecting_cell(GCell_1, Edge_10);
    // assert(getGCellID(connected_cell) == 2);
    auto get_connecting_cell = [this](const GCell &from, const IdType across_edge_id) -> GCell & {
        const auto &edge = grEdgeArr[across_edge_id];
        const auto *from_ptr = &from;
        if (from_ptr == edge.gcell1) {
            return *edge.gcell2;
        } else {
            return *edge.gcell1;
        }
    };

    //@brief checks if we have reached the destination cell
    //
    //  this function is only written to provide readability, and it's expected
    //  that it will be inlined at compile time
    auto reached_destination = [&dest_cell_id](const IdType &cellId) { return dest_cell_id == cellId; };

    //@brief Given two cells that are adjacent to one other, return a reference to
    //  the edge between the cells.
    auto get_edge = [this](const IdType cell1_id, const IdType cell2_id) -> Edge & {
        const auto &cell1 = getGCell(cell1_id);

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
    };

    //@brief Checks if traversing an edge will cause overflow
    auto causes_overflow = [this, &allow_overflow](const IdType edge_id) -> bool {
        if (allow_overflow) { return false; }

        const auto &edge = grEdgeArr[edge_id];
        const CapType demand = (edge.type == VIA) ? 0 : minWidths[edge.layer] + minSpacings[edge.layer];

        const auto will_overflow = (edge.usage + demand) > edge.capacity;

        return will_overflow;
    };

    // A* search algorithm
    //
    // The *best* cell in the priority queue is determined by the cell's "total cost", which
    // is calculated by summing the distance from source cell to the cell with the the manhattan
    // distance from the cell to the destination cell.
    //
    // In a tradiional A* search you would only insert the single next best location from the
    // current location, but this implmentation inserts all the next possible locations from
    // current location
    do {
        const auto this_cell_id = priorityQueue.getBestGCell();
        const auto this_cell_data = priorityQueue.getGCellData(this_cell_id);

        priorityQueue.rmBestGCell();

        // if the current cell is the dest cell we can pop out of this loop
        if (reached_destination(this_cell_id)) { break; }

        // get all the edges from the current cell in order to look at the
        // next possible cells
        std::vector<IdType> edges;
        {
            const auto &this_cell = getGCell(this_cell_id);
            edges = getConnectedEdges(this_cell);
        }

        for (const auto &edgeId : edges) {

            if (causes_overflow(edgeId)) { continue; }

            // get references to the cell connected to the current cell by the current edge
            const auto &this_cell = getGCell(this_cell_id);
            const GCell &connecting_cell = get_connecting_cell(this_cell, edgeId);

            // if the cell is out of the bounding box we can skip this loop
            if (!in_bounding_box(connecting_cell)) { continue; }

            // calculate the two types of cost
            // manh_cost : heuristic cost between the connecting cell and the destination
            // edge_cost : the cost from the source cell to the connecting cell
            const auto manh_cost = manhattanDistance(connecting_cell, dest_cell);
            const auto path_cost = edge_cost(edgeId) + this_cell_data.pathCost;

            // Calculate the total cost as detailed in the PQueue.setGCellCost function
            const auto total_cost = manh_cost + path_cost;

            // get the connecting cell id to insert it into the priority queue
            const auto connecting_cell_id = getGCellId(connecting_cell);

            // insert the neighbor cell into the priority queue
            if (!priorityQueue.isGCellVsted(connecting_cell_id)) {
                priorityQueue.setGCellCost(connecting_cell_id, manh_cost, total_cost, this_cell_id);
            } else {
                const auto old_cost = priorityQueue.getGCellData(connecting_cell_id).totalCost;
                if (old_cost > total_cost) {
                    priorityQueue.setGCellCost(connecting_cell_id, manh_cost, total_cost, this_cell_id);
                }
            }
        }

    } while (!priorityQueue.isEmpty());

    // now backtrace and build up the path, if we found one
    // back-track from sink to source, and fill up 'path' vector with all the edges that are traversed
    if (priorityQueue.isGCellVsted(dest_cell_id)) {
        auto current_id = dest_cell_id;

        // pre-allocate estimated space for the route path in order to minimize allocations
        {
            const auto estimated_size = manhattanDistance(getGCell(source_cell_id), getGCell(dest_cell_id));
            path.reserve(static_cast<std::size_t>(estimated_size) * 2);
        }

        // clear the path in case it was already used
        path.clear();

        while (current_id != source_cell_id) {
            const auto &current_node = priorityQueue.getGCellData(current_id);
            const auto parent_id = current_node.parentGCell;

            auto &edge = get_edge(current_id, parent_id);
            path.push_back(&edge);// this vector expects a pointer, for some reason

            current_id = parent_id;
        }
    }

    // calculate the accumulated cost of the path
    const CostType finalCost = priorityQueue.isGCellVsted(dest_cell_id)
                                   ? priorityQueue.getGCellData(dest_cell_id).pathCost
                                   : std::numeric_limits<CostType>::max();

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
