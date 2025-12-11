#pragma once

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include "util/Types.h"
#include "navigation/Costmap.h"

class Planner {
public:
    // Constructor taking reference to Costmap
    explicit Planner(Costmap& costmap);

    // Plan path from start to goal, returns vector of Point2D waypoints
    std::vector<Point2D> planPath(Point2D start, Point2D goal);

    // Check if path is still traversable
    bool isPathValid(const std::vector<Point2D>& path) const;

private:
    // Euclidean distance heuristic
    float heuristic(const GridCell& a, const GridCell& b) const;

    // Get 8-connected grid neighbors
    std::vector<GridCell> getNeighbors(const GridCell& node) const;

    // Backtrack to build path
    std::vector<Point2D> reconstructPath(
        const std::unordered_map<GridCell, GridCell>& came_from,
        GridCell current) const;

    // Distance between adjacent cells
    float distance(const GridCell& a, const GridCell& b) const;

    Costmap& costmap_;
};
