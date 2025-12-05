#include "navigation/Planner.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <limits>
#include <algorithm>

Planner::Planner(Costmap& costmap) : costmap_(costmap) {
}

std::vector<Point2D> Planner::planPath(Point2D start, Point2D goal) {
    // Priority queue: (f_score, node) - min-heap
    std::priority_queue<std::pair<float, GridCell>,
                        std::vector<std::pair<float, GridCell>>,
                        std::greater<>> open_set;

    std::unordered_map<GridCell, GridCell> came_from;
    std::unordered_map<GridCell, float> g_score;  // Cost from start
    std::unordered_set<GridCell> closed_set;      // Already processed nodes

    auto start_cell = costmap_.worldToGrid(start.x, start.y);
    auto goal_cell = costmap_.worldToGrid(goal.x, goal.y);

    // Check if start or goal are in obstacles
    if (costmap_.getCost(start_cell.x, start_cell.y) >= OBSTACLE_THRESHOLD) {
        return {};  // Start is in obstacle
    }
    if (costmap_.getCost(goal_cell.x, goal_cell.y) >= OBSTACLE_THRESHOLD) {
        return {};  // Goal is in obstacle
    }

    g_score[start_cell] = 0;
    open_set.push({heuristic(start_cell, goal_cell), start_cell});

    while (!open_set.empty()) {
        auto current = open_set.top().second;
        open_set.pop();

        // Skip if already processed (handles duplicate entries in priority queue)
        if (closed_set.count(current)) {
            continue;
        }
        closed_set.insert(current);

        if (current == goal_cell) {
            return reconstructPath(came_from, current);
        }

        for (auto& neighbor : getNeighbors(current)) {
            if (closed_set.count(neighbor)) {
                continue;  // Skip already processed
            }
            if (costmap_.getCost(neighbor.x, neighbor.y) >= OBSTACLE_THRESHOLD) {
                continue;  // Skip obstacles
            }

            float tentative_g = g_score[current] + distance(current, neighbor);

            auto it = g_score.find(neighbor);
            float current_g = (it != g_score.end()) ? it->second : std::numeric_limits<float>::infinity();

            if (tentative_g < current_g) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                float f = tentative_g + heuristic(neighbor, goal_cell);
                open_set.push({f, neighbor});
            }
        }
    }

    return {};  // No path found
}

bool Planner::isPathValid(const std::vector<Point2D>& path) const {
    for (const auto& point : path) {
        auto cell = costmap_.worldToGrid(point.x, point.y);
        if (costmap_.getCost(cell.x, cell.y) >= OBSTACLE_THRESHOLD) {
            return false;
        }
    }
    return true;
}

float Planner::heuristic(const GridCell& a, const GridCell& b) const {
    float dx = static_cast<float>(a.x - b.x);
    float dy = static_cast<float>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy) * costmap_.getResolution();
}

std::vector<GridCell> Planner::getNeighbors(const GridCell& node) const {
    std::vector<GridCell> neighbors;
    neighbors.reserve(8);

    // 8-connected neighbors
    const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};

    for (int i = 0; i < 8; ++i) {
        int nx = node.x + dx[i];
        int ny = node.y + dy[i];
        if (nx >= 0 && nx < costmap_.getWidth() &&
            ny >= 0 && ny < costmap_.getHeight()) {
            neighbors.push_back({nx, ny});
        }
    }

    return neighbors;
}

std::vector<Point2D> Planner::reconstructPath(
    const std::unordered_map<GridCell, GridCell>& came_from,
    GridCell current) const {

    std::vector<Point2D> path;
    path.push_back(costmap_.gridToWorld(current.x, current.y));

    while (came_from.find(current) != came_from.end()) {
        current = came_from.at(current);
        path.push_back(costmap_.gridToWorld(current.x, current.y));
    }

    std::reverse(path.begin(), path.end());
    return path;
}

float Planner::distance(const GridCell& a, const GridCell& b) const {
    float dx = static_cast<float>(a.x - b.x);
    float dy = static_cast<float>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy) * costmap_.getResolution();
}
