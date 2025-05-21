#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <vector>
#include <utility>  // for std::pair
#include <unordered_set>

class PathFinder {
public:
    using Point = std::pair<int, int>;
    using Grid = std::vector<std::vector<int>>;
    using Path = std::vector<Point>;

    // Core pathfinding function (Theta* variant)
    static Path findPath(const Grid& grid, const Point& start, const Point& end);


private:
    // Helper functions
    static float heuristic(const Point& a, const Point& b);
    static bool lineOfSight(const Grid& grid, const Point& a, const Point& b);
};

#endif // PATHFINDER_H