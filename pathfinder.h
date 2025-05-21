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

    // Path optimization functions
    static Path optimizePath(const Grid& grid, const Path& path);
    static Path reverseOptimizePath(const Grid& grid, const Path& path);
    static std::vector<Point> splitLongSegments(const Path& path, float max_length = 10);
    static Path multiPassOptimize(const Grid& grid, const Path& path, int passes = 5);

private:
    // Helper functions
    static float heuristic(const Point& a, const Point& b);
    static bool lineOfSight(const Grid& grid, const Point& a, const Point& b);
};

#endif // PATHFINDER_H