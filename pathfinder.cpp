#include "pathfinder.h"
#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm>

struct Node {
    PathFinder::Point position;
    Node* parent;
    float g;  // Cost from start to current node
    float h;  // Heuristic estimate to end
    float f;  // Total cost (g + h)
    
    Node() : position({0,0}), parent(nullptr), g(0), h(0), f(0) {}  // Default constructor
    Node(PathFinder::Point pos, Node* p = nullptr)
        : position(pos), parent(p), g(0), h(0), f(0) {}
        
    bool operator==(const Node& other) const {
        return position == other.position;
    }
    
    bool operator<(const Node& other) const {
        return f > other.f;  // For min-heap
    }
};

namespace std {
    template<>
    struct hash<PathFinder::Point> {
        size_t operator()(const PathFinder::Point& p) const {
            return hash<int>()(p.first) ^ (hash<int>()(p.second) << 1);
        }
    };
}

float PathFinder::heuristic(const Point& a, const Point& b) {
    return sqrtf(powf(a.first - b.first, 2) + powf(a.second - b.second, 2));
}

bool PathFinder::lineOfSight(const Grid& grid, const Point& a, const Point& b) {
    int x1 = a.first, y1 = a.second;
    int x2 = b.first, y2 = b.second;
    
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int x = x1;
    int y = y1;
    int n = 1 + dx + dy;
    int x_inc = (x2 > x1) ? 1 : -1;
    int y_inc = (y2 > y1) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;
    
    for (int i = 0; i < n; i++) {
        // Check grid bounds
        if (x < 0 || x >= (int)grid.size() || y < 0 || y >= (int)grid[0].size()) {
            return false;
        }
        
        // Check if current cell is blocked
        if (grid[x][y] != 0) {
            return false;
        }
        
        if (error > 0) {
            x += x_inc;
            error -= dy;
        } else if (error < 0) {
            y += y_inc;
            error += dx;
        } else {  // Exactly diagonal
            x += x_inc;
            y += y_inc;
            error -= dy;
            error += dx;
            n--;
        }
    }
    
    return true;
}

PathFinder::Path PathFinder::findPath(const Grid& grid, const Point& start, const Point& end) {
    // Create start and end nodes
    Node start_node(start);
    Node end_node(end);
    
    // Priority queue for open list
    std::priority_queue<Node> open_list;
    open_list.push(start_node);
    
    // Closed list
    std::unordered_set<Point> closed_list;
    
    // Possible movement directions (4-way)
    const std::vector<Point> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
    
    // Node storage and lookup
    std::unordered_map<Point, Node> node_map;
    node_map[start] = start_node;
    
    while (!open_list.empty()) {
        Node current_node = open_list.top();
        open_list.pop();
        
        // Skip if already processed
        if (closed_list.count(current_node.position)) {
            continue;
        }
        closed_list.insert(current_node.position);
        
        // Found the goal
        if (current_node == end_node) {
            Path path;
            Node* current = &current_node;
            while (current != nullptr) {
                path.push_back(current->position);
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        // Generate children
        for (const auto& dir : directions) {
            Point node_position(
                current_node.position.first + dir.first,
                current_node.position.second + dir.second
            );
            
            // Check bounds
            if (node_position.first < 0 || node_position.first >= (int)grid.size() ||
                node_position.second < 0 || node_position.second >= (int)grid[0].size()) {
                continue;
            }
            
            // Check walkable
            if (grid[node_position.first][node_position.second] != 0) {
                continue;
            }
            
            // Create new node
            Node new_node(node_position, &node_map[current_node.position]);
            
            // Calculate costs
            if (current_node.parent && lineOfSight(grid, current_node.parent->position, node_position)) {
                // Theta*: try to connect to grandparent
                new_node.g = current_node.parent->g + heuristic(current_node.parent->position, node_position);
                new_node.parent = current_node.parent;
            } else {
                // Regular A*
                new_node.g = current_node.g + 1;
            }
            
            new_node.h = heuristic(node_position, end);
            new_node.f = new_node.g + new_node.h;
            
            // Add to open list if better path found
            if (!node_map.count(node_position) || new_node.g < node_map[node_position].g) {
                node_map[node_position] = new_node;
                open_list.push(new_node);
            }
        }
    }
    
    return {};  // Return empty path if none found
}

PathFinder::Path PathFinder::optimizePath(const Grid& grid, const Path& path) {
    if (path.size() < 3) return path;
    
    Path optimized = {path[0]};
    size_t current_index = 0;
    
    while (current_index < path.size() - 1) {
        // Check from furthest point first
        for (size_t next_index = path.size() - 1; next_index > current_index; next_index--) {
            if (lineOfSight(grid, path[current_index], path[next_index])) {
                optimized.push_back(path[next_index]);
                current_index = next_index;
                break;
            }
        }
        
        // No direct path found, move to next point
        if (optimized.back() == path[current_index]) {
            current_index++;
            optimized.push_back(path[current_index]);
        }
    }
    
    // Final check for direct path from start to end
    if (optimized.size() > 2 && lineOfSight(grid, optimized[0], optimized.back())) {
        return {optimized[0], optimized.back()};
    }
    
    return optimized;
}

PathFinder::Path PathFinder::reverseOptimizePath(const Grid& grid, const Path& path) {
    if (path.size() < 3) return path;
    
    Path optimized = {path.back()};
    size_t current_index = path.size() - 1;
    
    while (current_index > 0) {
        // Check from start point first
        for (size_t next_index = 0; next_index < current_index; next_index++) {
            if (lineOfSight(grid, path[next_index], path[current_index])) {
                optimized.push_back(path[next_index]);
                current_index = next_index;
                break;
            }
        }
        
        // No direct path found, move to previous point
        if (optimized.back() == path[current_index]) {
            current_index--;
            optimized.push_back(path[current_index]);
        }
    }
    
    // Reverse to maintain start-to-end order
    std::reverse(optimized.begin(), optimized.end());
    return optimized;
}

PathFinder::Path PathFinder::splitLongSegments(const Path& path, float max_length) {
    if (path.size() < 2) return path;
    
    Path new_path = {path[0]};
    for (size_t i = 1; i < path.size(); i++) {
        float dx = path[i].first - path[i-1].first;
        float dy = path[i].second - path[i-1].second;
        float distance = sqrtf(dx*dx + dy*dy);
        
        if (distance > max_length) {
            int segments = static_cast<int>(distance / max_length) + 1;
            for (int s = 1; s < segments; s++) {
                float ratio = static_cast<float>(s) / segments;
                int new_x = static_cast<int>(path[i-1].first + dx * ratio);
                int new_y = static_cast<int>(path[i-1].second + dy * ratio);
                new_path.emplace_back(new_x, new_y);
            }
        }
        new_path.push_back(path[i]);
    }
    
    return new_path;
}

PathFinder::Path PathFinder::multiPassOptimize(const Grid& grid, const Path& path, int passes) {
    if (path.size() < 3) return path;
    
    Path optimized = path;
    for (int i = 0; i < passes; i++) {
        optimized = splitLongSegments(optimized);
        optimized = optimizePath(grid, optimized);
        optimized = splitLongSegments(optimized);
        optimized = reverseOptimizePath(grid, optimized);
    }
    return optimized;
}