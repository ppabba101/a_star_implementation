#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <queue>
#include <set>
#include <utility>
#include <stdexcept>

using namespace std;

// ----------------------------------------------------------
// Node struct representing a position in the grid and 
// associated costs for A* pathfinding.
// ----------------------------------------------------------
struct Node {
    int x, y;          // Coordinates of the node in the grid
    int s_cost;        // Cost from the start node to this node
    int g_cost;        // Heuristic (estimated) cost from this node to the goal node
    bool walkable;     // True if the node can be walked on
    Node* parent;      // Pointer to the parent node (for path reconstruction)

    // Constructor to initialize the node
    Node(int x, int y, int s_cost, int g_cost, bool walkable, Node* parent)
        : x(x), y(y), s_cost(s_cost), g_cost(g_cost), walkable(walkable), parent(parent) {}

    // Overload output operator for debugging
    friend ostream& operator<<(ostream& os, const Node& node) {
        os << "Node(" << node.x << ", " << node.y << ") - "
           << "Walkable: " << (node.walkable ? "Yes" : "No") << ", "
           << "s_cost: " << node.s_cost << ", "
           << "g_cost: " << node.g_cost << ", ";
        if (node.parent) {
            os << "parent coords: " << node.parent->x << ", " << node.parent->y << endl;
        }
        return os;
    }
};

// ----------------------------------------------------------
// Map class loads the grid from a file or a given vector.
// Provides a method to get neighbors and visualize the path.
// ----------------------------------------------------------
class Map {
private:
    vector<vector<bool> >  grid; 
    int rows, cols;

    // Check if a given coordinate is inside the grid boundaries
    bool isInBounds(int x, int y) const {
        return x >= 0 && x < rows && y >= 0 && y < cols;
    }

public:
    // Constructor: initialize the map from a 2D grid of booleans
    Map(const vector<vector<bool> > & grid) : grid(grid) {
        rows = (int)grid.size();
        cols = rows > 0 ? (int)grid[0].size() : 0;
    }

    // Constructor: initialize the map from a file
    // File lines contain '0' for walkable and '1' for non-walkable squares
    Map(const string& filename) {
        ifstream file(filename);
        if (!file.is_open()) {
            throw runtime_error("Could not open file");
        }

        string line;
        while (getline(file, line)) {
            vector<bool> row;
            istringstream stream(line);
            char value;
            while (stream >> value) {
                if (value == '0') {
                    // '0' = walkable cell
                    row.push_back(true);
                } else if (value == '1') {
                    // '1' = non-walkable cell
                    row.push_back(false);
                }
            }
            if (!row.empty()) {
                grid.push_back(row);
            }
        }

        file.close();
        rows = (int)grid.size();
        cols = rows > 0 ? (int)grid[0].size() : 0;
    }

    // Returns the walkable neighbors of a cell (including diagonals)
    // Uses 8-directional movement.
    vector<pair<int, int> >  get_neighbors(int x, int y) const {
        vector<pair<int, int> >  neighbors;
        static const int directions[8][2] = {
            {1, 0}, {1, 1}, {0, 1}, {-1, 1},
            {0, -1}, {-1, -1}, {0, -1}, {1, -1}
        };

        for (const auto& dir : directions) {
            int nx = x + dir[0];
            int ny = y + dir[1];
            // Check bounds and walkability
            if (isInBounds(nx, ny) && grid[nx][ny]) {
                neighbors.emplace_back(nx, ny);
            }
        }
        return neighbors;
    }

    // Print the map and mark the path found by A*
    void visualizePath(const pair<int, int>& start, const pair<int, int>& end, const vector<pair<int, int> > & path) const {
        vector<vector<char> >  visualization(grid.size(), vector<char>(grid[0].size(), ' '));

        // Fill visualization based on walkable/non-walkable
        for (size_t i = 0; i < grid.size(); ++i) {
            for (size_t j = 0; j < grid[i].size(); ++j) {
                visualization[i][j] = grid[i][j] ? '.' : '#';
            }
        }

        // Mark the path found by A*
        for (const auto& point : path) {
            visualization[point.first][point.second] = '*';
        }

        // Mark start and end positions
        visualization[start.first][start.second] = 'S';
        visualization[end.first][end.second] = 'E';

        // Print the visualization to the console
        for (const auto& row : visualization) {
            for (char cell : row) {
                cout << cell << ' ';
            }
            cout << '\n';
        }
    }
};

// ----------------------------------------------------------
// Heuristic function for A*: approximate distance (cost) 
// between two points considering diagonal movement.
// This uses a combination of diagonal (14 cost) and straight 
// moves (10 cost).
// ----------------------------------------------------------
int approx_dist(int x1, int y1, int x2, int y2) {
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int min_d = min(dx, dy);

    // If dx == dy, distance is roughly 14 per diagonal step, 
    // otherwise combine diagonal (14 each) and straight moves (10 each)
    if (dx >= dy) {
        return 14 * min_d + 10 * (dx - dy);
    } else {
        return 14 * min_d + 10 * (dy - dx);
    }
}

// ----------------------------------------------------------
// AStar class encapsulates the entire A* search logic, so that
// main does very little besides calling it. We do not change 
// any logic, just reorganize the code and add comments.
// ----------------------------------------------------------
class AStar {
public:
    // Public function to run A* given a map, start node, and end node
    // Returns true if a path is found, false otherwise.
    bool run(const Map& map, Node* start, Node* end) {
        // Custom comparator for the priority queue, sorts by f_cost = s_cost + g_cost
        struct f_cost {
            bool operator()(Node* const& n1, Node* const& n2) {
                // f1 and f2 are the total cost estimates (f = s + g)
                int f1 = n1->s_cost + n1->g_cost;
                int f2 = n2->s_cost + n2->g_cost;

                // If we have a tie, break ties by comparing g_cost
                // The node with the smaller g_cost should have priority.
                if (f1 == f2) {
                    return n1->g_cost > n2->g_cost; 
                }
                return f1 > f2;
            }
        };

        // Priority queue (min-heap) for open set (nodes to explore)
        // Using the f_cost struct to determine priority
        priority_queue<Node*, vector<Node*>, f_cost> open;

        // Closed set stores the nodes that have been fully explored
        set<pair<int, int> >  closed;

        // Initialize the start node's heuristic cost
        start->g_cost = approx_dist(start->x, start->y, end->x, end->y);
        open.push(start);

        bool finished = false;

        // A* search loop: continues until we find the end node or run out of nodes
        while (!open.empty() && !finished) {
            // Get the node with the lowest overall cost (f_cost)
            Node* priority = open.top();
            open.pop();

            int px = priority->x;
            int py = priority->y;

            // Check if the priority node is the goal
            if (px == end->x && py == end->y) {
                // If we reached the goal, reconstruct the path
                cout << "Found path, reconstructing..." << endl;
                vector<pair<int, int> >  path;
                Node* current = priority;

                // Backtrack from the end node to the start node via parents
                while (!(current->x == start->x && current->y == start->y)) {
                    path.push_back(pair<int, int>(current->x, current->y));
                    current = current->parent;
                }
                path.push_back(pair<int, int>(current->x, current->y));

                // Visualize the path on the map
                map.visualizePath(pair<int, int>(start->x, start->y), pair<int, int>(end->x, end->y), path);
                finished = true;
                break;
            }

            // If not reached goal, explore neighbors if the current node hasn't been processed
            if (closed.find(pair<int, int>(px, py)) == closed.end()) {
                // Mark the current node as explored by adding to closed set
                closed.insert(pair<int, int>(px, py));

                // Get all walkable neighbors of the current node
                vector<pair<int, int> >  neighbors = map.get_neighbors(px, py);

                // For each walkable neighbor
                for (auto& nb : neighbors) {
                    // If neighbor isn't already processed
                    if (closed.find(nb) == closed.end()) {
                        int nx = nb.first;
                        int ny = nb.second;

                        // Calculate new costs for neighbor
                        // s_cost: distance from start
                        // g_cost: heuristic distance to goal
                        int s_cost = priority->s_cost + approx_dist(px, py, nx, ny);
                        int g_cost = approx_dist(end->x, end->y, nx, ny);

                        // Create a new node for this neighbor and push to open set
                        Node* current = new Node(nx, ny, s_cost, g_cost, true, priority);
                        open.push(current);
                    }
                }
            }
        }

        if (!finished) {
            cout << "No valid path found." << endl;
        }

        return finished;
    }
};

// ----------------------------------------------------------
// main function:
//  - Parses command line arguments
//  - Loads the map from file
//  - Creates start and end nodes
//  - Runs A* via the AStar class
//  - No logic changed, just reorganized and added comments.
// ----------------------------------------------------------
int main(int argc, char* argv[]) {
    // Check for proper usage
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <map_filename> [start_x start_y end_x end_y]" << endl;
        return 1;
    }

    string filename = argv[1];

    // Default start and end coordinates if not provided by the user
    int startX = 1;
    int startY = 2;
    int endX   = 7;
    int endY   = 6;

    // If user provided start and end positions via command line
    if (argc >= 6) {
        startX = stoi(argv[2]);
        startY = stoi(argv[3]);
        endX   = stoi(argv[4]);
        endY   = stoi(argv[5]);
    }

    // Create start and end nodes
    Node* start = new Node(startX, startY, 0, 0, true, nullptr);
    Node* end   = new Node(endX, endY, 0, 0, true, nullptr);

    // Load the map from the given file
    Map map(filename);

    // Create an AStar instance and run the search
    // All A* logic is now encapsulated in the AStar class.
    AStar astar;
    astar.run(map, start, end);

    return 0;
}