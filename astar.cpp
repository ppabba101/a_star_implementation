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

struct Node {
    int x, y;          // Coordinates of the node in the grid
    int s_cost;        // Cost from the start node to this node
    int g_cost;        // Heuristic (estimated) cost from this node to the goal node
    bool walkable;     // True if the node can be walked on
    Node* parent;      // Pointer to the parent node (for path reconstruction)

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
                    row.push_back(true);   // '0' = walkable cell
                } else if (value == '1') {
                    row.push_back(false);  // '1' = non-walkable cell
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
    vector<pair<int, int> >  get_neighbors(int x, int y) const {
        vector<pair<int, int> >  neighbors;

        // Possible directions (8-directional movement)
        static const int directions[8][2] = {
            {1, 0}, {1, 1}, {0, 1}, {-1, 1},
            {0, -1}, {-1, -1}, {0, -1}, {1, -1}
        };

        for (const auto& dir : directions) {
            int nx = x + dir[0];
            int ny = y + dir[1];
            if (isInBounds(nx, ny) && grid[nx][ny]) {
                neighbors.emplace_back(nx, ny);
            }
        }
        return neighbors;
    }

    // Print the map with the found path, start and end points
    void visualizePath(const pair<int, int>& start, const pair<int, int>& end, const vector<pair<int, int> > & path) const {
        vector<vector<char> >  visualization(grid.size(), vector<char>(grid[0].size(), ' '));

        // Fill visualization based on walkable/non-walkable
        for (size_t i = 0; i < grid.size(); ++i) {
            for (size_t j = 0; j < grid[i].size(); ++j) {
                visualization[i][j] = grid[i][j] ? '.' : '#';
            }
        }

        // Mark the path
        for (const auto& point : path) {
            visualization[point.first][point.second] = '*';
        }

        // Mark start and end
        visualization[start.first][start.second] = 'S';
        visualization[end.first][end.second] = 'E';

        // Print the visualization
        for (const auto& row : visualization) {
            for (char cell : row) {
                cout << cell << ' ';
            }
            cout << '\n';
        }
    }
};

// Approximate distance (cost) between two points using a diagonal movement-friendly heuristic
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

int main(int argc, char* argv[]) {
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <map_filename> [start_x start_y end_x end_y]" << endl;
        return 1;
    }

    string filename = argv[1];

    // Default start and end coordinates if not provided
    int startX = 1;
    int startY = 2;
    int endX   = 7;
    int endY   = 6;

    if (argc >= 6) {
        startX = stoi(argv[2]);
        startY = stoi(argv[3]);
        endX   = stoi(argv[4]);
        endY   = stoi(argv[5]);
    }

    Node* start = new Node(startX, startY, 0, 0, true, nullptr);
    Node* end   = new Node(endX, endY, 0, 0, true, nullptr);

    Map map(filename);

    // Custom comparator for the priority queue, sorts by f_cost = s_cost + g_cost
    struct f_cost {
        bool operator()(Node* const& n1, Node* const& n2) {
            int f1 = n1->s_cost + n1->g_cost;
            int f2 = n2->s_cost + n2->g_cost;
            if (f1 == f2) {
                return n1->g_cost > n2->g_cost; // Tie-break by g_cost
            }
            return f1 > f2;
        }
    };

    // Priority queue (min-heap) for open set (nodes to explore)
    priority_queue<Node*, vector<Node*>, f_cost> open;
    set<pair<int, int> >  closed; // Set of visited (closed) nodes

    // Initialize start node's heuristic cost
    start->g_cost = approx_dist(start->x, start->y, end->x, end->y);
    open.push(start);

    bool finished = false;

    // A* search loop
    while (!open.empty() && !finished) {
        // Get the node with the lowest f_cost
        Node* priority = open.top();
        open.pop();

        int px = priority->x;
        int py = priority->y;

        // Check if we reached the goal
        if (px == end->x && py == end->y) {
            cout << "Found path, reconstructing..." << endl;
            vector<pair<int, int> >  path;
            Node* current = priority;

            // Trace back the path from goal to start
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

        // If not reached goal, process neighbors if this node wasn't already processed
        if (closed.find(pair<int, int>(px, py)) == closed.end()) {
            closed.insert(pair<int, int>(px, py));
            vector<pair<int, int> >  neighbors = map.get_neighbors(px, py);

            // For each walkable neighbor
            for (auto& nb : neighbors) {
                if (closed.find(nb) == closed.end()) {
                    int nx = nb.first;
                    int ny = nb.second;

                    // Calculate new costs
                    int s_cost = priority->s_cost + approx_dist(px, py, nx, ny);
                    int g_cost = approx_dist(end->x, end->y, nx, ny);

                    // Create a new node for this neighbor
                    Node* current = new Node(nx, ny, s_cost, g_cost, true, priority);
                    open.push(current);
                }
            }
        }
    }

    if (!finished) {
        cout << "No valid path found." << endl;
    }

    return 0;
}