#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <ostream>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <queue>
#include <set>
#include <unordered_set>

using namespace std;

struct Node {
  int x, y;               // Coordinates
  double s_cost;          // Cost from start node
  double g_cost;          // Heuristic cost to goal node
  bool walkable;          // True if the node is traversable
  Node* parent;

  friend ostream& operator<<(ostream& os, const Node& node) {
    os << "Node(" << node.x << ", " << node.y << ") - ";
    os << "Walkable: " << (node.walkable ? "Yes" : "No") << ", ";
    os << "s_cost: " << node.s_cost << ", ";
    os << "g_cost: " << node.g_cost << ", ";
    return os;
  }
};

class Map {
private:
    std::vector<std::vector<bool> > grid; // 2D grid: true = walkable, false = non-walkable
    int rows, cols;

    // Helper function to check if a position is within bounds
    bool isInBounds(int x, int y) const {
        return x >= 0 && x < rows && y >= 0 && y < cols;
    }

public:
    // Constructor to initialize the map with a given grid
    Map(const std::vector<std::vector<bool> >& grid) : grid(grid) {
        rows = grid.size();
        cols = rows > 0 ? grid[0].size() : 0;
    }
    Map(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file");
        }

        std::string line;
        while (std::getline(file, line)) {
            std::vector<bool> row;
            std::istringstream stream(line);
            char value;

            while (stream >> value) {
                if (value == '0') {
                    row.push_back(true); // 0 = walkable
                } else if (value == '1') {
                    row.push_back(false); // 1 = non-walkable
                }
            }

            if (!row.empty()) {
                grid.push_back(row);
            }
        }

        file.close();
        rows = grid.size();
        cols = rows > 0 ? grid[0].size() : 0;
    }

    // Function to get walkable neighbors of a square, including diagonals
    std::vector<std::pair<int, int> > get_neighbors(int x, int y) const {
        std::vector<std::pair<int, int> > neighbors;

        /*
        // All possible directions (up, down, left, right, diagonals)
        std::vector<std::pair<int, int> > directions = {
            pair<int, int> (0, 1),   // Right
            pair<int, int> (0, -1),  // Left
            pair<int, int> (1, 0),   // Down
            pair<int, int> (-1, 0),  // Up
            pair<int, int> (1, 1),   // Bottom-right diagonal
            pair<int, int> (1, -1),  // Bottom-left diagonal
            pair<int, int> (-1, 1),  // Top-right diagonal
            pair<int, int> (-1, -1) // Top-left diagonal
        };
        */
        static const int directions[8][2] = {{1, 0}, {1, 1}, {0, 1}, {-1, 1}, {0, -1}, {-1, -1}, {0, -1}, {1, -1}};  // 4-directional


        for (const auto& dir : directions) {
            int nx = x + dir[0];
            int ny = y + dir[1];

            if (isInBounds(nx, ny) && grid[nx][ny]) {
                neighbors.emplace_back(nx, ny);
            }
        }

        return neighbors;
    }
    void visualizePath(const std::pair<int, int>& start, const std::pair<int, int>& end, const std::vector<std::pair<int, int> >& path) const {
        std::vector<std::vector<char> > visualization(grid.size(), std::vector<char>(grid[0].size(), ' '));

        // Fill the visualization grid based on walkable (0) and non-walkable (1) cells
        for (size_t i = 0; i < grid.size(); ++i) {
            for (size_t j = 0; j < grid[i].size(); ++j) {
                visualization[i][j] = grid[i][j] ? '.' : '#'; // '.' = walkable, '#' = non-walkable
            }
        }

        // Mark the path
        for (const auto& point : path) {
            visualization[point.first][point.second] = '*'; // '' marks the path
        }

        // Mark the start and end points
        visualization[start.first][start.second] = 'S'; // 'S' = Start
        visualization[end.first][end.second] = 'E';     // 'E' = End

        // Output the visualization
        for (const auto& row : visualization) {
            for (char cell : row) {
                std::cout << cell << ' ';
            }
            std::cout << '\n';
        }
    }

};

double enc_dist(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

int main(int argc, char* argv[]) {
  Map map("ten_ten.txt");
  //Node start
  Node start = {1, 2, 0, 0, true, nullptr};
  //Node end
  Node end = {7, 6, 0, 0, true, nullptr};
  struct f_cost {
    bool operator()(Node const& n1, Node const& n2){
      return n1.s_cost + n1.g_cost < n2.s_cost + n2.g_cost;
    }
  };

  priority_queue<Node, std::vector<Node>, f_cost> open;
  set<pair<int, int> > closed;

  start.g_cost = enc_dist(start.x, start.y, end.x, end.y);

  open.push(start);
  bool finished = false;
  while (open.size() != 0 && finished == false) {
    Node priority = open.top();
    int px = priority.x;
    int py = priority.y;
    
    open.pop();
    if (priority.x == end.x && priority.y == end.y) {
      Node * current = &priority;
      vector<pair<int, int> > path;
      while(current->x != start.x && current->y != start.y) {
        path.push_back(pair<int, int> (current->x, current->y));
        current = current->parent;
      }
      path.push_back(pair<int, int> (current->x, current->y));
      finished = true;
      map.visualizePath(pair<int, int> (start.x, start.y), pair<int, int> (end.x, end.y), path);
      
    }
    else {
      closed.insert(pair<int, int> (px, py));
      vector<pair<int, int> > neighbors = map.get_neighbors(px, py);
      for (int i = 0; i < neighbors.size(); i++) {
        int nx = neighbors[i].first;
        int ny = neighbors[i].second;
        //if the neighbor is not in the closed set
        if (closed.find(neighbors[i]) != closed.end()) {
          double s_cost = priority.s_cost + enc_dist(px, py, nx, ny);
          double g_cost = enc_dist(end.x, end.y, nx, ny);
          Node current = {nx, ny ,s_cost, g_cost, true, &priority};

          open.push(current);
        }
      }
    }
  }  
}