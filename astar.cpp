#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <ostream>
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

  Node(int x, int y, bool walkable = true) 
    : x(x), y(y), s_cost(INFINITY), g_cost(INFINITY), walkable(walkable) {}

  double f_cost() const { return s_cost + g_cost; }

  friend ostream& operator<<(ostream& os, const Node& node) {
    os << "Node(" << node.x << ", " << node.y << ") - ";
    os << "Walkable: " << (node.walkable ? "Yes" : "No") << ", ";
    os << "s_cost: " << node.s_cost << ", ";
    os << "g_cost: " << node.g_cost << ", ";
    os << "f_cost: " << node.f_cost();
    return os;
  }
};

// Custom hash function for Node to use in unordered_set
struct NodeHash {
  size_t operator()(const Node& node) const {
    return hash<int>()(node.x) ^ hash<int>()(node.y);
  }
};

class UniqueNodeQueue {
private:
  queue<Node> q;
  unordered_set<Node, NodeHash> nodeSet; // To track unique nodes

public:
  // Adds a node if it's not already in the queue
  bool push(const Node& node) {
    if (nodeSet.find(node) == nodeSet.end()) { // Not found in the set
      q.push(node);
      nodeSet.insert(node);
      return true; // Node was successfully added
    }
    return false; // Node was already in the queue
  }

  // Removes a node from the front of the queue
  bool pop() {
    if (q.empty()) return false;

    Node node = q.front();
    q.pop();
    nodeSet.erase(node); // Remove from set as well
    return true;
  }

  // Access the front node in the queue
  Node front() const {
    return q.front();
  }

  // Check if the queue is empty
  bool empty() const {
    return q.empty();
  }

  // Get the size of the queue
  size_t size() const {
    return q.size();
  }
};

class Map {
private:
  vector<vector<Node>> grid;
  int width, height;

public:
  // Constructor to initialize map with given width and height
  Map(int width, int height) : width(width), height(height) {
    grid.resize(height, vector<Node>(width, Node(0, 0, true)));
    for (int y = 0; y < height; y++)
      for (int x = 0; x < width; x++)
        grid[y][x] = Node(x, y);
  }

  Map(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
      cerr << "Error: Unable to open file " << filename << endl;
      return;
    }

    string line;
    vector<vector<bool>> temp_grid;
    while (getline(file, line)) {
      vector<bool> row;
      for (char ch : line) {
        if (ch == '1') {
          row.push_back(false);  // Wall
        } else if (ch == '0') {
          row.push_back(true);   // Walkable space
        }
      }
      temp_grid.push_back(row);
    }
    file.close();

    // Set dimensions based on file input
    height = temp_grid.size();
    width = height > 0 ? temp_grid[0].size() : 0;

    // Initialize the grid of nodes
    grid.resize(height, vector<Node>(width, Node(0, 0, true)));
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        grid[y][x] = Node(x, y, temp_grid[y][x]);
      }
    }
  }


  // Accessor for a node at specific coordinates
  Node& getNode(int x, int y) {
    return grid[y][x];
  }

  // Check if a coordinate is within map boundaries
  bool inBounds(int x, int y) const {
    return x >= 0 && x < width && y >= 0 && y < height;
  }

  // Get neighbors for a given node (4 or 8 directions)
  vector<Node*> get_neighbors(int x, int y) {
    vector<Node*> neighbors;
    static const int directions[8][2] = {{1, 0}, {1, 1}, {0, 1}, {-1, 1}, {0, -1}, {-1, -1}, {0, -1}, {1, -1}};  // 4-directional

    for (auto& d : directions) {
      int nx = x + d[0], ny = y + d[1];
      if (inBounds(nx, ny) && grid[ny][nx].walkable)
        neighbors.push_back(&grid[ny][nx]);
    }
    return neighbors;
  }

  

  // Calculate the heuristic (e.g., Manhattan distance)
  double heuristic(const Node& start, const Node& goal) {
    return (start.x - goal.x) + (start.y - goal.y);
  }


  vector<Node*> find_path(Node &start, const Node &goal) {
    // UniqueNodeQueue q;
    // set<Node> done;
    // vector<Node> path;

    struct f_cost {
      bool operator()(Node * const& n1, Node * const& n2){
        return n1->f_cost() < n2->f_cost();
      }
    };

    priority_queue<Node*, std::vector<Node*>, f_cost> open;
    set<pair<int, int>> open_set; 
    set<Node> closed; 
    

    open.push(&start);
  

    while (open.size() > 0){
      Node * priority = open.top();
      open.pop();
      open_set.erase(pair(priority->x, priority->y));
     
      // Check if we've reached the goal
      if (priority->x == goal.x && priority->y == goal.y) {
        vector<Node*> path; 
        // Loop back through the parents to get the entire path
        while (priority->parent){
          path.push_back(priority->parent);
          priority = priority->parent; 
        }
      }

      //haven't reached the goal, add the neighbors to the queue
      else {
        closed.insert(*priority);

        // change get_neighbors so that it has nodes whose s_score is a delta from the current node
        vector<Node*> n = get_neighbors(priority->x, priority->y);
        for (int i = 0; i < n.size(); i++) {
          //if it hasn't already been looked at
          if (closed.find(*n[i]) == closed.end()){
            open.push(n[i]);
            // If not in the open set
            if (open_set.find(pair(priority->x, priority->y)) == open_set.end()) {
              open_set.insert(pair(priority->x, priority->y));
              open.push(n[i]);
            } 
            // if the s_score of the current node is less than what it is in the prioirty queue

            if (n[i]->s_cost < priority->s_cost) {
              n[i]->s_cost = 
            }
            //modify the s score of the element in the priority queue
            
          }
          
          
          
        } 
      }
      
    }

    
      

    //while openset isn't empty and the current node is not the goal node
    
    //
    int min_g_cost = -1;
    int index = 0;
    for (int i = 0; i < n.size(); i++) {
      if (n[i]->g_cost < min_g_cost) {
        min_g_cost = n[i]->g_cost;
        index = 0;
      }
    }
    path.push_back(*n[index]);
    
  }


};

int main(int argc, char* argv[]) {
  UniqueNodeQueue q;
  // Example usage
  int map_width = 10, map_height = 10;
  Map map(map_width, map_height);

  // Set up some obstacles
  map.getNode(3, 3).walkable = false;
  bool found = false;

  // Example neighbors access
  auto neighbors = map.getNeighbors(2, 2);
  for (auto n : neighbors) {
    cout << "Neighbor at (" << n->x << ", " << n->y << ")\n";
  }
}
