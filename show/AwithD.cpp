#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <vector>

using namespace std;

const int ROW = 5;  // 网格的行数
const int COL = 5;  // 网格的列数

struct Node {
    int x, y;
    double g, h;  // g: 从起点到当前节点的实际代价，h: 启发式估计代价
    Node* parent;

    Node(int x, int y, double g, double h, Node* p = nullptr)
        : x(x), y(y), g(g), h(h), parent(p) {}

    double f() const { return g + h; }
};

struct CompareNode {
    bool operator()(const Node* lhs, const Node* rhs) const {
        return lhs->f() > rhs->f();
    }
};

double heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);  // 曼哈顿距离
}

void aStar(const vector<vector<double>>& graph, int startX, int startY,
           int goalX, int goalY) {
    priority_queue<Node*, vector<Node*>, CompareNode> openSet;
    vector<vector<bool>> closedSet(ROW, vector<bool>(COL, false));
    vector<vector<Node*>> nodes(ROW, vector<Node*>(COL, nullptr));

    Node* start =
        new Node(startX, startY, 0.0, heuristic(startX, startY, goalX, goalY));
    openSet.push(start);
    nodes[startX][startY] = start;

    while (!openSet.empty()) {
        Node* current = openSet.top();
        openSet.pop();

        int x = current->x;
        int y = current->y;
        if (x == goalX && y == goalY) {
            cout << "Path found with cost: " << current->g << endl;
            vector<pair<int, int>> path;
            while (current) {
                path.emplace_back(current->x, current->y);
                current = current->parent;
            }
            reverse(path.begin(), path.end());
            for (const auto& p : path) {
                cout << "(" << p.first << ", " << p.second << ") ";
            }
            cout << endl;
            return;
        }

        closedSet[x][y] = true;

        vector<pair<int, int>> neighbors = {
            {x - 1, y}, {x + 1, y}, {x, y - 1}, {x, y + 1}};
        for (const auto& neighbor : neighbors) {
            int nx = neighbor.first;
            int ny = neighbor.second;

            if (nx >= 0 && nx < ROW && ny >= 0 && ny < COL &&
                !closedSet[nx][ny] &&
                graph[nx][ny] != numeric_limits<double>::infinity()) {
                double newDist = current->g + 1;  // 每个移动的代价为1
                if (nodes[nx][ny] == nullptr || newDist < nodes[nx][ny]->g) {
                    if (nodes[nx][ny] == nullptr) {
                        nodes[nx][ny] =
                            new Node(nx, ny, newDist,
                                     heuristic(nx, ny, goalX, goalY), current);
                        openSet.push(nodes[nx][ny]);
                    } else {
                        nodes[nx][ny]->g = newDist;
                        nodes[nx][ny]->parent = current;
                        openSet.push(nodes[nx][ny]);
                    }
                }
            }
        }
    }

    cout << "No path found" << endl;
}

void loadGraphFromFile(const string& filename, vector<vector<double>>& graph) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Unable to open file " << filename << endl;
        return;
    }

    string line;
    int row = 0;
    while (getline(file, line)) {
        if (line.find("Graph (Adjacency Matrix):") != string::npos) {
            continue;  // 跳过标题行
        }

        istringstream iss(line);
        for (int col = 0; col < COL; ++col) {
            string weight;
            if (!(iss >> weight)) {
                cerr << "Error reading matrix value at row " << row
                     << ", column " << col << endl;
                continue;
            }

            if (weight == "0") {
                graph[row][col] = numeric_limits<double>::infinity();
            } else {
                try {
                    graph[row][col] = stod(weight);
                } catch (const invalid_argument& e) {
                    cerr << "Invalid number format: " << weight << endl;
                    graph[row][col] = numeric_limits<double>::infinity();
                }
            }
        }
        ++row;
    }

    file.close();
}

int main() {
    vector<vector<double>> graph(
        ROW, vector<double>(COL, numeric_limits<double>::infinity()));

    loadGraphFromFile("grid_data.txt", graph);

    // 设置起点和终点
    int startX = 0, startY = 0;
    int goalX = ROW - 1, goalY = COL - 1;

    aStar(graph, startX, startY, goalX, goalY);

    return 0;
}
