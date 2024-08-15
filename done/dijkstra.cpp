#include <climits>
#include <cmath>
#include <ctime>
#include <iostream>
#include <limits>
#include <queue>
#include <vector>

using namespace std;

const int V = 50;  // 图中节点的数量

// 找到未访问节点中距离源节点最近的节点
int minDistance(const vector<int>& dist, const vector<bool>& sptSet) {
    int min = INT_MAX, min_index = -1;

    for (int v = 0; v < V; v++) {
        if (!sptSet[v] && dist[v] <= min) {
            min = dist[v];
            min_index = v;
        }
    }
    return min_index;
}

void printPath(const vector<int>& parent, int j) {
    if (parent[j] == -1) {
        cout << j;
        return;
    }
    printPath(parent, parent[j]);
    cout << "-->" << j;
}

// 实现 Dijkstra 算法
void dijkstra(vector<vector<int>>& graph, int src, int goal) {
    vector<int> dist(V, INT_MAX);  // 保存从源节点到每个节点的最短距离
    vector<bool> sptSet(
        V, false);  // sptSet[i] 将为真，表示节点 i 的最短路径已确定
    vector<int> parent(V, -1);

    dist[src] = 0;  // 源节点到自身的距离为 0

    for (int count = 0; count <= V - 1; count++) {
        int u = minDistance(dist, sptSet);  // 找到最近的未访问节点
        if (u == -1) {
            cout << "No valid node found. Exiting.\n";
            return;
        }

        sptSet[u] = true;  // 标记为已访问

        // 检查是否已经达到目标节点
        if (u == goal) {
            cout << "The distance from " << src << " to " << goal << " is "
                 << dist[goal] << endl;
            cout << "The path is: ";
            printPath(parent, u);
            cout << endl;
            return;
        }

        // 更新与节点 u 相邻的节点的距离
        for (int v = 0; v < V; v++) {
            if (!sptSet[v] && graph[u][v] != INT_MAX && dist[u] != INT_MAX &&
                dist[u] + graph[u][v] < dist[v]) {
                dist[v] = dist[u] + graph[u][v];
                parent[v] = u;
            }
        }
    }

    // 检查是否找到路径
    if (dist[goal] == INT_MAX) {
        cout << "No path from " << src << " to " << goal << endl;
    }
}

// 启发式函数：假设图是平面图，使用欧几里得距离估计（简单起见，这里假设每个节点都有
// x, y 坐标）
double heuristic(int u, int goal) {
    // 由于没有给定实际的坐标，我们使用一个简单的估计
    // 实际应用中应该根据具体问题定义启发式函数
    return abs(u - goal);
}

// 实现 A* 算法
void aStar(const vector<vector<int>>& graph, int src, int goal) {
    vector<double> dist(
        V, numeric_limits<
               double>::infinity());  // 保存从源节点到每个节点的最短距离
    vector<int> parent(V, -1);         // 保存路径
    vector<bool> closedSet(V, false);  // 已经处理过的节点集合

    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>>
        openSet;  // 单调栈栈顶是最小元素

    dist[src] = 0;
    openSet.emplace(heuristic(src, goal), src);

    while (!openSet.empty()) {
        int u = openSet.top().second;
        openSet.pop();

        if (u == goal) {
            cout << "The distance from " << src << " to " << goal << " is "
                 << dist[goal] << endl;
            cout << "The path is: ";
            printPath(parent, goal);
            cout << endl;
            return;
        }

        if (closedSet[u]) continue;
        closedSet[u] = true;

        for (int v = 0; v < V; ++v) {
            if (graph[u][v] != INT_MAX && !closedSet[v]) {
                double newDist = dist[u] + graph[u][v];
                if (newDist < dist[v]) {
                    dist[v] = newDist;
                    parent[v] = u;
                    double f = newDist + heuristic(v, goal);
                    openSet.emplace(f, v);
                }
            }
        }
    }

    cout << "No path from " << src << " to " << goal << endl;
}

int main() {
    // 图的邻接矩阵表示，使用 vector<vector<int>> 来动态分配
    vector<vector<int>> graph(V, vector<int>(V, INT_MAX));
    srand(static_cast<unsigned int>(time(NULL)));  // 随机种子

    for (int i = 0; i < V; i++) {
        for (int j = i; j < V; j++) {
            if (i == j) {
                graph[i][i] = 0;
                continue;
            }
            graph[i][j] = (rand() % 1001) + 100;  // 生成 100 到 1100 的随机权重
            graph[j][i] = graph[i][j];            // 保证对称性
        }
    }

    cout << "Start Dijkstra" << endl;
    clock_t startTimeVal = clock();
    dijkstra(graph, 0, 4);  // 从节点 0 开始
    clock_t finishTimeVal = clock();
    printf("Dijkstra time taken: %f 秒\n",
           (double)(finishTimeVal - startTimeVal) / CLOCKS_PER_SEC);

    cout << "Start A* algorithm" << endl;
    startTimeVal = clock();
    aStar(graph, 0, 4);  // 从节点 0 开始
    finishTimeVal = clock();
    printf("A* time taken: %f 秒\n",
           (double)(finishTimeVal - startTimeVal) / CLOCKS_PER_SEC);

    return 0;
}
