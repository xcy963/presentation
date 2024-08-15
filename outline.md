# 大纲

Explain how does Dijkstra algorithm for computing shortest paths in a graph from asource vertex to all other vertices work. Is it an informed or uninformed search algorithm? Compare it with A* algorithm in the task of searching for shortest paths; 1the comparison can be theoretical (explaining theoretical differences) and/or practical (providing experimental evidence).

1. dijkstra原理
2. dijkstra非启发式算法（h(n)恒为零的A*）
3. A*描述

- 原理
- 可接受性和一致性
- 路径最优的充分必要条件

4. 比较

- 演示用

```c++
#include <climits>
#include <iostream>
#include <vector>

using namespace std;

const int V = 5;  // 图中节点的数量

// 找到未访问节点中距离源节点最近的节点
int minDistance(const vector<int>& dist, const vector<bool>& sptSet) {
    int min = INT_MAX, min_index;

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
    return;
}

// 实现 Dijkstra 算法
void dijkstra(int graph[V][V], int src, int goal) {
    vector<int> dist(V, INT_MAX);  // 保存从源节点到每个节点的最短距离
    vector<bool> sptSet(
        V, false);  // sptSet[i] 将为真，表示节点 i 的最短路径已确定
    vector<int> parent(V, -1);

    dist[src] = 0;  // 源节点到自身的距离为 0

    for (int count = 0; count <= V - 1; count++) {
        int u = minDistance(dist, sptSet);  // 找到最近的未访问节点
        sptSet[u] = true;                   // 标记为已访问
        if (u == goal) {
            cout << "the distance from " << src << " to " << goal << " is "
                 << dist[goal] << endl;
            cout << "the path is:";
            printPath(parent, u);
            cout << endl;
            return;
        }

        // 更新与节点 u 相邻的节点的距离
        for (int v = 0; v < V; v++) {
            if (!sptSet[v] && graph[u][v] && dist[u] != INT_MAX &&
                dist[u] + graph[u][v] < dist[v]) {
                dist[v] = dist[u] + graph[u][v];
                parent[v] = u;
            }
        }
    }
}

int main() {
    // 图的邻接矩阵表示
    int graph[V][V] = {{0, 10, 0, 30, 100},
                       {10, 0, 50, 0, 0},
                       {0, 50, 0, 20, 10},
                       {30, 0, 20, 0, 60},
                       {100, 0, 10, 60, 0}};

    dijkstra(graph, 0, 4);  // 从节点 0 开始

    return 0;
}

```

问题描述：网格路径规划
问题：

给定一个二维网格地图，其中有一些障碍物和开放区域，你需要从起点找到到达终点的最短路径。网格的每个单元格可以是：

- 开放区域：可以在这些区域移动。
障碍物：这些区域不可通行。
A*算法将用于在网格上找到从起点到终点的最短路径。

- 网格地图
网格的每个单元格用 (i, j) 表示，其中 i 和 j 分别是行和列。

起点和终点的坐标分别为 (start_x, start_y) 和 (goal_x, goal_y)。

- 实现步骤

1. 定义网格地图：创建一个 M x N 的二维矩阵，其中 M 和 N 是网格的行数和列数，设置障碍物的位置。
2. 定义A*算法：使用A*算法在网格上寻找从起点到终点的路径。
3. 输出结果：输出路径长度和实际路径（如果找到的话）。
