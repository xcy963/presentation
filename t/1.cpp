#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

using namespace std;

const int ROW = 10000;  // 网格的行数
const int COL = 10000;  // 网格的列数

double heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);  // 曼哈顿距离
}

void saveGridToFile(const string& filename) {
    ofstream file(filename);
    if (!file.is_open()) {
        cerr << "Unable to open file " << filename << endl;
        return;
    }

    file << "Graph (Adjacency Matrix):\n";
    vector<vector<double>> grid(
        ROW, vector<double>(COL, numeric_limits<double>::infinity()));

    // 随机生成网格数据，非边界点的值为1，边界点为INF
    for (int i = 0; i < ROW; ++i) {
        for (int j = 0; j < COL; ++j) {
            // 随机决定是否设置为可通行的点
            grid[i][j] =
                (rand() % 10 < 8)
                    ? 1.0
                    : numeric_limits<double>::infinity();  // 80%概率为1.0
        }
    }

    // 打印网格到文件
    for (int i = 0; i < ROW; ++i) {
        for (int j = 0; j < COL; ++j) {
            if (grid[i][j] == numeric_limits<double>::infinity()) {
                file << "0 ";
            } else {
                file << grid[i][j] << " ";
            }
        }
        file << endl;
    }

    file.close();
}

int main() {
    srand(static_cast<unsigned int>(time(0)));  // 使用当前时间作为随机数种子

    saveGridToFile("grid_data.txt");

    return 0;
}