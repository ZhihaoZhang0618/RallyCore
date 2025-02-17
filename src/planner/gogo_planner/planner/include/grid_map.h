#pragma once

#include <vector>
#include <unordered_map>
#include <iostream>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <memory>

#define PRINTF_WHITE(STRING) std::cout<<STRING
#define PRINT_GREEN(STRING) std::cout<<"\033[32m"<<STRING<<"\033[m\n"
#define PRINT_RED(STRING) std::cout<<"\033[31m"<<STRING<<"\033[m\n"
#define PRINT_YELLOW(STRING) std::cout<<"\033[33m"<<STRING<<"\033[m\n"

#define IN_CLOSE 97
#define IN_OPEN 98
#define IS_UNKNOWN 99

using namespace std;
using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using RowMatrixXi = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{   
    Eigen::Vector2i index;
    double fScore;
    double gScore;
    GridNodePtr parent;
    char id;

    GridNode() : 
        index(Eigen::Vector2i::Zero()), 
        fScore(0.0), 
        gScore(0.0), 
        parent(nullptr), 
        id(IS_UNKNOWN) {}
    
    GridNode(const Eigen::Vector2i& _index) : 
        index(_index), 
        fScore(0.0), 
        gScore(0.0), 
        parent(nullptr), 
        id(IS_UNKNOWN) {}

    void reset()
    {
        parent = nullptr;
        gScore = fScore = 0.0;
        id = IS_UNKNOWN;
        return;
    }
    
    ~GridNode(){};
};

class GridMap
{
    public:
        // params
        int             buffer_size;
        double          resolution, resolution_inv;
        Eigen::Vector2d map_origin;
        Eigen::Vector2d map_size;
        Eigen::Vector2d min_boundary;
        Eigen::Vector2d max_boundary;
        Eigen::Vector2i min_idx;
        Eigen::Vector2i max_idx;
        Eigen::Vector2i voxel_num;

    private:
        //datas
        bool is_ready = false;
        GridNodePtr* grid_node_map = nullptr;
        vector<char>   occ_buffer;
        vector<double> esdf_buffer;

    public:
        GridMap() {}
        // ~GridMap()
        // { 
        //     for (int i=0; i<buffer_size; i++) 
        //         delete[] grid_node_map[i];

        //     delete[] grid_node_map;
        //     return;
        // }
        ~GridMap()
        { 
            for (int i=0; i<buffer_size; i++) 
                delete grid_node_map[i];  // 使用 delete 释放单个对象

            delete[] grid_node_map;  // 释放指针数组
            return;
        }

        void init(std::unordered_map<std::string, double> params);
        void setMap(RowMatrixXi map);
        void updateESDF2d();
        std::pair<std::vector<Eigen::Vector2d>, double> astarPlan(const Eigen::Vector2d& start, const Eigen::Vector2d& end, double safe_threshold);
        std::vector<RowMatrixXd> getRectCorridor(std::vector<Eigen::VectorXd> state_list);
        std::vector<RowMatrixXd> getFiriCorridor(std::vector<Eigen::VectorXd> state_list);

        inline void getDistance(const Eigen::Vector2d& pos, double& distance);
        inline void getDisWithGradI(const Eigen::Vector2d& pos, double& distance, Eigen::Vector2d& grad);
        inline bool isCollision(const Eigen::Vector2d& pos, double safe_threshold);
        inline void boundIndex(Eigen::Vector2i& id);
        inline void posToIndex(const Eigen::Vector2d& pos, Eigen::Vector2i& id);
        inline void indexToPos(const Eigen::Vector2i& id, Eigen::Vector2d& pos);
        inline int toAddress(const Eigen::Vector2i& id);
        inline int toAddress(const int& x, const int& y);
        inline bool isInMap(const Eigen::Vector2d& pos);
        inline bool isInMap(const Eigen::Vector2i& idx);
        inline bool isOccupancy(const Eigen::Vector2d& pos);
        inline bool isOccupancy(const Eigen::Vector2i& id);
        inline bool isLineOccupancy(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);
        inline int getGridNum();
        inline double getResolution();
        inline RowMatrixXd getMap();

        typedef std::shared_ptr<GridMap> Ptr;
};

inline void GridMap::getDistance(const Eigen::Vector2d& pos, double& distance)
{
    if (!isInMap(pos))
    {
        distance = 1e+10;
        return;
    }

    /* use trilinear interpolation */
    Eigen::Vector2d pos_m = pos;
    pos_m(0) -= 0.5 * resolution;
    pos_m(1) -= 0.5 * resolution;

    Eigen::Vector2i idx;
    posToIndex(pos_m, idx);

    Eigen::Vector2d idx_pos;
    indexToPos(idx, idx_pos);

    Eigen::Vector2d diff = pos - idx_pos;
    diff(0) *= resolution_inv;
    diff(1) *= resolution_inv;

    double values[2][2];
    for (int x = 0; x < 2; x++)
        for (int y = 0; y < 2; y++)
        {
            Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
            boundIndex(current_idx);
            values[x][y] = esdf_buffer[toAddress(current_idx)];
        }
    
    // value & grad
    double v0 = values[0][0] * (1 - diff[0]) + values[1][0] * diff[0];
    double v1 = values[0][1] * (1 - diff[0]) + values[1][1] * diff[0];
    distance = v0 * (1 - diff[1]) + v1 * diff[1];

    return;
}

inline void GridMap::getDisWithGradI(const Eigen::Vector2d& pos, double& distance, Eigen::Vector2d& grad)
{
    if (!isInMap(pos))
    {
        distance = 0.0;
        grad.setZero();
        return;
    }

    /* use trilinear interpolation */
    Eigen::Vector2d pos_m = pos;
    pos_m(0) -= 0.5 * resolution;
    pos_m(1) -= 0.5 * resolution;

    Eigen::Vector2i idx;
    posToIndex(pos_m, idx);

    Eigen::Vector2d idx_pos;
    indexToPos(idx, idx_pos);

    Eigen::Vector2d diff = pos - idx_pos;
    diff(0) *= resolution_inv;
    diff(1) *= resolution_inv;

    double values[2][2];
    for (int x = 0; x < 2; x++)
        for (int y = 0; y < 2; y++)
        {
            Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
            boundIndex(current_idx);
            values[x][y] = esdf_buffer[toAddress(current_idx)];
        }
    
    // value & grad
    double v0 = values[0][0] * (1 - diff[0]) + values[1][0] * diff[0];
    double v1 = values[0][1] * (1 - diff[0]) + values[1][1] * diff[0];
    distance = v0 * (1 - diff[1]) + v1 * diff[1];
    grad(1) = (v1 - v0) * resolution_inv;
    grad(0) = (1 - diff[1]) * (values[1][1] - values[0][1]);
    grad(0) += diff[1] * (values[1][0] - values[0][0]);
    grad(0) *= resolution_inv;

    return;
}

inline bool GridMap::isCollision(const Eigen::Vector2d& pos, double safe_threshold = 0.0)
{
    if (isInMap(pos))
    {
        double dist;
        getDistance(pos, dist);
        return dist < safe_threshold;
    }
    else
        return true;
}

inline void GridMap::boundIndex(Eigen::Vector2i& id)
{
    id(0) = max(min(id(0), max_idx(0)), min_idx(0));
    id(1) = max(min(id(1), max_idx(1)), min_idx(1));

    return;
}

inline void GridMap::posToIndex(const Eigen::Vector2d& pos, Eigen::Vector2i& id)
{
    id(0) = floor((pos(0) - map_origin(0)) * resolution_inv);
    id(1) = floor((pos(1) - map_origin(1)) * resolution_inv);
    return;
}

inline void GridMap::indexToPos(const Eigen::Vector2i& id, Eigen::Vector2d& pos)
{
    pos(0) = (id(0) + 0.5) * resolution  + map_origin(0);
    pos(1) = (id(1) + 0.5) * resolution  + map_origin(1);
    return;
}

inline int GridMap::toAddress(const Eigen::Vector2i& id) 
{
    return id(0) * voxel_num(1) + id(1);
}

inline int GridMap::toAddress(const int& x, const int& y) 
{
    return x * voxel_num(1) + y;
}

inline bool GridMap::isInMap(const Eigen::Vector2d& pos) 
{
    if (pos(0) < min_boundary(0) + 1e-4 || \
        pos(1) < min_boundary(1) + 1e-4     ) 
    {
        return false;
    }

    if (pos(0) > max_boundary(0) - 1e-4 || \
        pos(1) > max_boundary(1) - 1e-4     ) 
    {
        return false;
    }

    return true;
}

inline bool GridMap::isInMap(const Eigen::Vector2i& idx)
{
    if (idx(0) < 0 || idx(1) < 0)
    {
        return false;
    }

    if (idx(0) > voxel_num(0) - 1 || \
        idx(1) > voxel_num(1) - 1     ) 
    {
        return false;
    }

    return true;
}

inline bool GridMap::isOccupancy(const Eigen::Vector2d& pos)
{
    Eigen::Vector2i id;

    posToIndex(pos, id);
    
    return isOccupancy(id);
}

inline bool GridMap::isOccupancy(const Eigen::Vector2i& id)
{
    if (!isInMap(id))
        return true;

    return occ_buffer[toAddress(id)] == 1;
}

inline bool GridMap::isLineOccupancy(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2)
{
    bool occ = false;
    Eigen::Vector2d diff = p2 - p1;
    int max_diff = (diff * resolution_inv).lpNorm<Eigen::Infinity>() / 0.8;
    Eigen::Vector2d step = diff / (1.0 * max_diff);

    for (int i=1; i<max_diff; i++)
    {
        Eigen::Vector2d pt = p1 + step * i;
        if (isOccupancy(pt))
        {
            occ = true;
            break;
        }
    }

    return occ;
}

inline int GridMap::getGridNum()
{
    return voxel_num(0)*voxel_num(1);
}

inline double GridMap::getResolution()
{
    return resolution;
}

inline RowMatrixXd GridMap::getMap()
{
    RowMatrixXd map(voxel_num(0), voxel_num(1));
    for (int i=0; i<voxel_num(0); i++)
        for (int j=0; j<voxel_num(1); j++)
        {
            map(i,j) = esdf_buffer[toAddress(i, j)];
        }
    return map;
}
