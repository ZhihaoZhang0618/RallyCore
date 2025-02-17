#include "grid_map.h"

void GridMap::init(std::unordered_map<std::string, double> params)
{
    map_size[0] = params["map_size_x"];
    map_size[1] = params["map_size_y"];
    resolution = params["resolution"];

    // origin and boundary
    min_boundary = -map_size / 2.0;
    max_boundary = map_size / 2.0;
    map_origin = min_boundary;

    // resolution
    resolution_inv = 1.0 / resolution;

    // voxel num
    voxel_num(0) = ceil(map_size(0) / resolution);
    voxel_num(1) = ceil(map_size(1) / resolution);

    // idx
    min_idx = Eigen::Vector2i::Zero();
    max_idx = voxel_num - Eigen::Vector2i::Ones();

    // datas
    buffer_size  = voxel_num(0) * voxel_num(1);
    esdf_buffer = vector<double>(buffer_size, 0.0);
    occ_buffer = vector<char>(buffer_size, 0);
    grid_node_map = new GridNodePtr[buffer_size];
    for (int i=0; i<buffer_size; i++)
        grid_node_map[i] = new GridNode();
}

void GridMap::setMap(RowMatrixXi map)
{
    for (int i=0; i<voxel_num(0); i++)
    {
        for (int j=0; j<voxel_num(1); j++)
        {
            occ_buffer[toAddress(i, j)] = map(i, j);
        }
    }
        
    updateESDF2d();
    is_ready = true;
    return;
}

template <typename F_get_val, typename F_set_val>
void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int size) 
{
    int v[size];
    double z[size + 1];

    int k = start;
    v[start] = start;
    z[start] = -std::numeric_limits<double>::max();
    z[start + 1] = std::numeric_limits<double>::max();

    for (int q = start + 1; q <= end; q++) {
        k++;
        double s;

        do {
            k--;
            s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
        } while (s <= z[k]);

        k++;

        v[k] = q;
        z[k] = s;
        z[k + 1] = std::numeric_limits<double>::max();
    }

    k = start;

    for (int q = start; q <= end; q++) {
        while (z[k + 1] < q) k++;
        double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
        f_set_val(q, val);
    }
}

void GridMap::updateESDF2d()
{
    int rows = voxel_num[0];
    int cols = voxel_num[1];

    Eigen::MatrixXd tmp_buffer;
    Eigen::MatrixXd neg_buffer;
    Eigen::MatrixXi neg_map;
    Eigen::MatrixXd dist_buffer;
    tmp_buffer.resize(rows, cols);
    neg_buffer.resize(rows, cols);
    neg_map.resize(rows, cols);
    dist_buffer.resize(rows, cols);

    /* ========== compute positive DT ========== */

    for (int x = min_idx[0]; x <= max_idx[0]; x++)
    {
        fillESDF(
            [&](int y)
            {
                return occ_buffer[toAddress(x, y)] == 1 ?
                    0 :
                    std::numeric_limits<double>::max();
            },
            [&](int y, double val) { tmp_buffer(x, y) = val; }, min_idx[1],
            max_idx[1], cols
        );
    }

    for (int y = min_idx[1]; y <= max_idx[1]; y++) {
        fillESDF(
            [&](int x) { return tmp_buffer(x, y); },
            [&](int x, double val)
            {
                dist_buffer(x, y) = resolution * std::sqrt(val);
            },
            min_idx[0], max_idx[0], rows
        );
    }

    /* ========== compute negative distance ========== */
    for (int x = min_idx(0); x <= max_idx(0); ++x)
        for (int y = min_idx(1); y <= max_idx(1); ++y)
        {
            if (occ_buffer[toAddress(x, y)] == 0)
            {
                neg_map(x, y) = 1;
            } else if (occ_buffer[toAddress(x, y)] == 1)
            {
                neg_map(x, y) = 0;
            } else 
            {
                PRINT_RED("what?");
            }
        }

    for (int x = min_idx[0]; x <= max_idx[0]; x++) {
        fillESDF(
            [&](int y)
            {
                return neg_map(x, y) == 1 ?
                    0 :
                    std::numeric_limits<double>::max();
            },
            [&](int y, double val) { tmp_buffer(x, y) = val; }, min_idx[1],
            max_idx[1], cols
        );
    }

    for (int y = min_idx[1]; y <= max_idx[1]; y++)
    {
        fillESDF(
            [&](int x) { return tmp_buffer(x, y); },
            [&](int x, double val)
            {
                neg_buffer(x, y) = resolution * std::sqrt(val);
            },
            min_idx[0], max_idx[0], rows
        );
    }

    /* ========== combine pos and neg DT ========== */
    for (int x = min_idx(0); x <= max_idx(0); ++x)
        for (int y = min_idx(1); y <= max_idx(1); ++y)
        {
            esdf_buffer[toAddress(x, y)] = dist_buffer(x, y);
            if (neg_buffer(x, y) > 0.0)
                esdf_buffer[toAddress(x, y)] += (-neg_buffer(x, y) + resolution);
        }
    
    return;
}

std::pair<std::vector<Eigen::Vector2d>, double> GridMap::astarPlan(const Eigen::Vector2d& start, 
                                                                    const Eigen::Vector2d& end,
                                                                    double safe_threshold = 0.0)
{
    std::vector<Eigen::Vector2d> path;

    for (int i=0; i<buffer_size; i++)
    {
        grid_node_map[i]->reset();
    }

    if (!is_ready)
    {
        PRINT_RED("[Astar] map not ready.");
        return make_pair(path, 0.0);
    }

    if(!isInMap(start) || !isInMap(end))
    {
        PRINT_RED("[Astar] boundary points out of map.");
        return make_pair(path, 0.0);
    }

    std::multimap<double, GridNodePtr> openSet;
    Eigen::Vector2i start_index;
    posToIndex(start, start_index);
    GridNodePtr start_point = grid_node_map[toAddress(start_index)];
    start_point->index = start_index;
    GridNodePtr currentPtr = nullptr;

    openSet.insert(make_pair(0.0, start_point));

    Eigen::Vector2i end_index;
    posToIndex(end, end_index);

    while ( !openSet.empty() )
    {
        auto iter  = std::begin(openSet);
        currentPtr = iter -> second;
        openSet.erase(iter);

        grid_node_map[toAddress(currentPtr->index)]->id = IN_CLOSE;

        if( currentPtr->index == end_index )
        {
            GridNode* p = currentPtr;
            double cost = p->gScore;
            Eigen::Vector2d p_world;
            while (p->parent != nullptr)
            {
                indexToPos(p->index, p_world);
                path.push_back(p_world);
                p = p->parent;
            }
            indexToPos(p->index, p_world);
            path.push_back(p_world);

            reverse(path.begin(), path.end());

            return make_pair(path, cost);
        }

        Eigen::Vector2i neighbor_index;
        for(int i = -1; i <= 1; i++)
        {
            for(int j = -1; j <= 1; j++)
            {
                if(i == 0 && j == 0) { continue; }
                neighbor_index = currentPtr->index + Eigen::Vector2i(i ,j);

                if(isInMap(neighbor_index))
                {  
                    GridNodePtr neighborPtr = grid_node_map[toAddress(neighbor_index)];

                    bool occ = false;

                    if (safe_threshold > 1e-6)
                    {
                        double dist;
                        Eigen::Vector2d neighbor_world;
                        indexToPos(neighbor_index, neighbor_world);
                        getDistance(neighbor_world, dist);
                        if (dist < safe_threshold)
                            occ = true;
                    }
                    else
                        occ = isOccupancy(neighbor_index);
                    if (neighborPtr->id == IS_UNKNOWN && !occ)
                    {
                        double tg = ((i * j == 0) ? resolution : (resolution * 1.41)) + currentPtr -> gScore;
                        double heu = 1.0001 * (end_index - neighbor_index).norm() * resolution;

                        neighborPtr -> parent = currentPtr;
                        neighborPtr -> gScore = tg;
                        neighborPtr -> fScore = tg + heu;
                        neighborPtr -> index = neighbor_index;
                        neighborPtr -> id = IN_OPEN;
                        openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                    }
                    else if (neighborPtr->id == IN_OPEN)
                    {
                        double tg = ((i * j == 0) ? resolution : (resolution * 1.414)) + currentPtr -> gScore;
                        if (tg < neighborPtr->gScore)
                        {
                            double heu = 1.0001 * (end_index - neighbor_index).norm() * resolution;
                            neighborPtr -> parent = currentPtr;
                            neighborPtr -> gScore = tg;
                            neighborPtr -> fScore = tg + heu;
                        }
                    }
                    else
                    {
                        double tg = ((i * j == 0) ? resolution : (resolution * 1.414)) + currentPtr -> gScore;
                        if(tg < neighborPtr -> gScore)
                        {
                            double heu = 1.0001 * (end_index - neighbor_index).norm() * resolution;
                            neighborPtr -> parent = currentPtr;
                            neighborPtr -> gScore = tg;
                            neighborPtr -> fScore = tg + heu;
                            
                            neighborPtr -> id = IN_OPEN;
                            openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                        }
                    }
                }
            }
        }   
    }

    PRINT_RED("[Astar] Fails!!!");
    path.clear();

    return make_pair(path, 0.0);
}

std::vector<RowMatrixXd> GridMap::getRectCorridor(std::vector<Eigen::VectorXd> state_list)
{
    std::vector<RowMatrixXd> corridor_list;
    double step = resolution * 0.8;

    for (size_t i=0; i<state_list.size(); i++)
    {
        Eigen::Vector3d state = state_list[i].head(3);
        RowMatrixXd hPoly;
        hPoly.resize(4, 4);
        Eigen::Matrix<bool, 4, 1> NotFinishTable = Eigen::Matrix<bool, 4, 1>(true, true, true, true);      
        Eigen::Vector2d sourcePt = state.head(2);
        double yaw = state[2];
        // change to simple rectangle
        yaw = 0.0;
        Eigen::Matrix2d egoR;
        egoR << cos(yaw), -sin(yaw),
                sin(yaw), cos(yaw);
        Eigen::Vector4d expandLength(Eigen::Vector4d::Zero());
        double d_cr = 0.0;
        double w = 0.4;
        double l = 0.8;
        while (NotFinishTable.any())
        {
            for(int k = 0; k<4; k++)
            {
                if(!NotFinishTable[k]) continue;

                Eigen::Vector2d point1, point2, newpoint1, newpoint2;
                Eigen::Vector2d step_vec;
                Eigen::Vector2d wl_vec1;
                Eigen::Vector2d wl_vec2;
                
                switch (k)
                {
                    //+dy
                    case 0:
                        step_vec = Eigen::Vector2d(0.0, step);
                        wl_vec1 = Eigen::Vector2d(l/2.0+d_cr, w/2.0);
                        wl_vec2 = Eigen::Vector2d(-l/2.0+d_cr, w/2.0);
                        break;
                    //+dx
                    case 1:
                        step_vec = Eigen::Vector2d(step, 0.0);
                        wl_vec1 = Eigen::Vector2d(l/2.0+d_cr, -w/2.0);
                        wl_vec2 = Eigen::Vector2d(l/2.0+d_cr, w/2.0);
                        break;
                    //-dy
                    case 2:
                        step_vec = Eigen::Vector2d(0.0, -step);
                        wl_vec1 = Eigen::Vector2d(-l/2.0+d_cr, -w/2.0);
                        wl_vec2 = Eigen::Vector2d(l/2.0+d_cr, -w/2.0);
                        break;
                    //-dx
                    case 3:
                        step_vec = Eigen::Vector2d(-step, 0.0);
                        wl_vec1 = Eigen::Vector2d(-l/2.0+d_cr, w/2.0);
                        wl_vec2 = Eigen::Vector2d(-l/2.0+d_cr, -w/2.0);
                        break;
                }
                point1 = sourcePt + egoR * wl_vec1;
                point2 = sourcePt + egoR * wl_vec2;
                newpoint1 = point1 + egoR * step_vec;
                newpoint2 = point2 + egoR * step_vec;
                if (isLineOccupancy(point1, newpoint1) || 
                    isLineOccupancy(newpoint1, newpoint2) || 
                    isLineOccupancy(newpoint2, point2) )
                {
                    NotFinishTable[k] = false;
                    continue;
                }
                expandLength[k] += step;
                if(expandLength[k] >= 1.0)
                {
                    NotFinishTable[k] = false;
                    continue;
                }
                sourcePt = sourcePt + egoR * (step_vec / 2.0);
                if (k==0 || k==2)   w += step;
                else    l += step;
            }
        }

        Eigen::Vector2d point1, norm1;
        point1 = sourcePt + egoR*Eigen::Vector2d(l/2.0+d_cr,w/2.0);
        norm1 << -sin(yaw), cos(yaw);
        hPoly.col(0).head<2>() = norm1;
        hPoly.col(0).tail<2>() = point1;
        Eigen::Vector2d point2, norm2;
        point2 = sourcePt+egoR*Eigen::Vector2d(l/2.0+d_cr,-w/2.0);
        norm2 << cos(yaw), sin(yaw);
        hPoly.col(1).head<2>() = norm2;
        hPoly.col(1).tail<2>() = point2;
        Eigen::Vector2d point3, norm3;
        point3 = sourcePt+egoR*Eigen::Vector2d(-l/2.0+d_cr,-w/2.0);
        norm3 << sin(yaw), -cos(yaw);
        hPoly.col(2).head<2>() = norm3;
        hPoly.col(2).tail<2>() = point3;
        Eigen::Vector2d point4, norm4;
        point4 = sourcePt+egoR*Eigen::Vector2d(-l/2.0+d_cr,w/2.0);
        norm4 << -cos(yaw), -sin(yaw);
        hPoly.col(3).head<2>() = norm4;
        hPoly.col(3).tail<2>() = point4;
        corridor_list.push_back(hPoly);
    }

    return corridor_list;
}
