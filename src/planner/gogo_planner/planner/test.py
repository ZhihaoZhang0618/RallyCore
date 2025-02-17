from planner import GridMap
from planner import indexToPos
from planner import PenalTrajOpt
from matplotlib import pyplot as plt
import matplotlib as mpl
import numpy as np

if __name__ == "__main__":
    grid_map_parms = {
        "map_size_x": 12.0,
        "map_size_y": 10.0,
        "resolution": 0.1
    }
    
    # set obstacles
    map = GridMap()
    map.init(grid_map_parms)
    sdf_map = map.getMap()
    b = np.zeros(sdf_map.shape)
    for i in range(sdf_map.shape[0]):
        for j in range(sdf_map.shape[1]):
            if i > 5 and i < 8 and j > 5 and j < 8:
                b[i][j] = 1
            if i > 70 and i < 120 and j > 45 and j < 55:
                b[i][j] = 1
            if i == 0 or i == sdf_map.shape[0]-1 or j == 0 or j == sdf_map.shape[1]-1:
                b[i][j] = 1
    rand_obstacle_num = 10
    np.random.seed(101)
    for i in range(rand_obstacle_num):
        x = np.random.randint(0, sdf_map.shape[0])
        y = np.random.randint(0, sdf_map.shape[1])
        b[x][y] = 1
                
    map.setMap(b)
    sdf_map = map.getMap()

    # plot esdf map and grid map
    x = []
    y = []
    z = []
    zi = []
    for i in range(sdf_map.shape[0]):
        for j in range(sdf_map.shape[1]):
            idx = np.array([[i], [j]], dtype=np.int32)
            pos = np.zeros((2, 1), dtype=np.float64)
            indexToPos(map, idx, pos)
            x.append(pos[0])
            y.append(pos[1])
            z.append(sdf_map[i][j])
            zi.append(b[i][j])

    x = np.array(x).squeeze()
    y = np.array(y).squeeze()
    z = np.array(z).squeeze()
    zi = np.array(zi).squeeze()

    fig, viz_ax = plt.subplots(1, 2)

    levels = mpl.ticker.MaxNLocator(50).tick_values(z.min(), z.max())
    cp = viz_ax[0].tricontourf(x, y, z, levels)
    viz_ax[0].set_title("esdf map")
    viz_ax[0].set_xlabel("X")
    viz_ax[0].set_ylabel("Y")
    viz_ax[0].set_aspect('equal', 'box')
    fig.colorbar(cp, ax=viz_ax[0])

    cp = viz_ax[1].tricontourf(x, y, zi)
    viz_ax[1].set_title("grid map")
    viz_ax[1].set_xlabel("X")
    viz_ax[1].set_ylabel("Y")
    viz_ax[1].set_aspect('equal', 'box')
    fig.colorbar(cp, ax=viz_ax[1])

    # plot astar path
    astar_res = map.astarPlan([4, -2], [4, 2], 0.5)
    astar_path = astar_res[0]
    astar_path = np.array(astar_path)
    viz_ax[1].plot(astar_path[:, 0], astar_path[:, 1], 'ro')

    # test planner
    traj_opt = PenalTrajOpt()
    traj_opt.init(grid_map_parms)
    traj_opt.setMap(b)
    if traj_opt.plan([4, -2, 1.57], [4, 2, 0.0], 0.5):
        traj = traj_opt.getTraj()
        traj_time = traj.getTotalDuration()
        trajps = []
        for i in range(100):
            t = i / 100.0 * traj_time
            pos = traj.getPos(t)
            trajps.append(pos)
        trajps = np.array(trajps)
        viz_ax[1].plot(trajps[:, 0], trajps[:, 1], 'g-')
    plt.show()

