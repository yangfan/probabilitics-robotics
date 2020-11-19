import numpy as np
import matplotlib.pyplot as plt


def get_neighborhood(cell, occ_map_shape):
    '''
    Arguments:
    cell -- cell coordinates as [x, y]
    occ_map_shape -- shape of the occupancy map (nx, ny)

    Output:
    neighbors -- list of up to eight neighbor coordinate tuples [(x1, y1), (x2, y2), ...]
    '''

    x, y = cell
    xlim = occ_map_shape[0] - 1
    ylim = occ_map_shape[1] - 1

    neighbors = []

    for delta_x in [-1, 0, 1]:
        for delta_y in [-1, 0, 1]:

            if np.array_equal([x + delta_x, y + delta_y], cell):
                continue
            if x + delta_x < 0 or x + delta_x > xlim:
                continue
            if y + delta_y < 0 or y + delta_y > ylim:
                continue

            neighbors.append((x + delta_x, y + delta_y))

    return neighbors


def get_edge_cost(parent, child, occ_map):
    '''
    Calculate cost for moving from parent to child.

    Arguments:
    parent, child -- cell coordinates as [x, y]
    occ_map -- occupancy probability map

    Output:
    edge_cost -- calculated cost
    '''

    edge_cost = 0

    prob_occ = occ_map[child[0], child[1]]

    if prob_occ >= 0.5:
        return np.inf

    dist = np.linalg.norm(parent - child)

    edge_cost = dist + 10 * prob_occ

    return edge_cost


def get_heuristic(cell, goal):
    '''
    Estimate cost for moving from cell to goal based on heuristic.

    Arguments:
    cell, goal -- cell coordinates as [x, y]

    Output:
    cost -- estimated cost
    '''
    # heuristic is equal to zero for Dijkstra
    heuristic = 0

    # heuristic for A*
    # By multiplying a constant factor > 1, the solution may be found more quickly
    # but solution will not be guarantee to be optimal
    # h2 = [1, 2, 5, 10]

    # heuristic = h2[0] * np.linalg.norm(cell - goal)

    return heuristic


def plot_map(occ_map, start, goal):
    plt.imshow(occ_map.T, cmap=plt.get_cmap('gray'),
               interpolation='none', origin='upper')
    plt.plot([start[0]], [start[1]], 'ro')
    plt.plot([goal[0]], [goal[1]], 'go')
    plt.axis([0, occ_map.shape[0]-1, 0, occ_map.shape[1]-1])
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Path planning')


def plot_expanded(expanded, start, goal):
    if np.array_equal(expanded, start) or np.array_equal(expanded, goal):
        return
    plt.plot([expanded[0]], [expanded[1]], 'yo')
    plt.pause(1e-6)


def plot_path(path, goal):
    if np.array_equal(path, goal):
        return
    plt.plot([path[0]], [path[1]], 'bo')
    plt.pause(1e-6)


def plot_costs(cost):
    plt.figure()
    plt.imshow(cost.T, cmap=plt.get_cmap('gray'),
               interpolation='none', origin='upper')
    plt.axis([0, cost.shape[0]-1, 0, cost.shape[1]-1])
    plt.xlabel('x')
    plt.ylabel('y')


def run_path_planning(occ_map, start, goal):
    '''
    This implements the
    - Dikstra algorithm (in case heuristic is 0)
    - A* algorithm (in case heuristic is not 0)
    '''

    plot_map(occ_map, start, goal)

    # cost values for each cell, filled incrementally.
    # Initialize with infinity
    costs = np.ones(occ_map.shape) * np.inf

    # cells that have already been visited
    closed_flags = np.zeros(occ_map.shape)

    # store predecessors for each visited cell
    predecessors = -np.ones(occ_map.shape + (2,), dtype=np.int)

    # heuristic for A*
    heuristic = np.zeros(occ_map.shape)
    for x in range(occ_map.shape[0]):
        for y in range(occ_map.shape[1]):
            heuristic[x, y] = get_heuristic([x, y], goal)

    # start search
    parent = start
    costs[start[0], start[1]] = 0

    # loop until goal is found
    while not np.array_equal(parent, goal):

        # costs of candidate cells for expansion (i.e. not in the closed list)
        open_costs = np.where(closed_flags == 1, np.inf, costs) + heuristic

        # find cell with minimum cost in the open list
        x, y = np.unravel_index(open_costs.argmin(), open_costs.shape)

        # break loop if minimal costs are infinite (no open cells anymore)
        if open_costs[x, y] == np.inf:
            break

        # set as parent and put it in closed list
        parent = np.array([x, y])
        closed_flags[x, y] = 1

        # update costs and predecessor for neighbors

        # get neighbors of parent node
        neighbors = get_neighborhood(parent, occ_map.shape)

        # compute tentative cost to each neighbor
        for neighbor in neighbors:
            # compute edge cost from parent to neighbor
            edge_cost = get_edge_cost(parent, neighbor, occ_map)
            tentative_past_cost = edge_cost + costs[x, y]
            if tentative_past_cost < costs[neighbor]:
                costs[neighbor] = tentative_past_cost
                predecessors[neighbor] = parent

        # visualize grid cells that have been expanded
        plot_expanded(parent, start, goal)

    # rewind the path from goal to start (at start predecessor is [-1,-1])
    if np.array_equal(parent, goal):
        path_length = 0
        while predecessors[parent[0], parent[1]][0] >= 0:
            plot_path(parent, goal)
            predecessor = predecessors[parent[0], parent[1]]
            path_length += np.linalg.norm(parent - predecessor)
            parent = predecessor

        print("found goal     : " + str(parent))
        print("cells expanded : " + str(np.count_nonzero(closed_flags)))
        print("path cost      : " + str(costs[goal[0], goal[1]]))
        print("path length    : " + str(path_length))
    else:
        print("no valid path found")

    # plot the costs
    plot_costs(costs)
    plt.show(block=True)


def main():
    # load the occupancy map
    occ_map = np.loadtxt('map.txt')

    # start and goal position [x, y]
    start = np.array([22, 33])
    goal = np.array([40, 15])

    run_path_planning(occ_map, start, goal)


if __name__ == "__main__":
    main()
