__package__ = "Drones.ALNS"


import copy

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import numpy.random as rnd

# import sys
# sys.path.append('Drones/ALNS/alns')
from ALNS.alns import ALNS, State
# from alns import ALNS, State
# from alns.accept import RecordToRecordTravel
# from alns.select import RouletteWheel
# from alns.stop import MaxRuntime

SEED = 1234

filename = "test_drones_MTSP.tsp"

def change_to_float(list):
    return [float(x) for x in list]


# %% open file

with open(filename, 'r') as file:
    lines = file.readlines()

# Extract the necessary information
dimension = None
node_coords = []
drone_coords = []
read_drone_coords = False

line_index = 0

while not lines[line_index].startswith("DIMENSION"):
    line_index+=1
dimension = int(lines[line_index].split(":")[1].strip())
while not lines[line_index].startswith("DRONE_COORD_SECTION"):
    line_index+=1
line_index+=1
while not lines[line_index].startswith("NODE_COORD_SECTION"):
    drone_coords.append(change_to_float(lines[line_index].split()[1:]))
    line_index+=1
line_index+=1
while not lines[line_index].startswith("EOF"):
    node_coords.append(change_to_float(lines[line_index].split()[1:]))
    line_index+=1

#%% Solution State
class CvrpState(State):
    """
    Solution state for CVRP. It has two data members, routes and unassigned.
    Routes is a list of list of integers, where each inner list corresponds to
    a single route denoting the sequence of customers to be visited. A route
    does not contain the start and end depot. Unassigned is a list of integers,
    each integer representing an unassigned customer.
    """

    def __init__(self, routes, unassigned=None):
        self.routes = routes #include only the nodes, not the drones at begining andd at the end
        #0 for the first node, ...
        self.unassigned = unassigned if unassigned is not None else []

    def copy(self):
        return CvrpState(copy.deepcopy(self.routes), self.unassigned.copy())

    def objective_per_route(self):
        """
        Computes the total route costs.
        """
        return [[route_cost(route,drone) for route in self.routes[drone]] for drone in range(len(drone_coords))]

    def objective(self):
        return sum(sum(list) for list in self.objective_per_route())

    @property
    def cost(self):
        """
        Alias for objective method. Used for plotting.
        """
        return self.objective()

    def find_route(self, customer):
        """
        Return the route that contains the passed-in customer.
        """
        for drone_route in self.routes:
            for route in drone_route:
                if customer in route:
                    return route

        raise ValueError(f"Solution does not contain customer {customer}.")


def route_cost(route,drone):
    tour = [drone-len(drone_coords)] + route + [drone-len(drone_coords)]
    return sum(Distance[tour[idx]+len(drone_coords)][tour[idx + 1]+len(drone_coords)] for idx in range(len(tour) - 1))

degree_of_destruction = 0.05
customers_to_remove = int((len(node_coords) - 1) * degree_of_destruction)

##### Operator
def random_removal(state, rnd_state):
    """
    Removes a number of randomly selected customers from the passed-in solution.
    """
    destroyed = state.copy()

    for customer in rnd_state.choice(
        range(1, len(node_coords)), customers_to_remove, replace=False
    ):
        destroyed.unassigned.append(customer)
        route = destroyed.find_route(customer)
        route.remove(customer)

    return remove_empty_routes(destroyed)


def remove_empty_routes(state):
    """
    Remove empty routes after applying the destroy operator.
    """
    state.routes = [[route for route in drone_route if len(route) != 0] for drone_route in state.routes]
    return state

def greedy_repair(state, rnd_state):
    """
    Inserts the unassigned customers in the best route. If there are no
    feasible insertions, then a new route is created.
    """
    rnd_state.shuffle(state.unassigned)

    while len(state.unassigned) != 0:
        customer = state.unassigned.pop()
        drone, route_index, idx = best_insert(customer, state)

        if state.routes[drone][route_index] is not None:
            state.routes[drone][route_index].insert(idx, customer)
        else:
            state.routes[drone].append([customer])

    return state


def best_insert(customer, state):
    """
    Finds the best feasible route and insertion idx for the customer.
    Return (None, None) if no feasible route insertions are found.
    """
    best_cost, best_route, best_idx = None, None, None

    for drone in range(len(state.routes)):
        for route_index in range(len(state.routes[drone])):
            for idx in range(len(state.routes[drone][route_index]) + 1):

                if can_insert(state, customer, idx, drone,route_index):
                    cost = insert_cost(state, customer, drone, route_index, idx)

                    if best_cost is None or cost < best_cost:
                        best_cost, best_drone, best_route_index, best_idx = cost, drone, route_index, idx

    return best_drone, best_route_index, best_idx


def can_insert(state, customer, idx, drone, route_index):
    """
    Checks if inserting customer does not exceed vehicle capacity.
    """
    tour_to_check = state.routes[drone][route_index]
    tour_to_check.insert(idx,customer)
    distance_done = route_cost(tour_to_check,drone)
    return distance_done <= read_drone_coords[drone][-1]


def insert_cost(state, customer, drone, route_index, idx):
    """
    Computes the insertion cost for inserting customer in route at idx.
    """
    pred = drone-len(drone_coords) if idx == 0 else state.routes[drone][route_index][idx - 1]
    succ = drone-len(drone_coords) if idx == len(state.routes[drone][route_index]) else state.routes[drone][route_index][idx]

    # Increase in cost by adding the customer
    cost = Distance[pred+len(drone_coords)][customer+len(drone_coords)] + Distance[customer+len(drone_coords)][succ+len(drone_coords)]

    # Decrease in cost by removing old edge (pred, succ)
    cost -= Distance[pred+len(drone_coords)][succ+len(drone_coords)]

    return cost

def distance(Node):
    Distance = []
    for i in range(len(Node)):
        Dist = []
        for j in range(len(Node)):
            Dist.append(((Node[i][0]-Node[j][0])**2+(Node[i][1]-Node[j][1])**2)**0.5)
        Distance.append(Dist)
    return Distance

# %%##### Initial Solution
def neighbors(customer,drone):
    """
    Return the nearest neighbors of the customer, excluding the depot.
    """
    if drone == True:
        locations = np.argsort([Distance[customer][i] for i in range(len(drone_coords),len(drone_coords) + len(node_coords))])
    else:
        locations = np.argsort([Distance[customer+len(drone_coords)][i] for i in range(len(drone_coords),len(drone_coords) + len(node_coords))])
        locations = locations[1:]
    return locations


def nearest_neighbor():
    """
    Build a solution by iteratively constructing routes, where the nearest
    customer is added until the route has met the vehicle capacity limit.
    """
    number_of_drone = len(drone_coords)
    routes = []
    unvisited = list(range(0, len(node_coords)))
    #cration of the first routes, one per drone
    for i in range(number_of_drone):
        nearest = [nb for nb in neighbors(i,True) if nb in unvisited][0]
        routes.append([[i-number_of_drone,nearest]])
        unvisited.remove(nearest)
    route_demands = [[2 * Distance[routes[i][-1][0]][routes[i][-1][1]+number_of_drone]] for i in range(len(routes))]
    drone = 0
    while unvisited:
        # for each unvisited node, it is added to a drones route
        # after each insertion, drone --> +1 in order to change drones route
        # we check that the drone capacity is not overwhelmed
        route = routes[drone][-1]
        current = route[-1]
        nearest = [nb for nb in neighbors(current, False) if nb in unvisited][0]
        l = 1
        while route_demands[drone][-1] + Distance[route[-1]][nearest] + Distance[nearest][route[0]] - Distance[route[-1]][
            route[0]] > drone_coords[drone][-1] and l < len(unvisited):
            nearest = [nb for nb in neighbors(current, False) if nb in unvisited][l]
            l+=1
        if l < len(unvisited):
            route.append(nearest)
            unvisited.remove(nearest)
            route_demands[drone][-1] += Distance[route[-1]][nearest] + Distance[nearest][route[0]] - Distance[route[-1]][route[0]]
        else:
            nearest = [nb for nb in neighbors(drone, True) if nb in unvisited][0]
            routes[drone].append([drone-number_of_drone, nearest])
            route_demands.append(2 * Distance[routes[drone][-1][0]][routes[drone][-1][1] + number_of_drone])
            unvisited.remove(nearest)
        drone += 1
        drone = drone % len(drone_coords)
    for drone in range(number_of_drone):
        for r in range(len(routes[drone])):
            routes[drone][r] = routes[drone][r][1:]
    return CvrpState(routes)

Distance = distance(drone_coords+node_coords)

init_solution = nearest_neighbor()
# print(init_solution.routes)
# print(init_solution.objective())

# %% ANLS
def compute_MTSP():
    alns = ALNS(rnd.RandomState(SEED))

    alns.add_destroy_operator(random_removal)

    alns.add_repair_operator(greedy_repair)

    init = nearest_neighbor()
    select = RouletteWheel([25, 5, 1, 0], 0.8, 1, 1)
    accept = RecordToRecordTravel.autofit(init.objective(), 0.02, 0, 9000)
    stop = MaxRuntime(2)

    result = alns.iterate(init, select, accept, stop)

    solution = result.best_state
    objective = solution.objective()
    print(solution.routes)
    print(f"Best heuristic objective is {objective}.")
    _, ax = plt.subplots(figsize=(12, 6))
    result.plot_objectives(ax=ax)
    plt.show()
    return solution.routes