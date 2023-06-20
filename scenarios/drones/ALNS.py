import copy

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import numpy.random as rnd
import tsplib95
import tsplib95.distances as distances
from typing import List,Dict
import pandas as pd
import random

import sys
sys.path.append('C:/Users/frede/PycharmProjects/tsp_alns')
from alns import ALNS, State
from alns.accept import HillClimbing
from alns.select import RouletteWheel
from alns.stop import MaxRuntime, NoImprovement




class Compute_TSP_with_ALNS:
    """
    return a class with the computed and true solution of a TSP problem.
    -------
    data_name : refers to the benchmarking data set in the folder dataset
    PARAMETER_SET : {'DEGREE_OF_DESTRUCTION' : 0.1,
                    'DECAY' : 0.8,
                    'SCORES': [3,2,1,0.5],
                    'MAX_ITERATIONS':500,
                    'SEED':100}
    """
    def __init__(
            self,
            PARAMETER_SET : Dict
    ):
        self.DATA = tsplib95.load('C:\\Users\\frede\\eclipse-workspace\\matsim-example-project\\scenarios\\drones\\matsim_points.tsp')
        self.CITIES = list(self.DATA.node_coords.keys())
        self.COORDS = self.DATA.node_coords.values()
        self.DIST = np.empty((len(self.COORDS) + 1, len(self.COORDS) + 1))
        ### State
        self.nodes = self.CITIES
        self.edges = {}
        ### parameter
        self.DEGREE_OF_DESTRUCTION = PARAMETER_SET['Degree_of_Destruction']
        self.SCORES = PARAMETER_SET['Scores']
        self.DECAY = PARAMETER_SET['Decay']
        self.Max_Iterations = PARAMETER_SET['Max_Iterations_without_Improvement']
        self.SEED = PARAMETER_SET['Seed']
        ### part of the solution
        self.tsp_solution = 0
        self.tsp_objective = 0
        self.objectives_history = []

        self.compute = self.compute_solution()

    def compute_solution(self):
        # Precompute the distance matrix - this saves a bunch of time evaluating moves.
        # + 1 since the cities start from one (not zero).
        for row, coord1 in enumerate(self.COORDS, 1):
            for col, coord2 in enumerate(self.COORDS, 1):
                self.DIST[row, col] = distances.euclidean(coord1, coord2)

        random_state = rnd.RandomState(self.SEED)
        state = TspState(self.CITIES, {},self.DIST,self.DATA)

        init_sol = self.greedy_repair(state, random_state)

        alns = ALNS(random_state)

        alns.add_destroy_operator(self.random_removal)
        alns.add_destroy_operator(self.path_removal)
        alns.add_destroy_operator(self.worst_removal)

        alns.add_repair_operator(self.greedy_repair)

        select = RouletteWheel(self.SCORES, self.DECAY, 3, 1)
        accept = HillClimbing()
        stop = NoImprovement(self.Max_Iterations)

        result = alns.iterate(init_sol, select, accept, stop)

        solution = result.best_state
        self.tsp_solution = solution.edges
        self.tsp_objective = solution.objective()
        self.objectives_history = result._statistics._objectives[:]
        return solution.objective()

    def draw_graph(self,graph, only_nodes=False):
        """
        Helper method for drawing TSP (tour) graphs.
        """
        fig, ax = plt.subplots(figsize=(12, 6))

        if only_nodes:
            nx.draw_networkx_nodes(graph, self.DATA.node_coords, node_size=25, ax=ax)
        else:
            nx.draw_networkx(graph, self.DATA.node_coords, node_size=25, with_labels=False, ax=ax)

    def edges_to_remove(self,state):
        return int(len(state.edges) * self.DEGREE_OF_DESTRUCTION)

    def worst_removal(self,current, rnd_state):
        """
        Worst removal iteratively removes the 'worst' edges, that is,
        those edges that have the largest distance.
        """
        destroyed = copy.deepcopy(current)

        worst_edges = sorted(destroyed.nodes,
                             key=lambda node: self.DIST[node, destroyed.edges[node]])

        for idx in range(self.edges_to_remove(current)):
            del destroyed.edges[worst_edges[-(idx + 1)]]

        return destroyed

    def path_removal(self,current, rnd_state):
        """
        Removes an entire consecutive sub-path, that is, a series of
        contiguous edges.
        """
        destroyed = copy.deepcopy(current)

        node_idx = rnd_state.choice(len(destroyed.nodes))
        node = destroyed.nodes[node_idx]

        for _ in range(self.edges_to_remove(current)):
            node = destroyed.edges.pop(node)

        return destroyed

    def random_removal(self,current, rnd_state):
        """
        Random removal iteratively removes random edges.
        """
        destroyed = copy.deepcopy(current)

        for idx in rnd_state.choice(len(destroyed.nodes),
                                    self.edges_to_remove(current),
                                    replace=False):
            del destroyed.edges[destroyed.nodes[idx]]

        return destroyed

    def would_form_subcycle(self,from_node, to_node, state):
        """
        Ensures the proposed solution would not result in a cycle smaller
        than the entire set of nodes. Notice the offsets: we do not count
        the current node under consideration, as it cannot yet be part of
        a cycle.
        """
        for step in range(1, len(state.nodes)):
            if to_node not in state.edges:
                return False

            to_node = state.edges[to_node]

            if from_node == to_node and step != len(state.nodes) - 1:
                return True

        return False

    def greedy_repair(self,current, rnd_state):
        """
        Greedily repairs a tour, stitching up nodes that are not departed
        with those not visited.
        """
        visited = set(current.edges.values())

        # This kind of randomness ensures we do not cycle between the same
        # destroy and repair steps every time.
        shuffled_idcs = rnd_state.permutation(len(current.nodes))
        nodes = [current.nodes[idx] for idx in shuffled_idcs]

        while len(current.edges) != len(current.nodes):
            node = next(node for node in nodes
                        if node not in current.edges)

            # Computes all nodes that have not currently been visited,
            # that is, those that this node might visit. This should
            # not result in a subcycle, as that would violate the TSP
            # constraints.
            unvisited = {other for other in current.nodes
                         if other != node
                         if other not in visited
                         if not self.would_form_subcycle(node, other, current)}

            # Closest visitable node.
            nearest = min(unvisited,
                          key=lambda other: self.DIST[node, other])

            current.edges[node] = nearest
            visited.add(nearest)

        return current


class TspState(State):
    """
    Solution class for the TSP problem. It has two data members, nodes, and edges.
    nodes is a list of IDs. The edges data member, then, is a mapping from each node
    to their only outgoing node.
    """

    def __init__(self, nodes, edges, DIST, DATA):

        self.nodes = nodes
        self.edges = edges
        self.DIST = DIST
        self.DATA = DATA

    def objective(self):
        """
        The objective function is simply the sum of all individual edge lengths,
        using the rounded Euclidean norm.
        """
        return sum(self.DIST[node, self.edges[node]] for node in self.nodes)

    def to_graph(self):
        """
        NetworkX helper method.
        """
        graph = nx.Graph()

        for node in self.nodes:
            graph.add_node(node, pos=self.DATA.node_coords[node])

        for node_from, node_to in self.edges.items():
            graph.add_edge(node_from, node_to)

        return graph

TSP = Compute_TSP_with_ALNS(
    {
        'Degree_of_Destruction' : 0.1,
        'Decay' : 0.8,
        'Scores': [3,2,1,0.5],
        'Max_Iterations_without_Improvement':10000,
        'Seed':100
    })
# print(TSP.tsp_solution)
solution = [1]
for i in range(len(TSP.tsp_solution)):
    solution.append(TSP.tsp_solution[solution[-1]])
print(solution)