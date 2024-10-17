from search import Problem, Node
from utils import memoize, PriorityQueue
from collections import deque
import matplotlib.pyplot as plt
import os
import numpy as np
import networkx as nx
import plotly.graph_objects as go

orientation_vectors = {
    0: (-1, 0),     # North
    1: (-1, 1),     # Northeast
    2: (0, 1),      # East
    3: (1, 1),      # Southeast
    4: (1, 0),      # South
    5: (1, -1),     # Southwest
    6: (0, -1),     # West
    7: (-1, -1)     # Northwest
}

class Map:
    
    def __init__(self, file, origin_dir = os.getcwd()):
        file_path = os.path.join(origin_dir, file)
        with open(file_path, 'r') as file:
            lines = file.readlines()

        dimension = list(map(int, lines[0].split()))
        self.rows, self.cols = dimension
        self.weights = np.array([list(map(int, lines[i + 1].split())) for i in range(self.rows)])
        self.start_position = tuple(map(int, lines[-2].split()))
        self.end_position = tuple(map(int, lines[-1].split()))[:2]

    def draw_initial(self):
        weights_array = np.array(self.weights)
        plt.imshow(weights_array, cmap="Blues", origin="upper")
        for i in range(self.rows):
            for j in range(self.cols):
                plt.text(j, i, str(weights_array[i, j]), color='black', fontsize=12, ha='center', va='center')

        orientation = self.start_position[2]
        if orientation == 7:
            plt.text(self.start_position[1]+0.3, self.start_position[0]+0.3, 'S', color='black', fontsize=12, ha='center', va='center', fontweight='bold')
        else:
            plt.text(self.start_position[1]-0.3, self.start_position[0]-0.3, 'S', color='black', fontsize=12, ha='center', va='center', fontweight='bold')
        plt.text(self.end_position[1]-0.3, self.end_position[0]-0.3, 'E', color='green', fontsize=12, ha='center', va='center', fontweight='bold')
        
        arrow_dy, arrow_dx = [x/2 for x in orientation_vectors[orientation]]
        plt.arrow(self.start_position[1]+arrow_dx/2, self.start_position[0]+arrow_dy/2, arrow_dx, arrow_dy, head_width=0.1, head_length=0.1, fc='black', ec='black')

        plt.axis('off')

        plt.show()
    
    def draw_solution(self, solution, path = None):
        weights_array = np.array(self.weights)
        plt.imshow(weights_array, cmap="Blues", origin="upper")
        for i in range(self.rows):
            for j in range(self.cols):
                plt.text(j, i, str(weights_array[i, j]), color='black', fontsize=12, ha='center', va='center')

        arrow_dy, arrow_dx = [x/2 for x in orientation_vectors[self.start_position[2]]]
        plt.arrow(self.start_position[1]+arrow_dx/2, self.start_position[0]+arrow_dy/2, arrow_dx, arrow_dy, head_width=0.1, head_length=0.1, fc='black', ec='black')
        
        for i in range(1, len(solution)-2):
            position = solution[i][:2]
            orientation = solution[i][2]
            arrow_dy, arrow_dx = [x/2 for x in orientation_vectors[orientation]]
            next_position = solution[i+1][:2]
            
            if next_position != position:
                plt.arrow(position[1]+arrow_dx/2, position[0]+arrow_dy/2, arrow_dx, arrow_dy, head_width=0.1, head_length=0.1, fc='red', ec='red')
            else:
                plt.arrow(position[1]+arrow_dx/2, position[0]+arrow_dy/2, arrow_dx, arrow_dy, head_width=0.1, head_length=0.1, fc='blue', ec='blue')
        
        position = solution[-2][:2]
        orientation = solution[-2][2]
        arrow_dy, arrow_dx = [x/2 for x in orientation_vectors[orientation]]
        plt.arrow(position[1]+arrow_dx/2, position[0]+arrow_dy/2, arrow_dx, arrow_dy, head_width=0.1, head_length=0.1, fc='green', ec='green')
                
        plt.axis('off')

        if path:
            plt.savefig(path)
        else:
            plt.show()
        
        plt.close()
            
Node.solution = lambda self: [node.state for node in self.path()]

def visualize_tree(parent_map, title, path = None):
    G = nx.DiGraph()

    for child, parent in parent_map.items():
        G.add_edge(parent, child)

    pos = nx.drawing.nx_agraph.graphviz_layout(G, prog='dot')

    edge_x = []
    edge_y = []
    for edge in G.edges():
        x0, y0 = pos[edge[0]]
        x1, y1 = pos[edge[1]]
        edge_x.append(x0)
        edge_x.append(x1)
        edge_x.append(None)
        edge_y.append(y0)
        edge_y.append(y1)
        edge_y.append(None)

    edge_trace = go.Scatter(
        x=edge_x, y=edge_y,
        line=dict(width=1, color='#888'),
        hoverinfo='none',
        mode='lines')

    node_x = []
    node_y = []
    for node in G.nodes():
        x, y = pos[node]
        node_x.append(x)
        node_y.append(y)

    node_trace = go.Scatter(
        x=node_x, y=node_y,
        mode='markers+text',
        hoverinfo='text',
        text=[str(node) for node in G.nodes()],
        textposition="bottom center",
        marker=dict(
            showscale=False,
            color='lightblue',
            size=20,
            line_width=2))

    layout = go.Layout(
        title=title,
        titlefont_size=16,
        showlegend=False,
        hovermode='closest',
        margin=dict(b=0, l=0, r=0, t=40),
        annotations=[dict(
            text = "",
            showarrow=False,
            xref="paper", yref="paper",
            x=0.005, y=-0.002)],
        xaxis=dict(showgrid=False, zeroline=False),
        yaxis=dict(showgrid=False, zeroline=False)
    )

    fig = go.Figure(data=[edge_trace, node_trace], layout=layout)

    if path:
        fig.write_html(path)
    else:
        fig.show()


class RobotProblem(Problem):

    def __init__(self, map):
        Problem.__init__(self, map.start_position, map.end_position)
        self.weights = map.weights
        self.dimensions = map.weights.shape

    def actions(self, state):
        actions = [-1,1]
        new_position = [state[0] + orientation_vectors[state[2]][0], state[1] + orientation_vectors[state[2]][1]]
        if 0 <= new_position[0] < self.dimensions[0] and 0 <= new_position[1] < self.dimensions[1]:
            actions.append(0)
        return actions
    
    def result(self, state, action):
        if action != 0:
            return (state[0], state[1], (state[2] + action) % 8)
        return (state[0] + orientation_vectors[state[2]][0], state[1] + orientation_vectors[state[2]][1], state[2])
    
    def goal_test(self, state):
        return state[:2] == self.goal[:2]
    
    def path_cost(self, cost_so_far, state1, action, state2):
        if state1[:2] == state2[:2]:
            return cost_so_far + 1
        else:
            return cost_so_far + self.weights[state2[0], state2[1]]

def breadth_first_graph_search(problem):
    node = Node(problem.initial)
    if problem.goal_test(node.state):
        return node
    frontier = deque([node])
    explored = []
    parent_map = {}
    while frontier:
        node = frontier.popleft()
        explored.append(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                parent_map[child.state] = node.state
                if problem.goal_test(child.state):
                    return child, explored, frontier, parent_map
                frontier.append(child)
    return None

def depth_first_graph_search(problem):
    frontier = [(Node(problem.initial))]
    explored = []
    parent_map = {}
    while frontier:
        node = frontier.pop()
        if problem.goal_test(node.state):
            return node, explored, frontier, parent_map
        explored.append(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                parent_map[child.state] = node.state
                frontier.append(child)
    return None

def best_first_graph_search(problem, f, display=False):
    f = memoize(f, 'f')
    node = Node(problem.initial)
    frontier = PriorityQueue('min', f)
    frontier.append(node)
    explored = []
    parent_map = {}
    while frontier:
        node = frontier.pop()
        if problem.goal_test(node.state):
            if display:
                print(len(explored), "paths have been expanded and", len(frontier), "paths remain in the frontier")
            return node, explored, frontier, parent_map
        explored.append(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                parent_map[child.state] = node.state
                frontier.append(child)
            elif child in frontier:
                if f(child) < frontier[child]:
                    del frontier[child]
                    frontier.append(child)
    return None

def astar_search(problem, h=None, display=False):
    h = memoize(h or problem.h, 'h')
    return best_first_graph_search(problem, lambda n: n.path_cost + h(n), display)

def chebyshev_distance(start, goal):
    x1, y1 = start
    x2, y2 = goal
    
    return max(abs(x2 - x1), abs(y2 - y1))

def euclidean_scaled_distance(start, goal):
    x1, y1 = start
    x2, y2 = goal
    
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)*np.sqrt(2)/2

def surrounding_hardness(state, weights, goal):
    if state[:2] == goal:
        return 0
    x, y = state[:2]
    hardness = 0
    total_neighbors = 0
    for i in range(-1, 2):
        for j in range(-1, 2):
            if (i == 0 and j == 0):
                continue
            if 0 <= x+i < weights.shape[0] and 0 <= y+j < weights.shape[1]:
                hardness += weights[x+i, y+j]
                total_neighbors += 1
    return (hardness/(total_neighbors*9))

class RobotProblem_Euclidean(RobotProblem):
    
    def __init__(self, map):
        RobotProblem.__init__(self, map)
        
    def h(self, node):
        return euclidean_scaled_distance(node.state[:2], self.goal[:2])
    
class RobotProblem_Chebyshev(RobotProblem):

    def __init__(self, map):
        RobotProblem.__init__(self, map)
        
    def h(self, node):
        return chebyshev_distance(node.state[:2], self.goal[:2])
    
class RobotProblem_Hardness(RobotProblem):
    
        def __init__(self, map):
            RobotProblem.__init__(self, map)
            
        def h(self, node):
            return surrounding_hardness(node.state, self.weights, self.goal[:2])
