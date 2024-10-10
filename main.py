import os
import argparse
import numpy as np
from robot_search import *
import re
import curses
import sys

def generate_random_map(map_size, filename, origin_dir = os.getcwd(), random_start_end_points = False):
    # Generate the path to the new map file
    path = os.path.join(origin_dir, filename)
    
    # Generate random weights for the map
    weights = np.random.randint(1, 10, size=map_size)
    
    # Generate random start and end points
    if random_start_end_points:
        start_point = np.random.randint(0, [map_size[0], map_size[1], 8], size=3)
        end_point = np.random.randint(0, [map_size[0], map_size[1]], size=2)
        # Ensure start and end points are different
        while start_point[0] == end_point[0] and start_point[1] == end_point[1]:
            end_point = np.random.randint(0, [map_size[0], map_size[1]], size=2)
        end_point = np.append(end_point, 8)
    # Use fixed start and end points
    else:
        start_point = [0, 0, np.random.randint(1, 8)]
        end_point = [map_size[0] - 1, map_size[1] - 1, 8]
    
    # Save the map to a .txt file
    with open(path, 'w') as f:
        f.write(f"{map_size[0]} {map_size[1]}\n")
        for row in weights:
            f.write(' '.join(map(str, row)) + '\n')
        f.write(f"{start_point[0]} {start_point[1]} {start_point[2]}\n")
        f.write(f"{end_point[0]} {end_point[1]} {end_point[2]}\n")

def get_next_map_index(folder_name):
    # Search for all files matching the pattern 'map_*.txt'
    existing_files = os.listdir(folder_name)
    map_files = [f for f in existing_files if re.match(r'map_\d+\.txt', f)]
    
    # Extract the numbers from the filenames and find the highest one
    indices = [int(re.search(r'\d+', f).group()) for f in map_files]
    
    if indices:
        return max(indices) + 1  # The next free index
    else:
        return 1  # Start from 1 if no maps exist
    
def parse_size(size_arg):
    sizes = list(map(int, size_arg.split('x')))
    if len(sizes) == 1:
        return [sizes[0], sizes[0]]  # (n, n) for square map
    elif len(sizes) == 2:
        return [sizes[0], sizes[1]]  # (n, m) for rectangular map
    else:
        raise ValueError("Size must be an integer or two integers separated by 'x'.")

def calculate_averages(results):
    if not results:
        return 0, 0, 0, 0
    
    total_depth = total_cost = total_explored = total_frontier = 0
    for _, depth, cost, explored_count, frontier_count in results:
        total_depth += depth
        total_cost += cost
        total_explored += explored_count
        total_frontier += frontier_count
    
    count = len(results)
    avg_depth = total_depth / count
    avg_cost = total_cost / count
    avg_explored = total_explored / count
    avg_frontier = total_frontier / count
    
    return avg_depth, avg_cost, avg_explored, avg_frontier

def output_trace(final_node, explored, frontier, path, to_file = True):
    original_output = sys.stdout
    if to_file:
        sys.stdout = open(path, "w")
        
    solution = final_node.path()
    
    depth, path_cost, operator, state = print_node(solution[0])

    if hasattr(solution[0], 'h'):
        heuristic = solution[0].h
        print(f"Node 0: ({depth}, {path_cost}, {operator}, {heuristic}, {state})")
    else:
        print(f"Node 0: ({depth}, {path_cost}, {operator}, {state})")

    for i, node in enumerate(solution[1:], start=1):
        # Get details of the current node
        depth, path_cost, operator, state = print_node(node)

        print(f"Operator {i}: {operator}")
            
        # Output format based on the presence of heuristic
        if hasattr(node, 'h'):
            heuristic = node.h
            print(f"Node {i}: ({depth}, {path_cost}, {operator}, {heuristic}, {state})")
        else:
            print(f"Node {i}: ({depth}, {path_cost}, {operator}, {state})")

    print(f"Total number of items in explored list: {len(explored)}")
    print(f"Total number of items in frontier: {len(frontier)}")
    
    sys.stdout = original_output
        
        
def execute_algorithm(map_data, algorithm, heuristic = None):
    # Create the robot problem based on the chosen heuristic
    if heuristic == 'h1':
        robot_problem = RobotProblem_Chebyshev(map_data)
    elif heuristic == 'h2':
        robot_problem = RobotProblem_Euclidean(map_data)
    elif heuristic == 'h3':            
        robot_problem = RobotProblem_Hardness(map_data)
    else:
        robot_problem = RobotProblem(map_data)
            
    # Run the chosen algorithm
    result = None
    if algorithm == 'breadth':
        result = breadth_first_graph_search(robot_problem)
    elif algorithm == 'depth':
        result = depth_first_graph_search(robot_problem)
    elif algorithm == 'a*':
        result = astar_search(robot_problem)

    return result

def execute_algorithm_for_single_map(map_path, algorithm, heuristic = None):
    # Run the chosen algorithm
    map_data = Map(map_path)
    print(f"Processing map: {map_path}")
    result = execute_algorithm(map_data, algorithm, heuristic)
    
    if result is None or isinstance(result, tuple) and len(result) == 0:
        print(f"No solution found for {map_path}.")
        return

    # Unpack results
    final_node, explored, frontier, parent_map = result
    
    # Calculate cost and depth directly from the final_node
    if isinstance(final_node, Node):
        cost = final_node.path_cost
        depth = final_node.depth
    else:
        print(f"Unexpected result type for final_node in {map_path}.")
        return
    
    # Print output for the single map
    output_trace(final_node, explored, frontier, path = None, to_file = False)
    print(f"Depth: {depth} | Cost: {cost} | Explored nodes: {len(explored)} | Final frontier: {len(frontier)}")
    
    # Show the search tree visualization
    visualize_tree(parent_map, "Search tree")
        
    # Show the visualization of the solution path
    solution = final_node.solution()
    solution.insert(0, map_data.start_position)
    map_data.draw_solution(solution)

def execute_algorithm_for_many_maps(map_size, algorithm, heuristic = None):
    # Directories for maps and execution results
    maps_folder_path = f"./maps_{map_size[0]}x{map_size[1]}/maps_info_{map_size[0]}x{map_size[1]}"
    if heuristic:
        exec_folder_path = f"./maps_{map_size[0]}x{map_size[1]}/execution_{map_size[0]}x{map_size[1]}/a_{heuristic}"
    else:
        exec_folder_path = f"./maps_{map_size[0]}x{map_size[1]}/execution_{map_size[0]}x{map_size[1]}/{algorithm}"
    
    # Ensure execution folder exists
    os.makedirs(exec_folder_path, exist_ok=True)

    # List maps from the maps folder
    try:
        maps = os.listdir(maps_folder_path)
    except FileNotFoundError:
        print(f"Error: The directory {maps_folder_path} does not exist.")
        return []

    results = []
    
    for map_file in maps:
        map_data = Map(os.path.join(maps_folder_path, map_file))
        
        result = execute_algorithm(map_data, algorithm, heuristic)
            
        trace_file = os.path.join(exec_folder_path, f"{map_file.split('.')[0]}_{algorithm}{heuristic or ''}_trace.txt")
        tree_file = os.path.join(exec_folder_path, f"{map_file.split('.')[0]}_{algorithm}{heuristic or ''}_tree.html")
        solution_file = os.path.join(exec_folder_path, f"{map_file.split('.')[0]}_{algorithm}{heuristic or ''}_solution.png")

        # Check if a solution was found
        if result is None or isinstance(result, tuple) and len(result) == 0:
            print(f"No solution found for {map_file}.\n")
            continue
        
        # Unpack results
        final_node, explored, frontier, parent_map = result
        
        # Calculate cost and depth directly from the final_node
        if isinstance(final_node, Node):
            cost = final_node.path_cost
            depth = final_node.depth
        else:
            print(f"Unexpected result type for final_node in {map_file}.")
            continue
        
        results.append((map_file, depth, cost, len(explored), len(frontier)))
        
        # Save detailed trace to a .txt file for each map
        output_trace(final_node, explored, frontier, trace_file)
        print(f"Trace saved to {trace_file}")

        # Save the tree visualization for each map
        visualize_tree(parent_map, f"{map_file.split('.')[0]}_{algorithm}{heuristic or ''}", path = tree_file)
        print(f"Tree visualization saved to {tree_file}")
        
        # Save the visualization of the solution path
        solution = final_node.solution()
        solution.insert(0, map_data.start_position)
        map_data.draw_solution(solution, path = solution_file)
        print(f"Solution visualization saved to {solution_file}\n")
    
    # Output statistics for all maps and write to file
    output_template = "Map: {0} | Depth: {1} | Cost: {2} | Explored nodes: {3} | Final frontier: {4}\n"
    avg_template = "\nAverages for all maps using {0} search: | Depth: {1:.2f} | Cost: {2:.2f} | Explored nodes: {3:.2f} | Final frontier: {4:.2f}"
    with open(f"{exec_folder_path}/metrics.txt", "w") as f:
        for result in results:
            output = output_template.format(*result)
            print(output.strip())
            f.write(output)
    
        # Calculate and print averages
        avg_depth, avg_cost, avg_explored, avg_frontier = calculate_averages(results)
        avg_output = avg_template.format(f"{algorithm}{heuristic or ''}", avg_depth, avg_cost, avg_explored, avg_frontier)
        print(avg_output)
        f.write(avg_output)
        
    print(f"Metrics saved to {exec_folder_path}/metrics.txt")
            
    return results

def print_node(node):
    # Get details of the current node
    depth = node.depth  # Ensure node is a Node object
    path_cost = node.path_cost
    operator = node.action if node.action is not None else 'N/A'
    state = node.state
    
    return depth, path_cost, operator, state

def menu(stdscr, options, title):
    curses.curs_set(0)
    current_row = 0

    while True:
        stdscr.clear()
        stdscr.addstr(0, 0, title)
        for idx, row in enumerate(options):
            if idx == current_row:
                stdscr.addstr(idx + 1, 0, row, curses.A_REVERSE)
            else:
                stdscr.addstr(idx + 1, 0, row)
        key = stdscr.getch()

        if key == curses.KEY_UP and current_row > 0:
            current_row -= 1
        elif key == curses.KEY_DOWN and current_row < len(options) - 1:
            current_row += 1
        elif key == curses.KEY_ENTER or key in [10, 13]:
            return options[current_row]

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Random map generator and search algorithm executor.')
    parser.add_argument('-g', action='store_true', help='Generate random maps')
    parser.add_argument('-s', action='store_true', help='Execute algorithms on generated maps')
    parser.add_argument('-t', action='store_true', help='Run algorithm on a single map')
    parser.add_argument('--size', type=str, help='Size of the map (n or n x m)')
    parser.add_argument('--num_maps', type=int, default=1, help='Number of maps to generate (optional for -g only)')
    parser.add_argument('--map_path', type=str, help='Path to the map file (optional for -t only)')

    args = parser.parse_args()

    # Create the necessary directories
    if args.g:
        try:
            map_size = parse_size(args.size)
            num_maps = args.num_maps
            
            # Directories for maps
            origin_dir = os.getcwd()
            folder_name = f"{origin_dir}/maps_{map_size[0]}x{map_size[1]}/maps_info_{map_size[0]}x{map_size[1]}"
            os.makedirs(folder_name, exist_ok=True)
            print(f"Generating maps in {folder_name}")

            # Get the next map index to start from
            next_map_index = get_next_map_index(folder_name)
            last_map_index = next_map_index + num_maps
            
            # Generate the specified number of maps
            for i in range(next_map_index, last_map_index):
                file_name = f"map_{i}.txt"
                generate_random_map(map_size, file_name, folder_name)
                print(f"Generated map: {file_name}")

        except ValueError as e:
            print(f"Error: {e}")

    elif args.s:
        try:
            map_size = parse_size(args.size)
            algorithm_options = ['breadth', 'depth', 'a*']
            heuristic_options = ['h1 (Chebyshev)', 'h2 (Euclidean)', 'h3 (Hardness)']

            algorithm = curses.wrapper(menu, algorithm_options, "Choose the algorithm:")
            
            if algorithm == 'a*':
                heuristic = curses.wrapper(menu, heuristic_options, "Choose the heuristic function:")
                heuristic = heuristic.split()[0]  # Extract 'h1' or 'h2'
                results = execute_algorithm_for_many_maps(map_size, algorithm, heuristic)
            else:
                results = execute_algorithm_for_many_maps(map_size, algorithm)
        
        except ValueError as e:
            print(f"Error: {e}")
    
    elif args.t:
        try:
            map_path = args.map_path
            
            if map_path is None:
                raise ValueError("Path to the map file is required for -t.")
            
            algorithm_options = ['breadth', 'depth', 'a*']
            heuristic_options = ['h1 (Chebyshev)', 'h2 (Euclidean)', 'h3 (Hardness)']

            algorithm = curses.wrapper(menu, algorithm_options, "Choose the algorithm:")
            
            if algorithm == 'a*':
                heuristic = curses.wrapper(menu, heuristic_options, "Choose the heuristic function:")
                heuristic = heuristic.split()[0]
                results = execute_algorithm_for_single_map(map_path, algorithm, heuristic)
            else:
                results = execute_algorithm_for_single_map(map_path, algorithm)
        
        except ValueError as e:
            print(f"Error: {e}")