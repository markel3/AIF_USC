import os
import argparse
import numpy as np

def generate_random_map(map_size, filename, origin_dir = os.getcwd()):
    path = os.path.join(origin_dir, filename)
    weights = np.random.randint(1, 10, size=map_size)
    start_point = np.random.randint(0, [map_size[0], map_size[1], 9], size=3)
    end_point = np.random.randint(0, [map_size[0], map_size[1]], size=2)
    end_point = np.append(end_point, 8)
    
    with open(path, 'w') as f:
        f.write(f"{map_size[0]} {map_size[1]}\n")
        for row in weights:
            f.write(' '.join(map(str, row)) + '\n')
        f.write(f"{start_point[0]} {start_point[1]} {start_point[2]}\n")
        f.write(f"{end_point[0]} {end_point[1]} {end_point[2]}\n")

def parse_size(size_arg):
    sizes = list(map(int, size_arg.split('x')))
    if len(sizes) == 1:
        return [sizes[0], sizes[0]]  # (n, n) for square map
    elif len(sizes) == 2:
        return [sizes[0], sizes[1]]  # (n, m) for rectangular map
    else:
        raise ValueError("Size must be an integer or two integers separated by 'x'.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Random map generator.')
    parser.add_argument('-g', action='store_true', help='Generate random maps')
    parser.add_argument('size', type=str, help='Size of the map (n or n x m)')
    parser.add_argument('num_maps', type=int, help='Number of maps to generate')

    args = parser.parse_args()

    if args.g:
        try:
            map_size = parse_size(args.size)
            num_maps = args.num_maps
            
            folder_name = f"maps_{map_size[0]}x{map_size[1]}"
            folder_name = os.path.join(os.getcwd(), folder_name)
            os.makedirs(folder_name, exist_ok=True)

            for i in range(num_maps):
                file_name = f"map_{i + 1}.txt"
                generate_random_map(map_size, file_name, folder_name)

        except ValueError as e:
            print(f"Error: {e}")