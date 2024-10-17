<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a id="readme-top"></a>

[![Contributors][contributors-shield]][contributors-url]
[![MIT License][license-shield]][license-url]

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
    </li>
    <li>
      <a href="#installation">Installation</a>
    </li>
    <li>
        <a href="#usage">Usage</a>
        <ul>
          <li>  <a href="#generate-maps">Generate maps</a> </li>
          <li>  <a href="#execute-several-searchs">Execute several searchs</a> </li>
          <li>  <a href="#execute-one-search">Execute one search</a> </li>
        </ul>        
    </li>
  </ol>
</details>


<!-- ABOUT THE PROJECT -->
## About The Project

This project was developed as part of the first assignment of the Artificial Intelligence Fundamentals course at the University of Santiago de Compostela. The goal was to implement and compare different search algorithms for a robot search problem. 

This project based on the code from the [aima](https://github.com/aimacode/aima-python) repository. In particular, the script called `search.py` was used as a base to implement the search algorithms.

In this problem, a robot must find a path from its initial position to a goal position in a grid-like map, in which each cell represents a rock with an associated hardness value, from 1 to 9. The restrictions are as follows:
<ul>
    <li> <b> Robot Orientation: </b> It must consider one of the eight possible orientations (North, Northeast, East, Southeast, South, Southwest, West, Northwest). </li>
    <li> <b> Allowed Actions:: </b> At each position, the robot can:
    <ul>
        <li> Move (drill) in the direction of its current orientation. </li>
        <li>Rotate 45 degrees clockwise or counterclockwise.</li>
    </ul>
    </li>
</ul>
Each 45-degree rotation has an associated cost of one unit, and the cost of moving forward depends on the hardness of the rock at the target position.

To solve the problem, the following search algorithms were implemented:
<ul>
    <li> <b> Breadth-First Search (BFS) </b> </li>
    <li> <b> Depth-First Search (DFS) </b> </li>
    <li> <b> A* with two different heuristics: </b>
    <ul>
        <li> <b> Chebyshev Distance: </b> The maximum of the absolute differences between the x and y coordinates of the current position and the goal position. </li>
        <li> <b> Surrounding hardness: </b> A normalized mean of the hardness values of the adjacent cells to the studied position. </li>
    </ul>
    </li>
</ul>



<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- INSTALLATION -->
## Installation

Once you have cloned the repository, you can install the required packages by running the following command:

```sh
conda env create -f environment.yml
```

This will create a new conda environment named `aif_P1` with all the necessary packages. To activate the environment, run:

```sh
conda activate aif_P1
```

After activating the environment, you can run the program. To deactivate the environment, run:

```sh
conda deactivate
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- USAGE EXAMPLES -->
## Usage

<p>To run the program, execute the <code>main.py</code> script. It offers three execution options:</p>

<h3 id="generate-maps"> 1. Generate Maps </h3>

This function allows you to create random maps for the robot search problem. To use it, type `-g` followed by two arguments:
<ul>
  <li><b>Map Size:</b> Either a single positive integer for a square map or two positive integers separated by 'x' (e.g., <code>5x7</code>). Specified by <code>--size</code>.</li>
  <li><b>Number of Maps:</b> A positive integer indicating how many maps to generate. Specified by <code>--num_maps</code>.</li>
</ul>

#### Examples:
<ul>
  <li><code>python main.py -g --size 10 --num_maps 5</code> — Generates 5 square maps of size 10x10.</li>
  <li><code>python main.py -g --size 5x7 --num_maps 3</code> — Generates 3 rectangular maps of size 5x7.</li>
</ul>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<h3 id="execute-several-searchs"> 2. Execute Multiple Searches </h3>

<p>This function allows you to execute your preferred search algorithm for all maps of a specified size. To use it, type <code>-s</code> followed by one argument:</p>
<ul>
  <li><b>Map Size:</b> This can be a single positive integer for a square map or two positive integers separated by an 'x' (e.g., <code>5x7</code>). Specified by <code>--size</code>.</li>
</ul>
<p>You will then be prompted to select one of the available search algorithms.</p>

<h4>Examples:</h4>
<ul>
  <li><code>python main.py -s --size 5</code> — Executes the search for all square maps of size 5x5.</li>
  <li><code>python main.py -s --size 5x7</code> — Executes the search for all rectangular maps of size 5x7.</li>
</ul>

<p>The program will generate files that include the output trace, search tree visualization, and solution visualization for each map. Additionally, it computes the following metrics for each map:</p>
<ul>
  <li><b>Depth:</b> The depth of the exploration tree where the solution was found.</li>
  <li><b>Cost:</b> The cost of the obtained solution path.</li>
  <li><b>Total Number of Explored Nodes:</b> The total number of nodes (expanded) during the execution of the corresponding algorithm.</li>
  <li><b>Final Number of Nodes in the Frontier:</b> The number of nodes stored in the frontier list at the end of execution.</li>
</ul>

<p>Finally, it computes the averages of all metrics across the processed maps and stores the results of each map separately, along with the averages in a single output file.</p>


<p align="right">(<a href="#readme-top">back to top</a>)</p>

<h3 id="execute-one-search"> 3. Execute one search </h3>

<p>This function allows you to execute your chosen search algorithm on a single specific map. To use it, type <code>-t</code> followed by one argument:</p>
<ul>
  <li><b>Map Path:</b> The path to the map file you want to use. This can be an absolute or relative path (e.g., <code>/path/to/your/mapfile.txt</code>). Specified by <code>--map_path</code>.</li>
</ul>

<h4>Example:</h4>
<ul>
  <li><code>python main.py -t --map_path /path/to/your/mapfile.txt</code> — Executes the search on <code>mapfile.txt</code>.</li>
</ul>

<p>The program will display the results, including the depth, cost, number of explored nodes, and the final frontier. It will also visualize the search tree and the solution path on the map.</p>
<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/markel3/AIF_USC.svg?style=for-the-badge
[contributors-url]: https://github.com/markel3/AIF_USC/graphs/contributors
[license-shield]: https://img.shields.io/github/license/markel3/AIF_USC.svg?style=for-the-badge
[license-url]: https://github.com/markel3/AIF_USC/blob/main/LICENSE