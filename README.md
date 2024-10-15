<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a id="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->

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

Here's a blank template to get started: To avoid retyping too much info. Do a search and replace with your text editor for the following: `github_username`, `repo_name`, `twitter_handle`, `linkedin_username`, `email_client`, `email`, `project_title`, `project_description`

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- INSTALLATION -->
## Installation

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- USAGE EXAMPLES -->
## Usage

<p>To run the program, execute the <code>main.py</code> script. It offers three execution options:</p>

<h3 id="generate-maps"> 1. Generate Maps </h3>

This function allows you to create random maps for the robot search problem. To use it, type `-g` followed by two arguments:
<ul>
  <li><b>Map size:</b> Either a single positive integer for a square map or two positive integers separated by 'x' (e.g., <code>5x7</code>). Specified by <code>--size</code>.</li>
  <li><b>Number of maps:</b> A positive integer indicating how many maps to generate. Specified by <code>--num_maps</code>.</li>
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
  <li><code>python main.py --size 5</code> — Executes the search for all square maps of size 5x5.</li>
  <li><code>python main.py --size 5x7</code> — Executes the search for all rectangular maps of size 5x7.</li>
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

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/github_username/repo_name.svg?style=for-the-badge
[contributors-url]: https://github.com/markel3/AIF_USC/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/github_username/repo_name.svg?style=for-the-badge
[forks-url]: https://github.com/github_username/repo_name/network/members
[stars-shield]: https://img.shields.io/github/stars/github_username/repo_name.svg?style=for-the-badge
[stars-url]: https://github.com/github_username/repo_name/stargazers
[issues-shield]: https://img.shields.io/github/issues/github_username/repo_name.svg?style=for-the-badge
[issues-url]: https://github.com/github_username/repo_name/issues
[license-shield]: https://img.shields.io/github/license/github_username/repo_name.svg?style=for-the-badge
[license-url]: https://github.com/github_username/repo_name/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/linkedin_username
[product-screenshot]: images/screenshot.png
[Next.js]: https://img.shields.io/badge/next.js-000000?style=for-the-badge&logo=nextdotjs&logoColor=white
[Next-url]: https://nextjs.org/
[React.js]: https://img.shields.io/badge/React-20232A?style=for-the-badge&logo=react&logoColor=61DAFB
[React-url]: https://reactjs.org/
[Vue.js]: https://img.shields.io/badge/Vue.js-35495E?style=for-the-badge&logo=vuedotjs&logoColor=4FC08D
[Vue-url]: https://vuejs.org/
[Angular.io]: https://img.shields.io/badge/Angular-DD0031?style=for-the-badge&logo=angular&logoColor=white
[Angular-url]: https://angular.io/
[Svelte.dev]: https://img.shields.io/badge/Svelte-4A4A55?style=for-the-badge&logo=svelte&logoColor=FF3E00
[Svelte-url]: https://svelte.dev/
[Laravel.com]: https://img.shields.io/badge/Laravel-FF2D20?style=for-the-badge&logo=laravel&logoColor=white
[Laravel-url]: https://laravel.com
[Bootstrap.com]: https://img.shields.io/badge/Bootstrap-563D7C?style=for-the-badge&logo=bootstrap&logoColor=white
[Bootstrap-url]: https://getbootstrap.com
[JQuery.com]: https://img.shields.io/badge/jQuery-0769AD?style=for-the-badge&logo=jquery&logoColor=white
[JQuery-url]: https://jquery.com 