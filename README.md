# Roundtrip Path Planning

Project Assignment: Roundtrip Path Planning

## Project overview

The task was to create a roundtrip path planner which would compute path with one start position and multiple goals. The solution makes use of serveral probability roadmap algorithms (RRMs)

1. Basic PRM
2. Lazy PRM
3. Visibility PRM

and a Customized version of the visibility PRM:

4. Visibility PRM with
    - Early stopping
    - Crossconnected nodes

## Setup Instructions

This repository contains all the required PRM algorithms, testing environments and an evaluation environment. To install the project locally, follow these steps:

### 1. Clone the Repository

   ```sh
   git clone https://github.com/moritzscheckenbach/Roundtrip_Path_Planning
   ```

### 2. make sure you have the following python librarys installed:

- copy
- descartes
- heapq
- math
- matplotlib
- networkX
- numpy
- pandas
- random
- scipy
- shapely
- sys
- time
 
### 3. make sure you have a version of [Jupyter Notebook](https://jupyter.org/) installed
 
 
## Usage Instructions
 
To jump right into using the project, open any of the Evaluation_xxxxx.ipynb files. You will find a Jupyter Notebook with the following content:
 
 
 
## Benchmarking
 
You can find a comparison of the PRMs within the Evaluation_XXXX.pynb files, at the bottom.
 
 
#########################################################################
 
Deliverables
1. Code:
  Submit your implementation in a well-organized GitHub repository.
  Include a README file with instructions for
  - setup,
  - usage, and
  - benchmarking.
 
 
3. Documentation:
  Explain the rationale behind your design decisions, including:
  How the algorithms were chosen and implemented.
  Steps taken to optimize or smooth the motion paths.
  Challenges faced during the development and how they were addressed.
  Include an analysis of the results, highlighting metrics such as path efficiency and computational performance.
  Please check the notebook "Profiling_pstats_example" and "IP-X-0- Automated_PlanerTest" for profiling and statistics.