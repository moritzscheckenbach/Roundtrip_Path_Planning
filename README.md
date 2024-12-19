# Roundtrip_Path_Planning


Project Assignment: Roundtrip Path Planning

Your task is to implement and evaluate a roundtrip path planner for a robotic navigation system. The planner will compute a collision-free path from a given starting position through multiple target positions, visiting each target exactly once. You will design the planner to support different path-planning algorithms and ensure a modular interface. Additionally, you will develop and evaluate a customized version of the Visibility PRM that leverages its unique advantages.

Task 
  1: Roundtrip Path Planner:
    Input:
      A defined start position.
      Multiple target positions to visit exactly once.
    Output:
      A collision-free path encoded to distinguish the start point, collision-free intermediate paths, and target points.
  Features:
  The path-planning algorithm should be selectable and passed as an argument to the planner.
Ensure modularity and compatibility with existing path-planning interfaces.

2. Evaluation:
  Use Basic PRM, Lazy PRM, and Visibility PRM algorithms.
  Test the planner on at least five benchmark environments.
  Present results graphically, including solution paths, and discuss performance metrics.
  Develop a customized version of the Visibility PRM to enhance its effectiveness, evaluate it on the same benchmarks, and compare the results.

Design Considerations
1. Ensure the planner is modular and integrates seamlessly with other planners.
2. Evaluate paths based on length, smoothness, computational time, and obstacle avoidance.
3. Consider optimizing or smoothing the computed paths as part of the solution, it should be discussed in the presentation.

Deliverables
1. Code:
  Submit your implementation in a well-organized GitHub repository.
  Include a README file with instructions for setup, usage, and benchmarking.
2. Presentation:
  Present your results using one of the following formats: A PowerPoint presentation.
  A Jupyter Notebook.
3. Documentation:
Explain the rationale behind your design decisions, including:
  How the algorithms were chosen and implemented.
  Steps taken to optimize or smooth the motion paths.
  Challenges faced during the development and how they were addressed.
Include an analysis of the results, highlighting metrics such as path efficiency and computational performance.
Please check the notebook "Profiling_pstats_example" and "IP-X-0- Automated_PlanerTest" for profiling and statistics.
