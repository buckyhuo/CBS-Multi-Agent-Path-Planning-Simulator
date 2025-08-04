# CBS Multi-Agent Path Planning Simulator

**Author:** Jinhong Huo  
**Institution:** Hong Kong University of Science and Technology (HKUST)  
**Project:** Optimizing Warehouse Throughput via Conflict-Based Search for Multi-Agent Pathfinding

## Project Description

This repository contains the implementation of **Conflict-Based Search (CBS)** algorithm for multi-agent path planning. The CBS algorithm is designed to efficiently solve multi-agent pathfinding problems while minimizing conflicts between agents in warehouse environments.

**Note:** This implementation is designed to be integrated as part of the `dorabot_minions-master` codebase, which is not provided in this repository due to license protection requirements.

## Algorithm Performance Comparison

The following table shows the performance comparison between different pathfinding algorithms tested with varying numbers of agents:

| Algorithm   | Agents | PPM | Collisions | Exec. Time (s) |
|-------------|--------|-----|------------|----------------|
| RRT*        | 5      | N/A | N/A        | N/A            |
| Layered A*  | 5      | 378 | 2          | 366            |
| **CBS**     | **5**  | **398** | **1**  | **392**        |
| RRT*        | 8      | N/A | N/A        | N/A            |
| Layered A*  | 8      | 636 | 17         | 506            |
| **CBS**     | **8**  | **640** | **5**  | **552**        |

**Key Findings:**
- CBS demonstrates superior collision reduction compared to Layered A*
- For 5 agents: CBS reduces collisions by 50% (from 2 to 1)
- For 8 agents: CBS reduces collisions by 70.6% (from 17 to 5)
- CBS maintains competitive throughput (PPM) while significantly improving safety

## Project Resources

### Documentation
- [Final Report](https://github.com/buckyhuo/CBS-Multi-Agent-Path-Planning-Simulator/blob/main/paperwork/cbs%20final%20report.pdf)
- [Result Data](https://github.com/buckyhuo/CBS-Multi-Agent-Path-Planning-Simulator/blob/main/paperwork/result_comparison.csv)

### Video Demonstrations

#### CBS Algorithm Results
- [CBS - 5 Agents](https://github.com/buckyhuo/CBS-Multi-Agent-Path-Planning-Simulator/blob/main/video_results/cbs_5.mov)
- [CBS - 8 Agents](https://github.com/buckyhuo/CBS-Multi-Agent-Path-Planning-Simulator/blob/main/video_results/cbs_8.mov)

#### Comparison Algorithms
- [Layered A* - 5 Agents](https://github.com/buckyhuo/CBS-Multi-Agent-Path-Planning-Simulator/blob/main/video_results/la_5.mov)
- [Layered A* - 8 Agents](https://github.com/buckyhuo/CBS-Multi-Agent-Path-Planning-Simulator/blob/main/video_results/la_8.mov)
- [RRT* - 5 Agents](https://github.com/buckyhuo/CBS-Multi-Agent-Path-Planning-Simulator/blob/main/video_results/rtt_5.mov)

## About CBS (Conflict-Based Search)

Conflict-Based Search is a two-level algorithm that finds optimal solutions for multi-agent pathfinding problems. It operates by:

1. **High-level search:** Maintains a constraint tree where each node represents a set of constraints
2. **Low-level search:** Finds optimal paths for individual agents while respecting constraints
3. **Conflict detection:** Identifies and resolves conflicts between agent paths systematically

This approach ensures completeness and optimality while maintaining practical performance for warehouse automation applications.
