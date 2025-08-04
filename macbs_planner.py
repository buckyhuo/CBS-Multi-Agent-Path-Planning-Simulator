import heapq
import numpy as np
import pygame
from geometry import Point
from global_planners.global_planner import MapType
from multiagent_global_planners.macbs_planner_helper import *
from multiagent_global_planners.multiagent_planner import MultiAgentPlanner
from representation.float_to_grid import agent_to_gridmap, float_to_grid, grid_to_float
from representation.gridmap_a import GridmapWithNeighbors
import pprint

pp = pprint.PrettyPrinter(indent=4)


class MaCBS(MultiAgentPlanner):
    MAX_NODES_EXPANDED = 10000
    MAP = MapType.GRID
    PAUSE_DURATION = 20

    def compute_path(self, environment):
        solution_paths_dict = self.cbs_start(self.server.collect_agents_info(self.agents), environment)
        self.server.refresh_agents_path(solution_paths_dict)
        return solution_paths_dict

    def cbs_start(self, all_agents_dict, environment):
        temp_path_dict = {}
        for agent_idx in range(len(all_agents_dict)):
            path = self.find_path_with_constraints(
                all_agents_dict[agent_idx],
                environment,
                []
            )
            temp_path_dict[agent_idx] = path if path else []

        if len(all_agents_dict) >= 2:
            path_dict = self.cbs_main(temp_path_dict, all_agents_dict, environment)
            return {i: [Point(node[0], node[1]) for node in p] for i,p in path_dict.items()}

        # convert to point
        return {i: [Point(node[0], node[1]) for node in p] for i,p in temp_path_dict.items()}

    def cbs_main(self, temp_path_dict, all_agents_dict, environment):
        open_list = [{'constraints': {}, 'paths': temp_path_dict.copy(), 'cost': self.get_solution_cost(temp_path_dict)}]
        max_iter = 10
        iter = 0
        prev_conflict = None

        while open_list and iter <= max_iter:
            iter+=1
            node = min(open_list, key=lambda x: x['cost'])
            open_list.remove(node)


            conflict = self.find_conflict(node['paths'], all_agents_dict)
            # print conflict

            if not conflict:
                return node['paths']

            # unresolvable conflict handling (pause agent with higher id)
            if conflict == prev_conflict:
                agent1, agent2, location, timestep = conflict
                
                paused_agent = max(agent1, agent2)
                
                delayed_path = self.create_delayed_path(node['paths'][paused_agent], self.PAUSE_DURATION)
                # delayed_path = node['paths'][paused_agent][0:]
                
                final_paths = node['paths'].copy()
                final_paths[paused_agent] = delayed_path
                
                return final_paths
            
            
            prev_conflict = conflict
            agent1, agent2, location, timestep = conflict

            for agent_id in [agent1, agent2]:
                new_node = {
                    'constraints': node['constraints'].copy(),
                    'paths': node['paths'].copy(),
                    'cost': float('inf')
                }


                if agent_id not in new_node['constraints']:
                    new_node['constraints'][agent_id] = []

                new_node['constraints'][agent_id].append((location, timestep))

                new_path = self.find_path_with_constraints(
                    all_agents_dict[agent_id],
                    environment,
                    new_node['constraints'].get(agent_id, [])
                )

                if new_path:
                    new_node['paths'][agent_id] = new_path
                    new_node['cost'] = self.get_solution_cost(new_node['paths'])
                    open_list.append(new_node)


        return temp_path_dict

    def find_conflict(self, paths, all_agents_dict):
        # Check for vertex conflicts (two agents at same location at same time)
        max_path_length = max(len(path) for path in paths.values())

        # Extend shorter paths by repeating their last position
        extended_paths = {}
        for agent_id, path in paths.items():
            if len(path) < max_path_length:
                dest = all_agents_dict[agent_id].destination_location.to_tuple()
                extended_paths[agent_id] = path + [dest] * (max_path_length - len(path))
            else:
                extended_paths[agent_id] = path

        # Check for vertex conflicts
        for t in range(max_path_length):
            for agent_id, path in extended_paths.items():
                for agent_id_other, path_other in extended_paths.items():
                    if agent_id < agent_id_other:
                        location = path[t]
                        location_other = path_other[t]
                        # if (location == location_other and
                        #         location!= all_agents_dict[agent_id_other].destination_location.to_tuple()):                        
                        if location == location_other:
                            return agent_id, agent_id_other, location_other, t

        # Check for edge conflicts (two agents swap positions)
        for t in range(max_path_length - 1):
            for agent1 in extended_paths:
                for agent2 in extended_paths:
                    if agent1 != agent2:
                        if (t < len(extended_paths[agent1]) - 1 and
                                t < len(extended_paths[agent2]) - 1):
                            if (extended_paths[agent1][t] == extended_paths[agent2][t + 1] and
                                    extended_paths[agent1][t + 1] == extended_paths[agent2][t]):
                                return agent1, agent2, (extended_paths[agent1][t], extended_paths[agent1][t + 1]), t

        return None  # No conflicts found

    def find_path_with_constraints(self, agent, environment, constraints):
        resolution = environment.resolution
        gridmap = GridmapWithNeighbors(environment)
        start = agent_to_gridmap(agent.position, gridmap, resolution, centroid=False)
        goal = float_to_grid(agent.destination_location.to_tuple(), resolution, centroid=False)

        # Initialize A* search
        initial_f_score = self.heuristic(start, goal)
        open_set = []
        heapq.heappush(open_set, (initial_f_score, start, 0))  # (f_score, position, timestep)
        came_from = {}
        g_score = {(start, 0): 0}
        f_score = {(start, 0): initial_f_score}

        open_set_hash = {(start, 0)}
        iteration = 0
        max_iterations = 20000
        while open_set and iteration < max_iterations:

            iteration += 1
            _, current, timestep = heapq.heappop(open_set)
            open_set_hash.remove((current, timestep))

            # If we've reached the goal
            if current == goal:
                # Reconstruct path
                path = []
                current_key = (current, timestep)
                while current_key in came_from:
                    path.append(current_key[0])
                    current_key = came_from[current_key]
                path.append(start)
                path.reverse()
                path_temp = [grid_to_float(node, resolution, centroid=False) for node in path]
                return path_temp

            # Get neighbors including waiting in place
            neighbors = gridmap.neighbors(current) + [current]

            for neighbor in neighbors:
                next_timestep = timestep + 1
                neighbor_key = (neighbor, next_timestep)

                # Check if this move violates any constraints
                if self.is_constrained(neighbor, next_timestep, constraints, gridmap, resolution):
                    continue

                # Check edge constraint (moving from current to neighbor)
                if self.is_edge_constrained(current, neighbor, timestep, constraints, gridmap, resolution):
                    continue

                tentative_g_score = g_score[(current, timestep)] + gridmap.cost(current, neighbor)

                if neighbor_key not in g_score or tentative_g_score < g_score[neighbor_key]:
                    came_from[neighbor_key] = (current, timestep)
                    g_score[neighbor_key] = tentative_g_score
                    f_score[neighbor_key] = tentative_g_score + self.heuristic(neighbor, goal)

                    if neighbor_key not in open_set_hash:
                        heapq.heappush(open_set, (f_score[neighbor_key], neighbor, next_timestep))
                        open_set_hash.add(neighbor_key)
        print 'iteration', iteration
        print 'open_set', open_set
        return None  # No path found

    def is_constrained(self, location, timestep, constraints, gridmap, resolution):
        for constraint in constraints:
            # Handle different constraint formats
            if isinstance(constraint[0][0], tuple):
                # This is an edge constraint, skip in vertex check
                continue
            else:
                constraint_loc, constraint_time = constraint
                constraint_loc = agent_to_gridmap(Point(constraint_loc[0], constraint_loc[1]), gridmap, resolution,
                                                  centroid=False)
                if constraint_time == timestep and location == constraint_loc:
                    return True
        return False

    def is_edge_constrained(self, from_loc, to_loc, timestep, constraints, gridmap, resolution):
        for constraint in constraints:
            if isinstance(constraint[0][0], tuple) and len(constraint[0]) == 2:
                edge_constraint, constraint_time = constraint
                edge_constraint = (
                    agent_to_gridmap(Point(edge_constraint[0][0], edge_constraint[0][1]), gridmap, resolution,
                                     centroid=False),
                    agent_to_gridmap(Point(edge_constraint[1][0], edge_constraint[1][1]), gridmap, resolution,
                                     centroid=False))
                # Check if this is the edge we're moving along
                if constraint_time == timestep and edge_constraint == (from_loc, to_loc):
                    return True
        return False

    def get_solution_cost(self, paths):
        return sum(len(path) for path in paths.values())

    @staticmethod
    def heuristic(a, b):
        return np.abs(a[0] - b[0]) + np.abs(a[1] - b[1])
        # return np.sqrt(np.square(a[0] - b[0]) + np.square(a[1] - b[1]))


    def create_delayed_path(self, original_path, delay_timesteps):

        if not original_path or delay_timesteps <= 0:
            return original_path
        
        # Add delay by duplicating the start position
        start_position = original_path[0]
        delayed_path = [start_position] * delay_timesteps + original_path
        
        return delayed_path