import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        print(self.my_map)
        start_time = timer.time()
        result = []
        constraints = []

        longest_path_so_far = 0
        map_size = len(self.my_map)*len(self.my_map[0])

        for i in range(self.num_of_agents):  # Find path for each agent
            path_length_bound =  map_size + longest_path_so_far

            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints, path_length_bound)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            longest_path_so_far = max(longest_path_so_far, len(path))

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            for  time_step, (curr_loc, next_loc) in enumerate(zip(path[:-1], path[1:]), start=1):
                for j in range(self.num_of_agents):
                    if i == j:
                        continue
                    constraints.append({'agent': j, 
                    'loc': next_loc,
                    'time_step': time_step})

                    constraints.append({'agent': j, 
                    'loc': (curr_loc, next_loc),
                    'time_step': time_step})
            
            #Add endless vertex constraints
            for j in range(self.num_of_agents):
                    if i == j:
                        continue
                    constraints.append({'agent': j, 
                    'loc': path[-1],
                    'time_step': -time_step})
            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
