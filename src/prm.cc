//
// prm.cc
//
// Author: David Robinson
// Date: 2019-11-22
//

#include <prm.hh>


bool PRM::dijkstra(tree &T, Config start, Config goal)
{
    std::vector<Config> plan;

    /*
    * Setup Data Structures for Search
    */
    std::vector<int> open_set, closed_set;
    open_set.push_back(start.id);

    /*
    * Compute Cost for start cell
    */
    T[start.id].cost = 0;

    /*
    * Begin main search loop
    */
    int count (0);
    int curr;
    while(!open_set.empty() && count < timeout)
    {
        /*
        * Get Config with lowest f value
        */
        float min_f = 1e99;
        int min_idx = 0;
        for (int i=0; i<open_set.size(); i++)
        {
            float comp_f = T[open_set[i]].cost;
            if (comp_f < min_f)
            {
                min_f = comp_f;
                min_idx = i;
            }
        }
        curr = open_set[min_idx];

        /*
        * Check if we're at the goal and return final path
        */
        if (T[curr] == goal) return true;

        /*
        * Deal with sets
        */
        open_set.erase(open_set.begin() + min_idx);
        closed_set.push_back(curr);

        /*
        * Search through neighbours
        */
        for (auto edge : T[curr].edges)
        {
            /*
            * See if the neighbor is in the closed set
            */
            if(std::find(closed_set.begin(), closed_set.end(), edge) != closed_set.end())
            {
                continue;
            }

            /*
            * Compute new G-Value and cost
            */
            int cost = config_dist(T[curr], T[edge]);
            if (cost >= 0)
            {
                /*
                * Compute cost to come
                */
                float cost_to_come = T[curr].cost + cost;

                /*
                * Add to open set if it's not,
                * Otherwise just skip this neighbor because it's not a better path
                */
                if( std::find(open_set.begin(), open_set.end(), edge) == open_set.end() )
                {
                    open_set.push_back(edge);
                }
                else if(cost_to_come >= T[edge].cost)
                {
                    continue;
                }

                /*
                * This is the best path yet, Let's add it to the Open Set!
                */
                T[edge].parent_id = curr;
                T[edge].cost = cost_to_come;
            }
        }
        count++;
    }
    return false;
}


int PRM::plan(
        double*** plan_out,
        int* planlength,
        double* num_samples,
        double* path_quality)
{
    /************* Generate PRM *************/

    tree PRM;
    Config PRM_start_config, PRM_goal_config;
    bool valid_start_goal_connections = false;
    int PRM_samples(0);
    while (true)
    {
        // Uniform random sampling
        Config alpha = sample_config(0);

        if (IsValidArmConfiguration(alpha.angles, map, x_size, y_size))
        {
            // Get neighbors
            auto Q = get_neighbors(PRM, alpha, PRM_thresh);

            // Add node / edges to PRM
            PRM_samples++;
            alpha.id = PRM.size();
            PRM[alpha.id] = alpha;
            for (auto q : Q)
            {
                if (no_collisions(PRM[q], PRM[alpha.id]))
                {
                    PRM[alpha.id].edges.push_back(q);
                    PRM[q].edges.push_back(alpha.id);
                }
            }

            // Check for valid path to goal
            PRM_start_config = get_nearest_neighbor(PRM, start_config);
            PRM_goal_config = get_nearest_neighbor(PRM, goal_config);

            if (no_collisions(start_config, PRM_start_config) &&
                no_collisions(goal_config, PRM_goal_config))
            {
                valid_start_goal_connections = true;
            }
        }

        // Nested exit condition
        if (PRM_samples > num_PRM_samples)
        {
            if (valid_start_goal_connections)
            {
                break;
            }
            else if (PRM_samples > timeout)
            {
                std::cout << "Timeout" << std::endl;
                return 0;
            }
        }
    }

    std::cout << "Finished Building PRM" << std::endl;

    /************* Compute Path *************/

    std::vector<Config> plan;
    if (!dijkstra(PRM, PRM_start_config, PRM_goal_config))
    {
        std::cout << "Invalid Graph" << std::endl;
        return 0;
    }

    plan.push_back(goal_config);
    generate_path(plan, PRM, PRM[PRM_goal_config.id]);
    plan.push_back(start_config);
    std::reverse(plan.begin(), plan.end());

    *num_samples = PRM_samples;
    assign_plan(plan_out, planlength, numofDOFs, plan, path_quality);
    return 1;
}