//
// rrt_connect.cc
//
// Author: David Robinson
// Date: 2019-11-22
//

#include <rrt_connect.hh>


bool RRTConnect::connect(tree &T, const ArmConfiguration &new_config)
{
    ArmConfiguration nearest_config = get_nearest_neighbor(T, new_config);
    ArmConfiguration to_extend = nearest_config;

    std::vector<ArmConfiguration> extentions;

    int count(0);
    while (count < timeout)
    {
        ArmConfiguration extended_config = extend(to_extend, new_config);

        if (no_collisions(extended_config, nearest_config))
        {
            // If we've connected
            if (extended_config == new_config)
            {
                // Add to real tree
                for (auto e : extentions) T[e.id] = e;
                return true;
            }

            // Attach parent and add to tree
            extended_config.id = T.size() + extentions.size();
            extended_config.parent_id = to_extend.id;
            extentions.push_back(extended_config);

            // Update current position
            to_extend = extended_config;
        }
        else
        {
            // Add to real tree because apparently we should
            for (auto e : extentions) T[e.id] = e;
            return false;
        }
        count++;
    }
    return false;
}


int RRTConnect::plan(
        double*** plan_out,
        int* planlength,
        double* num_samples,
        double* path_quality)
{
    /************* Generate RRT-Connect *************/

    // Setup graph
    tree T_start, T_goal;
    T_start[start_config.id] = start_config;
    T_goal[goal_config.id] = goal_config;

    ArmConfiguration T_start_end, T_goal_end, extended_config;

    bool T_switch(true);
    int count(0);
    while (true)
    {
        if (T_switch)
        {
            if (generate_RRT_tree(T_start, goal_config, extended_config))
            {
                if (connect(T_goal, extended_config)) break;
            }
        }
        else
        {
            if (generate_RRT_tree(T_goal, start_config, extended_config))
            {
                if (connect(T_start, extended_config)) break;
            }
        }
        T_switch = !T_switch;

        count++;
        if (count > timeout)
        {
            std::cout << "Timeout" << std::endl;
            return 0;
        }
    }

    /************* Return Path *************/

    std::vector<ArmConfiguration> plan;

    // Go from meeting point toward the start and reverse
    generate_path(plan, T_start, get_nearest_neighbor(T_start, extended_config));
    std::reverse(plan.begin(), plan.end());

    // Append from meeting point to goal
    generate_path(plan, T_goal, get_nearest_neighbor(T_goal,  extended_config));

    assign_plan(plan_out, planlength, numofDOFs, plan, path_quality);
    *num_samples = count;
    return 1;
}
