//
// rrt_connect.cc
//
// Author: David Robinson
// Date: 2019-11-22
//

#include <rrt_connect.hh>


RRTConnect::RRTConnect(
    const PlannerOptions &opts,
    const Map &map,
    const ArmConfiguration &start_config,
    const ArmConfiguration &goal_config,
    const double arm_link_length) :
    Planner(opts, map, start_config, goal_config, arm_link_length)
{
}

bool RRTConnect::connect(Tree &T, const ArmConfiguration &new_config)
{
    ArmConfiguration nearest_config = get_nearest_neighbor(T, new_config);
    ArmConfiguration to_extend = nearest_config;

    std::vector<ArmConfiguration> extentions;

    const auto start_time = std::chrono::steady_clock::now();
    while ((std::chrono::steady_clock::now() - start_time).count() / 1e9 < opts_.timeout_s)
    {
        ArmConfiguration extended_config = extend(to_extend, new_config);

        if (no_collisions(extended_config, nearest_config))
        {
            // If we've connected
            if (extended_config == new_config)
            {
                // Add to real tree
                for (const auto &e : extentions)
                    T[e.id] = e;
                return true;
            }

            // Attach parent and add to tree
            extended_config.id = T.size() + extentions.size();
            extended_config.parent_id = to_extend.id;
            extentions.emplace_back(extended_config);

            // Update current position
            to_extend = extended_config;
        }
        else
        {
            // Add to real tree because apparently we should
            for (const auto &e : extentions)
                T[e.id] = e;

            return false;
        }
    }

    return false;
}

Plan RRTConnect::plan()
{
    const auto start_time = std::chrono::steady_clock::now();

    /************* Generate RRT-Connect *************/

    // Setup graph
    Tree T_start, T_goal;
    T_start[start_config_.id] = start_config_;
    T_goal[goal_config_.id] = goal_config_;

    ArmConfiguration T_start_end, T_goal_end, extended_config;

    bool T_switch(true);
    while (true)
    {
        if (T_switch)
        {
            if (generate_RRT_tree(T_start, goal_config_, extended_config))
                if (connect(T_goal, extended_config))
                    break;
        }
        else
        {
            if (generate_RRT_tree(T_goal, start_config_, extended_config))
                if (connect(T_start, extended_config))
                    break;
        }
        T_switch = !T_switch;

        if ((std::chrono::steady_clock::now() - start_time).count() / 1e9 > opts_.timeout_s)
            return Plan(start_time);
    }

    /************* Return Path *************/

    std::vector<ArmConfiguration> plan;

    // Go from meeting point toward the start and reverse
    generate_path(plan, T_start, get_nearest_neighbor(T_start, extended_config));
    std::reverse(plan.begin(), plan.end());

    // Append from meeting point to goal
    generate_path(plan, T_goal, get_nearest_neighbor(T_goal,  extended_config));

    return Plan(plan, start_time);
}
