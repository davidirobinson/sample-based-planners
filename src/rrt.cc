//
// rrt.cc
//
// Author: David Robinson
// Date: 2019-11-22
//

#include <rrt.hh>


RRT::RRT(
    const RRTOptions &opts,
    const Map &map,
    const ArmConfiguration &start_config,
    const ArmConfiguration &goal_config,
    const double arm_link_length) :
    rrt_opts_(opts),
    Planner(opts.general, map, start_config, goal_config, arm_link_length)
{
}

Plan RRT::plan()
{
    const auto start_time = std::chrono::steady_clock::now();

    /************* Generate RRT *************/

    // Setup graph
    Tree T_start;
    T_start[start_config_.id] = start_config_;

    double goal_dist = config_dist(start_config_, goal_config_);
    while (true)
    {
        ArmConfiguration extended_config;
        if (generate_RRT_tree(T_start, goal_config_, extended_config))
        {
            // Check the distance to the goal as a status update
            ArmConfiguration nearest_to_goal = get_nearest_neighbor(T_start, goal_config_);
            goal_dist = config_dist(nearest_to_goal, goal_config_);

            if (goal_dist < opts_.angle_step_size_rad)
            {
                // Add goal to tree
                goal_config_.id = T_start.size();
                goal_config_.parent_id = nearest_to_goal.id;
                T_start[goal_config_.id] = goal_config_;

                // Exit loop
                break;
            }
        }

        if ((std::chrono::steady_clock::now() - start_time).count() / 1e9 > opts_.timeout_s)
            return Plan(start_time);
    }

    /************* Return Path *************/

    std::vector<ArmConfiguration> plan;
    generate_path(plan, T_start, goal_config_);
    std::reverse(plan.begin(), plan.end());

    return Plan(plan, start_time);
}
