//
// rrt_star.cc
//
// Author: David Robinson
// Date: 2019-11-22
//

#include <rrt_star.hh>


RRTStar::RRTStar(
    const RRTStarOptions &opts,
    const Map &map,
    const ArmConfiguration &start_config,
    const ArmConfiguration &goal_config,
    const double arm_link_length) :
    rrtstar_opts_(opts),
    Planner(opts.general, map, start_config, goal_config, arm_link_length)
{
}

Plan RRTStar::plan()
{
    const auto start_time = std::chrono::steady_clock::now();

    /************* Generate RRT Star *************/

    // Initialize Graph
    Tree T;
    T[start_config_.id] = start_config_;

    double goal_dist = config_dist(start_config_, goal_config_);
    while (true)
    {
        ArmConfiguration x_new;
        if (generate_RRT_tree(T, goal_config_, x_new))
        {
            // Get neighbors of T around x_new
            const auto X_near = get_neighbors(T, x_new, rrtstar_opts_.rewire_radius);

            // Compute cost of x_new to origin using currently wired path
            std::vector<ArmConfiguration> path;
            generate_path(path, T, x_new);

            double current_cost = 0.0;
            for(size_t i = 1; i < path.size(); i++)
                current_cost += config_dist(path[i - 1],path[i]);

            // As far as I interpret the RRT* algorithm,
            // we can just do the extention in 1x loop
            for (auto x_near : X_near)
            {
                // Perform rewiring
                if (no_collisions(T[x_near], T[x_new.id]))
                {
                    // Compute cost of new path
                    generate_path(path, T, T[x_near]);

                    double new_cost = config_dist(x_new, T[x_near]);
                    for(size_t i = 1; i < path.size(); i++)
                        new_cost += config_dist(path[i-1],path[i]);

                    if (new_cost < current_cost)
                    {
                        T[x_new.id].parent_id = x_near;
                        current_cost = new_cost;
                    }
                }
            }

            // Check the distance to the goal as a status update
            goal_dist = config_dist(get_nearest_neighbor(T, goal_config_), goal_config_);
            if (goal_dist < opts_.angle_step_size_rad)
                break;
        }

        if ((std::chrono::steady_clock::now() - start_time).count() / 1e9 > opts_.timeout_s)
            return Plan(start_time);
    }

    /************* Return Path *************/

    // Add goal to path
    ArmConfiguration nearest_to_goal = get_nearest_neighbor(T, goal_config_);

    goal_config_.id = T.size();
    goal_config_.parent_id = nearest_to_goal.id;
    T[goal_config_.id] = goal_config_;

    // Generate plan
    std::vector<ArmConfiguration> plan;
    generate_path(plan, T, goal_config_);
    std::reverse(plan.begin(), plan.end());

    return Plan(plan, start_time);
}
