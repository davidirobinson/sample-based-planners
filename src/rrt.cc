//
// rrt.cc
//
// Author: David Robinson
// Date: 2019-11-22
//

#include <rrt.hh>


RRT::RRT(
    const PlannerOptions &opts,
    const Map &map,
    const ArmConfiguration &start_config,
    const ArmConfiguration &goal_config) :
    Planner(opts, map, start_config, goal_config)
{
}

int RRT::plan(
        double*** plan_out,
        int* planlength,
        double* num_samples,
        double* path_quality
)
{
    /************* Generate RRT *************/

    // Setup graph
    tree T_start;
    T_start[start_config_.id] = start_config_;

    double goal_dist = config_dist(start_config_, goal_config_);
    int count(0);
    while (true)
    {
        ArmConfiguration extended_config;
        if (generate_RRT_tree(T_start, goal_config_, extended_config))
        {
            // Check the distance to the goal as a status update
            ArmConfiguration nearest_to_goal = get_nearest_neighbor(T_start, goal_config_);
            goal_dist = config_dist(nearest_to_goal, goal_config_);

            if (goal_dist < angle_step_size)
            {
                // Add goal to tree
                goal_config_.id = T_start.size();
                goal_config_.parent_id = nearest_to_goal.id;
                T_start[goal_config_.id] = goal_config_;

                // Exit loop
                break;
            }
        }

        if (count > timeout)
        {
            std::cout << "Timeout" << std::endl;
            return 0;
        }
        count++;
    }

    /************* Return Path *************/

    std::vector<ArmConfiguration> plan;
    generate_path(plan, T_start, goal_config_);
    std::reverse(plan.begin(), plan.end());

    assign_plan(plan_out, planlength, opts_.arm_dof, plan, path_quality);
    *num_samples = count;
    return 1;
}