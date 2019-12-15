//
// prm.cc
//
// Author: David Robinson
// Date: 2019-11-22
//

#include <prm.hh>


PRM::PRM(
    const PRMOptions &opts,
    const Map &map,
    const ArmConfiguration &start_config,
    const ArmConfiguration &goal_config,
    const double arm_link_length) :
    prm_opts_(opts),
    Planner(opts.general, map, start_config, goal_config, arm_link_length)
{
}

bool PRM::dijkstra(
    Tree &T,
    const ArmConfiguration &start_config,
    const ArmConfiguration &end_config)
{
    std::vector<ArmConfiguration> plan;

    // Setup Data Structures for Search
    std::vector<size_t> open_set, closed_set;
    open_set.emplace_back(start_config.id);

    // Compute Cost for start cell
    T[start_config.id].cost = 0.0;

    // Begin main search loop
    size_t curr;
    const auto start_time = std::chrono::steady_clock::now();
    while(!open_set.empty() && (std::chrono::steady_clock::now() - start_time).count() / 1e9 < opts_.timeout_s)
    {
        // Get ArmConfiguration with lowest f value
        float min_f = 1e99;
        int min_idx = 0;
        for (size_t i = 0; i < open_set.size(); i++)
        {
            const auto comp_f = T[open_set[i]].cost;
            if (comp_f < min_f)
            {
                min_f = comp_f;
                min_idx = i;
            }
        }
        curr = open_set[min_idx];

        // Check if we're at the goal and return final path
        if (T[curr] == end_config)
            return true;

        // Deal with sets
        open_set.erase(open_set.begin() + min_idx);
        closed_set.emplace_back(curr);

        // Search through neighbours
        for (const auto &edge : T[curr].edges)
        {
            // See if the neighbor is in the closed set
            if(std::find(closed_set.begin(), closed_set.end(), edge) != closed_set.end())
            {
                continue;
            }

            // Compute new G-Value and cost
            const auto cost = config_dist(T[curr], T[edge]);
            if (cost >= 0)
            {
                // Compute cost to come
                const auto cost_to_come = T[curr].cost + cost;

                // Add to open set if it's not, otherwise just skip this neighbor because
                // it's not a better path
                if (std::find(open_set.begin(), open_set.end(), edge) == open_set.end())
                    open_set.emplace_back(edge);
                else if(cost_to_come >= T[edge].cost)
                    continue;

                // This is the best path yet, Let's add it to the Open Set!
                T[edge].parent_id = curr;
                T[edge].cost = cost_to_come;
            }
        }
    }

    return false;
}

Plan PRM::plan()
{
    const auto start_time = std::chrono::steady_clock::now();

    /************* Generate PRM *************/

    Tree PRM;
    ArmConfiguration PRM_start_config, PRM_goal_config;
    bool valid_start_goal_connections = false;
    int PRM_samples(0);
    while (true)
    {
        // Uniform random sampling
        ArmConfiguration alpha = sample_config(0);

        if (is_valid_arm_config(alpha, map_, arm_link_length_))
        {
            // Get neighbors
            const auto Q = get_neighbors(PRM, alpha, prm_opts_.thresh);

            // Add node / edges to PRM
            PRM_samples++;
            alpha.id = PRM.size();
            PRM[alpha.id] = alpha;
            for (const auto &q : Q)
            {
                if (no_collisions(PRM[q], PRM[alpha.id]))
                {
                    PRM[alpha.id].edges.emplace_back(q);
                    PRM[q].edges.emplace_back(alpha.id);
                }
            }

            // Check for valid path to goal
            PRM_start_config = get_nearest_neighbor(PRM, start_config_);
            PRM_goal_config = get_nearest_neighbor(PRM, goal_config_);

            if (no_collisions(start_config_, PRM_start_config) &&
                no_collisions(goal_config_, PRM_goal_config))
            {
                valid_start_goal_connections = true;
            }
        }

        // Nested exit condition
        if (PRM_samples > prm_opts_.num_samples)
            if (valid_start_goal_connections)
                break;

        if ((std::chrono::steady_clock::now() - start_time).count() / 1e9 > opts_.timeout_s)
            return Plan(start_time);
    }

    /************* Compute Path *************/

    std::vector<ArmConfiguration> plan;
    if (!dijkstra(PRM, PRM_start_config, PRM_goal_config))
        return Plan(start_time);

    plan.emplace_back(goal_config_);
    generate_path(plan, PRM, PRM[PRM_goal_config.id]);
    plan.emplace_back(start_config_);
    std::reverse(plan.begin(), plan.end());

    return Plan(plan, start_time);
}
