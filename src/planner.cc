//
// planner.cc
//
// Author: David Robinson
// Date: 2019-11-22
//

#include <planner.hh>


Planner::Planner(const PlannerOptions &opts, const Map &map) :
    opts_(opts),
    map_(map),
    random_(std::default_random_engine(rd()))
{
}

double Planner::config_dist(const ArmConfiguration &a, const ArmConfiguration &b)
{
    // www.codeproject.com/Articles/59789/Calculate-the-Real-Difference-Between-Two-Angles-K
    double ssd = 0.0;
    for (size_t i = 0; i < a.angles.size(); i++)
    {
        double diff = a.angles[i] - b.angles[i];
        while (diff < -M_PI) diff += 2 * M_PI;
        while (diff > +M_PI) diff -= 2 * M_PI;

        ssd += pow(diff, 2);
    }
    return sqrt(ssd);
}

void Planner::assign_plan(
    double*** plan_array,
    int* planlength,
    int numofDOFs,
    std::vector<ArmConfiguration> &plan_vector,
    double* path_quality)
{
    int length = plan_vector.size();

    double path_length = 0;
    for (int i=1; i<length; i++)
    {
        path_length += config_dist(plan_vector[i-1], plan_vector[i]);
    }

    *plan_array = (double**) malloc(length*sizeof(double*));
    for (int i=0; i<length; i++)
    {
        (*plan_array)[i] = (double*) malloc(numofDOFs*sizeof(double));
        for(int j = 0; j < numofDOFs; j++)
        {
            (*plan_array)[i][j] = plan_vector[i].angles[j];
        }
    }
    *planlength = length;
    *path_quality = path_length;
}

ArmConfiguration Planner::sample_config(const double &p_goal)
{
    std::uniform_real_distribution<double> probability(0,1);
    std::uniform_real_distribution<double> angle(0,2*M_PI);

    if (probability(random_) < p_goal)
        return opts_.goal_config;

    ArmConfiguration random_config;
    random_config.angles.resize(opts_.goal_config.angles.size());

    for (int i = 0; i < opts_.goal_config.angles.size(); i++)
        random_config.angles[i] = angle(random_);

    return random_config;
}

ArmConfiguration Planner::get_nearest_neighbor(
    const tree &configs,
    const ArmConfiguration &new_config)
{
    double min_dist = 1e9;
    ArmConfiguration nearest;
    for (auto c : configs)
    {
        double dist = config_dist(c.second, new_config);
        if (dist < min_dist)
        {
            min_dist = dist;
            nearest = c.second;
        }
    }
    return nearest;
}

ArmConfiguration Planner::extend(const ArmConfiguration &nearest, const ArmConfiguration &sampled)
{
    double dist = config_dist(nearest, sampled);

    if (angle_step_size < dist)
    {
        double u = angle_step_size / dist;

        ArmConfiguration interp;
        interp.angles.resize(nearest.angles.size());
        for (int i=0; i<interp.angles.size(); i++)
        {
            interp.angles[i] = (1-u)*nearest.angles[i] + (u)*sampled.angles[i];
        }
        return interp;
    }
    else
    {
        return sampled;
    }
}

bool Planner::no_collisions(ArmConfiguration start_config, ArmConfiguration goal_config)
{
    for (double i=0; i<=interp_samples; i+=1.0)
    {
        double u = i / interp_samples;

        ArmConfiguration interp;
        interp.angles.resize(start_config.angles.size());

        for (int i=0; i<interp.angles.size(); i++)
        {
            interp.angles[i] = (1-u)*start_config.angles[i] + (u)*goal_config.angles[i];
        }

        if (!IsValidArmConfiguration(interp, map_))
            return false;
    }
    return true;
}

bool Planner::generate_RRT_tree(
    tree &T_start,
    const ArmConfiguration &goal_config,
    ArmConfiguration &extended_config)
{
    ArmConfiguration new_config = sample_config(p_goal_sample);
    ArmConfiguration nearest_config = get_nearest_neighbor(T_start, new_config);
    extended_config = extend(nearest_config, new_config);

    if (no_collisions(nearest_config, extended_config))
    {
        // Attach parent and add to tree
        extended_config.id = T_start.size();
        extended_config.parent_id = nearest_config.id;
        T_start[extended_config.id] = extended_config;
        return true;
    }
    return false;
}

void Planner::generate_path(std::vector<ArmConfiguration> &plan, tree &T, ArmConfiguration parent)
{
    int count = 0;
    while (count < timeout)
    {
        plan.push_back(parent);

        if (parent.parent_id == -1)
        {
            break;
        }
        else
        {
            parent = T[parent.parent_id];
            count++;
        }
    }
}

std::vector<int> Planner::get_neighbors(
    const tree &T,
    const ArmConfiguration &config,
    const double &radius)
{
    std::vector<int> neighbors;

    for (auto t : T)
    {
        double dist = config_dist(t.second, config);

        if (dist < radius &&
            config.parent_id != t.second.id &&
            config.id != t.second.id)
        {
            neighbors.push_back(t.second.id);
        }
    }
    return neighbors;
}
