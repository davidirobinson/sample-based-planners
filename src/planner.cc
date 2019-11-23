//
// planner.cc
//
// Author: David Robinson
// Date: 2019-11-22
//

#include <planner.hh>


bool operator==(const Config &a, const Config &b)
{
	if (a.angles.size() != b.angles.size()) return false;

	for (int i=0; i<a.angles.size(); i++)
	{
		double dist = fabs(a.angles[i] - b.angles[i]);
		if (dist > 0.001) return false;
	}
	return true;
}

double Planner::config_dist(const Config &a, const Config &b)
{
    // www.codeproject.com/Articles/59789/Calculate-the-Real-Difference-Between-Two-Angles-K
    double ssd(0.0);
    for (int i=0; i<a.angles.size(); i++)
    {
        double diff = a.angles[i] - b.angles[i];
        while (diff < -M_PI) diff += 2*M_PI;
        while (diff > +M_PI) diff -= 2*M_PI;

        ssd += pow(diff, 2);
    }
    return sqrt(ssd);
}

Config Planner::convert_to_config(int numofDOFs, double* array)
{
    Config ret;
    ret.angles.resize(numofDOFs);
    for (int i=0; i<numofDOFs; i++)
    {
        ret.angles[i] = array[i];
    }
    return ret;
}

void Planner::assign_plan(
    double*** plan_array,
    int* planlength,
    int numofDOFs,
    std::vector<Config> &plan_vector,
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

void Planner::print_config(const Config &config)
{
    for (int i=0; i<config.angles.size(); i++)
    {
        std::cout << ", " << config.angles[i];
    }
    std::cout << std::endl;
}

Config Planner::sample_config(const double &p_goal)
{
    std::uniform_real_distribution<double> probability(0,1);
    std::uniform_real_distribution<double> angle(0,2*M_PI);

    if (probability(random) < p_goal)
    {
        return goal_config;
    }
    else
    {
        Config random_config;
        random_config.angles.resize(goal_config.angles.size());

        for (int i=0; i<goal_config.angles.size(); i++)
        {
            random_config.angles[i] = angle(random);
        }

        return random_config;
    }
}

Config Planner::get_nearest_neighbor(
    const tree &configs,
    const Config &new_config)
{
    double min_dist = 1e9;
    Config nearest;
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

Config Planner::extend(const Config &nearest, const Config &sampled)
{
    double dist = config_dist(nearest, sampled);

    if (angle_step_size < dist)
    {
        double u = angle_step_size / dist;

        Config interp;
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

bool Planner::no_collisions(Config start_config, Config goal_config)
{
    for (double i=0; i<=interp_samples; i+=1.0)
    {
        double u = i / interp_samples;

        Config interp;
        interp.angles.resize(start_config.angles.size());

        for (int i=0; i<interp.angles.size(); i++)
        {
            interp.angles[i] = (1-u)*start_config.angles[i] + (u)*goal_config.angles[i];
        }

        // TODO: Implement this function
        // if (!IsValidArmConfiguration(interp.angles, map, x_size, y_size)) return false;
    }
    return true;
}

bool Planner::generate_RRT_tree(
    tree &T_start,
    const Config &goal_config,
    Config &extended_config)
{
    Config new_config = sample_config(p_goal_sample);
    Config nearest_config = get_nearest_neighbor(T_start, new_config);
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

void Planner::generate_path(std::vector<Config> &plan, tree &T, Config parent)
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
    const Config &config,
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
