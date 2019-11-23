//
// planner.hh
//
// Author: David Robinson
// Date: 2019-11-22
//

#pragma once

#include <math.h>
#include <ctime>
#include <unistd.h>
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <random>
#include <ctime>
#include <functional>

#include <json/json.h>


enum class PlannerType
{
    RRT,
    RRT_CONNECT,
    RRT_STAR,
    PRM
};

struct PlannerOptions
{
    PlannerType planner_type = PlannerType::RRT;

    explicit PlannerOptions(const Json::Value &json)
    {
        std::string planner_type_str = json["general"]["planner_type"].asString();
        std::transform(planner_type_str.begin(), planner_type_str.end(), planner_type_str.begin(),
            [](unsigned char c){ return std::tolower(c); });

        if (planner_type_str == "rrt")
            planner_type = PlannerType::RRT;
        else if (planner_type_str == "rrt_connect" || planner_type_str == "rrtconnect")
            planner_type = PlannerType::RRT_CONNECT;
        else if (planner_type_str == "rrt_star" || planner_type_str == "rrtstar")
            planner_type = PlannerType::RRT_STAR;
        else if (planner_type_str == "prm")
            planner_type = PlannerType::PRM;
        else
            throw std::runtime_error(std::string("Unknown planner type: " + planner_type_str));
    }
};

/*
 * Planning state representation
 */
struct Config
{
	int id;
	int parent_id;
	std::vector<double> angles;

	// For PRM:
	double cost;
	std::vector<int> edges;

	Config()
	{
		id = 0;
		parent_id = -1;
	}

	Config(std::vector<double> _angles) :
		angles(_angles)
	{
		id = 0;
		parent_id = -1;
	}
};

bool operator==(const Config &a, const Config &b);

/*
 * Base Class for Planners
 */
class Planner
{
	public:
		Planner() {}
		~Planner() {}

		void set_values(
			double*	_map,
			int _x_size,
			int _y_size,
			double* _start_config,
			double* _goal_config,
			int _numofDOFs)
		{
			// Save values
			map = _map;
			x_size = _x_size;
			y_size = _y_size;
			numofDOFs = _numofDOFs;

			// Get start / end configs
			start_config = convert_to_config(numofDOFs, _start_config);
			goal_config = convert_to_config(numofDOFs, _goal_config);

			// Intialize Random
			random = std::default_random_engine(rd());
		}

		virtual int plan(
			double*** plan_out,
			int* planlength,
			double* num_samples,
			double* path_quality) = 0;

	protected:
		typedef std::unordered_map<int, Config> tree;

		/*
		 * Params as member vars
		 */
		double*	map;
		int x_size, y_size;
		int numofDOFs;

		Config start_config, goal_config;

		std::random_device rd;
		std::default_random_engine random;

		/*
		 * Params
		 */
		const long timeout 			 = 200000;
		const int interp_samples 	 = 50;
		const double angle_step_size = 0.5;
		const double p_goal_sample   = 0.05;

		double config_dist(const Config &a, const Config &b);

		Config convert_to_config(int numofDOFs, double* array);

		void assign_plan(
			double*** plan_array,
			int* planlength,
			int numofDOFs,
			std::vector<Config> &plan_vector,
			double* path_quality);

		void print_config(const Config &config);

		Config sample_config(const double &p_goal);

		Config get_nearest_neighbor(const tree &configs, const Config &new_config);

		Config extend(const Config &nearest, const Config &sampled);

		bool no_collisions(Config start_config, Config goal_config);

		bool generate_RRT_tree(
			tree &T_start,
			const Config &goal_config,
			Config &extended_config);

		void generate_path(std::vector<Config> &plan, tree &T, Config parent);

		std::vector<int> get_neighbors(
			const tree &T,
			const Config &config,
			const double &radius);
};

class RRT : public Planner
{
	public:
		int plan(
			double*** plan_out,
			int* planlength,
			double* num_samples,
			double* path_quality
		);
};

class RRT_Connect : public Planner
{
	public:
		int plan(
			double*** plan_out,
			int* planlength,
			double* num_samples,
			double* path_quality
		);

	private:
		bool connect(tree &T, const Config &new_config);
};

class RRT_Star : public Planner
{
	public:
		int plan(
			double*** plan_out,
			int* planlength,
			double* num_samples,
			double* path_quality
		);
	private:
		const long timeout = 50000; // lower than the rest!
		const double rewire_radius = 0.5;
};

class PRM : public Planner
{
	public:
		int plan(
			double*** plan_out,
			int* planlength,
			double* num_samples,
			double* path_quality
		);

	private:
		const double PRM_thresh = 1.5;
		const int num_PRM_samples = 4000;

		bool dijkstra(tree &T, Config start, Config goal);
};
