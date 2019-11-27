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
#include <check_valid.hh>

#include <json/json.h>


enum class PlannerType
{
    RRT,
    RRTConnect,
    RRTStar,
    PRM
};

enum class PlannerUnits
{
	Radians,
	Degrees
};

struct ArmConfiguration
{
	int id;
	int parent_id;
	std::vector<double> angles;

	// For PRM:
	double cost;
	std::vector<int> edges;

	ArmConfiguration()
	{
		id = 0;
		parent_id = -1;
	}

	ArmConfiguration(std::vector<double> _angles) :
		angles(_angles)
	{
		id = 0;
		parent_id = -1;
	}
};

struct PlannerOptions
{
    PlannerType planner_type = PlannerType::RRTConnect;
	PlannerUnits planner_units = PlannerUnits::Degrees;
	ArmConfiguration arm_start_rads;
	ArmConfiguration arm_end_rads;
	size_t arm_dof;
	double image_display_scale = 5.0;

    explicit PlannerOptions(const Json::Value &json)
    {
        std::string planner_type_str = json["general"]["planner"].asString();
        std::transform(planner_type_str.begin(), planner_type_str.end(), planner_type_str.begin(),
            [](unsigned char c){ return std::tolower(c); });
        if (planner_type_str == "rrt")
            planner_type = PlannerType::RRT;
        else if (planner_type_str == "rrtconnect")
            planner_type = PlannerType::RRTConnect;
        else if (planner_type_str == "rrtstar")
            planner_type = PlannerType::RRTStar;
        else if (planner_type_str == "prm")
            planner_type = PlannerType::PRM;
        else
            throw std::runtime_error(std::string("Unknown planner type: " + planner_type_str));

		std::string planner_units_str = json["general"]["units"].asString();
        std::transform(planner_units_str.begin(), planner_units_str.end(), planner_units_str.begin(),
            [](unsigned char c){ return std::tolower(c); });
        if (planner_units_str == "radians")
            planner_units = PlannerUnits::Radians;
        else if (planner_units_str == "degrees")
            planner_units = PlannerUnits::Degrees;
        else
            throw std::runtime_error(std::string("Unknown planner units: " + planner_units_str));

		for (const auto &angle_json : json["general"]["arm_start"])
		{
			const auto angle = planner_units == PlannerUnits::Degrees ?
				angle_json.asDouble() / 180 * M_PI : angle_json.asDouble();
			arm_start_rads.angles.emplace_back(angle);
		}

		for (const auto &angle_json : json["general"]["arm_end"])
		{
			const auto angle = planner_units == PlannerUnits::Degrees ?
				angle_json.asDouble() / 180 * M_PI : angle_json.asDouble();
			arm_end_rads.angles.emplace_back(angle);
		}

		if (arm_start_rads.angles.size() != arm_end_rads.angles.size())
			throw std::runtime_error("Starting arm dofs does not match ending arm dofs");

		arm_dof = arm_start_rads.angles.size();
		if (arm_dof < 2)
			throw std::runtime_error("invalid dofs: " + std::to_string(arm_dof) + ", it should be at least 2");

		image_display_scale = json["general"]["image_display_scale"].asDouble();
    }
};

std::ostream &operator<<(std::ostream &stream, const ArmConfiguration &arm_config);

bool operator==(const ArmConfiguration &a, const ArmConfiguration &b);

using tree = std::unordered_map<size_t, ArmConfiguration>;

/*
 * Base Class for Planners
 */
class Planner
{
	public:
		Planner()
		{
		}

		void set_values(
			double*	_map,
			int _x_size,
			int _y_size,
			ArmConfiguration _start_config,
			ArmConfiguration _goal_config,
			size_t _numofDOFs)
		{
			// TODO: move to ctor
			// Save values
			map = _map;
			x_size = _x_size;
			y_size = _y_size;
			numofDOFs = _numofDOFs;

			// Get start / end configs
			start_config = _start_config;
			goal_config = _goal_config;

			// Intialize Random
			random = std::default_random_engine(rd());
		}

		virtual int plan(
			double*** plan_out,
			int* planlength,
			double* num_samples,
			double* path_quality) = 0;

	protected:
		double config_dist(const ArmConfiguration &a, const ArmConfiguration &b);

		void assign_plan(
			double*** plan_array,
			int* planlength,
			int numofDOFs,
			std::vector<ArmConfiguration> &plan_vector,
			double* path_quality);

		ArmConfiguration sample_config(const double &p_goal);

		ArmConfiguration get_nearest_neighbor(const tree &configs, const ArmConfiguration &new_config);

		ArmConfiguration extend(const ArmConfiguration &nearest, const ArmConfiguration &sampled);

		bool no_collisions(ArmConfiguration start_config, ArmConfiguration goal_config);

		bool generate_RRT_tree(
			tree &T_start,
			const ArmConfiguration &goal_config,
			ArmConfiguration &extended_config);

		void generate_path(std::vector<ArmConfiguration> &plan, tree &T, ArmConfiguration parent);

		std::vector<int> get_neighbors(
			const tree &T,
			const ArmConfiguration &config,
			const double &radius);

		double*	map;
		size_t x_size;
		size_t y_size;
		size_t numofDOFs;

		ArmConfiguration start_config;
		ArmConfiguration goal_config;

		std::random_device rd;
		std::default_random_engine random;

		/*
		 * Params
		 */
		const long timeout 			 = 200000;
		const int interp_samples 	 = 50;
		const double angle_step_size = 0.5;
		const double p_goal_sample   = 0.05;
};
