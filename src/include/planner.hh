//
// planner.hh
//
// Author: David Robinson
// Date: 2019-11-22
//

#pragma once

#include <math.h>
#include <chrono>
#include <unistd.h>
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <random>
#include <ctime>
#include <functional>

#include <types.hh>
#include <check_valid.hh>

#include <opencv2/opencv.hpp>
#include <json/json.h>


double config_dist(const ArmConfiguration &a, const ArmConfiguration &b);

struct PlannerOptions
{
    PlannerType planner_type = PlannerType::RRTConnect;
	PlannerUnits planner_units = PlannerUnits::Degrees;
	ArmConfiguration start_config;
	ArmConfiguration goal_config;
	size_t arm_dof;
	double arm_link_length;
	double display_scale = 5.0;

	PlannerOptions()
	{
	}

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
			start_config.angles.emplace_back(angle);
		}

		for (const auto &angle_json : json["general"]["arm_end"])
		{
			const auto angle = planner_units == PlannerUnits::Degrees ?
				angle_json.asDouble() / 180 * M_PI : angle_json.asDouble();
			goal_config.angles.emplace_back(angle);
		}

		if (start_config.angles.size() != goal_config.angles.size())
			throw std::runtime_error("Starting arm dofs does not match ending arm dofs");

		arm_dof = start_config.angles.size();
		if (arm_dof < 2)
			throw std::runtime_error("invalid dofs: " + std::to_string(arm_dof) + ", it should be at least 2");

		arm_link_length = json["general"]["arm_link_length"].asDouble();
		display_scale = json["general"]["image_display_scale"].asDouble();
    }
};

struct Plan
{
	bool valid;
	std::vector<ArmConfiguration> configs;
	double length;
	std::chrono::duration<double> duration;

	Plan(const std::chrono::steady_clock::time_point &start_time) :
		valid(false),
		duration(std::chrono::steady_clock::now() - start_time)
	{
	}

	explicit Plan(
		const std::vector<ArmConfiguration> &config_vector,
		const std::chrono::steady_clock::time_point &start_time) :
		valid(true),
		configs(config_vector),
		length(0.0),
		duration(std::chrono::steady_clock::now() - start_time)
	{
		for (size_t i = 1; i < configs.size(); i++)
			length += config_dist(configs[i - 1], configs[i]);
	}
};

class Planner
{
	public:
		Planner(
			const PlannerOptions &opts,
			const Map &map,
			const ArmConfiguration &start_config,
			const ArmConfiguration &goal_config);

		// TODO: Rule of 5
		Planner(Planner&& other);

		virtual Plan plan() = 0;

	protected:
		ArmConfiguration sample_config(const double &p_goal);

		ArmConfiguration get_nearest_neighbor(const tree &configs, const ArmConfiguration &new_config);

		ArmConfiguration extend(const ArmConfiguration &nearest, const ArmConfiguration &sampled);

		bool no_collisions(ArmConfiguration start_config, ArmConfiguration goal_config);

		bool generate_RRT_tree(
			tree &T_start,
			const ArmConfiguration &goal_config,
			ArmConfiguration &extended_config);

		void generate_path(
			std::vector<ArmConfiguration> &plan,
			tree &T,
			ArmConfiguration parent);

		std::vector<int> get_neighbors(
			const tree &T,
			const ArmConfiguration &config,
			const double &radius);

		const PlannerOptions opts_;
		const Map map_;

		ArmConfiguration start_config_;
		ArmConfiguration goal_config_;

		std::random_device rd_;
		std::default_random_engine random_;

		/*
		 * Params
		 */
		const long timeout 			 = 200000;
		const int interp_samples 	 = 50;
		const double angle_step_size = 0.5;
		const double p_goal_sample   = 0.05;
};
