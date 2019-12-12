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
	double timeout_s = 30;
	unsigned interp_samples = 50;
	double angle_step_size = 0.5;
	double p_goal_sample = 0.05;

	PlannerOptions()
	{
	}

    explicit PlannerOptions(const Json::Value &json)
    {
		timeout_s = json["planner"]["timeout_s"].asDouble();
		interp_samples = json["planner"]["interp_samples"].asUInt();
		angle_step_size = json["planner"]["angle_step_size"].asDouble();
		p_goal_sample = json["planner"]["p_goal_sample"].asDouble();
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
			const ArmConfiguration &goal_config,
			const double arm_link_length);

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
		const double arm_link_length_;
		std::random_device rd_;
		std::default_random_engine random_;
};
