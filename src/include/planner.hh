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

#include <json/json.h>


struct PlannerOptions
{
	double timeout_s = 30.0;
	unsigned interp_samples = 50;
	double angle_step_size_rad = 0.5;
	double p_goal_sample = 0.05;

	PlannerOptions()
	{
	}

    explicit PlannerOptions(const Json::Value &json)
    {
		timeout_s = json["planner"]["timeout_s"].asDouble();
		interp_samples = json["planner"]["interp_samples"].asUInt();
		angle_step_size_rad = json["planner"]["angle_step_size_degrees"].asDouble() / 180 * M_PI;
		p_goal_sample = json["planner"]["p_goal_sample"].asDouble();
    }
};

/**
 * Compute the angular distance between each link of the arm configuration
 * @param a First arm configuration
 * @param b Second arm configuration
 * @return Total angular distance between arms
 */
double config_dist(const ArmConfiguration &a, const ArmConfiguration &b);

/**
 * Data structure containing information describing an arm plan, i.e. a sequence of
 * steps that the arm should take to move from the start to end config without colliding
 * with any obstacles in the map
 */
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
		/**
		 * Constructor
		 * @param opts General options for the planner
		 * @param map Map to plan within
		 * @param start_config Initial arm configuration
 		 * @param end_config Final arm configuration
	     * @param arm_link_length Length of the arm to planning for
		 */
		Planner(
			const PlannerOptions &opts,
			const Map &map,
			const ArmConfiguration &start_config,
			const ArmConfiguration &goal_config,
			const double arm_link_length);

		/**
		 * Move constructor
		 * @param other Reference (rvalue) to construct class with
		 */
		Planner(Planner&& other);

		/**
		 * Pure virtual function to define the interface to compute the plan
		 * @return Computed plan for the arm
		 */
		virtual Plan plan() = 0;

	protected:
		/**
		 * Sample a uniformly random arm configuarion in the map
		 * @param p_goal probability of sampling the goal
		 * @param goal_config Goal config to sample with some probability
		 * @return Randomly sampled arm configuration
		 */
		ArmConfiguration sample_config(
			const double &p_goal,
			const ArmConfiguration &goal_config);

		/**
		 * Find closest arm configuration in tree to a given configuration
		 * @param configs Arm configurations tree to search through
		 * @param new_config Arm configuration to find nearest neighbor for
		 * @return Nearest arm configuration
		 */
		ArmConfiguration get_nearest_neighbor(
			const Tree &configs,
			const ArmConfiguration &new_config);

		/**
		 * Get arm configuration within a fixed angle step size towards a sampled config
		 * @param nearest Arm configuration to step from
		 * @param sampled Arm configuration to step towards
		 * @return Extended arm configuration
		 */
		ArmConfiguration extend(
			const ArmConfiguration &nearest,
			const ArmConfiguration &sampled);

		/**
		 * Interpolates between and start and end arm config to determine if there are
		 * any collisions
		 * @param start_config Config to begin collision search at
		 * @param goal_config Config to end collision search at
		 * @return Success bool
		 */
		bool no_collisions(
			const ArmConfiguration &start_config,
			const ArmConfiguration &goal_config);

		/**
		 * Function specific to RRT planners which consolidates function calls to get a new sample,
		 * find it's nearest neighbor, extent towards that neighbor, and then check collisions for
		 * the extended configuration.
		 * @param tree_start Tree to sample neighbor from an append extended config to
		 * @param goal_config Goal config to sample with some probability
		 * @param extended_config Extended configuration to return by reference
		 * @return Success bool
		 */
		bool generate_RRT_tree(
			Tree &tree_start,
			const ArmConfiguration &goal_config,
			ArmConfiguration &extended_config);

		/**
		 * Search through configuration tree to find sequence of arm configurations to return
		 * @param plan Vector of arm configuration to append to
		 * @param tree Arm configuration tree
		 * @param parent Parent node to begin tree search with
		 */
		void generate_path(
			std::vector<ArmConfiguration> &plan,
			const Tree &tree,
			const ArmConfiguration &parent);

		/**
		 * Search through configuration tree to find neighbors of an arm configuration
		 * @param tree Arm configuration tree
		 * @param config Arm configuration to find neighbors of
		 * @param radius Distance radius to collect neighbors within
		 */
		std::vector<size_t> get_neighbors(
			const Tree &tree,
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
