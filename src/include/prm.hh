//
// prm.hh
//
// Author: David Robinson
// Date: 2019-11-22
//

#pragma once

#include <planner.hh>


struct PRMOptions
{
	PlannerOptions general;
	double thresh = 1.5;
	size_t num_samples = 4000;

    explicit PRMOptions(const Json::Value &json)
    {
		general = PlannerOptions(json);
		thresh = json["planner"]["prm"]["thresh"].asDouble();
		num_samples = json["planner"]["prm"]["num_samples"].asUInt64();
	}
};

class PRM : public Planner
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
		PRM(
			const PRMOptions &opts,
			const Map &map,
			const ArmConfiguration &start_config,
			const ArmConfiguration &goal_config,
			const double arm_link_length);

		/**
		 * Compute the plan between the start and goal config
		 * @return Computed plan for the arm
		 */
		Plan plan();

	private:
		/**
		 * Perform a dijkstra graph search on a tree to find a path between a
		 * start and end arm configuration
 		 * @param start_config Initial arm configuration
 		 * @param end_config Final arm configuration
		 * @return Success bool
		 */
		bool dijkstra(
			Tree &T,
			const ArmConfiguration &start_config,
			const ArmConfiguration &end_config);

		const PRMOptions prm_opts_;
};
