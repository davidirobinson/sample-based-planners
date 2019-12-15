//
// rrt_connect.hh
//
// Author: David Robinson
// Date: 2019-11-22
//

#pragma once

#include <planner.hh>


struct RRTConnectOptions
{
	PlannerOptions general;

    explicit RRTConnectOptions(const Json::Value &json)
    {
		general = PlannerOptions(json);
    }
};

class RRTConnect : public Planner
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
		RRTConnect(
			const RRTConnectOptions &opts,
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
		 * Attempt to connect a new arm config sample to the existing tree by checking
		 * for collisions while extending towards that sample
		 * @param T Tree to try and add the new config to
		 * @param new_config New arm config to use
		 * @return Success bool
		 */
		bool connect(Tree &T, const ArmConfiguration &new_config);

		const RRTConnectOptions rrtconnect_opts_;
};
