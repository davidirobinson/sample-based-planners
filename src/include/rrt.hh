//
// rrt.hh
//
// Author: David Robinson
// Date: 2019-11-22
//

#pragma once

#include <planner.hh>


struct RRTOptions
{
	PlannerOptions general;

    explicit RRTOptions(const Json::Value &json)
    {
		general = PlannerOptions(json);
    }
};

class RRT : public Planner
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
		RRT(
			const RRTOptions &opts,
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
		const RRTOptions rrt_opts_;
};
