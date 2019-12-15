//
// rrt_star.hh
//
// Author: David Robinson
// Date: 2019-11-22
//

#pragma once

#include <planner.hh>


struct RRTStarOptions
{
	PlannerOptions general;
	double rewire_radius = 0.5;

    explicit RRTStarOptions(const Json::Value &json)
    {
		general = PlannerOptions(json);
		rewire_radius = json["planner"]["rrtstar"]["rewire_radius"].asDouble();
    }
};

class RRTStar : public Planner
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
		RRTStar(
			const RRTStarOptions &opts,
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
		const RRTStarOptions rrtstar_opts_;
};
