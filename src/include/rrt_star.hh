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

    explicit RRTStarOptions(const Json::Value &json)
    {
		general = PlannerOptions(json);
    }
};

class RRTStar : public Planner
{
	public:
		RRTStar(
			const RRTStarOptions &opts,
			const Map &map,
			const ArmConfiguration &start_config,
			const ArmConfiguration &goal_config,
			const double arm_link_length);

		Plan plan();

	private:
		const double rewire_radius = 0.5;
};
