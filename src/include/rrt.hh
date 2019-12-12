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
    explicit RRTOptions(const Json::Value &json)
    {
    }
};

class RRT : public Planner
{
	public:
		RRT(
			const PlannerOptions &opts,
			const Map &map,
			const ArmConfiguration &start_config,
			const ArmConfiguration &goal_config,
			const double arm_link_length);

		Plan plan();
};
