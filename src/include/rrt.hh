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
		RRT(
			const RRTOptions &opts,
			const Map &map,
			const ArmConfiguration &start_config,
			const ArmConfiguration &goal_config,
			const double arm_link_length);

		Plan plan();

	private:
		const RRTOptions rrt_opts_;
};
