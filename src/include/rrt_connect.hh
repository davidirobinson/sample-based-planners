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
		RRTConnect(
			const RRTConnectOptions &opts,
			const Map &map,
			const ArmConfiguration &start_config,
			const ArmConfiguration &goal_config,
			const double arm_link_length);

		Plan plan();

	private:
		bool connect(Tree &T, const ArmConfiguration &new_config);

		const RRTConnectOptions rrtconnect_opts_;
};
