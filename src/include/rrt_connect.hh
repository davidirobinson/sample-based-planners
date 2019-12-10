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
    explicit RRTConnectOptions(const Json::Value &json)
    {
    }
};

class RRTConnect : public Planner
{
	public:
		RRTConnect(
			const PlannerOptions &opts,
			const Map &map,
			const ArmConfiguration &start_config,
			const ArmConfiguration &goal_config);

		Plan plan();

	private:
		bool connect(tree &T, const ArmConfiguration &new_config);
};
