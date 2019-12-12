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
    explicit PRMOptions(const Json::Value &json)
    {
    }
};

class PRM : public Planner
{
	public:
		PRM(
			const PlannerOptions &opts,
			const Map &map,
			const ArmConfiguration &start_config,
			const ArmConfiguration &goal_config,
			const double arm_link_length);

		Plan plan();

	private:
		const double PRM_thresh = 1.5;
		const int num_PRM_samples = 4000;

		bool dijkstra(tree &T, ArmConfiguration start, ArmConfiguration goal);
};
