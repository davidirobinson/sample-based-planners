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
    explicit RRTStarOptions(const Json::Value &json)
    {
    }
};

class RRTStar : public Planner
{
	public:
		RRTStar(
			const PlannerOptions &opts,
			const Map &map,
			const ArmConfiguration &start_config,
			const ArmConfiguration &goal_config);

		int plan(
			double*** plan_out,
			int* planlength,
			double* num_samples,
			double* path_quality);

	private:
		const long timeout = 50000; // lower than the rest!
		const double rewire_radius = 0.5;
};
