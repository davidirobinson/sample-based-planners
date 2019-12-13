//
// types.hh
//
// Author: David Robinson
// Date: 2019-12-06
//

#pragma once

#include <opencv2/opencv.hpp>


enum class MapState
{
	Occupied,
	Free
};

struct Map
{
	cv::Mat image;
	size_t size_x;
	size_t size_y;
	std::unordered_map<size_t, std::unordered_map<size_t, MapState>> data;
};

enum class PlannerType
{
    RRT,
    RRTConnect,
    RRTStar,
    PRM
};

enum class PlannerUnits
{
	Radians,
	Degrees
};

struct ArmConfiguration
{
	size_t id;
	size_t parent_id;
	std::vector<double> angles;

	// For PRM
	double cost;
	std::vector<int> edges;

	ArmConfiguration()
	{
		id = 0;
		parent_id = -1;
	}

	ArmConfiguration(std::vector<double> _angles) :
		angles(_angles)
	{
		id = 0;
		parent_id = -1;
	}
};

using Tree = std::unordered_map<size_t, ArmConfiguration>;

inline std::ostream &operator<<(std::ostream &stream, const ArmConfiguration &arm_config)
{
    for (const auto &angle : arm_config.angles)
        stream << angle * 180 / M_PI << " ";
    return stream;
}

inline bool operator==(const ArmConfiguration &a, const ArmConfiguration &b)
{
	if (a.angles.size() != b.angles.size())
        return false;

	for (size_t i = 0; i < a.angles.size(); i++)
	{
		double dist = fabs(a.angles[i] - b.angles[i]);
		if (dist > 0.001) return false;
	}

	return true;
}
