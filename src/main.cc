//
// main.cc
//
// Author: David Robinson
// Date: 2019-11-22
//

#include <planner.hh>
#include <prm.hh>
#include <rrt_connect.hh>
#include <rrt_star.hh>
#include <rrt.hh>

#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>

#include <json/json.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>


void print_usage(char **argv)
{
    std::cerr << "USAGE: " << argv[0]
              << " -c <path-to-config> -m <path-to-map> [-v (visualize)]" << std::endl;
}

Json::Value read_json(const std::string &path)
{
    std::ifstream ifs(path);
    if (ifs.fail())
        throw std::runtime_error(path + " does not exist");

    Json::Value root;
    ifs >> root;
    return root;
}

cv::Point2d swap_coords(const cv::Mat &image, const cv::Point2d &coord)
{
    return cv::Point2d(coord.x, image.rows - coord.y - 1);
}

Map load_map(const std::string &map_path)
{
    const auto image = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
    if (!image.data)
    {
        std::cout << "Could not open or find the map_image" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    cv::Mat image_uchar;
    image.convertTo(image_uchar, CV_8U);

    std::unordered_map<size_t, std::unordered_map<size_t, MapState>> data;
    for (size_t row = 0; row < image_uchar.rows; row++)
    {
        for (size_t col = 0; col < image_uchar.cols; col++)
        {
            const auto px = static_cast<unsigned>(image_uchar.at<uchar>(row, col));
            const auto map_coords = swap_coords(image_uchar, cv::Point2d(col, row));

            if (px > 0)
                data[map_coords.x][map_coords.y] = MapState::Free;
            else
                data[map_coords.x][map_coords.y] = MapState::Occupied;
        }
    }

    return Map{ image_uchar, data };
}

cv::Mat draw_configs(cv::Mat map_image, const std::vector<ArmConfiguration> &configs, const double arm_link_length, const double scale)
{
    cv::resize(map_image, map_image, cv::Size(), scale, scale, cv::INTER_NEAREST);
    cv::cvtColor(map_image, map_image, CV_GRAY2RGB);

    for (const auto &config : configs)
    {
        cv::Point2d current(0.0, 0.0);
        cv::circle(map_image, swap_coords(map_image, current), scale * 0.5, cv::Scalar(0, 0, 255), CV_FILLED);

        for (size_t i = 0; i < config.angles.size(); ++i)
        {
            cv::Point2d next(current.x + arm_link_length * scale * cos(config.angles.at(i)),
                             current.y + arm_link_length * scale * sin(config.angles.at(i)));
            cv::line(map_image, swap_coords(map_image, current), swap_coords(map_image, next), cv::Scalar(0, 0, 255), scale * 0.3);
            cv::circle(map_image, swap_coords(map_image, next), scale * 0.6, cv::Scalar(0, 0, 255), CV_FILLED);
            current = next;
        }
    }
    return map_image;
}

int main(int argc, char *argv[])
{
    std::string config_path;
    std::string map_path;
    bool visualize = false;

    int option = -1;
    while ((option = getopt(argc, argv, "c:m:vh")) != -1)
    {
        switch (option)
        {
            case 'c':
                config_path = std::string(optarg);
                break;
            case 'm':
                map_path = std::string(optarg);
                break;
            case 'h':
                print_usage(argv);
                std::exit(EXIT_FAILURE);
            case 'v':
                visualize = true;
            default:
                break;
        }
    }

    if (config_path.empty() || map_path.empty())
    {
        print_usage(argv);
        std::exit(EXIT_FAILURE);
    }

    const auto map = load_map(map_path);
    const auto config_json = read_json(config_path);
    const auto opts = PlannerOptions(config_json);

    std::cout << std::endl << "Loaded " << map.data.size() << " x " << map.data.at(0).size() << " map." << std::endl;
    std::cout << std::endl << "Loaded " << opts.arm_dof << " dof arm." << std::endl;
    std::cout << "Start angles (degrees): " << opts.arm_start_rads << std::endl;
    std::cout << "End angles (degrees): " << opts.arm_end_rads << std::endl;

    // TODO: Check start and end have no collisions!

    if (visualize)
    {
        const auto configs = std::vector<ArmConfiguration>{ opts.arm_start_rads, opts.arm_end_rads };
        cv::imshow("Map", draw_configs(map.image, configs, opts.arm_link_length, opts.image_display_scale));

        std::cout << std::endl << "Press any key to begin planning..." << std::endl << std::endl;
        cv::waitKey(0);
    }

    // Choose planner
    Planner *planner;
    switch (opts.planner_type)
    {
        case PlannerType::RRT:
            std::cout << "RRT Planner" << std::endl;
            // TODO: Use modern ptrs
            planner = new RRT;
            break;
        case PlannerType::RRTConnect:
            std::cout << "RRT-Connect Planner" << std::endl;
            planner = new RRTConnect;
            break;
        case PlannerType::RRTStar:
            std::cout << "RRT* Planner" << std::endl;
            planner = new RRTStar;
            break;
        case PlannerType::PRM:
            std::cout << "PRM Planner" << std::endl;
            planner = new PRM;
            break;
        default:
            throw std::runtime_error("Unsupported planner type");
    }

    // TODO: Compute Plan

    // TODO: Playback finished plan

    std::exit(EXIT_SUCCESS);
}