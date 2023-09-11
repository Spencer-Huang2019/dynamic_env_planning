//
// Created by spencer on 1/7/23.
//

#include <move_dynamic/file_operation_tools/write_trajectory.h>
#include <string.h>
#include <math.h>
#include <fstream>


namespace move_dynamic
{
WriteTrajectory::WriteTrajectory()
{
}

WriteTrajectory::~WriteTrajectory()
{
};

void WriteTrajectory::writeTrajectoryMsg2Yaml(const moveit_msgs::RobotTrajectory& trajectory,
                                             const std::string& filename)
{
    std::ofstream fout(filename);
    YAML::Node config_ = YAML::LoadFile(filename);

    auto points = trajectory.joint_trajectory.points;
    int i = 0;
    for (const auto& point : points)
    {
        std::string num = std::to_string(i);
        if (num.size() == 1)
            num = "00" + num;
        else if(num.size() == 2)
            num = "0" + num;
        std::string point_num = "point_" + num;
        config_[point_num]["positions"] = point.positions;
        config_[point_num]["time_from_start"] = point.time_from_start.toSec();
        config_[point_num]["velocities"] = point.velocities;
        config_[point_num]["accelerations"] = point.accelerations;
        i++;
    }

    fout << config_;
    fout.close();
}

void WriteTrajectory::writeTrajectoryMsg2Yaml(robot_trajectory::RobotTrajectoryPtr& trajectory,
                                              const std::string& filename)
{
    std::ofstream fout(filename);
    YAML::Node config_ = YAML::LoadFile(filename);

    std::size_t wpts_count = trajectory->getWayPointCount();

    for(std::size_t i = 0; i < wpts_count; ++i)
    {
        moveit::core::RobotStatePtr point = trajectory->getWayPointPtr(i);
        std::vector<double> positions;
        positions.resize(7);
        for(std::size_t j = 0; j < 7; ++j)
        {
            positions.push_back(point->getVariablePosition(j));
        }

        std::string num = std::to_string(i);
        if (num.size() == 1)
            num = "00" + num;
        else if(num.size() == 2)
            num = "0" + num;
        std::string point_num = "point_" + num;
        config_[point_num]["positions"] = positions;
    }

    fout << config_;
    fout.close();
}

void WriteTrajectory::writeTrajectoryMsg2Yaml(const std::vector<moveit_msgs::RobotTrajectory>& trajectories,
                                             const std::string& filename)
{
    std::ofstream fout(filename);
    YAML::Node config_ = YAML::LoadFile(filename);

    int j = 0;
    for (const auto trajectory : trajectories)
    {
        std::string num = std::to_string(j);
        if (num.size() == 1)
            num = "00" + num;
        else if(num.size() == 2)
            num = "0" + num;
        std::string traj_num = "traj_" + num;

        auto points = trajectory.joint_trajectory.points;
        int i = 0;
        for (const auto& point : points)
        {
            std::string point_num = "point_" + std::to_string(i);
            config_[traj_num][point_num]["positions"] = point.positions;
            config_[traj_num][point_num]["time_from_start"] = point.time_from_start.toSec();
            config_[traj_num][point_num]["velocities"] = point.velocities;
            config_[traj_num][point_num]["accelerations"] = point.accelerations;
            i++;
        }
        j++;
    }

    fout << config_;
    fout.close();
}
}