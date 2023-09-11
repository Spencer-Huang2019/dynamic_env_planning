//
// Created by spencer on 12/13/22.
//

#include "move_dynamic/move_group/capability_names.h"
#include "move_dynamic/move_group/move_dynamic_capability.h"
#include <moveit/macros/console_colors.h>
#include <pluginlib/class_loader.hpp>

#include <boost/tokenizer.hpp>
#include <memory>
#include <set>

constexpr char LOGNAME[] = "move_dynamic_planning_interface";

namespace move_group
{
static const char* DEFAULT_CAPABILITIES[] = {
        "move_group/MoveDynamicPlanningService",
        "move_group/MoveDynamicExecuteWptsAction",
};

class MoveDynamicExe
{
public:
    MoveDynamicExe() : node_handle_("~")
    {
        configureCapabilities();
    }

    ~MoveDynamicExe()
    {
        capabilities_.clear();
        capability_plugin_loader_.reset();
    }

    void status()
    {
        if (capabilities_.empty())
            printf(MOVEIT_CONSOLE_COLOR_BLUE "\nmove_dynamic_planning_interface is running but no move_group for move_dynamic_planning_interface are "
                                             "loaded.\n\n" MOVEIT_CONSOLE_COLOR_RESET);
        else
        printf(MOVEIT_CONSOLE_COLOR_GREEN "\nYou can start dynamic planning now!\n\n" MOVEIT_CONSOLE_COLOR_RESET);
        fflush(stdout);
    }

private:
    void configureCapabilities()
    {
        try
        {
            capability_plugin_loader_ = std::make_shared<pluginlib::ClassLoader<MoveDynamicCapability>>(
                    "move_dynamic_move_group", "move_group::MoveDynamicCapability");
        }
        catch (pluginlib::PluginlibException& ex)
        {
            ROS_FATAL_STREAM_NAMED(LOGNAME,
                                   "Exception while creating plugin loader for move_dynamic_planning_interface move_group: " << ex.what());
            return;
        }

        std::set<std::string> capabilities;

        // add default move_group
        for (const char* capability: DEFAULT_CAPABILITIES)
            capabilities.insert(capability);

        // add move_group listed in ROS parameter
        std::string capability_plugins;
        if (node_handle_.getParam("capabilities", capability_plugins))
        {
            boost::char_separator<char> sep(" ");
            boost::tokenizer<boost::char_separator<char>> tok(capability_plugins, sep);
            capabilities.insert(tok.begin(), tok.end());
        }

        for (const std::string& capability : capabilities)
        {
            try
            {
                printf(MOVEIT_CONSOLE_COLOR_CYAN "Loading '%s'...\n" MOVEIT_CONSOLE_COLOR_RESET, capability.c_str());
                MoveDynamicCapabilityPtr cap = capability_plugin_loader_->createUniqueInstance(capability);
                cap->initialize();
                capabilities_.push_back(cap);
            }
            catch (pluginlib::PluginlibException& ex)
            {
                ROS_FATAL_STREAM_NAMED(LOGNAME,
                                       "Exception while creating plugin loader for move_dynamic_planning_interface move_group: " << ex.what());
            }
        }

        std::stringstream ss;
        ss << std::endl;
        ss << std::endl;
        ss << "********************************************************" << std::endl;
        ss << "* MoveDynamic using: " << std::endl;
        for (const MoveDynamicCapabilityPtr& cap : capabilities_)
            ss << "*     - " << cap->getName() << std::endl;
        ss << "********************************************************" << std::endl;
        ROS_INFO_STREAM_NAMED(LOGNAME, ss.str());
    }

    ros::NodeHandle node_handle_;
    std::shared_ptr<pluginlib::ClassLoader<MoveDynamicCapability>> capability_plugin_loader_;
    std::vector<MoveDynamicCapabilityPtr> capabilities_;
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_dynamic_planning_interface");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    move_group::MoveDynamicExe mde;
    mde.status();

    ros::waitForShutdown();

    return 0;
}