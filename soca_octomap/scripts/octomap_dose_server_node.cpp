/** @file octomap_dose_server_node.cpp
 * @brief The octomap dose server node.
 * 
 * @author Tito Irfan (titoirfan)
 * @version 1.0 26/3/2021
 */

#include <ros/ros.h>
#include <soca_octomap/OctomapDoseServer.hpp>

#define USAGE "\nUSAGE: octomap_dose_server <map.[bt|ot]>\n" \
    "  map.bt: inital octomap 3D map file to read\n"

using namespace octomap_server;

int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_dose_server");
    const ros::NodeHandle nh;
    const ros::NodeHandle private_nh("~");
    
    std::string mapFilename(""), mapFilenameParam("");

    if (argc > 2 || (argc == 2 && std::string(argv[1]) == "-h")){
        ROS_ERROR("%s", USAGE);
        exit(-1);
    }

    OctomapDoseServer server(private_nh, nh);
    ros::spinOnce();

    if (argc == 2) {
        mapFilename = std::string(argv[1]);
    }

    if (private_nh.getParam("map_file", mapFilenameParam))
        if (mapFilename != "")
            ROS_WARN("map_file is specified by the argument '%s' and rosparam '%s'. now loads '%s'", 
                mapFilename.c_str(), 
                mapFilenameParam.c_str(), 
                mapFilename.c_str());
        else
            mapFilename = mapFilenameParam;
    
    if (mapFilename != "")
        if (!server.openFile(mapFilename)) {
            ROS_ERROR("Could not open file %s", mapFilename.c_str());
            exit(1);
        }

    double frequency;
    private_nh.getParam("frequency", frequency);
    ros::Rate loop_rate(frequency);

    while (ros::ok()) {
        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }
}