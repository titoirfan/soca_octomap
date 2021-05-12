/** @file octomap_dose_saver_node.cpp
 * @brief The octomap dose saver node.
 * 
 * @author Tito Irfan (titoirfan)
 * @version 1.0 26/3/2021
 */

#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <fstream>

// Workaround for OcTreeDose
#include <soca_octomap/OcTreeDose.hpp>
octomap::OcTreeDose dummy(0.1);

#include <octomap_msgs/GetOctomap.h>
using octomap_msgs::GetOctomap;

#define USAGE "\nUSAGE: octomap_saver [-f] <mapfile.[bt|ot]>\n" \
    "  -f: Query for the full occupancy octree, instead of just the compact binary one\n" \
    "  mapfile.bt: filename of map to be saved (.bt: binary tree, .ot: general octree)\n"

using namespace std;
using namespace octomap;

class MapSaver{
public:
  MapSaver(const std::string& mapname, bool full){
    ros::NodeHandle n;
    std::string servname = "octomap_binary";
    if (full)
      servname = "octomap_full";
    ROS_INFO("Requesting the map from %s...", n.resolveName(servname).c_str());
    GetOctomap::Request req;
    GetOctomap::Response resp;
    while(n.ok() && !ros::service::call(servname, req, resp)) {
      ROS_WARN("Request to %s failed; trying again...", n.resolveName(servname).c_str());
      usleep(1000000);
    }

    if (n.ok()) { // skip when CTRL-C
      AbstractOcTree* tree = octomap_msgs::msgToMap(resp.map);
      AbstractOccupancyOcTree* octree = NULL;
      if (tree) {
        octree = dynamic_cast<AbstractOccupancyOcTree*>(tree);
      } 
      else {
        ROS_ERROR("Error creating octree from received message");
        if (resp.map.id == "ColorOcTree")
          ROS_WARN("You requested a binary map for a ColorOcTree - this is currently not supported. Please add -f to request a full map");
      }

      if (octree) {
        ROS_INFO("Map received (%zu nodes, %f m res), saving to %s", octree->size(), octree->getResolution(), mapname.c_str());
        
        std::string suffix = mapname.substr(mapname.length()-3, 3);
        if (suffix== ".bt") { // write to binary file:
          if (!octree->writeBinary(mapname)) {
            ROS_ERROR("Error writing to file %s", mapname.c_str());
          }
        } 
        else if (suffix == ".ot") { // write to full .ot file:
          if (!octree->write(mapname)) {
            ROS_ERROR("Error writing to file %s", mapname.c_str());
          }
        } 
        else {
          ROS_ERROR("Unknown file extension, must be either .bt or .ot");
        }
      } 
      else {
        ROS_ERROR("Error reading OcTree from stream");
      }

      delete octree;
    }
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_dose_saver");
  std::string mapFilename("");
  bool fullmap = false;
  if (argc == 3 && strcmp(argv[1], "-f")==0) {
    fullmap = true;
    mapFilename = std::string(argv[2]);
  } 
  else if (argc == 2)
    mapFilename = std::string(argv[1]);
  else {
    ROS_ERROR("%s", USAGE);
    exit(1);
  }

  try {
    MapSaver ms(mapFilename, fullmap);
  }
  catch(std::runtime_error& e) {
    ROS_ERROR("octomap_dose_saver exception: %s", e.what());
    exit(2);
  }

  exit(0);
}


