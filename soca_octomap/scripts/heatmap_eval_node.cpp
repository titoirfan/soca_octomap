/** @file heatmap_eval_node.cpp
 * @brief Compare a dose octomap against a ground truth octomap
 *  to assess its coverage and statistics
 * 
 * @author Tito Irfan (titoirfan)
 * @version 1.0 27/3/2021
 */

#include <ros/ros.h>

#include <octomap/octomap.h>
#include <soca_octomap/OcTreeDose.hpp>

#include <string.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <cmath>

using namespace octomap;

bool compareDouble(double a, double b) {
    return fabs(a - b) <= 1e-6;
}

bool isInBounds(
    point3d point, 
    double x_min, 
    double x_max, 
    double y_min, 
    double y_max, 
    double z_min, 
    double z_max) {
    if (point.x() >= x_min && point.x() <= x_max 
        && point.y() >= y_min && point.y() <= y_max 
        && point.z() >= z_min && point.z() <= z_max)
        return true;

    return false;
}

int main(int argc, char **argv) {
    // Node initialization
    ros::init(argc, argv, "heatmap_eval_node");

    ros::NodeHandle n("~");

    double dose_threshold;
    n.getParam("dose_threshold", dose_threshold);

    // Measurement octree filename
    std::string meas_octree_filename;
    // Ground truth octree filename
    std::string gtruth_octree_filename;

    // Whether to save difference map
    bool save_difference_map;

    n.getParam("measurement_octree_filename", meas_octree_filename);
    n.getParam("ground_truth_octree_filename", gtruth_octree_filename);
    n.getParam("save_difference_map", save_difference_map);

    double x_min, x_max, y_min, y_max, z_min, z_max;
    n.getParam("x_min", x_min);
    n.getParam("x_max", x_max);
    n.getParam("y_min", y_min);
    n.getParam("y_max", y_max);
    n.getParam("z_min", z_min);
    n.getParam("z_max", z_max);    

    // Read the octrees from the files
    ROS_INFO("Reading octree files.");
    OcTreeDose* meas_octree = dynamic_cast<OcTreeDose*>(OcTreeDose::read(meas_octree_filename));
    OcTree* gtruth_octree = new OcTree(gtruth_octree_filename);

    // Validate the octrees' resolution
    ROS_INFO("Measured octree resolution: %lf", meas_octree->getResolution());
    ROS_INFO("Ground truth octree resolution: %lf", gtruth_octree->getResolution());

    if (fabs(meas_octree->getResolution() - gtruth_octree->getResolution()) > 1e-6) {
        ROS_ERROR("Octrees resolution doesn't match, the result might be wrong!");
    }
    
    double meas_x, meas_y, meas_z;
    double min_x, min_y, min_z;
    meas_octree->getMetricSize(meas_x, meas_y, meas_z);
    ROS_INFO("Measurement octree size: %lf\t\t%lf\t\t%lf", meas_x, meas_y, meas_z);

    meas_octree->getMetricMin(min_x, min_y, min_z);
    ROS_INFO("Measurement octree center: %lf\t\t%lf\t\t%lf", min_x + meas_x / 2, min_y + meas_y / 2, min_z + meas_z / 2);

    double gtruth_x, gtruth_y, gtruth_z;
    gtruth_octree->getMetricSize(gtruth_x, gtruth_y, gtruth_z);
    ROS_INFO("Ground truth octree size: %lf\t\t%lf\t\t%lf", gtruth_x, gtruth_y, gtruth_z);

    if (!(compareDouble(meas_x, gtruth_x) && compareDouble(meas_y, gtruth_y) && compareDouble(meas_z, gtruth_z)))
        ROS_ERROR("Octrees size doesn't match, the result might be wrong!");

    // Expand the previously pruned octrees to full resolution
    ROS_INFO("Expanding octrees.");
    meas_octree->expand();
    gtruth_octree->expand();

    // Compare the trees
    ROS_INFO("Comparing trees.");

    // Count the matching nodes amount
    int match_amt = 0;
    int meas_occupied_nodes_amt = 0;
    int irradiated_nodes_amt = 0;
    int gtruth_occupied_nodes_amt = 0;

    auto gtruth_clamp_threshold = gtruth_octree->getClampingThresMax();

    std::vector<OcTreeNode*> irradiated_nodes;

    for (auto it = meas_octree->begin_leafs(); it != meas_octree->end_leafs(); ++it) {
        if (it->isDoseSet()) {
            if (isInBounds(it.getCoordinate(), x_min, x_max, y_min, y_max, z_min, z_max)) {
                ++meas_occupied_nodes_amt;

                // Only process the node when the dose is sufficient
                if (it->getDose() >= dose_threshold) {
                    // Count sufficiently irradiated node
                    ++irradiated_nodes_amt;

                    // Look for the measured node in the ground truth octree
                    OcTreeNode* node = gtruth_octree->search(it.getKey());

                    // If the position of the measured node matches a node in the ground truth octree
                    if (node && compareDouble(node->getOccupancy(), gtruth_clamp_threshold)) {
                        irradiated_nodes.push_back(node);
                        ++match_amt;
                    }
                }
            }
        }
    }

    std::vector<OcTreeNode*> out_of_bounds_nodes;

    // Count all occupied nodes in the ground truth octree
    for (OcTree::leaf_iterator it = gtruth_octree->begin_leafs(); it != gtruth_octree->end_leafs(); ++it)
        if (compareDouble(it->getOccupancy(), gtruth_clamp_threshold))
            if (isInBounds(it.getCoordinate(), x_min, x_max, y_min, y_max, z_min, z_max))
                ++gtruth_occupied_nodes_amt;
            else
                out_of_bounds_nodes.push_back(gtruth_octree->search(it.getKey()));

    double coverage = (double) match_amt * 100 / (double) gtruth_occupied_nodes_amt;

    ROS_INFO("Measured octree has %d irradiated nodes in total.", meas_occupied_nodes_amt);
    ROS_INFO("Measured octree has %d sufficiently irradiated nodes.", irradiated_nodes_amt);
    ROS_INFO("Measured octree has %d insufficiently irradiated nodes.", meas_occupied_nodes_amt - irradiated_nodes_amt);
    ROS_INFO("Got %d matches out of %d ground truth occupied nodes.", match_amt, gtruth_occupied_nodes_amt);
    ROS_INFO("Coverage: %.2lf percent.", coverage);

    if (save_difference_map) {
        std::string difference_map_save_path;
        n.getParam("difference_map_save_path", difference_map_save_path);

        ROS_INFO("Saving difference map to %s.", difference_map_save_path.c_str());

        // Delete the sufficiently irradiated nodes from the ground truth octree
        // -6.90 log-odds ~ 0.001 probability
        for (auto node : irradiated_nodes)
            node->setLogOdds(-6.90);

        for (auto node : out_of_bounds_nodes)
            node->setLogOdds(-6.90);

        gtruth_octree->updateInnerOccupancy();

        if(gtruth_octree->writeBinary(difference_map_save_path))
            ROS_INFO("Difference map saved successfully.");
        else
            ROS_INFO("Save failed.");
    }

    // Clear the memory occupied by the octrees
    delete meas_octree;
    delete gtruth_octree;

    return 0;
}