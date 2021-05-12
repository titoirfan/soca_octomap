/** @file coverage_eval_node.cpp
 * @brief Compare an octomap against a ground truth octomap
 *  to assess its coverage
 * 
 * @author Tito Irfan (titoirfan)
 * @version 1.0 20/3/2021
 */

#include <ros/ros.h>

#include <octomap/octomap.h>

#include <string.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <cmath>

using namespace octomap;

int main(int argc, char **argv) {
    // Node initialization
    ros::init(argc, argv, "coverage_eval_node");

    ros::NodeHandle n("~");

    // Measurement octree filename
    std::string meas_octree_filename;
    // Ground truth octree filename
    std::string gtruth_octree_filename;    

    n.getParam("measurement_octree_filename", meas_octree_filename);
    n.getParam("ground_truth_octree_filename", gtruth_octree_filename);

    // Read the octrees from the files
    ROS_INFO("Reading octree files.");

    OcTree* meas_octree = new OcTree(meas_octree_filename);
    OcTree* gtruth_octree = new OcTree(gtruth_octree_filename);

    // Validate the octrees' resolution
    ROS_INFO("Measured octree resolution: %lf", meas_octree->getResolution());
    ROS_INFO("Ground truth octree resolution: %lf", gtruth_octree->getResolution());
    if (fabs(meas_octree->getResolution() - gtruth_octree->getResolution()) > 1e-6) {
        ROS_FATAL("Tree resolution doesn't match!");
    }

    // Expand the previously pruned octrees to full resolution
    ROS_INFO("Expanding octrees.");

    meas_octree->expand();
    gtruth_octree->expand();

    // Validate the octrees' leaf node amount 
    if (meas_octree->getNumLeafNodes() != gtruth_octree->getNumLeafNodes()) {
      ROS_FATAL("Octrees have different size: %zu != %zu", 
        meas_octree->getNumLeafNodes(), 
        gtruth_octree->getNumLeafNodes());
    }

    ROS_INFO("Expanded octrees' leaf amount: %zu", meas_octree->getNumLeafNodes());

    // Compare the trees
    ROS_INFO("Comparing trees.");

    // Count the matching nodes amount
    int match_amt = 0;
    int meas_occupied_nodes_amt = 0;
    int gtruth_occupied_nodes_amt = 0;

    for (OcTree::leaf_iterator it = meas_octree->begin_leafs(); it != meas_octree->end_leafs(); ++it)
        if (it->getOccupancy() == meas_octree->getClampingThresMax()) {
            // Count measured occupied node
            ++meas_occupied_nodes_amt;

            // Look for the measured node in the ground truth octree
            OcTreeNode* node = gtruth_octree->search(it.getKey());

            // If the position of the measured node matches a node in the ground truth octree
            if (node)
                ++match_amt;
        }

    // Count all occupied nodes in the ground truth octree
    for (OcTree::leaf_iterator it = gtruth_octree->begin_leafs(); it != gtruth_octree->end_leafs(); ++it)
        if (it->getOccupancy() == gtruth_octree->getClampingThresMax())
            ++gtruth_occupied_nodes_amt;

    ROS_INFO("Measured octree has %d occupied nodes.", meas_occupied_nodes_amt);
    ROS_INFO("Got %d matches out of %d ground truth occupied nodes.", match_amt, gtruth_occupied_nodes_amt);

    double coverage = match_amt * 100 / gtruth_occupied_nodes_amt;

    ROS_INFO("Coverage: %.2lf percent.", coverage);

    // Clear the memory occupied by the octrees
    delete meas_octree;
    delete gtruth_octree;

    return 0;
}