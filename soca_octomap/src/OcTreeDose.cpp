/** @file OcTreeDose.cpp
 * @brief Ultraviolet-C dose voxels octree implementation
 * 
 * @author Tito Irfan (titoirfan)
 * @version 1.0 23/3/2021
 */

#include <soca_octomap/OcTreeDose.hpp>

namespace octomap {

std::ostream& OcTreeNodeDose::writeData(std::ostream &s) const {
    // Occupancy
    s.write((const char*) &value, sizeof(value));
    // Dose
    s.write((const char*) &dose, sizeof(dose));

    return s;
}

std::istream& OcTreeNodeDose::readData(std::istream &s) {
    // Occupancy
    s.read((char*) &value, sizeof(value));
    // Dose
    s.read((char*) &dose, sizeof(dose));

    return s;
}

float OcTreeNodeDose::getAverageChildDose() const {
    float sum = 0;
    float n = 0;

    if (children != NULL)
        for (unsigned int i = 0; i < 8; ++i) {
            OcTreeNodeDose* child = static_cast<OcTreeNodeDose*>(children[i]);

            if (child != NULL & child->isDoseSet()) {
                sum += child->getDose();
                ++n;
            }
        }
    
    if (n > 0)
        return sum / n;
    else
        return 0;
}

float OcTreeNodeDose::getSumChildDose() const {
    float sum = 0;

    if (children != NULL)
        for (unsigned int i = 0; i < 8; ++i) {
            OcTreeNodeDose* child = static_cast<OcTreeNodeDose*>(children[i]);

            if (child != NULL && child->isDoseSet())
                sum += child->getDose();
        }
    
    return sum;
}

void OcTreeNodeDose::updateDoseChildren() {
    dose = getSumChildDose();
}

OcTreeDose::OcTreeDose(double resolution): 
    OccupancyOcTreeBase<OcTreeNodeDose>(resolution) {
    ocTreeDoseMemberInit.ensureLinking();
}

OcTreeNodeDose* OcTreeDose::setNodeDose(const OcTreeKey& key, float dose) {
    OcTreeNodeDose* n = search(key);

    if (n)
        n->setDose(dose);
    
    return n;
}

bool OcTreeDose::pruneNode(OcTreeNodeDose* node) {
    if (!isNodeCollapsible(node))
        return false;

    // Set value to children's value
    node->copyData(*(getNodeChild(node, 0)));

    if (node->isDoseSet())
        node->setDose(node->getSumChildDose());

    // Delete children
    for (unsigned int i = 0; i < 8; ++i)
        deleteNodeChild(node, i);

    delete[] node->children;
    node->children = NULL;

    return true;
}

void OcTreeDose::expandNode(OcTreeNodeDose* node){
    assert(!nodeHasChildren(node));
    float node_dose = node->getDose();

    for (unsigned int k=0; k<8; k++) {
        OcTreeNodeDose* newNode = createNodeChild(node, k);
        newNode->copyData(*node);
        newNode->setDose(node_dose / 8);
    }
}

bool OcTreeDose::isNodeCollapsible(const OcTreeNodeDose* node) const {
    // If the node doesn't have children 
    if (!nodeChildExists(node, 0))
        return false;

    // If the child is not a leaf node
    const OcTreeNodeDose* first_child = getNodeChild(node, 0);
    if (nodeHasChildren(first_child))
        return false;

    // Ignore dose for pruning
    // Check whether there are any node of different occupancy
    for (unsigned int i = 0; i < 8; ++i)
        if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i))
            || !(getNodeChild(node, i)->getValue() == first_child->getValue()))
            return false;
    
    return true;
}

OcTreeNodeDose* OcTreeDose::incrementNodeDose(const OcTreeKey& key, float dose) {
    OcTreeNodeDose* n = search(key);

    if (n)
        if (n->isDoseSet()) {
            float prev_dose = n->getDose();
            n->setDose(prev_dose + dose);
        }
        else
            n->setDose(dose);

    return n;
}

void OcTreeDose::updateInnerOccupancy() {
    this->updateInnerOccupancyRecurs(this->root, 0);
}

void OcTreeDose::updateInnerOccupancyRecurs(OcTreeNodeDose* node, unsigned int depth) {
    // Only recurse and update inner nodes
    if (nodeHasChildren(node)) {
        if (depth < this->tree_depth)
            for (unsigned int i = 0; i < 8; ++i)
                if (nodeChildExists(node, i))
                    updateInnerOccupancyRecurs(getNodeChild(node, i), depth + 1);
    
        node->updateOccupancyChildren();
        node->updateDoseChildren();
    }
}

float OcTreeDose::getMaxDose() {
    float max = 0.0;

    for (auto it = this->begin_leafs(); it != this->end_leafs(); ++it)
        if (it->getDose() > max)
            max = it->getDose();
    
    return max;
}

OcTreeDose::StaticMemberInitializer OcTreeDose::ocTreeDoseMemberInit;

}