/** @file OcTreeDose.hpp
 * @brief Ultraviolet-C dose voxels octree definition
 * 
 * @author Tito Irfan (titoirfan)
 * @version 1.0 21/3/2021
 */

#ifndef OCTOMAP_OCTREE_DOSE_H
#define OCTOMAP_OCTREE_DOSE_H

#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>

namespace octomap {

class OcTreeNodeDose: public OcTreeNode {
  public:
    // Needed to access inherited node children
    friend class OcTreeDose;

    // Default constructor
    OcTreeNodeDose(): OcTreeNode(), dose(0) {}

    // Copy constructor
    OcTreeNodeDose(const OcTreeNodeDose& o): OcTreeNode(o), dose(o.dose) {};

    /** Comparison operator.
     * 
     * @return true if both occupancy values and dose data are equal
     */
    bool operator==(const OcTreeNodeDose& o) const{
      return (o.value == value && o.dose == dose);
    }

    /** copyData. Copy and assign another node's data to self.
     * 
     * @result from's data is copied to self
     */
    void copyData(const OcTreeNodeDose& from){
      OcTreeNode::copyData(from);
      dose = from.getDose();
    }

    // Dose getter
    inline float getDose() const { return dose; }
    inline void setDose(float d) { dose = d; }

    inline bool isDoseSet() const { return dose != 0; }

    void updateDoseChildren();

    float getAverageChildDose() const;

    float getSumChildDose() const;

    // File I/O
    std::istream& readData(std::istream &s);
    std::ostream& writeData(std::ostream &s) const;

  protected:
    // Dose data
    float dose;
};

class OcTreeDose: public OccupancyOcTreeBase<OcTreeNodeDose> {
  public:
    // Default constructor, sets resolution of leafs
    OcTreeDose(double resolution);

    // Virtual constructor
    OcTreeDose* create() const { return new OcTreeDose(resolution); }

    std::string getTreeType() const { return "OcTreeDose"; }

    /** pruneNode. Prunes a node when it is collapsible. 
     * This overloaded version only considers the node 
     * occupancy for pruning, different doses of child 
     * nodes are ignored.
     * 
     * @param[in] node The node to be pruned.
     * @return true if pruning was successful.
     */
    virtual bool pruneNode(OcTreeNodeDose* node);

    virtual void expandNode(OcTreeNodeDose* node);
    
    virtual bool isNodeCollapsible(const OcTreeNodeDose* node) const;
       
    // Set node dose at given key or coordinate. Replaces previous dose.
    OcTreeNodeDose* setNodeDose(const OcTreeKey& key, float dose);

    OcTreeNodeDose* setNodeDose(float x, float y, 
      float z, float dose) {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
      return setNodeDose(key,dose);
    }

    // Update node dose
    OcTreeNodeDose* incrementNodeDose(const OcTreeKey& key, float dose);
    
    OcTreeNodeDose* incrementNodeDose(float x, float y, 
      float z, float dose) {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
      return incrementNodeDose(key,dose);
    }

    float getMaxDose(); 

    // update inner nodes, sets color to average child dose
    void updateInnerOccupancy();

  protected:
    void updateInnerOccupancyRecurs(OcTreeNodeDose* node, unsigned int depth);

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once. You need this as a
     * static member in any derived octree class in order to read .ot
     * files through the AbstractOcTree factory. You should also call
     * ensureLinking() once from the constructor.
     */
    class StaticMemberInitializer{
    public:
      StaticMemberInitializer() {
        OcTreeDose* tree = new OcTreeDose(0.1);
        tree->clearKeyRays();
        AbstractOcTree::registerTreeType(tree);
      }

      /**
      * Dummy function to ensure that MSVC does not drop the
      * StaticMemberInitializer, causing this tree failing to register.
      * Needs to be called from the constructor of this octree.
      */
      void ensureLinking() {};
    };
    /// to ensure static initialization (only once)
    static StaticMemberInitializer ocTreeDoseMemberInit;
};

} // end namespace

#endif
