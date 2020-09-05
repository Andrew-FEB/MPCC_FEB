#pragma once

#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <memory>

#include "definitions.h"
#include "visualisation.h"
#include "boundaryLogger.h"
#include "freeFunctions.h"

int constexpr MAX_CHILDREN = 4; //Constant indicating number of possible children for any node

struct tNode
{
    Coord pos;  //node central point
    bool checked{false};    //boolean to mark if node has been checked to find its possible children
    tNode *parent {nullptr};    //pointer to parent for this node
    std::vector<tNode *> children;  //pointers to children of this node
    int pathsDerived{0};    //for path extraction, iterates to indicate the number of path branches that have been extracted including this node (should be at most one for each of its children)
    tNode()
    {
        children.resize(MAX_CHILDREN);
    }
};

class Tree
{

    public:
        Tree(std::shared_ptr<Visualisation> visualisation_cont, int est_node_size);
        ~Tree() = default;
        //Add node to tree with coordinates for this point and its parent point if available
        void addNode(const Coord &point, const Coord &parent = {std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()});
        //Returns coordinate of a node in tree that hasn't been explored to find possible children
        const Coord *getNextUnexploredPoint();  
        //Returns a vector of vector of all paths contained in the tree up to the given length (so if length = 5, all paths with 2, 3, 4 or 5 nodes in them)
        std::vector<std::vector<Coord>> getPathsOfLength(int length);
        //Triggers output of tree to ros visualisation
        void visualiseTree();
        const std::vector<tNode> &getStorageList() const;
        const int &getTreeSize() const;
        

    private:
        #ifdef VISUALISE
        std::shared_ptr<Visualisation> visualisation{ nullptr };
        #endif
        #ifdef DEBUG
        std::unique_ptr<BoundaryLogger> log_add_node{nullptr};
        std::unique_ptr<BoundaryLogger> log_next_point{nullptr};
        std::unique_ptr<BoundaryLogger>  log_get_paths;
        #endif

        //Returns pointer to node containing given coordinate. If no node found, nullptr returned
        tNode * findNode(const Coord &point);
        //returns true if node with given coordinate AND parent coordinate already exists equal to arguments
        bool nodeAlreadyExists(const Coord &point, const Coord &parent);
        
        tNode *head{nullptr};
        std::vector<tNode> storageList; //Vector containing nodes. Vector chosen despite O(n) searches as size of tree usually small enough to hold whole vector in cache memory
        int treeSize{0};
};

std::ostream &operator<<(std::ostream& os, std::vector<tNode> &storageList);

