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

int constexpr MAX_CHILDREN = 4;
double constexpr MAX_DIST_TO_END_GOAL = 10;

struct tNode
{
    coord pos;
    bool checked{false};
    double bestFirstDist{std::numeric_limits<double>::max()};
    tNode *parent {nullptr};
    std::vector<tNode *> children;
    int pathsDerived{0};
    tNode()
    {
        children.resize(4);
    }
};

class Tree
{

    public:
        Tree(std::shared_ptr<Visualisation> visualisation_cont, int est_node_size);
        ~Tree() = default;
        void addNode(const coord &point, double bestFirstDist, const coord &parent = {std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()});
        const coord *getNextUnexploredPoint();  //Element one is vector, element two is error code
        std::vector<std::vector<coord>> getPathsOfLength(int length, const coord &section_end);
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

        tNode * findNode(const coord &point);
        bool nodeAlreadyExists(const coord &point, const coord &parent);
        friend double distBetweenPoints(const coord &a, const coord &b);    //implemented in triangulation.cpp
        void sortVecBestFirst();
        bool viablePathToEnd(const std::vector<coord> &path, const coord &section_end);
        
        tNode *head{nullptr};
        std::vector<tNode> storageList;
        int treeSize{0};
};

std::ostream &operator<<(std::ostream& os, std::vector<tNode> &storageList);

