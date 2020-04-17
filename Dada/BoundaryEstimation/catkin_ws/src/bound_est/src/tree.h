#pragma once

#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <memory>

#include "definitions.h"
#include "visualisation.h"

typedef struct treeNode
{
    coord midPoint;
    bool checked{false};
    int pathsDerived{0};
    double bestFirstDist{std::numeric_limits<double>::max()};
    treeNode *parent {nullptr};
    treeNode *left {nullptr};
    treeNode *right {nullptr};
}tNode;

class Tree
{

    public:
        Tree(std::shared_ptr<Visualisation> visualise_cont, int est_node_size);
        ~Tree();
        void addNode(const coord point, const coord parent, double bestFirstDist);
        const coord *getNextUnexploredPoint();  //Element one is vector, element two is error code
        std::vector<std::vector<coord>> getPathsOfGivenLength(int length);
        friend std::ostream& operator<<(std::ostream& os, Tree &tree);
        void visualiseTree();
        int size();

    private:
        std::shared_ptr<Visualisation> visualise{ nullptr };
        tNode * findNode(const coord &point);
        bool nodeAlreadyExists(const coord &point, const coord &parent);
        friend double distBetweenPoints(const coord &a, const coord &b);    //implemented in triangulation.cpp
        void sortVecBestFirst();
        
        tNode *head{nullptr};
        std::vector<tNode> storageList;
        int treeSize{0};
};

