#include "tree.h"

Tree::Tree(std::shared_ptr<Visualisation> visualise_cont, int est_node_size)
{
#ifdef VISUALISE
	if (visualise_cont!=nullptr)
	{
		visualise = visualise_cont;
	}
	else std::cerr << "ERROR: Visualise pre-processor statement defined but nullptr Visualise object passed to Triangulation object";
#endif

storageList.reserve(est_node_size);
}

Tree::~Tree()
{

}

const coord *Tree::getNextUnexploredPoint()
{
    for (std::vector<tNode>::iterator it = storageList.begin(); it!=storageList.end(); it++)
    {
        if (!(it->checked))
        {
            it->checked = true;
            return &(it->midPoint);
        }
    }
    return nullptr;
}
    
void Tree::addNode(const coord point, const coord parent, double bestFirstDist)
{
    //TEST
    std::cerr<<"Attempting to add node"<<std::endl;
    std::cerr<<"Node midpoint is x("<<point.x<<"), y("<<point.y<<")"<<std::endl;
    std::cerr<<"Parent midpoint is x("<<parent.x<<"), y("<<parent.y<<")"<<std::endl;
    //ETEST
    tNode node;
    if (head == nullptr)    //First node
    {
        head = &(storageList.back());
    }
    else    //Node other than first
    {
        if (nodeAlreadyExists(point, parent)) return;
        //Find existing stored parent node
        auto pNode = findNode(parent);
        if (pNode == nullptr) //Parent node not found - error.
        {
            std::cerr<<"ERROR - Failed to find parent in addNode() of tree.cpp"<<std::endl;
            return;
        }
        else node.parent = pNode;    //Parent found

        //TEST
        std::cerr<<"What was found in parent search had x("<<pNode->midPoint.x<<"), y("<<pNode->midPoint.y<<")"<<std::endl;
        std::cerr<<"What was linked in parent search had x("<<node.parent->midPoint.x<<"), y("<<node.parent->midPoint.y<<")"<<std::endl;
        //ETEST

        if (node.parent->left==nullptr) //first branch
        {
            node.parent->left = &node; 
        } 
        else if (node.parent->right == nullptr) //second branch
        {
            node.parent->right = &node;
        }
        else 
        {
            std::cerr<<"ERROR - parent node selected that had already set all pointers in addNode() of tree.cpp"<<std::endl;
            return;
        }
    }
    node.midPoint = point;
    node.bestFirstDist = bestFirstDist;
    treeSize++;
    storageList.push_back(node);
    //TEST
    if (storageList.back().parent !=nullptr)
    {
    std::cerr<<"What was place in vector had x("<<storageList.back().parent->midPoint.x<<"), y("<<storageList.back().parent->midPoint.y<<")"<<std::endl;
    }
    //ETEST
    //sortVecBestFirst(); //Maybe not needed
}

void Tree::visualiseTree()
{
#ifdef VISUALISE
    std::vector<coord> nodeCoords;
    std::vector<std::pair<coord, coord>> nodeParentLinks;
    for (auto node : storageList)
    {
        nodeCoords.push_back(node.midPoint);
        if (node.parent!=nullptr)
        {
            nodeParentLinks.push_back(std::make_pair(node.midPoint, node.parent->midPoint));
        }
    }
	visualise->showNodeMids(nodeCoords);
    visualise->showNodeParentLinks(nodeParentLinks);
#else
    return;
#endif
}

std::ostream& operator<<(std::ostream& os, Tree &tree)
{
    os<<"Tree of size: "<<tree.treeSize<<"."<<std::endl<<"Contains: "<<std::endl;
    for (const tNode &node : tree.storageList)
    {
        os<<"Node with position: "<<node.midPoint.x<<", "<<node.midPoint.y<<" and a parent with midpoint at: ";
        if (node.parent!=nullptr) os<<node.parent->midPoint.x<<", "<<node.parent->midPoint.y;
        else os<<"Parent is nullptr!";
        os<<std::endl;
    }

}

void Tree::sortVecBestFirst()
{
    std::sort(storageList.begin(), storageList.end(), [](const tNode &a, const tNode &b)
    {
        return (a.bestFirstDist < b.bestFirstDist);
    });
}

tNode *Tree::findNode(const coord &point)
{
    auto node = std::find_if(storageList.begin(), storageList.end(), [&point] (const tNode &n) 
    {
        return (point.x==n.midPoint.x && point.y == n.midPoint.y);
    });
    return (node!=storageList.end()) ? &(*node) : nullptr; 
}

bool Tree::nodeAlreadyExists(const coord &point, const coord &parent)
{
    auto node = std::find_if(storageList.begin(), storageList.end(), [&point, &parent] (const tNode &n) 
    {
        bool same {true};
        if (!(point.x==n.midPoint.x && point.y == n.midPoint.y)) same = false;
        if (n.parent!=nullptr)
        {
            if (!(parent.x==n.parent->midPoint.x && parent.y == n.parent->midPoint.y)) same = false;
        }
        return same;
    });
    return (node!=storageList.end());
}

std::vector<std::vector<coord>> Tree::getPathsOfGivenLength(int length)
{
    if (head==nullptr)
    {
        std::cerr<<"ERROR - request for paths while tree is empty!";
        return {};
    }
    if (length<2)
    {
        return {};
    }
    int expectedNumPaths = pow(2, length);
    std::vector<std::vector<coord>> collection;
    std::vector<coord> path;
    tNode *nodeP {nullptr};
    for (int pathsCollected = 0; pathsCollected<expectedNumPaths; pathsCollected++)
    {
        nodeP = head;
        path.push_back(nodeP->midPoint); //Get root position
        for (unsigned int step = (expectedNumPaths>>1); step>0; step>>1)
        {
            if (step&pathsCollected)
            { 
                nodeP = nodeP->right;
            }
            else
            {
                nodeP = nodeP->left;
            }
            if (nodeP!=nullptr)path.push_back(nodeP->midPoint);
            else break;
        }
        // 0 lll 000
        // 1 llr 001
        // 2 lrl 010
        // 3 lrr 011
        // 4 rll 100
        // 5 rlr 101
        // 6 rrl 110
        // 7 rrr 111
        collection.push_back(path);
    }
    return collection;
}

static inline bool intIsOdd(int x)
{
    return (x%2);   //Compiler should adjust to simple &1 operation, but be compilable on more systems
}

int Tree::size()
{
    return treeSize;
}

