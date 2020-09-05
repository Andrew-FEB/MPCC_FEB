#include "tree.h"
#include "boundGlobals.h"

Tree::Tree(std::shared_ptr<Visualisation> visualisation_cont, int est_node_size)
{
#ifdef VISUALISE
	if (visualisation_cont!=nullptr)
	{
		visualisation = visualisation_cont;
	}
	else std::cerr << "ERROR: Visualise pre-processor statement defined but nullptr Visualisation object passed to Tree object"<<std::endl;
#endif
#ifdef DEBUG
log_add_node = std::make_unique<BoundaryLogger>("DEBUG_TREE_ADDNODE", "Tree addNode()", reset_logs);
log_next_point = std::make_unique<BoundaryLogger>("DEBUG_TREE_GETNEXTPOINT", "Tree getNextUnexploredPoint()", reset_logs);
log_get_paths = std::make_unique<BoundaryLogger>("DEBUG_TREE_GETPATHSOFLENGTH", "Tree getPathsOfLength()", reset_logs);
#endif
storageList.reserve(est_node_size);
}

const Coord *Tree::getNextUnexploredPoint()
{
    #ifdef DEBUG
    std::stringstream ss;
	log_next_point->write(ss<<"Next Unexplored point requested");
    log_next_point->write(ss<<"Tree at this point has size: "<<getTreeSize());
    for (const tNode &node : storageList)
    {
        ss<<"Node with position: "<<node.pos.x<<", "<<node.pos.y<<", a checked status of "<<node.checked<<" and a parent with pos at: ";
        if (node.parent!=nullptr) ss<<node.parent->pos.x<<", "<<node.parent->pos.y;
        else ss<<"Parent is nullptr!";
        ss<<std::endl;
    }
    log_next_point->write(ss);
    #endif
    for (std::vector<tNode>::iterator it = storageList.begin(); it!=storageList.end(); it++)
    {
        if (!(it->checked))
        {
            #ifdef DEBUG
            log_next_point->write(ss<<"Next point selected to be checked has position x("<<it->pos.x<<"), y("<<it->pos.y<<")", true);
            #endif          
            it->checked = true;
            return &(it->pos);
        }
    }
    #ifdef DEBUG
    log_next_point->write(ss<<"No valid points found and nullptr returned", true);
    #endif    
    return nullptr;
}
    
void Tree::addNode(const Coord &point, const Coord &parent)
{
    #ifdef DEBUG
    std::stringstream ss;
	log_add_node->write(ss<<"Trying to add node:");
    log_add_node->write(ss<<"Node position x("<<point.x<<"), y("<<point.y<<")");
    log_add_node->write(ss<<"Node parent should have position x("<<parent.x<<"), y("<<parent.y<<")");
    #endif
    tNode node;
    if (head == nullptr)    //First node of already searched location
    {
        #ifdef DEBUG
            log_add_node->write(ss<<"Node placed in head position");
        #endif
        node.checked = true;    //Initial node has already been explored
        node.pos = point;
        treeSize++;
        storageList.push_back(node);
        head = &(storageList.back());
    }
    else    //Node other than first
    {
        if (nodeAlreadyExists(point, parent)) 
        {
            #ifdef DEBUG
            log_add_node->write(ss<<"Node was already found in tree. Point skipped", true);
            #endif
            return;
        }
        //Find existing stored parent node
        #ifdef DEBUG
        log_add_node->write(ss<<"Attempting to find parent...");
        #endif
        auto pNode = findNode(parent);
        if (pNode == nullptr) //Parent node not found - error.
        {
            #ifdef DEBUG
            log_add_node->write(ss<<"Failed to find parent in tree", true);
            #endif
            return;
        }
        else node.parent = pNode;    //Parent found
        #ifdef DEBUG
        log_add_node->write(ss<<"Parent found with position x("<<pNode->pos.x<<"), y("<<pNode->pos.y<<")");
        if (pNode->parent != nullptr)   log_add_node->write(ss<<"Parent's own parent has position x("<<pNode->parent->pos.x<<"), y("<<pNode->parent->pos.y<<")");
        #endif
        node.pos = point;
        treeSize++;
        storageList.push_back(node);

        bool parent_pointer_linked{false};
        for (int i = 0; i<=MAX_CHILDREN; i++)
        {
            if (node.parent->children[i] == nullptr)
            {
                node.parent->children[i] = &storageList.back();
                parent_pointer_linked = true;
                break;
                #ifdef DEBUG
                log_add_node->write(ss<<"Node linked to parent child pointer "<<i, true);
                #endif
            }
        }
        if (!parent_pointer_linked) 
        {
            #ifdef DEBUG
            log_add_node->write(ss<<"Parent node's pointers all used", true);
            #endif
            return;
        }
    }
    #ifdef DEBUG
    log_add_node->write(ss<<"Node addition operation complete", true);
    #endif
}

void Tree::visualiseTree()
{
#ifdef VISUALISE
    std::vector<Coord> nodeCoords;
    std::vector<std::pair<Coord, Coord>> nodeParentLinks;
    for (auto node : storageList)
    {
        nodeCoords.push_back(node.pos);
        if (node.parent!=nullptr)
        {
            nodeParentLinks.push_back(std::make_pair(node.pos, node.parent->pos));
        }
    }
	visualisation->showNodeMids(nodeCoords);
#else
    return;
#endif
}

tNode *Tree::findNode(const Coord &point)
{
    auto node = std::find_if(storageList.begin(), storageList.end(), [&point] (const tNode &n) 
    {
        return (point.x==n.pos.x && point.y == n.pos.y);
    });
    return (node!=storageList.end()) ? &(*node) : nullptr; 
}

bool Tree::nodeAlreadyExists(const Coord &point, const Coord &parent)
{
    auto node = std::find_if(storageList.begin(), storageList.end(), [&point, &parent] (const tNode &n) 
    {
        bool same {true};
        if (!(point.x==n.pos.x && point.y == n.pos.y)) same = false;
        if (n.parent!=nullptr)
        {
            if (!(parent.x==n.parent->pos.x && parent.y == n.parent->pos.y)) same = false;
        }
        return same;
    });
    return (node!=storageList.end());
}

std::vector<std::vector<Coord>> Tree::getPathsOfLength(int length)
{

    #ifdef DEBUG
    std::stringstream ss;
	log_get_paths->write(ss<<"Request made for paths of length "<<length);
    #endif
    if (head==nullptr)
    {
        #ifdef DEBUG
        log_get_paths->write(ss<<"Error has occurred! Paths have been requested from an empty tree. Empty vector returned", true);
        #endif
        std::cerr<<"ERROR - empty tree but paths requested. Returning empty vector"<<std::endl;
        return {};
    }
    if (length<2)
    {
        #ifdef DEBUG
        log_get_paths->write(ss<<"Paths of length 1 requested. Empty vector returned", true);
        #endif
        return {};
    }
    std::vector<std::vector<Coord>> paths;
    std::vector<Coord> path;

    tNode *node_p {head};    //moving pointer to indicate currently checked node

    #ifdef DEBUG
    log_get_paths->write(ss<<"Path collection begun...");
    #endif

    int current_tree_depth {1};
    bool paths_collected{false};    //While paths available
    while (!paths_collected)
    {
        #ifdef DEBUG
        log_get_paths->write(ss<<"Currently pointing at node with position x("<<node_p->pos.x<<"), y("<<node_p->pos.y<<")");
        #endif
        //If current node has not been added to path
        if (node_p->pathsDerived==0)
        {
            path.push_back(node_p->pos);
            #ifdef DEBUG
            log_get_paths->write(ss<<"Path collected of length "<<path.size()<<" containing:");
            int debug_coord_count{0};
            for (auto point : path)
            {
                ss<<"Coordinate "<<++debug_coord_count<<" with position x("<<point.x<<"), y("<<point.y<<")"<<std::endl;
            }
            log_get_paths->write(ss);
            #endif
            paths.push_back(path);
        }
        node_p->pathsDerived++;
        
        //If children list for current node has been exhausted
        if ((node_p->pathsDerived-1)>=MAX_CHILDREN || current_tree_depth>=length)
        {
            #ifdef DEBUG
            log_get_paths->write(ss<<"Children used up or at end of tree. Shifting backwards");
            #endif
            if (node_p == head)
            {
                #ifdef DEBUG
                log_get_paths->write(ss<<"Head node detected to have been completed. Tree returning paths collection, containing "<<paths.size()<<" paths", true);
                #endif
                paths_collected = true;
                break;
            }
            else
            { 
                node_p = node_p->parent;
                path.pop_back();
                current_tree_depth--;
                #ifdef DEBUG
                log_get_paths->write(ss<<"Moving back up tree to new tree depth of "<<current_tree_depth, true);
                #endif
            }  
        }
        //Still useful nodes left in child list
        else if (node_p->children[node_p->pathsDerived-1] != nullptr)
        {
            node_p = node_p->children[node_p->pathsDerived-1];
            current_tree_depth++;
            #ifdef DEBUG
            log_get_paths->write(ss<<"Moving down tree to new tree depth of "<<current_tree_depth, true);
            #endif
        } 
        //nullptr found
        else
        {
            #ifdef DEBUG
            log_get_paths->write(ss<<"No node at this children pointer. Ending search at this node", true);
            #endif
            node_p->pathsDerived = MAX_CHILDREN;
        }
    }

    return paths;
}

const std::vector<tNode> &Tree::getStorageList() const
{
    return storageList;
}

const int &Tree::getTreeSize() const
{
    return treeSize;
}

std::ostream& operator<<(std::ostream& os, std::vector<tNode> &storageList)
{
    os<<"Tree contains: "<<std::endl;
    for (const tNode &node : storageList)
    {
        os<<"Node with position: "<<node.pos.x<<", "<<node.pos.y<<", a checked status of "<<node.checked<<" and a parent with pos at: ";
        if (node.parent!=nullptr) os<<node.parent->pos.x<<", "<<node.parent->pos.y;
        else os<<"Parent is nullptr!";
        os<<std::endl;
    }
}



