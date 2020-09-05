#include "triangulation.h"
#include "include/delaunay/delaunator.hpp"
#include "boundGlobals.h"

Triangulation::Triangulation(std::shared_ptr<Visualisation> visualisation_cont)
{
	#ifdef VISUALISE
	if (visualisation_cont!=nullptr)
	{
		visualisation = visualisation_cont;
	}
	else std::cerr << "ERROR: Visualise pre-processor statement defined but nullptr Visualisation object passed to Triangulation object";
	#endif
}

std::vector<std::vector<Coord>> Triangulation::getTraversingPaths(const std::vector<std::unique_ptr<Cone>>& cone_list, const std::pair<std::vector<const Cone *>, std::vector<const Cone *>> &seperated_cone_lists, const Coord &entry_point, const Coord &exit_point, int max_path_length)
{
	//Apply triangulation on cone points
	std::vector<Triang> triangle_list = findTrianglePoints(cone_list, entry_point);

	//Connect cone positions to triangle values for easier processing later
	placeConesInTriangle(triangle_list, cone_list);

	#ifdef VISUALISE
		if (triangle_list.size()>0) visualisation->showTriangles(triangle_list);
		visualisation->showEndPoint(exit_point);
	#endif

	//Find triangle midpoint in section closest to entry_pont
	auto localTriangles = findTrianglesAroundPoint(false, triangle_list, entry_point);
	//Find valid target point around section start
	auto first = findFirstMidpoint(localTriangles, entry_point);	

	//Collect all possible paths in search space
	auto paths = findViablePaths(first, exit_point, triangle_list, max_path_length);
	return paths;
}

std::vector<Triang> Triangulation::findTrianglesAroundPoint(bool is_midpoint, const std::vector<Triang> &triangle_list, const Coord &point)
{
	#ifdef DEBUG
    std::unique_ptr<BoundaryLogger> log = std::make_unique<BoundaryLogger>("DEBUG_LOCALTRIANGLES", "findTrianglesAroundPoint()", reset_logs);
    std::stringstream ss;
	log->write(ss<<"At time of entering function key variables were:");
	log->write(ss<<"Value derived from car position: "<<is_midpoint);
	log->write(ss<<"Origin point x("<<point.x<<"), y("<<point.y<<")");
	log->write(ss<<"Triangle list size = "<<triangle_list.size()); 
	#endif
	std::vector<Triang> list;
	std::vector<Triang>::const_iterator it = triangle_list.begin();
	if (is_midpoint)
	{
		while (it!=triangle_list.end())
		{
			#ifdef DEBUG
			log->write(ss<<"Checking triangle with points a("<<it->a.x<<","<<it->a.y<<"), b("<<it->b.x<<","<<it->b.y<<"), c("<<it->c.x<<","<<it->c.y<<")");
			#endif
			if (it->onVertices(point))
			{
				#ifdef DEBUG
				log->write(ss<<"Triangle found on vertices and added to list");
				#endif
				list.push_back(*it);
			}
			it++;
			
		}

	}
	else
	{
		while (it!=triangle_list.end())
		{
			#ifdef DEBUG
			log->write(ss<<"Checking triangle with points a("<<it->a.x<<","<<it->a.y<<"), b("<<it->b.x<<","<<it->b.y<<"), c("<<it->c.x<<","<<it->c.y<<")" ,true);
			#endif
			if (it->touches(point))
			{
				#ifdef DEBUG
				log->write(ss<<"Triangle found on points and added to list");
				#endif
				list.push_back(*it);
			}
			it++;
			
		}
	}
	#ifdef DEBUG
	if (triangle_list.size()<1) log->write(ss<<"ERROR. Found no triangles around local point.");
	#endif
	return list;
}

Coord Triangulation::findFirstMidpoint(std::vector<Triang> &vec, const Coord &point)
{
	#ifdef DEBUG
    std::unique_ptr<BoundaryLogger> log = std::make_unique<BoundaryLogger>("DEBUG_FINDFIRSTMID", "findFirstMidpoint()", reset_logs);
    std::stringstream ss;
	log->write(ss<<"At time of entering function key variables were:");
	int vector_debug_count = 0;
	log->write(ss<<"Comparison  point x("<<point.x<<"), y("<<point.y<<")");
	log->write(ss<<"Vector of triangles of size "<<vec.size()<<" contains:");
	log->write(ss<<vec);
	#endif
	Coord first{0, 0};
	Coord second{0, 0};
	for (Triang &triangle : vec)
	{
		if (triangle.pointsAreEqual(point, triangle.a))
		{
			#ifdef DEBUG
			log->write(ss<<"Point identified as being point A on triangle "<<++vector_debug_count);
			#endif
			if (!triangle.pointsOnBoundary(triangle.b, triangle.c))
			{
				#ifdef DEBUG
				log->write(ss<<"Points B and C identified as not being on boundary, so this position selected");
				#endif
				first = triangle.b;
				second = triangle.c;
				break;
			}
		}
		else if (triangle.pointsAreEqual(point, triangle.b)) 
		{
			#ifdef DEBUG
			log->write(ss<<"Point identified as being point B on triangle "<<++vector_debug_count);
			#endif
			if (!triangle.pointsOnBoundary(triangle.a, triangle.c))
			{
				#ifdef DEBUG
				log->write(ss<<"Points A and C identified as not being on boundary, so this position selected");
				#endif
				first = triangle.a;
				second = triangle.c;
				break;
			}
		}
		else 
		{
			#ifdef DEBUG
			log->write(ss<<"Point identified as being point C on triangle "<<++vector_debug_count);
			#endif
			if (!triangle.pointsOnBoundary(triangle.a, triangle.b))
			{
				#ifdef DEBUG
				log->write(ss<<"Points A and B identified as not being on boundary, so this position selected");
				#endif
				first = triangle.a;
				second = triangle.b;
				break;
			}
		}
	}
	#ifdef DEBUG
	auto output = findMidpoint(first, second);
	log->write(ss<<"Returned coordinate has coordinates x("<<output.x<<"), y("<<output.y<<")", true);
	#endif
	return findMidpoint(first,second);
}

void Triangulation::placeConesInTriangle(std::vector<Triang> &triangle_list, const std::vector<std::unique_ptr<Cone>> &cone_list)
{
	for (Triang &triangle : triangle_list)
	{
		bool locatedA = false;
		bool locatedB = false;
		bool locatedC = false;
		for (std::vector<std::unique_ptr<Cone>>::const_reverse_iterator it = cone_list.rbegin() ; it!=cone_list.rend(); it++)
		{
			if (!locatedA)
			{
				if (triangle.a.x == (*it)->getX() && triangle.a.y == (*it)->getY())
				{
					locatedA = true;
					triangle.aPos = (*it)->getPos();
				}
			}
			if (!locatedB)
			{
				if (triangle.b.x == (*it)->getX() && triangle.b.y == (*it)->getY())
				{
					locatedB = true;
					triangle.bPos = (*it)->getPos();
				}
			}
			if (!locatedC)
			{
				if (triangle.c.x == (*it)->getX() && triangle.c.y == (*it)->getY())
				{
					locatedC = true;
					triangle.cPos = (*it)->getPos();
				}
			}
			if (locatedC && locatedB && locatedA) break;
		}
	}
}

std::vector<Triang> Triangulation::findTrianglePoints(const std::vector<std::unique_ptr<Cone>>& cone_list, const Coord &entry_point)
{
	std::vector<double> pointMap;
	pointMap.push_back(entry_point.x);
	pointMap.push_back(entry_point.y);
	for (auto& cone : cone_list)
	{
		pointMap.push_back(cone->getX());
		pointMap.push_back(cone->getY());
	}
	
	delaunator::Delaunator d = delaunator::Delaunator(pointMap);
	std::vector<Triang> triangles;
	for (std::size_t i = 0; i < d.triangles.size(); i += 3)
	{
		Triang triangle;
		triangle.a.x = d.coords[2*d.triangles[i]];
		triangle.a.y = d.coords[2*d.triangles[i]+1];
		triangle.b.x = d.coords[2*d.triangles[i+1]];
		triangle.b.y = d.coords[2*d.triangles[i+1]+1];
		triangle.c.x = d.coords[2*d.triangles[i+2]];
		triangle.c.y = d.coords[2*d.triangles[i+2]+1];
		triangles.push_back(triangle);
	}
	return triangles;
}


std::ostream& operator<<(std::ostream& os, std::vector<Triang>& triangle_list)
{
	int print_count{0};
	for (Triang &triangle : triangle_list)
	{
		//BoundStrTranslate defined in definitions.h. Used as map to translate enum class to string.
		os<<"Triangle "<<++print_count<<" with points A (x="<<triangle.a.x<<", y="<<triangle.a.y<<", pos="<<BoundStrTranslate.find(triangle.aPos)->second<<
		") B (x="<<triangle.b.x<<", y="<<triangle.b.y<<", pos="<<BoundStrTranslate.find(triangle.bPos)->second<<
		") and C (x="<<triangle.c.x<<", y="<<triangle.c.y<<", pos="<<BoundStrTranslate.find(triangle.cPos)->second<<")"<<std::endl;
	}
	return os;
} 


std::vector<Coord> Triangulation::findMidsInTriangVec(std::vector<Triang> &triangles, const Coord &current_position)
{
	std::vector<Coord> validMids;
	for (Triang &triangle : triangles)
	{
		Coord abMid = findMidpoint(triangle.a, triangle.b);
		Coord bcMid = findMidpoint(triangle.b, triangle.c);
		Coord caMid = findMidpoint(triangle.c, triangle.a);

		if (triangle.pointsAreEqual(current_position, abMid))
		{
			if (!triangle.pointsOnBoundary(triangle.b, triangle.c)) validMids.push_back(bcMid);
			if (!triangle.pointsOnBoundary(triangle.c, triangle.a)) validMids.push_back(caMid);
		}
		else if (triangle.pointsAreEqual(current_position, bcMid))
		{
			if (!triangle.pointsOnBoundary(triangle.a, triangle.b)) validMids.push_back(abMid);
			if (!triangle.pointsOnBoundary(triangle.c, triangle.a)) validMids.push_back(caMid);
		}
		else if (triangle.pointsAreEqual(current_position, caMid))	
		{
			if (!triangle.pointsOnBoundary(triangle.a, triangle.b)) validMids.push_back(abMid);
			if (!triangle.pointsOnBoundary(triangle.b, triangle.c)) validMids.push_back(bcMid);
		}
		else
		{
			if (!triangle.pointsOnBoundary(triangle.a, triangle.b)) validMids.push_back(abMid);
			if (!triangle.pointsOnBoundary(triangle.b, triangle.c)) validMids.push_back(bcMid);
			if (!triangle.pointsOnBoundary(triangle.c, triangle.a)) validMids.push_back(caMid);
		}
	}
	if (validMids.size()>0)
	{
		std::unique(validMids.begin(), validMids.end());
		return validMids;
	}
	else std::cerr<<"Currently returning no valid mid points - ERROR in triangulation"<<std::endl;
}

std::ostream& operator<<(std::ostream& os, std::vector<Coord> &vec) 
{
	for (Coord &point : vec)
	{
		auto i = &point - &vec[0];
		os<<"Centre point "<<i<<" = X pos "<<point.x<<", Y pos "<<point.y<<std::endl;
	}
	return os;
}

std::ostream& operator<<(std::ostream& os, Tree &tree)
{
    os<<"Tree of size: "<<tree.getTreeSize()<<"."<<std::endl<<"Contains: "<<std::endl;
    for (const tNode &node : tree.getStorageList())
    {
        os<<"Node with position: "<<node.pos.x<<", "<<node.pos.y<<", a checked status of "<<node.checked<<" and a parent with pos at: ";
        if (node.parent!=nullptr) os<<node.parent->pos.x<<", "<<node.parent->pos.y;
        else os<<"Parent is nullptr!";
        os<<std::endl;
    }
	return os;
}

inline bool Triangulation::movingToGoal(const double &distance_from_goal, const double &tolerance, const double &compare)
{
	return (compare+tolerance) > distance_from_goal;	//False if suggested point appears to moving away from goal including tolerance
}

std::vector<std::vector<Coord>> Triangulation::findViablePaths(const Coord &entry_point, const Coord &exit_point, std::vector<Triang>& triangle_list, int max_path_length)
{
	#ifdef DEBUG
    std::unique_ptr<BoundaryLogger> log = std::make_unique<BoundaryLogger>("DEBUG_PATHEXPLORATION", "Path Exploration", reset_logs);
    std::stringstream ss;
	log->write(ss<<"At time of entering function key variables were:");
	log->write(ss<<"Section start point x("<<entry_point.x<<"), y("<<entry_point.y<<")");
	log->write(ss<<"Section end point x("<<exit_point.x<<"), y("<<exit_point.y<<")");
	log->write(ss<<"Triangle list size = "<<triangle_list.size()); 
	auto debug_mids = findMidsInTriangVec(triangle_list, entry_point);
	log->write(ss<<"Number of midpoints in section = "<<debug_mids.size());
	log->write(ss<<"Given as follows:");
	log->write(ss<<debug_mids);
	#endif
	//Init tree
	std::unique_ptr<Tree> tree = std::make_unique<Tree>(visualisation, triangle_list.size()*4);
	bool filling_tree {true};
	auto origin_to_goal_dist = distBetweenPoints(entry_point, exit_point);
	tree->addNode(entry_point);
	#ifdef DEBUG
	log->write(ss<<"Distance to goal over full section: "<<origin_to_goal_dist, true);
	#endif
	//Move into regular search loop
	#ifdef DEBUG
	long debug_print_it = 1;
	log->write(ss<<"Entering tree exploration loop", true);
	#endif
	Coord parent = entry_point;
	while (filling_tree)
	{
		#ifdef DEBUG
		log->write(ss<<"Loop number: "<<debug_print_it++);
		log->write(ss<<"Search origin point x("<<parent.x<<"), y("<<parent.y<<")");
		#endif
		auto localTriangles = findTrianglesAroundPoint(true, triangle_list, parent);
		#ifdef DEBUG
		log->write(ss<<"Size of local triangles: "<<localTriangles.size());
		#endif
		auto midVec = findMidsInTriangVec(localTriangles, parent);
		#ifdef DEBUG
		log->write(ss<<"Size of vector of all triangle mids: "<<midVec.size());
		int midVec_it = 1;
		log->write(ss<<"Entering loop through vector of possible mids", true);
		#endif
		origin_to_goal_dist = distBetweenPoints(parent, exit_point);
		for (auto point : midVec)
		{
			auto best_first_dist = distBetweenPoints(point, exit_point);
			#ifdef DEBUG
			log->write(ss<<"Mid value "<<midVec_it++<<" x("<<point.x<<") y("<<point.y<<")");
			log->write(ss<<"Distance to goal from proposed midpoint: "<<best_first_dist);
			log->write(ss<<"Change in distance to goal if point selected: "<<(best_first_dist-origin_to_goal_dist));
			#endif
			if (movingToGoal(best_first_dist, BEST_FIRST_TOLERANCE, origin_to_goal_dist))
			{
				#ifdef DEBUG
				log->write(ss<<"This point selected as potentially valuable and node ended to tree for further exploration", true);
				#endif
				tree->addNode(point, parent);
			}
			#ifdef DEBUG
			else
			{
				log->write(ss<<"This point rejected and not added to tree", true);
			}
			#endif
		}
		//Collect new mid 
		#ifdef DEBUG
		log->write(ss<<"Requesting next point to explore...");
		#endif
		auto parentP = tree->getNextUnexploredPoint();
		#ifdef DEBUG
		if (parentP!=nullptr)
		{
			log->write(ss<<"Next suggested point by tree to investigate at x("<<parentP->x<<") y("<<parentP->y<<")", true);
		}
		#endif
		if (parentP != nullptr) parent = *parentP;
		else filling_tree = false;	//All nodes have been checked and children nodes added to tree
	}

	#ifdef DEBUG
	log->write(ss<<"Tree formation complete. Tree as follows: ");
	int i {0};
	for (const tNode &node : tree->getStorageList())
	{
		ss<<"Node "<<++i<<" with position: x("<<node.pos.x<<") y("<<node.pos.y<<"), a checked status of "<<node.checked<<" and a parent with pos at: ";
		if (node.parent!=nullptr) ss<<"x("<<node.parent->pos.x<<") y("<<node.parent->pos.y<<")";
		else ss<<"Parent is nullptr!";
		ss<<std::endl;
		int children_count{0};
		for (tNode *child : node.children)
		{
			ss<<"Child "<<++children_count;
			if (child !=nullptr)
			{
				ss<<" has position: x("<<child->pos.x<<") y("<<child->pos.y<<")"<<std::endl;
			}
			else
			{
				ss<<" is a nullptr"<<std::endl;
			}
		}
	}
	log->write(ss);
	log->write(ss<<"Moving into path development", true);
	#endif

	#ifdef VISUALISE
		if (tree->getTreeSize()>0) tree->visualiseTree();
	#endif
	//Move into path construction
	#ifdef DEBUG
	log->write(ss<<"Adding paths of length "<<max_path_length<<":");
	#endif
	auto paths = tree->getPathsOfLength(max_path_length); 
	#ifdef DEBUG
	log->write(ss<<"Paths collected. List of paths contains: ");
	int path_count{0};
	for (const std::vector<Coord> &path : paths)
	{
		log->write(ss<<"Path "<<++path_count);
		int point_count{0};
		for (const Coord &point : path)
		{
			ss<<"Point "<<++point_count<<" at x("<<point.x<<") y("<<point.y<<")"<<std::endl;
		}
		log->write(ss, true);
	}
	#endif
	return paths;
}




