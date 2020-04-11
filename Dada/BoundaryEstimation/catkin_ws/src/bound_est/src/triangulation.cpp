#include "triangulation.h"
#include "include/delaunay/delaunator.hpp"

Triangulation::Triangulation(std::shared_ptr<Visualise> visualise_cont)
{
#ifdef VISUALISE
	if (visualise_cont!=nullptr)
	{
		visualise = visualise_cont;
	}
	else std::cerr << "ERROR: Visualise pre-processor statement defined but nullptr Visualise object passed to Triangulation object";
#endif
}

std::vector<coord> Triangulation::getCentreCoords(std::vector<std::unique_ptr<Cone>>& coneList, coord lastPosition)
{
#ifdef VISUALISE
	if (coneList.size()>0) visualise->showCones(coneList);
#endif
	std::vector<coord> finalLine;
	std::vector<triang> triangleList = findTrianglePoints(coneList, lastPosition);
	placeConesInTriangle(triangleList, coneList);
#ifdef VISUALISE
	if (triangleList.size()>0) visualise->showTriangles(triangleList);
#endif

	coord sectionEnd = findEndGoal(coneList);
#ifdef VISUALISE
	visualise->showEndPoint(sectionEnd);
#endif
	calcFirstPoint(triangleList, finalLine, lastPosition);
	auto paths = findViablePaths(finalLine, triangleList, lastPosition, sectionEnd);
#ifdef VISUALISE
	if (finalLine.size()>0)visualise->showCentreCoords(finalLine);
#endif

	//auto finalPath = findBestPath(paths, coneList);
	//finalLine.insert(std::end(finalLine), std::begin(finalPath), std::end(finalPath));

	return finalLine;
}

std::vector<triang> Triangulation::findTrianglesAroundPoint(bool isMidpoint, const std::vector<triang> &triangList, const coord &point)
{
	std::vector<triang> list;
	std::vector<triang>::const_iterator it = triangList.begin();
	if (isMidpoint)
	{
		while (it!=triangList.end())
		{
			if (it->onVertices(point))
			{
				list.push_back(*it);
			}
			it++;
			
		}

	}
	else
	{
		while (it!=triangList.end())
		{
			if (it->touches(point))
			{
				list.push_back(*it);
			}
			it++;
			
		}
	}
	return list;
}

coord Triangulation::findFirstMidpoint(std::vector<triang> &vec, const coord &point)
{
	coord first;
	coord second;
	for (triang &triangle : vec)
	{
		if (triangle.pointsAreEqual(point, triangle.a))
		{
			if (!triangle.pointsOnBoundary(triangle.b, triangle.c))
			{
				first = triangle.b;
				second = triangle.c;
				break;
			}
		}
		else if (triangle.pointsAreEqual(point, triangle.b)) 
		{
			if (!triangle.pointsOnBoundary(triangle.a, triangle.c))
			{
				first = triangle.a;
				second = triangle.c;
				break;
			}
		}
		else 
		{
			if (!triangle.pointsOnBoundary(triangle.a, triangle.b))
			{
				first = triangle.a;
				second = triangle.b;
				break;
			}
		}
	}
	return findMidpoint(first,second);
}

void Triangulation::placeConesInTriangle(std::vector<triang> &triangList, const std::vector<std::unique_ptr<Cone>> &coneList)
{
	for (triang &triangle : triangList)
	{
		triangle.aPos = BoundPos::left;
		bool locatedA = false;
		bool locatedB = false;
		bool locatedC = false;
		for (std::vector<std::unique_ptr<Cone>>::const_reverse_iterator it = coneList.rbegin() ; it!=coneList.rend(); it++)
		{
			if (!locatedA)	//Avoid complex check if already located
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

inline coord Triangulation::findMidpoint(const coord &a, const coord &b)
{
	coord output{ (a.x + b.x) / 2, (a.y + b.y) / 2 };
	return output;
}

std::vector<triang> Triangulation::findTrianglePoints(std::vector<std::unique_ptr<Cone>>& coneList, coord &lastPoint)
{
	std::vector<double> pointMap;
	pointMap.push_back(lastPoint.x);
	pointMap.push_back(lastPoint.y);
	for (std::unique_ptr<Cone>& cone : coneList)
	{
		pointMap.push_back(cone->getX());
		pointMap.push_back(cone->getY());
	}
	
	delaunator::Delaunator d = delaunator::Delaunator(pointMap);
	std::vector<triang> triangles;
	for (std::size_t i = 0; i < d.triangles.size(); i += 3)
	{
		triang triangle;
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

coord Triangulation::findEndGoal(std::vector<std::unique_ptr<Cone>>& coneList)
{
	//Does cone list need sorting?? Shouldn't, but keep checking
	Cone *lastCone = coneList.back().get();
	auto comparePos = lastCone->getPos();
	Cone *lastConeOpp;
	for(std::vector<std::unique_ptr<Cone>>::reverse_iterator it = coneList.rbegin()++; it!=coneList.rend(); it++)
	{
		auto pos = (*it)->getPos();
		if (pos!=comparePos && pos!=BoundPos::undefined) 
		{
			lastConeOpp = it->get();
			break;
		}
	}
	auto endPoint = findMidpoint({lastCone->getX(), lastCone->getY()}, {lastConeOpp->getX(), lastConeOpp->getY()});
	return endPoint;
}

inline double distBetweenPoints(const coord &a, const coord &b)
{
	return (pow((a.x-b.x), 2) + pow((a.y-b.y),2));
}

std::ostream& operator<<(std::ostream& os, std::vector<triang>& triangList)
{
	for (triang &triangle : triangList)
	{
		//BoundStrTranslate defined in definitions.h. Used as map to translate enum class to string.
		os<<"Triangle with points A (x="<<triangle.a.x<<", y="<<triangle.a.y<<", pos="<<BoundStrTranslate.find(triangle.aPos)->second<<
		") B (x="<<triangle.b.x<<", y="<<triangle.b.y<<", pos="<<BoundStrTranslate.find(triangle.bPos)->second<<
		") and C (x="<<triangle.c.x<<", y="<<triangle.c.y<<", pos="<<BoundStrTranslate.find(triangle.cPos)->second<<")"<<std::endl;
	}
	return os;
} 


std::vector<coord> Triangulation::findMidsInTriangVec(std::vector<triang> &bdrTriangles, const coord &currentPoint)
{
	std::vector<coord> validMids;
	for (triang &triangle : bdrTriangles)
	{
		coord abMid = findMidpoint(triangle.a, triangle.b);
		coord bcMid = findMidpoint(triangle.b, triangle.c);
		coord caMid = findMidpoint(triangle.c, triangle.a);
		if (triangle.pointsAreEqual(currentPoint, abMid))
		{
			validMids.push_back(bcMid);
			validMids.push_back(caMid);
		}
		else if (triangle.pointsAreEqual(currentPoint, bcMid))
		{
			validMids.push_back(abMid);
			validMids.push_back(caMid);
		}
		else 	//point is midpoint between c and a
		{
			validMids.push_back(abMid);
			validMids.push_back(bcMid);
		}
	}
	if (validMids.size()>0) return validMids;
	else std::cerr<<"Currently returning no valid mid points - ERROR in triangulation"<<std::endl;
	
}

std::ostream& operator<<(std::ostream& os, std::vector<coord> &vec) 
{
	for (coord &point : vec)
	{
		auto i = &point - &vec[0];
		os<<"Centre point "<<i<<" = X pos "<<point.x<<", Y pos "<<point.y<<std::endl;
	}
	return os;
}

inline bool Triangulation::movingToGoal(const double &distanceFromGoal, const double &tolerance, const double &compare)
{
	return (compare+tolerance) > distanceFromGoal;	//False if suggested point appears to moving away from goal including tolerance
}

void Triangulation::calcFirstPoint(const std::vector<triang> &triangleList, std::vector<coord> &finalLine, const coord &startPoint)
{
	//Find first triangle mid from car
	auto localTriangles = findTrianglesAroundPoint(false, triangleList, startPoint);
	auto parent = findFirstMidpoint(localTriangles, startPoint);	//Find valid target point around section start
	finalLine.push_back(parent);	//add to vector
}

std::vector<std::vector<coord>> Triangulation::findViablePaths(std::vector<coord> &finalLine, std::vector<triang>& triangleList, const coord &startPoint, const coord &sectionEnd)
{
	
	//Init tree
	std::unique_ptr<Tree> tree = std::make_unique<Tree>(visualise, triangleList.size()*3);
	int limit{0};
	bool fillingTree {true};
	auto originToGoalDist = distBetweenPoints(startPoint, sectionEnd);
	auto parent = *finalLine.begin();

	//Move into regular search loop
	while (fillingTree && limit<=500)
	{
		auto localTriangles = findTrianglesAroundPoint(true, triangleList, parent);
		auto midVec = findMidsInTriangVec(localTriangles, parent);

		for (auto point : midVec)
		{
			auto bestFirstDist = distBetweenPoints(point, sectionEnd);
			originToGoalDist = distBetweenPoints(parent, sectionEnd);
			if (movingToGoal(bestFirstDist, FILTER_DISTANCE_TOLERANCE, originToGoalDist))
			{
				tree->addNode(point, parent, bestFirstDist);

			}
		}
		//Collect new mid 
		auto parentP = tree->getNextUnexploredPoint();
		if (parentP != nullptr) parent = *parentP;
		else fillingTree = false;	//All nodes have been checked and children nodes added to tree
		limit++;	
	}
	//TEST
	std::cerr<<*tree;
	//ETEST
#ifdef VISUALISE
	if (tree->size()>0) tree->visualiseTree();
#endif

	// std::vector<std::vector<coord>> paths;
	// //Move into path construction
	// for (int limit = MIN_PATH_LENGTH; limit < MAX_PATH_LENGTH ; limit++)
	// {
	// 	auto temp = tree->getPathsOfGivenLength(limit);
	// 	paths.insert(std::end(temp), std::begin(temp), std::end(temp));
	// }
	//return paths;
	return {};
}

std::vector<coord> Triangulation::findBestPath(const std::vector<std::vector<coord>> &paths, const std::vector<std::unique_ptr<Cone>> &coneList)
{
	std::vector<std::pair<double, int>> costScores;
	costScores.reserve(paths.size());
	int index{0};
	for (auto pathIt = paths.begin(); pathIt!=paths.end(); pathIt++)
	{
		index++;
		costScores[index] = std::make_pair(applyPathCostFunction(*pathIt), index);
	}	
	sortPathCosts(costScores);
	return paths[costScores.begin()->second];	//return path associated with lowest cost function score
}

inline void Triangulation::sortPathCosts(std::vector<std::pair<double, int>> &costScores)
{
	std::sort(costScores.begin(), costScores.end(), [](const std::pair<double,int> &a, const std::pair<double,int> b)
    {
        return (a.first < b.first);
    });
}

inline double Triangulation::applyPathCostFunction(const std::vector<coord> &path)
{
	double cost{0};
	double distFromEdges{0};
	for (auto &point : path)
	{

	}
}

inline double Triangulation::conesOnEitherSide(coord point)
{

}


