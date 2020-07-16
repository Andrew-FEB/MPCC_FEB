#pragma once

#include <vector>
#include <memory>
#include <cmath>

#include "cone.h"
#include "visualisation.h"
#include "definitions.h"
#include "tree.h"
#include "boundaryLogger.h"

constexpr double FILTER_DISTANCE_EDIT {3};
constexpr double FILTER_DISTANCE_TOLERANCE {FILTER_DISTANCE_EDIT*FILTER_DISTANCE_EDIT};
constexpr int MAX_PATH_LENGTH{20};

class Triangulation
{

public:
	Triangulation(std::shared_ptr<Visualisation> visualisation_cont);
	~Triangulation() = default;
	std::vector<std::vector<coord>> getTraversingPaths(std::vector<std::unique_ptr<Cone>>& coneList, coord lastPoint,
														std::pair<std::vector<const Cone *>, std::vector<const Cone *>> &seperated_cone_lists);
  
private:
	std::vector<triang> findTrianglePoints(std::vector<std::unique_ptr<Cone>>& coneList, coord &lastPoint);
	void placeConesInTriangle(std::vector<triang> &triangList, const std::vector<std::unique_ptr<Cone>> &coneList);

	coord findEndGoal(std::pair<std::vector<const Cone *>, std::vector<const Cone *>> &seperated_cone_lists);
	coord findMidpoint(const coord &a, const coord &b);
	bool movingToGoal(const double &distanceFromGoal, const double &tolerance, const double &compare);
	bool coordsAreEqual(const coord &a, const coord &b)
	{
		return (a.x == b.x && a.y == b.y);
	};
	std::vector<std::vector<coord>> findViablePaths(coord &parent, std::vector<triang> &triangleList, const coord &startPoint, const coord &sectionEnd);
	std::vector<triang> findTrianglesAroundPoint(bool isMidpoint, const std::vector<triang> &triangList, const coord &point);
	coord findFirstMidpoint(std::vector<triang> &vec, const coord &lastPoint);
	std::vector<coord> findMidsInTriangVec(std::vector<triang> &bdrTriangles, const coord &nextMid);
	std::vector<coord> findBestPath(const std::vector<std::vector<coord>> &paths, const std::vector<std::unique_ptr<Cone>> &coneList);
	coord calcFirstPoint(const std::vector<triang> &triangleList, const coord &startPoint);
	std::vector<coord> collectMidpoints(std::vector<triang> &triangList);
	std::shared_ptr<Visualisation> visualisation{ nullptr };
	
};

std::ostream& operator<<(std::ostream& os, std::vector<triang> &triangList);
std::ostream& operator<<(std::ostream& os, std::vector<coord> &vec);
std::ostream& operator<<(std::ostream& os, Tree &tree);