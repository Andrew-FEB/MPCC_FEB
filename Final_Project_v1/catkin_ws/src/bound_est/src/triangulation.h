#pragma once

#include <vector>
#include <memory>
#include <cmath>

#include "cone.h"
#include "visualisation.h"
#include "definitions.h"
#include "tree.h"
#include "boundaryLogger.h"
#include "freeFunctions.h"

constexpr double FILTER_DISTANCE_EDIT {3};
constexpr double FILTER_DISTANCE_TOLERANCE {FILTER_DISTANCE_EDIT*FILTER_DISTANCE_EDIT};
constexpr int MAX_PATH_LENGTH{15};

class Triangulation
{

public:
	Triangulation(std::shared_ptr<Visualisation> visualisation_cont);
	~Triangulation() = default;
	std::vector<std::vector<Coord>> getTraversingPaths(const std::vector<std::unique_ptr<Cone>>& cone_list, const Coord &last_point, const Coord &end_goal,
														const std::pair<std::vector<const Cone *>, std::vector<const Cone *>> &seperated_cone_lists, const bool &starting_from_car);
  
private:
	std::vector<Triang> findTrianglePoints(const std::vector<std::unique_ptr<Cone>>& coneList, const Coord &lastPoint);
	void placeConesInTriangle(std::vector<Triang> &triangList, const std::vector<std::unique_ptr<Cone>> &coneList);
	bool movingToGoal(const double &distanceFromGoal, const double &tolerance, const double &compare);
	bool coordsAreEqual(const Coord &a, const Coord &b)
	{
		return (a.x == b.x && a.y == b.y);
	};
	std::vector<std::vector<Coord>> findViablePaths(Coord &parent, std::vector<Triang> &triangleList, const Coord &sectionEnd);
	std::vector<Triang> findTrianglesAroundPoint(bool isMidpoint, const std::vector<Triang> &triangList, const Coord &point);
	Coord findFirstMidpoint(std::vector<Triang> &vec, const Coord &lastPoint);
	std::vector<Coord> findMidsInTriangVec(std::vector<Triang> &bdrTriangles, const Coord &nextMid);
	std::vector<Coord> findBestPath(const std::vector<std::vector<Coord>> &paths, const std::vector<std::unique_ptr<Cone>> &coneList);
	Coord calcFirstPoint(const std::vector<Triang> &triangleList, const Coord &startPoint);
	std::vector<Coord> collectMidpoints(std::vector<Triang> &triangList);

	std::shared_ptr<Visualisation> visualisation{ nullptr };
	
};

std::ostream& operator<<(std::ostream& os, std::vector<Triang> &triangList);
std::ostream& operator<<(std::ostream& os, std::vector<Coord> &vec);
std::ostream& operator<<(std::ostream& os, Tree &tree);
