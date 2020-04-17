#pragma once

#include <vector>
#include <memory>
#include <cmath>

#include "cone.h"
#include "visualisation.h"
#include "definitions.h"
#include "tree.h"

constexpr double FILTER_DISTANCE_EDIT {3};
constexpr double FILTER_DISTANCE_TOLERANCE {FILTER_DISTANCE_EDIT*FILTER_DISTANCE_EDIT};
constexpr int MIN_PATH_LENGTH{2};
constexpr int MAX_PATH_LENGTH{2};

class Triangulation
{

public:
	Triangulation(std::shared_ptr<Visualisation> visualise_cont);
	~Triangulation() = default;
	std::vector<coord> getCentreCoords(std::vector<std::unique_ptr<Cone>>& coneList, coord lastPoint);
  

private:
	std::vector<triang> findTrianglePoints(std::vector<std::unique_ptr<Cone>>& coneList, coord &lastPoint);
	void placeConesInTriangle(std::vector<triang> &triangList, const std::vector<std::unique_ptr<Cone>> &coneList);

	friend double distBetweenPoints(const coord &a, const coord &b);
	coord findEndGoal(std::vector<std::unique_ptr<Cone>>& coneList);
	coord findMidpoint(const coord &a, const coord &b);
	bool movingToGoal(const double &distanceFromGoal, const double &tolerance, const double &compare);
	void sortPathCosts(std::vector<std::pair<double, int>> &costScores);
	bool coordsAreEqual(const coord &a, const coord &b)
	{
		return (a.x == b.x && a.y == b.y);
	};
	double conesOnEitherSide(coord point);

	std::vector<std::vector<coord>> findViablePaths(std::vector<coord> &finalLine, std::vector<triang> &triangleList, const coord &startPoint, const coord &sectionEnd);
	std::vector<triang> findTrianglesAroundPoint(bool isMidpoint, const std::vector<triang> &triangList, const coord &point);
	coord findFirstMidpoint(std::vector<triang> &vec, const coord &lastPoint);
	std::vector<coord> findMidsInTriangVec(std::vector<triang> &bdrTriangles, const coord &nextMid);
	std::vector<coord> findBestPath(const std::vector<std::vector<coord>> &paths, const std::vector<std::unique_ptr<Cone>> &coneList);
	double applyPathCostFunction(const std::vector<coord> &path);
	void calcFirstPoint(const std::vector<triang> &triangleList, std::vector<coord> &finalLine, const coord &startPoint);

	std::vector<coord> collectMidpoints(std::vector<triang> &triangList);
	std::shared_ptr<Visualisation> visualise{ nullptr };
};

std::ostream& operator<<(std::ostream& os, std::vector<triang> &triangList);
std::ostream& operator<<(std::ostream& os, std::vector<coord> &vec);
