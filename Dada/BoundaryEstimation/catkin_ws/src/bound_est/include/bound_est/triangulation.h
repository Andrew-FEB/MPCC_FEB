#pragma once

#include <vector>
#include <memory>
#include <optional>

#include "cone.h"
#include "visualise.h"
#include "definitions.h"


class Triangulation
{
public:
	Triangulation(std::optional<std::shared_ptr<Visualise>> visualise_cont);
	~Triangulation() = default;
	std::vector<coord> getCentreLine(std::vector<std::unique_ptr<Cone>>& coneList);

private:
	std::vector<triang> findTrianglePoints(std::vector<std::unique_ptr<Cone>>& coneList);
	std::shared_ptr<Visualise> visualise{ nullptr };
};

