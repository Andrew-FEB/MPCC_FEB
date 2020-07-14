#pragma once

typedef struct coord
{
	double x{ 0 };
	double y{ 0 };
} coord;

typedef struct triangle
{
	coord a;
	coord b;
	coord c;
} triang;

typedef struct carDims
{
	double length{ 0 };
	double width{ 0 };
} carDims;