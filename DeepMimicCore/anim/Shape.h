#pragma once

#include <string>

class cShape
{
public:
	enum eShape
	{
		eShapeNull,
		eShapeBox,
		eShapeCapsule,
		eShapeSphere,
		eShapeCylinder,
		eShapePlane,
		eShapeMesh,
		eShapeSoft,
		eShapeMax,
	};

	static bool ParseShape(const std::string& str, cShape::eShape& out_shape);
};