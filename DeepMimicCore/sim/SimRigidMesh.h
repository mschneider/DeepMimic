#pragma once

#include "sim/SimRigidBody.h"
#include "render/DrawMesh.h"

class cSimRigidMesh : public cSimRigidBody
{
public:
	struct tParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		eType mType;
		double mMass;
		double mFriction;
		bool mUseQuantizedAabbCompression;

		tVector mPos;
		tVector mVel;
		tQuaternion mRot;

		std::vector<btScalar> mVertices;
		std::vector<int32_t> mIndizes;
		std::vector<float> mNormals;
		std::vector<float> mUVs;

		tParams();
	};

	cSimRigidMesh();
	virtual ~cSimRigidMesh();

	virtual void Init(const std::shared_ptr<cWorld>& world, const tParams& params);


	virtual cDrawMesh* GetDrawMesh() const;
	virtual tVector GetSize() const;
	virtual cShape::eShape GetShape() const;

	std::vector<btScalar> mVertices;
	std::vector<int32_t> mIndizes;
	std::vector<float> mNormals;
	std::vector<float> mUVs;

	cDrawMesh* mDrawMesh;
};

