#pragma once

#include "BulletSoftBody/btSoftBody.h"
#include "sim/SimObj.h"
#include "render/DrawMesh.h"

class cSimSoftBody :
	public cSimObj, public btDefaultMotionState
{

public:
	struct tParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		tVector mPos;
		tVector mVel;
		tQuaternion mRot;

		std::vector<btScalar> mVertices;
		std::vector<int32_t> mIndizes;
		std::vector<float> mNormals;
		std::vector<float> mUVs;

		tParams();
	};

	cSimSoftBody();
	virtual ~cSimSoftBody();

	virtual cDrawMesh* GetDrawMesh() const;
	virtual cShape::eShape GetShape() const;

	virtual tVector GetPos() const;

	virtual void GetRotation(tVector& out_axis, double& out_theta) const;

	virtual tQuaternion GetRotation() const;

	virtual void Init(const std::shared_ptr<cWorld>& world, const tParams& params);
	virtual void SetPos(const tVector& pos);
	virtual void SetRotation(const tQuaternion& q);
	virtual void UpdateShape();

	virtual const std::vector<btScalar>& GetVertexPositions() const;
	virtual tMatrix GetWorldTransform() const;

	virtual void CalcAABB(tVector& out_min, tVector& out_max) const;

	virtual tVector GetLinearVelocity(const tVector&) const;
	virtual tVector GetLinearVelocity() const;
	virtual void SetLinearVelocity(const tVector&);
	virtual tVector GetAngularVelocity() const;
	virtual void SetAngularVelocity(const tVector&);
	virtual void ApplyForce(const tVector&, const tVector&);
	virtual void ApplyForce(const tVector&);
	virtual void ApplyTorque(const tVector&);
	virtual void ClearForces();
	virtual tVector GetSize() const;
	virtual btCollisionObject* GetCollisionObject();
	virtual const btCollisionObject* GetCollisionObject() const;

protected:
	std::unique_ptr<btSoftBody> mSoftBody;
	cDrawMesh* mDrawMesh;
	std::vector<btScalar> mVertices;
	std::vector<int32_t> mIndizes;
	std::vector<float> mNormals;
	std::vector<float> mUVs;


	virtual void AddToWorld(const std::shared_ptr<cWorld>& world);
	virtual void RemoveFromWorld();
};

