#include "SimRigidMesh.h"

#include "render/MeshUtil.h"

cSimRigidMesh::tParams::tParams()
{
	mType = eTypeDynamic;
	mMass = 100;
	mFriction = 0.7;
	mPos = tVector(0, 0, 0, 0);
	mUseQuantizedAabbCompression = false;

	mVertices = {};
	mIndizes = {};
	mNormals = {};
	mUVs = {};
}

cSimRigidMesh::cSimRigidMesh()
{
	mCollissionMesh = nullptr;
	mDrawMesh = nullptr;
}

cSimRigidMesh::~cSimRigidMesh()
{
	if (mCollissionMesh != nullptr)
		delete(mCollissionMesh);
	if (mDrawMesh != nullptr)
		delete(mDrawMesh);
}

void cSimRigidMesh::Init(const std::shared_ptr<cWorld>& world, const tParams& params)
{
	mType = params.mType;

	btScalar mass = (params.mType == eTypeDynamic) ? static_cast<btScalar>(params.mMass) : 0;

	mVertices = params.mVertices;
	mIndizes = params.mIndizes;
	mNormals = params.mNormals;
	mUVs = params.mUVs;

	mDrawMesh = new cDrawMesh();
	cMeshUtil::BuildDrawMesh(
		mVertices.data(), mVertices.size(),
		mNormals.data(), mNormals.size(),
		mUVs.data(), mUVs.size(),
		mIndizes.data(), mIndizes.size(),
		mDrawMesh);

	mCollissionMesh = new btTriangleIndexVertexArray(
		mIndizes.size() / 3,
		mIndizes.data(),
		sizeof(int32_t),
		mVertices.size() / 3,
		mVertices.data(),
		sizeof(btScalar) * 3);


	mColShape = std::unique_ptr<btCollisionShape>(new btConvexTriangleMeshShape(mCollissionMesh, params.mUseQuantizedAabbCompression));

	btVector3 inertia(0, 0, 0);
	mColShape->calculateLocalInertia(mass, inertia);

	btRigidBody::btRigidBodyConstructionInfo cons_info(mass, this, mColShape.get(), inertia);
	mSimBody = std::unique_ptr<btRigidBody>(new btRigidBody(cons_info));
	mSimBody->setFriction(static_cast<btScalar>(params.mFriction));


	cSimRigidBody::Init(world);
	SetPos(params.mPos);
}

tVector cSimRigidMesh::GetSize() const
{
	return tVector3::Zero();
}

cShape::eShape cSimRigidMesh::GetShape() const
{
	return cShape::eShapeMesh;
}

cDrawMesh* cSimRigidMesh::GetDrawMesh() const
{
	return mDrawMesh;
}
