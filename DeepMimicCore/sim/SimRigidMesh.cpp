#include "SimRigidMesh.h"

#include "render/MeshUtil.h"
#include <iostream>
#include <iomanip>

cSimRigidMesh::tParams::tParams()
{
	mType = eTypeDynamic;
	mMass = 10;
	mFriction = 0.7;
	mUseQuantizedAabbCompression = false;

	mPos = tVector(0, 0, 0, 0);
	mVel = tVector(0, 0, 0, 0);
	mRot = tQuaternion::Identity();

	mVertices = {};
	mIndizes = {};
	mNormals = {};
	mUVs = {};
}

cSimRigidMesh::cSimRigidMesh()
{
	mDrawMesh = nullptr;
}

cSimRigidMesh::~cSimRigidMesh()
{
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
		mIndizes.data(), mIndizes.size(),
		mDrawMesh);
	
	
	/*
	cMeshUtil::BuildDrawMesh(
		mVertices.data(), mVertices.size(),
		mNormals.data(), mNormals.size(),
		mUVs.data(), mUVs.size(),
		mIndizes.data(), mIndizes.size(),
		mDrawMesh);
	*/


	// debug vertex buffer
	/*
	std::cout << "vertices: " << std::endl;

	for (int i = 0; i < mVertices.size(); ++i) {

		if (i % 3 == 0)
			std::cout << "(";

		auto v = mVertices[i];
		std::cout << v;

		if (i % 3 == 2)
			std::cout << "), " << std::endl;
		else
			std::cout << ",";
	}
	std::cout << std::endl;
	*/

	// debug index buffer
	/*
	std::cout << "indizes: " << std::endl;

	for (int i = 0; i < mIndizes.size(); ++i) {

		if (i % 3 == 0)
			std::cout << "(";

		auto v = mIndizes[i];
		std::cout << v;

		if (i % 3 == 2)
			std::cout << ")" << std::endl;
		else
			std::cout << ",";
	}

	std::cout << std::endl;
	*/

	// scale vertex buffer by world scale
	std::vector<btScalar> scaledVertices;

	for (auto & vertex : mVertices)
	{
		scaledVertices.push_back(vertex * world->GetScale());
	}

	auto convexHull = new btConvexHullShape(scaledVertices.data(), scaledVertices.size() / 3, sizeof(btScalar) * 3);
	mColShape = std::unique_ptr<btCollisionShape>(convexHull);

	// debug convex hull
	/*
	std::cout << "convex Hull vertices:" << std::endl;

	for (int i = 0; i < convexHull->getNumVertices(); ++i)
	{
		btVector3 vertex;
		convexHull->getVertex(i, vertex);

		std::cout << vertex.x() << ", " << vertex.y() << ", " << vertex.z() << std::endl;
	}
	*/

	btVector3 inertia(0, 0, 0);
	mColShape->calculateLocalInertia(mass, inertia);

	btRigidBody::btRigidBodyConstructionInfo cons_info(mass, this, mColShape.get(), inertia);
	mSimBody = std::unique_ptr<btRigidBody>(new btRigidBody(cons_info));
	mSimBody->setFriction(static_cast<btScalar>(params.mFriction));

	cSimRigidBody::Init(world);
	SetPos(params.mPos);
	SetLinearVelocity(params.mVel);
	SetRotation(params.mRot);

	// debug transform
	/*
	auto worldTransform = mSimBody->getWorldTransform();
	btScalar transform[16];
	worldTransform.getOpenGLMatrix(transform);

	std::cout << " pos: " <<  params.mPos.x() << ", " << params.mPos.y() << ", " << params.mPos.z() << std::endl;
	std::cout << " worldT: " << std::endl;

	for (int i = 0; i < 16; ++i)
	{
		std::cout << transform[i] << " ";
		if (i % 4 == 3)
			std::cout << std::endl;
	}

	std::cout << "mWorld->GetScale()" << world->GetScale() << std::endl;
	*/
}

tVector cSimRigidMesh::GetSize() const
{
	btVector3 aabbMin, aabbMax;
	mSimBody->getAabb(aabbMin, aabbMax);
	btVector3 diff = aabbMax - aabbMin;
	return tVector(diff.x(), diff.y(), diff.z(), 0);
}

cShape::eShape cSimRigidMesh::GetShape() const
{
	return cShape::eShapeMesh;
}

cDrawMesh* cSimRigidMesh::GetDrawMesh() const
{
	return mDrawMesh;
}
