#include "SimSoftBody.h"

#include <iostream>
#include <BulletSoftBody/btSoftMultiBodyDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include "render/MeshUtil.h"


cSimSoftBody::tParams::tParams()
{
	mPos = tVector(0, 0, 0, 0);
	mVel = tVector(0, 0, 0, 0);
	mRot = tQuaternion::Identity();

	mVertices = {};
	mIndizes = {};
	mNormals = {};
	mUVs = {};
}

cSimSoftBody::cSimSoftBody()
{
	mDrawMesh = new cDrawMesh();
}

cSimSoftBody::~cSimSoftBody()
{
	RemoveFromWorld();
	if (mDrawMesh != nullptr)
		delete(mDrawMesh);
}


cDrawMesh* cSimSoftBody::GetDrawMesh() const
{
	return mDrawMesh;
}

cShape::eShape cSimSoftBody::GetShape() const
{
	return cShape::eShapeSoft;
}



tVector cSimSoftBody::GetPos() const
{
	btTransform trans = mSoftBody->getWorldTransform();
	btVector3 origin = trans.getOrigin();
	tVector pos = tVector(origin[0], origin[1], origin[2], 0);
	pos /= mWorld->GetScale();
	return pos;
}

void cSimSoftBody::GetRotation(tVector& out_axis, double& out_theta) const
{
	btTransform trans = mSoftBody->getWorldTransform();
	btQuaternion bt_q = trans.getRotation();
	btVector3 axis = bt_q.getAxis();

	out_theta = bt_q.getAngle();
	out_axis[0] = axis[0];
	out_axis[1] = axis[1];
	out_axis[2] = axis[2];
	out_axis[3] = 0;
}

tQuaternion cSimSoftBody::GetRotation() const
{
	btTransform trans = mSoftBody->getWorldTransform();
	btQuaternion bt_q = trans.getRotation();
	tQuaternion q = tQuaternion(bt_q.w(), bt_q.x(), bt_q.y(), bt_q.z());
	return q;
}

void cSimSoftBody::SetPos(const tVector& pos)
{
	std::cout << "cSimSoftBody::SetPos" << std::endl;

	btTransform trans = mSoftBody->getWorldTransform();

	btScalar scale = static_cast<btScalar>(mWorld->GetScale());
	trans.setOrigin(scale * btVector3(static_cast<btScalar>(pos[0]),
		static_cast<btScalar>(pos[1]),
		static_cast<btScalar>(pos[2])));

	mSoftBody->setWorldTransform(trans);
}

void cSimSoftBody::SetRotation(const tQuaternion& q)
{
	std::cout << "cSimSoftBody::SetRotation" << std::endl;

	btTransform trans = mSoftBody->getWorldTransform();
	btQuaternion bt_q = btQuaternion(static_cast<btScalar>(q.x()),
		static_cast<btScalar>(q.y()),
		static_cast<btScalar>(q.z()),
		static_cast<btScalar>(q.w()));
	trans.setRotation(bt_q);
	mSoftBody->setWorldTransform(trans);
}


const std::vector<btScalar>& cSimSoftBody::GetVertexPositions() const
{
	return mVertices;
}

tMatrix cSimSoftBody::GetWorldTransform() const
{
	const btTransform& bt_trans = mSoftBody->getWorldTransform();
	const btMatrix3x3& basis = bt_trans.getBasis();
	const btVector3& origin = bt_trans.getOrigin();
	double scale = mWorld->GetScale();

	tMatrix trans = tMatrix::Identity();
	for (int i = 0; i < 3; ++i)
	{
		auto curr_row = trans.row(i);
		auto bt_row = basis.getRow(i);
		for (int j = 0; j < 3; ++j)
		{
			curr_row[j] = bt_row[j];
		}
		curr_row[3] = origin[i] / scale;
	}
	return trans;
}

void cSimSoftBody::CalcAABB(tVector& out_min, tVector& out_max) const
{
	const auto* shape = GetCollisionShape();

	btVector3 bt_min;
	btVector3 bt_max;
	shape->getAabb(mSoftBody->getWorldTransform(), bt_min, bt_max);
	double scale = mWorld->GetScale();

	out_min = tVector(bt_min[0], bt_min[1], bt_min[2], 0) / scale;
	out_max = tVector(bt_max[0], bt_max[1], bt_max[2], 0) / scale;
}


void cSimSoftBody::UpdateShape()
{
	mVertices.clear();
	for (int i = 0; i < mSoftBody->m_nodes.size(); ++i)
	{
		auto& nodePos = mSoftBody->m_nodes[i].m_x / mWorld->GetScale();
		mVertices.push_back(nodePos.x());
		mVertices.push_back(nodePos.y());
		mVertices.push_back(nodePos.z());
	}

	mIndizes.clear();
	for (int i = 0; i < mSoftBody->m_faces.size(); ++i)
	{
		auto face = mSoftBody->m_faces[i];
		auto base = &(mSoftBody->m_nodes[0]);
		for (int j = 0; j < 3; ++j)
		{
			auto idx = face.m_n[j] - base;
			mIndizes.push_back(idx);
		}
	}


	cMeshUtil::BuildDrawMesh(
		mVertices.data(), mVertices.size(),
		mIndizes.data(), mIndizes.size(),
		mDrawMesh);
}

void cSimSoftBody::Init(const std::shared_ptr<cWorld>& world, const tParams& params)
{
	RemoveFromWorld();

	mVertices = params.mVertices;
	mIndizes = params.mIndizes;
	mNormals = params.mNormals;
	mUVs = params.mUVs;

	auto softWorld = (btSoftMultiBodyDynamicsWorld*)(world->GetInternalWorldPtr());
	auto softWorldInfo = new btSoftBodyWorldInfo(softWorld->getWorldInfo());


	/*
	auto softBody = btSoftBodyHelpers::CreateFromTriMesh(
		*softWorldInfo,
		&mVertices[0],
		&mIndizes[0],
		mIndizes.size() / 3);
	softBody->m_cfg.kDF = 0.5;
	softBody->m_cfg.kMT = 0.05;
	softBody->m_cfg.piterations = 2;
	softBody->m_cfg.piterations = 5;
	softBody->randomizeConstraints();
	softBody->setTotalMass(100, true);
	softBody->setPose(false, true);
	*/


	/*
	auto softBody = btSoftBodyHelpers::CreateEllipsoid(
		*softWorldInfo,
		btVector3(params.mPos.x(), params.mPos.y(), params.mPos.z()),
		btVector3(1.0f, 1.0f, 1.0f),
		100);
	*/

	auto softBody = btSoftBodyHelpers::CreateFromTriMesh(
		*softWorldInfo,
		&mVertices[0],
		&mIndizes[0],
		mIndizes.size() / 3);
	softBody->setTotalMass(50);
	softBody->generateBendingConstraints(2);
	softBody->randomizeConstraints();


	auto cFlags = softBody->getCollisionFlags();

	std::cout << " collision flags " << cFlags << std::endl;


	// common settings
	softBody->m_cfg.kDF = .2;
	softBody->m_cfg.kDP = 0;
	softBody->m_cfg.kVC = 20;
	softBody->m_cfg.kPR = 0;
	softBody->m_cfg.kAHR = .7;
	softBody->m_cfg.maxvolume = 1;
	softBody->m_cfg.kMT = 0;
	softBody->m_cfg.kVCF = 1;
	softBody->m_cfg.timescale = 1;
	//softBody->m_cfg.collisions = btSoftBody::fCollision::Default;

	std::cout << " fCollisions " << softBody->m_cfg.collisions << std::endl;
	// contact hardness settings
	softBody->m_cfg.kCHR = 1;
	softBody->m_cfg.kKHR = .1;
	softBody->m_cfg.kSHR = 1;

	// solver settings
	softBody->m_cfg.piterations = 1;
	softBody->m_cfg.diterations = 0;
	softBody->m_cfg.viterations = 0;
	softBody->m_cfg.citerations = 0;
	
	// aero settings
	softBody->m_cfg.aeromodel = btSoftBody::eAeroModel::V_Point;
	softBody->m_cfg.kDG = 0;
	softBody->m_cfg.kLF = 0;

	// cluster settings
	softBody->m_cfg.kSRHR_CL = 0.1;
	softBody->m_cfg.kSKHR_CL = 1;
	softBody->m_cfg.kSSHR_CL = 0.5;
	softBody->m_cfg.kSR_SPLT_CL = 0.5;
	softBody->m_cfg.kSK_SPLT_CL = 0.5;
	softBody->m_cfg.kSS_SPLT_CL = 0.5;
	
	auto translation = btVector3(params.mPos.x(), params.mPos.y(), params.mPos.z());
	auto rotation = btQuaternion(params.mRot.x(), params.mRot.y(), params.mRot.z(), params.mRot.w());
	auto scale = world->GetScale() * btVector3(1, 1, 1);
	
	softBody->translate(translation);
	softBody->rotate(rotation);
	softBody->scale(scale);

	mSoftBody = std::unique_ptr<btSoftBody>(softBody);

	AddToWorld(world);

	/*
	SetPos(params.mPos);
	SetLinearVelocity(params.mVel);
	SetRotation(params.mRot);
	*/

	UpdateShape();
}

void cSimSoftBody::AddToWorld(const std::shared_ptr<cWorld>& world)
{
	mWorld = world;

	auto softWorld = (btSoftMultiBodyDynamicsWorld*) (mWorld->GetInternalWorld().get());
	softWorld->addSoftBody(mSoftBody.get());

	mColShape = std::unique_ptr<btCollisionShape>(mSoftBody->getCollisionShape());
}

void cSimSoftBody::RemoveFromWorld()
{
	if (mWorld != nullptr && mSoftBody != nullptr)
	{
		auto softWorld = (btSoftMultiBodyDynamicsWorld*)(mWorld->GetInternalWorld().get());
		softWorld->removeSoftBody(mSoftBody.get());
		mWorld.reset();
		mSoftBody.reset();
	}
}

tVector cSimSoftBody::GetLinearVelocity(const tVector&) const
{
	return tVector(0, 0, 0, 0);
}

tVector cSimSoftBody::GetLinearVelocity() const
{
	return tVector(0, 0, 0, 0);

}

tVector cSimSoftBody::GetAngularVelocity() const
{
	return tVector(0, 0, 0, 0);
}

void cSimSoftBody::SetLinearVelocity(const tVector&) {}
void cSimSoftBody::SetAngularVelocity(const tVector&) {}
void cSimSoftBody::ApplyForce(const tVector&, const tVector&) {}
void cSimSoftBody::ApplyForce(const tVector&) {}
void cSimSoftBody::ApplyTorque(const tVector&) {}
void cSimSoftBody::ClearForces() {}

tVector cSimSoftBody::GetSize() const
{
	return tVector(1, 1, 1, 0);
}

btCollisionObject* cSimSoftBody::GetCollisionObject()
{
	// this is ok, because I overrode all methods accessing this field
	return nullptr;
}

const btCollisionObject* cSimSoftBody::GetCollisionObject() const
{
	// this is ok, because I overrode all methods accessing this field
	return nullptr;
}