#include "SceneSimChar.h"

#include <memory>
#include <ctime>
#include <algorithm>

#include <Alembic/Abc/All.h>
#include <Alembic/AbcCoreAbstract/All.h>
#include <Alembic/AbcCoreOgawa/All.h>
#include <Alembic/AbcGeom/All.h>

#include "sim/SimBox.h"
#include "sim/SimRigidMesh.h"
#include "sim/SimSoftBody.h"
#include "sim/GroundPlane.h"
#include "sim/GroundBuilder.h"
#include "sim/DeepMimicCharController.h"

#include "util/FileUtil.h"

const int gDefaultCharID = 0;
const double gCharViewDistPad = 1;
const double cSceneSimChar::gGroundSpawnOffset = -1; // some padding to prevent parts of character from getting spawned inside obstacles

const size_t gInitGroundUpdateCount = std::numeric_limits<size_t>::max();

namespace serializeSceneSimChar {

	const float translationScale = 0.01;
	const tVector translationOffset(0, 0, 0, 0);

	typedef struct mesh {
		tVector rootPos;
		Eigen::AngleAxisd rootRot;
		std::vector<btScalar> vertices;
		std::vector<int32_t> indices;
		std::vector<float> normals;
		std::vector<float> uvs;

		void deserializeFromLastSample(
			Alembic::Abc::IArchive& archive,
			const std::string& transformName,
			const std::string& shapeName);
	} tMesh;

	void tMesh::deserializeFromLastSample(
		Alembic::Abc::IArchive& archive,
		const std::string& transformName,
		const std::string& shapeName)
	{
		auto rootTransform = Alembic::AbcGeom::IXform(archive.getTop(), transformName);
		auto polyMesh = Alembic::AbcGeom::IPolyMesh(rootTransform, shapeName);

		Alembic::AbcCoreAbstract::index_t lastIndexRootTransform = rootTransform.getSchema().getNumSamples() - 1;
		auto lastSampleSelectorRootTransform = Alembic::Abc::ISampleSelector(lastIndexRootTransform);

		Alembic::AbcCoreAbstract::index_t lastIndex = polyMesh.getSchema().getNumSamples() - 1;
		auto lastSampleSelectorMesh = Alembic::Abc::ISampleSelector(lastIndex);

		Alembic::AbcGeom::XformSample rootTransformSample;
		rootTransform.getSchema().get(rootTransformSample, lastSampleSelectorRootTransform);

		auto rootTransformTranslation = rootTransformSample.getTranslation();
		rootPos = tVector(rootTransformTranslation.x, rootTransformTranslation.y, rootTransformTranslation.z, 0);
		rootRot = Eigen::AngleAxisd(
			rootTransformSample.getAngle(),
			Eigen::Vector3d(
				rootTransformSample.getAxis().x,
				rootTransformSample.getAxis().y,
				rootTransformSample.getAxis().z));
		auto rootTransformScale = rootTransformSample.getScale();
		auto rootScale = tVector(rootTransformScale.x, rootTransformScale.y, rootTransformScale.z, 1.0);

		Eigen::IOFormat InlineVectorFmt(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "(", ")");
		std::cout << " found rootTransform with:" << std::endl \
			<< "  pos: " << rootPos.format(InlineVectorFmt) << std::endl \
			<< "  rot: " << rootRot.angle() << " @ " << rootRot.axis().format(InlineVectorFmt) << std::endl \
			<< "  sca: " << rootScale.format(InlineVectorFmt) << std::endl;

		// offset and scale transform positional values 
		rootPos = translationOffset + (rootPos * translationScale);

		Alembic::AbcGeom::IPolyMeshSchema::Sample meshSample;
		polyMesh.getSchema().get(meshSample, lastSampleSelectorMesh);

		int numVertices = meshSample.getPositions()->size();
		int numIndizes = meshSample.getFaceIndices()->size();

		auto vertexPtr = meshSample.getPositions()->get();
		auto indexPtr = meshSample.getFaceIndices()->get();

		/*
		auto normalsArray = polyMesh.getSchema().getNormalsParam().getExpandedValue().getVals();
		*/
		auto uvsArray = polyMesh.getSchema().getUVsParam().getExpandedValue().getVals();

		/*
		std::cout << " found " << numVertices << " vertices with " << numIndizes << " indizes and " <<
			normalsArray->size() << " normals " << uvsArray->size() << " uvs" << std::endl;
			*/

		vertices.clear();
		for (int i = 0; i < numVertices; ++i)
		{
			// scale by 100 to make cm unit export work between c4d and bullet
			auto& vec3 = vertexPtr[i] / 100.0;
			vertices.push_back(vec3.x);
			vertices.push_back(vec3.y);
			vertices.push_back(vec3.z);
		}

		indices.clear();
		std::vector<int32_t> indexCount(numVertices);
		std::vector<std::vector<int32_t>> adjacencies(numVertices);
		for (int i = 0; i < numIndizes / 3; ++i)
		{
			auto& i0 = indexPtr[i * 3 + 0];
			auto& i1 = indexPtr[i * 3 + 1];
			auto& i2 = indexPtr[i * 3 + 2];

			indexCount[i0]++;
			indexCount[i1]++;
			indexCount[i2]++;

			adjacencies[i0].push_back(i1);
			adjacencies[i1].push_back(i2);
			adjacencies[i2].push_back(i0);

			indices.push_back(i0);
			indices.push_back(i1);
			indices.push_back(i2);
		}

		/*
		normals = std::vector<float>((float*)normalsArray->get(), (float*)(normalsArray->get() + normalsArray->size()));
		*/
		uvs = std::vector<float>((float*)uvsArray->get(), (float*)(uvsArray->get() + uvsArray->size()));
		/*
		std::cout << " copied " << vertices.size() / 3 << " vertices with " << indices.size() << " indizes and " <<
			normals.size() / 3 << " normals " << uvs.size() / 2 << " uvs " << std::endl;
		*/
		/*
		auto posMinMax = std::minmax_element(vertices.begin(), vertices.end());
		auto idxMinMax = std::minmax_element(indices.begin(), indices.end());
		auto normMinMax = std::minmax_element(normals.begin(), normals.end());
		auto uvMinMax = std::minmax_element(uvs.begin(), uvs.end());

		std::cout << " pos min: " << *posMinMax.first << " max: " << *posMinMax.second << std::endl;
		std::cout << " idx min: " << *idxMinMax.first << " max: " << *idxMinMax.second << std::endl;
		std::cout << " norm min: " << *normMinMax.first << " max: " << *normMinMax.second << std::endl;
		std::cout << " uv min: " << *uvMinMax.first << " max: " << *uvMinMax.second << std::endl
		*/

		/*
		std::cout << " idx histogram:" << std::endl;
		for (int i = 0; i < numVertices; ++i)
		{
			std::cout << " " << i << ": " << indexCount[i] << std::endl;
		}
		*/

		/*
		std::cout << " vertex adjacencies:" << std::endl;
		for (int32_t i = 0; i < numVertices; ++i)
		{
			std::cout << " " << i << ": ";
			for (auto& adj : adjacencies[i])
			{
				std::cout << adj;

				auto& reverse = adjacencies[adj];
				auto found = std::find(reverse.begin(), reverse.end(), i);
				if (found == reverse.end())
				{
					std::cout << "! ";
				}
				else
				{
					std::cout << ", ";
				}
			}
			std::cout << std::endl;
		}
		*/
;
	}

	typedef struct rigidBodyRecording {
		std::string name;
		int handle = 0;
		tMesh shape;
		std::vector<tMatrix> frames;

		void serializeFrameCache(Alembic::Abc::OArchive& archive) const;
	} tRigidBodyRecording;

	typedef struct softBodyRecording {
		int handle = 0;
		tMesh shape;
		std::vector<std::vector<btScalar>> frames;

		void serializeFrameCache(Alembic::Abc::OArchive& archive) const;
	} tSoftBodyRecording;

	double timePassed = 0;

	std::vector<tRigidBodyRecording> rigidBodyRecordings;
	tSoftBodyRecording softBodyRecording;
	cSimSoftBody* simSoftBody = nullptr;

	void tRigidBodyRecording::serializeFrameCache(Alembic::Abc::OArchive& archive) const
	{
		auto frameDuration = timePassed / frames.size();
		auto timeSampling = Alembic::AbcCoreAbstract::TimeSampling(frameDuration, 0.0f);
		auto timeSamplingIdx = archive.addTimeSampling(timeSampling);

		auto abcXform = Alembic::AbcGeom::OXform(archive.getTop(), name);
		auto abcShape = Alembic::AbcGeom::OPolyMesh(abcXform, name + "Shape");

		abcXform.getSchema().setTimeSampling(timeSamplingIdx);
		abcShape.getSchema().setTimeSampling(timeSamplingIdx);
		abcShape.getSchema().setUVSourceName("uv");

		std::vector<Alembic::Abc::V3f> vertices;
		for (int i = 0; i < shape.vertices.size() / 3; ++i)
		{
			// scale by 100 to make cm unit export work between c4d and bullet

			auto x = shape.vertices[i * 3 + 0] * 100.0;
			auto y = shape.vertices[i * 3 + 1] * 100.0;
			auto z = shape.vertices[i * 3 + 2] * 100.0;

			vertices.push_back(Alembic::Abc::V3f(x, y, z));
		}

		std::vector<Alembic::Abc::int32_t> indices;
		for (int i = 0; i < shape.indices.size() / 3; ++i)
		{
			auto& i0 = shape.indices[i * 3 + 0];
			auto& i1 = shape.indices[i * 3 + 1];
			auto& i2 = shape.indices[i * 3 + 2];

			// invert index order to convert handedness
			indices.push_back(i0);
			indices.push_back(i1);
			indices.push_back(i2);
		}

		std::vector<Alembic::Abc::int32_t> faceCounts;
		for (int i = 0; i < indices.size() / 3; ++i)
		{
			faceCounts.push_back(3);
		}

		std::vector<Alembic::Abc::V2f> uvs;
		for (int i = 0; i < shape.uvs.size() / 2; ++i)
		{
			auto u = shape.uvs[i * 2 + 0];
			auto v = shape.uvs[i * 2 + 1];

			uvs.push_back(Alembic::Abc::V2f(u, v));
		}

		Alembic::AbcGeom::OV2fGeomParam::Sample uvSample(
			Alembic::Abc::V2fArraySample(
				uvs.data(),
				uvs.size()),
			Alembic::AbcGeom::kFacevaryingScope);

		std::vector<Alembic::Abc::N3f> normals;
		for (int i = 0; i < shape.normals.size() / 3; ++i)
		{
			auto x = shape.normals[i * 3 + 0];
			auto y = shape.normals[i * 3 + 1];
			auto z = shape.normals[i * 3 + 2];

			normals.push_back(Alembic::Abc::N3f(x, y, z));
		}

		Alembic::AbcGeom::ON3fGeomParam::Sample normalSample(
			Alembic::Abc::N3fArraySample(
				normals.data(),
				normals.size()),
			Alembic::AbcGeom::kFacevaryingScope);

		std::cout << " encoded " << vertices.size() << " vertices " << indices.size() << " indizes " << faceCounts.size() << " faces" << std::endl;

		bool firstFrame = true;

		for (const auto& f : frames)
		{
			Alembic::AbcGeom::M44d mat;
			for (auto y = 0; y < 4; ++y)
			{
				for (auto x = 0; x < 4; ++x)
				{
					if (x == 3 && y < 3)
						mat[x][y] = 100 * f(y, x);
					else
						mat[x][y] = f(y, x);
				}
			}

			Alembic::AbcGeom::XformSample xformSample;
			xformSample.setMatrix(mat);
			abcXform.getSchema().set(xformSample);

			Alembic::AbcGeom::OPolyMeshSchema::Sample meshSample;

			meshSample.setPositions(
				Alembic::Abc::P3fArraySample(
					vertices.data(),
					vertices.size()));

			if (firstFrame)
			{
				meshSample.setFaceIndices(
					Alembic::Abc::Int32ArraySample(
						indices.data(),
						indices.size()));

				meshSample.setFaceCounts(
					Alembic::Abc::Int32ArraySample(
						faceCounts.data(),
						faceCounts.size()
					)
				);

				meshSample.setUVs(uvSample);
				//meshSample.setNormals(normals);

				firstFrame = false;
			}

			abcShape.getSchema().set(meshSample);
		}
	}


	void tSoftBodyRecording::serializeFrameCache(Alembic::Abc::OArchive& archive) const
	{
		auto frameDuration = timePassed / frames.size();
		auto timeSampling = Alembic::AbcCoreAbstract::TimeSampling(frameDuration, 0.0f);
		auto timeSamplingIdx = archive.addTimeSampling(timeSampling);

		auto name = std::string("softBody");
		auto abcXform = Alembic::AbcGeom::OXform(archive.getTop(), name);
		auto abcShape = Alembic::AbcGeom::OPolyMesh(abcXform, name + "Shape");

		abcXform.getSchema().setTimeSampling(timeSamplingIdx);
		abcShape.getSchema().setTimeSampling(timeSamplingIdx);
		abcShape.getSchema().setUVSourceName("uv");


		std::vector<Alembic::Abc::int32_t> indices;
		for (int i = 0; i < shape.indices.size() / 3; ++i)
		{
			auto& i0 = shape.indices[i * 3 + 0];
			auto& i1 = shape.indices[i * 3 + 1];
			auto& i2 = shape.indices[i * 3 + 2];

			// invert index order to convert handedness
			indices.push_back(i0);
			indices.push_back(i1);
			indices.push_back(i2);
		}

		std::vector<Alembic::Abc::int32_t> faceCounts;
		for (int i = 0; i < indices.size() / 3; ++i)
		{
			faceCounts.push_back(3);
		}

		std::vector<Alembic::Abc::V2f> uvs;
		for (int i = 0; i < shape.uvs.size() / 2; ++i)
		{
			auto u = shape.uvs[i * 2 + 0];
			auto v = shape.uvs[i * 2 + 1];

			uvs.push_back(Alembic::Abc::V2f(u, v));
		}

		Alembic::AbcGeom::OV2fGeomParam::Sample uvSample(
			Alembic::Abc::V2fArraySample(
				uvs.data(),
				uvs.size()),
			Alembic::AbcGeom::kFacevaryingScope);

		std::vector<Alembic::Abc::N3f> normals;
		for (int i = 0; i < shape.normals.size() / 3; ++i)
		{
			auto x = shape.normals[i * 3 + 0];
			auto y = shape.normals[i * 3 + 1];
			auto z = shape.normals[i * 3 + 2];

			normals.push_back(Alembic::Abc::N3f(x, y, z));
		}

		Alembic::AbcGeom::ON3fGeomParam::Sample normalSample(
			Alembic::Abc::N3fArraySample(
				normals.data(),
				normals.size()),
			Alembic::AbcGeom::kFacevaryingScope);

		std::cout << " encoded " << indices.size() << " indizes " << faceCounts.size() << " faces" << std::endl;


		Alembic::AbcGeom::M44d mat;

		bool firstFrame = true;

		for (const auto& f : frames)
		{
			Alembic::AbcGeom::XformSample xformSample;
			xformSample.setMatrix(mat);
			abcXform.getSchema().set(xformSample);

			std::vector<Alembic::Abc::V3f> vertices;
			for (int i = 0; i < f.size() / 3; ++i)
			{
				// scale by 100 to make cm unit export work between c4d and bullet

				auto x = f[i * 3 + 0] * 100.0;
				auto y = f[i * 3 + 1] * 100.0;
				auto z = f[i * 3 + 2] * 100.0;

				vertices.push_back(Alembic::Abc::V3f(x, y, z));
			}


			Alembic::AbcGeom::OPolyMeshSchema::Sample meshSample;
			meshSample.setPositions(
				Alembic::Abc::P3fArraySample(
					vertices.data(),
					vertices.size()));

			if (firstFrame)
			{
				meshSample.setFaceIndices(
					Alembic::Abc::Int32ArraySample(
						indices.data(),
						indices.size()));

				meshSample.setFaceCounts(
					Alembic::Abc::Int32ArraySample(
						faceCounts.data(),
						faceCounts.size()
					)
				);

				meshSample.setUVs(uvSample);
				//meshSample.setNormals(normals);

				firstFrame = false;
			}

			abcShape.getSchema().set(meshSample);
		}
	}
}

using namespace serializeSceneSimChar;

cSceneSimChar::tObjEntry::tObjEntry()
{
	mObj = nullptr;
	mEndTime = std::numeric_limits<double>::infinity();
	mColor = tVector(0.5, 0.5, 0.5, 1);
	mPersist = false;
}

bool cSceneSimChar::tObjEntry::IsValid() const
{
	return mObj != nullptr;
}

cSceneSimChar::tJointEntry::tJointEntry()
{
	mJoint = nullptr;
}

bool cSceneSimChar::tJointEntry::IsValid() const
{
	return mJoint != nullptr;
}

cSceneSimChar::tPerturbParams::tPerturbParams()
{
	mEnableRandPerturbs = false;
	mTimer = 0;
	mTimeMin = std::numeric_limits<double>::infinity();
	mTimeMax = std::numeric_limits<double>::infinity();
	mNextTime = 0;
	mMinPerturb = 50;
	mMaxPerturb = 100;
	mMinDuration = 0.1;
	mMaxDuration = 0.5;
}

cSceneSimChar::cSceneSimChar()
{
	mEnableContactFall = true;
	mEnableRandCharPlacement = true;

	mWorldParams.mNumSubsteps = 1;
	mWorldParams.mScale = 1;
	mWorldParams.mGravity = gGravity;

	mRigidWorldInputPath = "";
	mRigidWorldOutputPath = "";
	mMattressInputPath = "";
	mMattressOutputPath = "";
}

cSceneSimChar::~cSceneSimChar()
{
	Clear();
}

void cSceneSimChar::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScene::ParseArgs(parser);

	bool succ = true;

	parser->ParseBool("enable_char_contact_fall", mEnableContactFall);
	parser->ParseBool("enable_rand_char_placement", mEnableRandCharPlacement);
	
	succ &= ParseCharTypes(parser, mCharTypes);
	succ &= ParseCharParams(parser, mCharParams);
	succ &= ParseCharCtrlParams(parser, mCtrlParams);
	if (mCharParams.size() != mCtrlParams.size())
	{
		printf("Char and ctrl file mismatch, %zi vs %zi\n", mCharParams.size(), mCtrlParams.size());
		assert(false);
	}

	std::string sim_mode_str = "";
	parser->ParseInt("num_sim_substeps", mWorldParams.mNumSubsteps);
	parser->ParseDouble("world_scale", mWorldParams.mScale);
	parser->ParseVector("gravity", mWorldParams.mGravity);

	parser->ParseBool("enable_rand_perturbs", mPerturbParams.mEnableRandPerturbs);
	parser->ParseDouble("perturb_time_min", mPerturbParams.mTimeMin);
	parser->ParseDouble("perturb_time_max", mPerturbParams.mTimeMax);
	parser->ParseDouble("min_perturb", mPerturbParams.mMinPerturb);
	parser->ParseDouble("max_perturb", mPerturbParams.mMaxPerturb);
	parser->ParseDouble("min_pertrub_duration", mPerturbParams.mMinDuration);
	parser->ParseDouble("max_perturb_duration", mPerturbParams.mMaxDuration);
	parser->ParseInts("perturb_part_ids", mPerturbParams.mPerturbPartIDs);

	parser->ParseInts("fall_contact_bodies", mFallContactBodies);

	parser->ParseString("rigid_world_input_path", mRigidWorldInputPath);
	parser->ParseString("rigid_world_output_path", mRigidWorldOutputPath);

	parser->ParseString("mattress_input_path", mMattressInputPath);
	parser->ParseString("mattress_output_path", mMattressOutputPath);

	ParseGroundParams(parser, mGroundParams);
}

void cSceneSimChar::Init()
{
	cScene::Init();

	if (mPerturbParams.mEnableRandPerturbs)
	{
		ResetRandPertrub();
	}

	BuildWorld();
	BuildGround();
	BuildCharacters();

	InitCharacterPos();
	ResolveCharGroundIntersect();

	ClearObjs();
}

void cSceneSimChar::Clear()
{
	cScene::Clear();

	mChars.clear();
	mGround.reset();
	mFallContactBodies.clear();
	ClearJoints();
	ClearObjs();
}

void cSceneSimChar::Update(double time_elapsed)
{
	cScene::Update(time_elapsed);

	if (time_elapsed < 0)
	{
		return;
	}

	if (mPerturbParams.mEnableRandPerturbs)
	{
		UpdateRandPerturb(time_elapsed);
	}

	PreUpdate(time_elapsed);

	// order matters!
	UpdateCharacters(time_elapsed);
	UpdateWorld(time_elapsed);
	UpdateGround(time_elapsed);
	UpdateObjs(time_elapsed);
	UpdateJoints(time_elapsed);

	PostUpdateCharacters(time_elapsed);
	PostUpdate(time_elapsed);
}

int cSceneSimChar::GetNumChars() const
{
	return static_cast<int>(mChars.size());
}

const std::shared_ptr<cSimCharacter>& cSceneSimChar::GetCharacter()  const
{
	return GetCharacter(gDefaultCharID);
}

const std::shared_ptr<cSimCharacter>& cSceneSimChar::GetCharacter(int char_id) const
{
	return mChars[char_id];
}

const std::shared_ptr<cWorld>& cSceneSimChar::GetWorld() const
{
	return mWorld;
}

tVector cSceneSimChar::GetCharPos() const
{
	return GetCharacter()->GetRootPos();
}

const std::shared_ptr<cGround>& cSceneSimChar::GetGround() const
{
	return mGround;
}

const tVector& cSceneSimChar::GetGravity() const
{
	return mWorldParams.mGravity;
}

bool cSceneSimChar::LoadControlParams(const std::string& param_file, const std::shared_ptr<cSimCharacter>& out_char)
{
	const auto& ctrl = out_char->GetController();
	bool succ = ctrl->LoadParams(param_file);
	return succ;
}

void cSceneSimChar::AddPerturb(const tPerturb& perturb)
{
	mWorld->AddPerturb(perturb);
}

void cSceneSimChar::ApplyRandForce(double min_force, double max_force, 
									double min_dur, double max_dur, cSimObj* obj)
{
	assert(obj != nullptr);
	tPerturb perturb = tPerturb::BuildForce();
	perturb.mObj = obj;
	perturb.mLocalPos.setZero();
	perturb.mPerturb[0] = mRand.RandDouble(-1, 1);
	perturb.mPerturb[1] = mRand.RandDouble(-1, 1);
	perturb.mPerturb[2] = mRand.RandDouble(-1, 1);
	perturb.mPerturb = mRand.RandDouble(min_force, max_force) * perturb.mPerturb.normalized();
	perturb.mDuration = mRand.RandDouble(min_dur, max_dur);

	AddPerturb(perturb);
}

void cSceneSimChar::ApplyRandForce()
{
	for (int i = 0; i < GetNumChars(); ++i)
	{
		ApplyRandForce(i);
	}
}

void cSceneSimChar::ApplyRandForce(int char_id)
{
	const std::shared_ptr<cSimCharacter>& curr_char = GetCharacter(char_id);
	int num_parts = curr_char->GetNumBodyParts();
	int part_idx = GetRandPerturbPartID(curr_char);
	assert(part_idx != gInvalidIdx);
	const auto& part = curr_char->GetBodyPart(part_idx);
	ApplyRandForce(mPerturbParams.mMinPerturb, mPerturbParams.mMaxPerturb, mPerturbParams.mMinDuration, mPerturbParams.mMaxDuration, part.get());
}

int cSceneSimChar::GetRandPerturbPartID(const std::shared_ptr<cSimCharacter>& character)
{
	int rand_id = gInvalidIdx;
	int num_part_ids = static_cast<int>(mPerturbParams.mPerturbPartIDs.size());
	if (num_part_ids > 0)
	{
		int idx = mRand.RandInt(0, num_part_ids);
		rand_id = mPerturbParams.mPerturbPartIDs[idx];
	}
	else
	{
		int num_parts = character->GetNumBodyParts();
		rand_id = mRand.RandInt(0, num_parts);
	}
	return rand_id;
}

void cSceneSimChar::RayTest(const tVector& beg, const tVector& end, cWorld::tRayTestResult& out_result) const
{
	cWorld::tRayTestResults results;
	mWorld->RayTest(beg, end, results);

	out_result.mObj = nullptr;
	if (results.size() > 0)
	{
		out_result = results[0];
	}
}

void cSceneSimChar::SetGroundParamBlend(double lerp)
{
	mGround->SetParamBlend(lerp);
}

int cSceneSimChar::GetNumParamSets() const
{
	return static_cast<int>(mGroundParams.mParamArr.rows());
}

void cSceneSimChar::OutputCharState(const std::string& out_file) const
{
	const auto& char0 = GetCharacter();
	tVector root_pos = char0->GetRootPos();
	double ground_h = mGround->SampleHeight(root_pos);
	tMatrix trans = char0->BuildOriginTrans();
	trans(1, 3) -= ground_h;

	char0->WriteState(out_file, trans);
}

void cSceneSimChar::OutputGround(const std::string& out_file) const
{
	mGround->Output(out_file);
}

std::string cSceneSimChar::GetName() const
{
	return "Sim Character";
}

bool cSceneSimChar::BuildCharacters()
{
	bool succ = true;
	mChars.clear();

	int num_chars = static_cast<int>(mCharParams.size());
	for (int i = 0; i < num_chars; ++i)
	{
		const cSimCharacter::tParams& curr_params = mCharParams[i];
		std::shared_ptr<cSimCharacter> curr_char;

		cSimCharBuilder::eCharType char_type = cSimCharBuilder::cCharGeneral;
		if (mCharTypes.size() > i)
		{
			char_type = mCharTypes[i];
		}
		cSimCharBuilder::CreateCharacter(char_type, curr_char);

		succ &= curr_char->Init(mWorld, curr_params);
		if (succ)
		{
			SetFallContacts(mFallContactBodies, *curr_char);
			curr_char->RegisterContacts(cWorld::eContactFlagCharacter, cWorld::eContactFlagEnvironment);

			InitCharacterPos(curr_char);

			if (i < mCtrlParams.size())
			{
				auto ctrl_params = mCtrlParams[i];
				ctrl_params.mChar = curr_char;
				ctrl_params.mGravity = GetGravity();
				ctrl_params.mGround = mGround;

				std::shared_ptr<cCharController> ctrl;
				succ = BuildController(ctrl_params, ctrl);
				if (succ && ctrl != nullptr)
				{
					curr_char->SetController(ctrl);
				}
			}

			mChars.push_back(curr_char);
		}
	}
	
	return succ;
}

bool cSceneSimChar::ParseCharTypes(const std::shared_ptr<cArgParser>& parser, std::vector<cSimCharBuilder::eCharType>& out_types) const
{
	bool succ = true;
	std::vector<std::string> char_type_strs;
	succ = parser->ParseStrings("char_types", char_type_strs);

	int num = static_cast<int>(char_type_strs.size());
	out_types.clear();
	for (int i = 0; i < num; ++i)
	{
		std::string str = char_type_strs[i];
		cSimCharBuilder::eCharType char_type = cSimCharBuilder::eCharNone;
		cSimCharBuilder::ParseCharType(str, char_type);

		if (char_type != cSimCharBuilder::eCharNone)
		{
			out_types.push_back(char_type);
		}
	}

	return succ;
}

bool cSceneSimChar::ParseCharParams(const std::shared_ptr<cArgParser>& parser, std::vector<cSimCharacter::tParams>& out_params) const
{
	bool succ = true;

	std::vector<std::string> char_files;
	succ = parser->ParseStrings("character_files", char_files);

	std::vector<std::string> state_files;
	parser->ParseStrings("state_files", state_files);

	std::vector<double> init_pos_xs;
	parser->ParseDoubles("char_init_pos_xs", init_pos_xs);
	
	int num_files = static_cast<int>(char_files.size());
	out_params.resize(num_files);
	for (int i = 0; i < num_files; ++i)
	{
		cSimCharacter::tParams& params = out_params[i];
		params.mID = i;
		params.mCharFile = char_files[i];
		
		params.mEnableContactFall = mEnableContactFall;

		if (state_files.size() > i)
		{
			params.mStateFile = state_files[i];
		}

		if (init_pos_xs.size() > i)
		{
			params.mInitPos[0] = init_pos_xs[i];
		}
	}

	if (!succ)
	{
		printf("No valid character file specified.\n");
	}

	return succ;
}

bool cSceneSimChar::ParseCharCtrlParams(const std::shared_ptr<cArgParser>& parser, std::vector<cCtrlBuilder::tCtrlParams>& out_params) const
{
	bool succ = true;

	std::vector<std::string> ctrl_files;
	parser->ParseStrings("char_ctrl_files", ctrl_files);

	int num_ctrls = static_cast<int>(ctrl_files.size());

	std::vector<std::string> char_ctrl_strs;
	parser->ParseStrings("char_ctrls", char_ctrl_strs);

	out_params.resize(num_ctrls);
	for (int i = 0; i < num_ctrls; ++i)
	{
		auto& ctrl_params = out_params[i];
		const std::string& type_str = char_ctrl_strs[i];
		cCtrlBuilder::ParseCharCtrl(type_str, ctrl_params.mCharCtrl);
		ctrl_params.mCtrlFile = ctrl_files[i];
	}

	return succ;
}

void cSceneSimChar::BuildWorld()
{
	mWorld = std::shared_ptr<cWorld>(new cWorld());
	mWorld->Init(mWorldParams);
}

void cSceneSimChar::BuildGround()
{
	mGroundParams.mHasRandSeed = mHasRandSeed;
	mGroundParams.mRandSeed = mRandSeed;
	cGroundBuilder::BuildGround(mWorld, mGroundParams, mGround);
}

bool cSceneSimChar::BuildController(const cCtrlBuilder::tCtrlParams& ctrl_params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = cCtrlBuilder::BuildController(ctrl_params, out_ctrl);
	return succ;
}

void cSceneSimChar::SetFallContacts(const std::vector<int>& fall_bodies, cSimCharacter& out_char) const
{
	int num_fall_bodies = static_cast<int>(fall_bodies.size());
	if (num_fall_bodies > 0)
	{
		for (int i = 0; i < out_char.GetNumBodyParts(); ++i)
		{
			out_char.SetBodyPartFallContact(i, false);
		}

		for (int i = 0; i < num_fall_bodies; ++i)
		{
			int b = fall_bodies[i];
			out_char.SetBodyPartFallContact(b, true);
		}
	}
}

void cSceneSimChar::InitCharacterPos()
{
	int num_chars = GetNumChars();
	for (int i = 0; i < num_chars; ++i)
	{
		InitCharacterPos(mChars[i]);
	}
}

void cSceneSimChar::InitCharacterPos(const std::shared_ptr<cSimCharacter>& out_char)
{
	if (mEnableRandCharPlacement)
	{
		SetCharRandPlacement(out_char);
	}
	else
	{
		InitCharacterPosFixed(out_char);
	}
}

void cSceneSimChar::InitCharacterPosFixed(const std::shared_ptr<cSimCharacter>& out_char)
{
	tVector root_pos = out_char->GetRootPos();
	int char_id = out_char->GetID();
	root_pos[0] = mCharParams[char_id].mInitPos[0];

	double h = mGround->SampleHeight(root_pos);
	root_pos[1] += h;

	out_char->SetRootPos(root_pos);
}

void cSceneSimChar::SetCharRandPlacement(const std::shared_ptr<cSimCharacter>& out_char)
{
	tVector rand_pos = tVector::Zero();
	tQuaternion rand_rot = tQuaternion::Identity();
	CalcCharRandPlacement(out_char, rand_pos, rand_rot);
	out_char->SetRootTransform(rand_pos, rand_rot);
}

void cSceneSimChar::CalcCharRandPlacement(const std::shared_ptr<cSimCharacter>& out_char, tVector& out_pos, tQuaternion& out_rot)
{
	tVector char_pos = out_char->GetRootPos();
	tQuaternion char_rot = out_char->GetRootRotation();

	tVector rand_pos;
	tQuaternion rand_rot;
	mGround->SamplePlacement(tVector::Zero(), rand_pos, rand_rot);

	out_pos = rand_pos;
	out_pos[1] += char_pos[1];
	out_rot = rand_rot * char_rot;
}

void cSceneSimChar::ResolveCharGroundIntersect()
{
	int num_chars = GetNumChars();
	for (int i = 0; i < num_chars; ++i)
	{
		ResolveCharGroundIntersect(mChars[i]);
	}
}

void cSceneSimChar::ResolveCharGroundIntersect(const std::shared_ptr<cSimCharacter>& out_char) const
{
	const double pad = 0.001;

	int num_parts = out_char->GetNumBodyParts();
	double min_violation = 0;
	for (int b = 0; b < num_parts; ++b)
	{
		if (out_char->IsValidBodyPart(b))
		{
			tVector aabb_min;
			tVector aabb_max;
			const auto& part = out_char->GetBodyPart(b);
			part->CalcAABB(aabb_min, aabb_max);

			tVector mid = 0.5 * (aabb_min + aabb_max);
			tVector sw = tVector(aabb_min[0], 0, aabb_min[2], 0);
			tVector nw = tVector(aabb_min[0], 0, aabb_max[2], 0);
			tVector ne = tVector(aabb_max[0], 0, aabb_max[2], 0);
			tVector se = tVector(aabb_max[0], 0, aabb_min[2], 0);

			double max_ground_height = 0;
			max_ground_height = mGround->SampleHeight(aabb_min);
			max_ground_height = std::max(max_ground_height, mGround->SampleHeight(mid));
			max_ground_height = std::max(max_ground_height, mGround->SampleHeight(sw));
			max_ground_height = std::max(max_ground_height, mGround->SampleHeight(nw));
			max_ground_height = std::max(max_ground_height, mGround->SampleHeight(ne));
			max_ground_height = std::max(max_ground_height, mGround->SampleHeight(se));
			max_ground_height += pad;

			double min_height = aabb_min[1];
			min_violation = std::min(min_violation, min_height - max_ground_height);
		}
	}

	if (min_violation < 0)
	{
		tVector root_pos = out_char->GetRootPos();
		root_pos[1] += -min_violation;
		out_char->SetRootPos(root_pos);
	}
}

void cSceneSimChar::UpdateWorld(double time_step)
{
	mWorld->Update(time_step);
}

void cSceneSimChar::UpdateCharacters(double time_step)
{
	int num_chars = GetNumChars();
	for (int i = 0; i < num_chars; ++i)
	{
		const auto& curr_char = GetCharacter(i);
		curr_char->Update(time_step);
	}
}

void cSceneSimChar::PostUpdateCharacters(double time_step)
{
	int num_chars = GetNumChars();
	for (int i = 0; i < num_chars; ++i)
	{
		const auto& curr_char = GetCharacter(i);
		curr_char->PostUpdate(time_step);
	}
}

void cSceneSimChar::UpdateGround(double time_elapsed)
{
	tVector view_min;
	tVector view_max;
	GetViewBound(view_min, view_max);
	mGround->Update(time_elapsed, view_min, view_max);
}

void cSceneSimChar::UpdateRandPerturb(double time_step)
{
	mPerturbParams.mTimer += time_step;
	if (mPerturbParams.mTimer >= mPerturbParams.mNextTime)
	{
		ApplyRandForce();
		ResetRandPertrub();
	}
}



void cSceneSimChar::ResetScene()
{
	cScene::ResetScene();

	if (mPerturbParams.mEnableRandPerturbs)
	{
		ResetRandPertrub();
	}

	ResetWorld();
	ResetCharacters();
	ResetGround();
	CleanObjs();

	InitCharacterPos();
	ResolveCharGroundIntersect();

	// manual breakpoint for debug
	// std::cin.get();

	if (timePassed > 0.1 && rigidBodyRecordings.size() > 0 && !mRigidWorldOutputPath.empty())
	{
		auto framesRecorded = rigidBodyRecordings[0].frames.size();
		auto frameDuration = timePassed / framesRecorded;

		std::cout << "cSceneSimChar::ResetScene " << framesRecorded \
			<< " frames in " << timePassed \
			<< " seconds, per frame " << frameDuration \
			<< " writing " << rigidBodyRecordings.size() \
			<< " rigidbodies to " << mRigidWorldOutputPath << std::endl;

		auto archive = Alembic::Abc::OArchive(
			Alembic::AbcCoreOgawa::WriteArchive(),
			mRigidWorldOutputPath,
			Alembic::Abc::ErrorHandler::kThrowPolicy);

		for (const auto& rigidBodyRecoding : rigidBodyRecordings)
		{
			rigidBodyRecoding.serializeFrameCache(archive);
		}
	}

	if (timePassed > 0.1 && !mMattressOutputPath.empty())
	{
		auto framesRecorded = softBodyRecording.frames.size();
		auto frameDuration = timePassed / framesRecorded;

		std::cout << "cSceneSimChar::ResetScene " << framesRecorded \
			<< " frames in " << timePassed \
			<< " seconds, per frame " << frameDuration \
			<< " writing softbody to " << mMattressOutputPath << std::endl;

		auto archive = Alembic::Abc::OArchive(
			Alembic::AbcCoreOgawa::WriteArchive(),
			mMattressOutputPath,
			Alembic::Abc::ErrorHandler::kThrowPolicy);

		softBodyRecording.serializeFrameCache(archive);
	}

	rigidBodyRecordings.clear();
	softBodyRecording = {};
	timePassed = 0;

	if (!mRigidWorldInputPath.empty())
	{
		std::cout << "cSceneSimChar::ResetScene reading rigid world from " << mRigidWorldInputPath << std::endl;

		auto archive = Alembic::Abc::IArchive(
			Alembic::AbcCoreOgawa::ReadArchive(),
			mRigidWorldInputPath,
			Alembic::Abc::ErrorHandler::kThrowPolicy);

		for (int i = 0; i < archive.getTop().getNumChildren(); ++i)
		{
			auto rigidBodyRoot = archive.getTop().getChild(i);
			auto rigidBodyName = rigidBodyRoot.getName();
			auto rigidBodyShapeName = rigidBodyRoot.getChild(0).getName();
			std::cout << " found rigidBody " << rigidBodyName << " with shape " << rigidBodyShapeName << std::endl;

			tRigidBodyRecording recording;
			recording.name = rigidBodyName;
			recording.shape.deserializeFromLastSample(archive, rigidBodyName, rigidBodyShapeName);

			auto isStatic = rigidBodyName.find("static_") == 0;
			recording.handle = SpawnRigidMesh(&recording.shape, isStatic);

			rigidBodyRecordings.push_back(recording);
		}
	}

	if (!mMattressInputPath.empty())
	{
		std::cout << "cSceneSimChar::ResetScene reading soft body from " << mMattressInputPath << std::endl;

		auto archive = Alembic::Abc::IArchive(
			Alembic::AbcCoreOgawa::ReadArchive(),
			mMattressInputPath,
			Alembic::Abc::ErrorHandler::kThrowPolicy);

		auto matressRoot = archive.getTop().getChild(0);
		auto matressName = matressRoot.getName();
		auto matressShapeName = matressRoot.getChild(0).getName();
		std::cout << " found rigidBody " << matressName << " with shape " << matressShapeName << std::endl;

		tSoftBodyRecording recording;
		recording.shape.deserializeFromLastSample(archive, matressName, matressShapeName);
		recording.handle = SpawnSoftMesh(&recording.shape);

		softBodyRecording = recording;
	}
}

void cSceneSimChar::ResetCharacters()
{
	int num_chars = GetNumChars();
	for (int i = 0; i < num_chars; ++i)
	{
		const auto& curr_char = GetCharacter(i);
		curr_char->Reset();
	}
}

void cSceneSimChar::ResetWorld()
{
	mWorld->Reset();
}

void cSceneSimChar::ResetGround()
{
	mGround->Clear();

	tVector view_min;
	tVector view_max;
	GetViewBound(view_min, view_max);

	tVector view_size = view_max - view_min;
	view_min = -view_size;
	view_max = view_size;

	view_min[0] += gGroundSpawnOffset;
	view_max[0] += gGroundSpawnOffset;
	view_min[2] += gGroundSpawnOffset;
	view_max[2] += gGroundSpawnOffset;

	mGround->Update(0, view_min, view_max);
}

void cSceneSimChar::PreUpdate(double timestep)
{
	ClearJointForces();
}

void cSceneSimChar::PostUpdate(double timestep)
{
	if (!mRigidWorldInputPath.empty() && !mRigidWorldOutputPath.empty())
	{
		for (auto& rigidBody : rigidBodyRecordings)
		{

			auto mesh = std::dynamic_pointer_cast<cSimRigidMesh>(GetObj(rigidBody.handle));
			auto worldTransform = mesh->GetWorldTransform();

			rigidBody.frames.push_back(worldTransform);
		}
	}
	if (!mMattressInputPath.empty() && !mMattressOutputPath.empty())
	{
		auto worldTransform = simSoftBody->GetWorldTransform();
		auto vertices = simSoftBody->GetVertexPositions();
		softBodyRecording.frames.push_back(vertices);
	}

	timePassed += timestep;
}

void cSceneSimChar::GetViewBound(tVector& out_min, tVector& out_max) const
{
	const std::shared_ptr<cSimCharacter>& character = GetCharacter();
	const cDeepMimicCharController* ctrl = reinterpret_cast<cDeepMimicCharController*>(character->GetController().get());

	out_min.setZero();
	out_max.setZero();
	if (ctrl != nullptr)
	{
		ctrl->GetViewBound(out_min, out_max);
	}
	else
	{
		character->CalcAABB(out_min, out_max);
	}

	out_min += tVector(-gCharViewDistPad, 0, -gCharViewDistPad, 0);
	out_max += tVector(gCharViewDistPad, 0, gCharViewDistPad, 0);
}

void cSceneSimChar::ParseGroundParams(const std::shared_ptr<cArgParser>& parser, cGround::tParams& out_params) const
{
	std::string terrain_file = "";
	parser->ParseString("terrain_file", terrain_file);
	parser->ParseDouble("terrain_blend", out_params.mBlend);

	if (terrain_file != "")
	{
		bool succ = cGroundBuilder::ParseParamsJson(terrain_file, out_params);
		if (!succ)
		{
			printf("Failed to parse terrain params from %s\n", terrain_file.c_str());
			assert(false);
		}
	}
}


void cSceneSimChar::UpdateObjs(double time_step)
{
	int num_objs = GetNumObjs();
	for (int i = 0; i < num_objs; ++i)
	{
		const tObjEntry& obj = mObjs[i];
		if (obj.IsValid() && obj.mEndTime <= GetTime())
		{
			RemoveObj(i);
		}
	}
}

void cSceneSimChar::UpdateJoints(double timestep)
{
	int num_joints = GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		const tJointEntry& joint = mJoints[j];
		if (joint.IsValid())
		{
			joint.mJoint->ApplyTau();
		}
	}
}

void cSceneSimChar::ClearJointForces()
{
	int num_joints = GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		const tJointEntry& joint = mJoints[j];
		if (joint.IsValid())
		{
			joint.mJoint->ClearTau();
		}
	}
}

void cSceneSimChar::ClearObjs()
{
	mObjs.Clear();
}

void cSceneSimChar::CleanObjs()
{
	int idx = 0;
	for (int i = 0; i < GetNumObjs(); ++i)
	{
		const tObjEntry& entry = mObjs[i];
		if (entry.IsValid() && !entry.mPersist)
		{
			RemoveObj(i);
		}
	}
}

int cSceneSimChar::AddObj(const tObjEntry& obj_entry)
{
	int handle = static_cast<int>(mObjs.Add(obj_entry));
	return handle;
}

void cSceneSimChar::RemoveObj(int handle)
{
	assert(handle != gInvalidIdx);
	mObjs[handle].mObj.reset();
	mObjs.Free(handle);
}


void cSceneSimChar::ClearJoints()
{
	mJoints.Clear();
}

int cSceneSimChar::AddJoint(const tJointEntry& joint_entry)
{
	int handle = static_cast<int>(mJoints.Add(joint_entry));
	return handle;
}

void cSceneSimChar::RemoveJoint(int handle)
{
	assert(handle != gInvalidIdx);
	mJoints[handle].mJoint.reset();
	mJoints.Free(handle);
}

int cSceneSimChar::GetNumJoints() const
{
	return static_cast<int>(mJoints.GetSize());
}

bool cSceneSimChar::HasFallen(const cSimCharacter& sim_char) const
{
	bool fallen = sim_char.HasFallen();

	tVector root_pos = sim_char.GetRootPos();
	tVector ground_aabb_min;
	tVector ground_aabb_max;
	mGround->CalcAABB(ground_aabb_min, ground_aabb_max);
	ground_aabb_min[1] = -std::numeric_limits<double>::infinity();
	ground_aabb_max[1] = std::numeric_limits<double>::infinity();
	bool in_aabb = cMathUtil::ContainsAABB(root_pos, ground_aabb_min, ground_aabb_max);
	fallen |= !in_aabb;

	return fallen;
}

void cSceneSimChar::SpawnProjectile()
{
	double density = 100;
	double min_size = 0.1;
	double max_size = 0.3;
	double min_speed = 10;
	double max_speed = 20;
	double life_time = 2;
	double y_offset = 0;
	SpawnProjectile(density, min_size, max_size, min_speed, max_speed, y_offset, life_time);
}

void cSceneSimChar::SpawnBigProjectile()
{
	double density = 100;
	double min_size = 1.25;
	double max_size = 1.75;
	double min_speed = 11;
	double max_speed = 12;
	double life_time = 2;
	double y_offset = 0.5;
	SpawnProjectile(density, min_size, max_size, min_speed, max_speed, y_offset, life_time);
}

int cSceneSimChar::GetNumObjs() const
{
	return static_cast<int>(mObjs.GetCapacity());
}

const std::shared_ptr<cSimRigidBody>& cSceneSimChar::GetObj(int id) const
{
	return mObjs[id].mObj;
}

const cSceneSimChar::tObjEntry& cSceneSimChar::GetObjEntry(int id) const
{
	return mObjs[id];
}

void cSceneSimChar::SetRandSeed(unsigned long seed)
{
	cScene::SetRandSeed(seed);
	if (mGround != nullptr)
	{
		mGround->SeedRand(seed);
	}
}

void cSceneSimChar::SpawnProjectile(double density, double min_size, double max_size,
									double min_speed, double max_speed, double y_offset,
									double life_time)
{
	double min_dist = 1;
	double max_dist = 2;
	tVector aabb_min;
	tVector aabb_max;

	int char_id = mRand.RandInt(0, GetNumChars());
	const auto& curr_char = GetCharacter(char_id);
	curr_char->CalcAABB(aabb_min, aabb_max);

	tVector aabb_center = (aabb_min + aabb_max) * 0.5;
	tVector obj_size = tVector(1, 1, 1, 0) * mRand.RandDouble(min_size, max_size);
	
	double rand_theta = mRand.RandDouble(0, M_PI);
	double rand_dist = mRand.RandDouble(min_dist, max_dist);

	double aabb_size_x = (aabb_max[0] - aabb_min[0]);
	double aabb_size_z = (aabb_max[2] - aabb_min[2]);
	double buffer_dist = std::sqrt(aabb_size_x * aabb_size_x + aabb_size_z * aabb_size_z);

	double rand_x = 0.5 * buffer_dist + rand_dist * std::cos(rand_theta);
	rand_x *= mRand.RandSign();
	rand_x += aabb_center[0];
	double rand_y = mRand.RandDouble(aabb_min[1], aabb_max[1]) + obj_size[1] * 0.5;
	rand_y += y_offset;

	double rand_z = aabb_center[2];
	rand_z = 0.5 * buffer_dist + rand_dist * std::sin(rand_theta);
	rand_z *= mRand.RandSign();
	rand_z += aabb_center[2];

	tVector pos = tVector(rand_x, rand_y, rand_z, 0);
	tVector target = tVector(mRand.RandDouble(aabb_min[0], aabb_max[0]),
		mRand.RandDouble(aabb_min[1], aabb_max[1]), aabb_center[2], 0);

	tVector com_vel = curr_char->CalcCOMVel();
	tVector vel = (target - pos).normalized();
	vel *= mRand.RandDouble(min_speed, max_speed);
	vel[0] += com_vel[0];
	vel[2] += com_vel[2];

	cSimBox::tParams params;
	params.mSize = obj_size;
	params.mPos = pos;
	params.mVel = vel;
	params.mFriction = 0.7;
	params.mMass = density * params.mSize[0] * params.mSize[1] * params.mSize[2];
	std::shared_ptr<cSimBox> box = std::shared_ptr<cSimBox>(new cSimBox());
	box->Init(mWorld, params);
	box->UpdateContact(cWorld::eContactFlagObject, cContactManager::gFlagNone);

	tObjEntry obj_entry;
	obj_entry.mObj = box;
	obj_entry.mEndTime = GetTime() + life_time;
	
	AddObj(obj_entry);
}

int cSceneSimChar::SpawnRigidMesh(void * _shape, bool isStatic)
{
	auto & shape = *(tMesh*)_shape;
	std::shared_ptr<cSimRigidMesh> simMesh = std::shared_ptr<cSimRigidMesh>(new cSimRigidMesh());

	cSimRigidMesh::tParams params;
	params.mType = isStatic ? cSimObj::eType::eTypeStatic : cSimObj::eType::eTypeDynamic;
	params.mPos = shape.rootPos;
	params.mRot = shape.rootRot;
	params.mVertices = shape.vertices;
	params.mIndizes = shape.indices;
	params.mNormals = shape.normals;
	params.mUVs = shape.uvs;

	simMesh->Init(mWorld, params);
	simMesh->UpdateContact(cWorld::eContactFlagObject, cContactManager::gFlagNone);

	tObjEntry obj_entry;
	obj_entry.mObj = simMesh;
	obj_entry.mEndTime = std::numeric_limits<float>::max();

	return AddObj(obj_entry);
}

int cSceneSimChar::SpawnSoftMesh(void * _shape)
{
	auto & shape = *(tMesh*)_shape;
	// FIXME: this leaks on purpose to bypass crash in cleanup code
	simSoftBody = new cSimSoftBody();

	cSimSoftBody::tParams params;
	params.mPos = shape.rootPos;
	params.mRot = shape.rootRot;
	params.mVertices = shape.vertices;
	params.mIndizes = shape.indices;
	params.mNormals = shape.normals;
	params.mUVs = shape.uvs;

	simSoftBody->Init(mWorld, params);
	//simSoftBody->UpdateContact(cWorld::eContactFlagObject, cContactManager::gFlagNone);
	/*
	tObjEntry obj_entry;
	obj_entry.mObj = simSoftBody;
	obj_entry.mEndTime = std::numeric_limits<float>::max();

	return AddObj(obj_entry);
	*/
	return 0xFF;
}

const cSimObj* cSceneSimChar::GetSoftBody() const
{
	return simSoftBody;
}

void cSceneSimChar::ResetRandPertrub()
{
	mPerturbParams.mTimer = 0;
	mPerturbParams.mNextTime = mRand.RandDouble(mPerturbParams.mTimeMin, mPerturbParams.mTimeMax);
}