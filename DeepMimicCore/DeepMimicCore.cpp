#include "DeepMimicCore.h"

#include "render/DrawUtil.h"
#include "scenes/SceneBuilder.h"
#include "scenes/DrawSceneImitate.h"
#include "scenes/RLSceneSimChar.h"

#include <Alembic/Abc/All.h>
#include <Alembic/AbcCoreAbstract/All.h>
#include <Alembic/AbcCoreOgawa/All.h>
#include <Alembic/AbcGeom/All.h>

#include <Eigen/Core>

#include <iterator>
#include <fstream>
#include <string>

namespace std
{
	double clamp(double v, double lo, double hi)
	{
		return std::min(std::max(v, lo), hi);
	}
}

cDeepMimicCore::cDeepMimicCore(bool enable_draw)
{
	mArgParser = std::shared_ptr<cArgParser>(new cArgParser());
	cDrawUtil::EnableDraw(enable_draw);

	mNumUpdateSubsteps = 1;
	mAlembicInputPath = "";
	mAlembicOutputPath = "";
	mMotionOutputPath = "";
	mPlaybackSpeed = 1;
	mUpdatesPerSec = 0;
}

cDeepMimicCore::~cDeepMimicCore()
{
}

void cDeepMimicCore::SeedRand(int seed)
{
	cMathUtil::SeedRand(seed);
}

void cDeepMimicCore::ParseArgs(const std::vector<std::string>& args)
{
	mArgParser->LoadArgs(args);

	std::string arg_file = "";
	mArgParser->ParseString("arg_file", arg_file);
	if (arg_file != "")
	{
		// append the args from the file to the ones from the commandline
		// this allows the cmd args to overwrite the file args
		bool succ = mArgParser->LoadFile(arg_file);
		if (!succ)
		{
			printf("Failed to load args from: %s\n", arg_file.c_str());
			assert(false);
		}
	}

	mArgParser->ParseInt("num_update_substeps", mNumUpdateSubsteps);
	mArgParser->ParseString("alembic_input_path", mAlembicInputPath);
	mArgParser->ParseString("alembic_output_path", mAlembicOutputPath);
	mArgParser->ParseString("motion_output_path", mMotionOutputPath);
}

void cDeepMimicCore::Init()
{
	if (EnableDraw())
	{
		cDrawUtil::InitDrawUtil();
		InitFrameBuffer();
	}
	SetupScene();
	Reset();
}


std::vector<std::vector<tMatrix>> frames(0);
double timePassed = 0;

const std::string const jointNames[] = {
  "root",
  "chest",
  "neck",
  "right_hip",
  "right_knee",
  "right_ankle",
  "right_shoulder",
  "right_elbow",
  "right_wrist",
  "left_hip",
  "left_knee",
  "left_ankle",
  "left_shoulder",
  "left_elbow",
  "left_wrist"
};



const std::string const jointTypes[] = {
	"root",
	"spherical",
	"spherical",
	"spherical",
	"revolute",
	"spherical",
	"spherical",
	"revolute",
	"fixed",
	"spherical",
	"revolute",
	"spherical",
	"spherical",
	"revolute",
	"fixed"
};



const int const jointParents[] = {
	-1,
	0,
	1,
	0,
	3,
	4,
	1,
	6,
	7,
	0,
	9,
	10,
	1,
	12,
	13
};

const std::string bonePrefix = "bn-deep_mimic-";
const std::string simPrefix = "sim-deep_mimic-";

void cDeepMimicCore::Update(double timestep)
{
	mScene->Update(timestep);

	auto drawScene = std::dynamic_pointer_cast<cDrawSceneImitate>(mScene);
	if (drawScene)
	{
		auto rlSimScene = dynamic_cast<cRLSceneSimChar*>(drawScene->GetRLScene());
		if (rlSimScene)
		{
			// record data
			auto simChar = rlSimScene->GetCharacter();
			const auto numJoints = simChar->GetNumJoints();
			auto transforms = std::vector<tMatrix>(numJoints);

			for (int i = 0; i < numJoints; ++i)
			{
				const auto & joint = simChar->GetJoint(i);
				transforms[i] = (joint.BuildWorldTrans());
			}

			frames.push_back(transforms);
			timePassed += timestep;
		}
	}
}

double frameDuration()
{
	return timePassed / frames.size();
}

void serializeFrameCache(Alembic::Abc::OArchive& archive)
{
	auto timeSampling = Alembic::AbcCoreAbstract::TimeSampling(frameDuration(), 0.0f);
	auto timeSamplingIdx = archive.addTimeSampling(timeSampling);

	const auto numJoints = frames[0].size();
	auto jointXforms = std::vector<Alembic::AbcGeom::OXform>(numJoints);

	for (int i = 0; i < numJoints; ++i)
	{
		jointXforms[i] = Alembic::AbcGeom::OXform(
			archive.getTop(),
			bonePrefix + jointNames[i]);

		jointXforms[i].getSchema().setTimeSampling(timeSamplingIdx);
	}

	for (const auto& f : frames)
	{
		for (int i = 0; i < numJoints; ++i)
		{
			Alembic::AbcGeom::M44d mat;
			for (auto y = 0; y < 4; ++y)
			{
				for (auto x = 0; x < 4; ++x)
				{
					mat[x][y] = f[i](y, x);
				}
			}

			Alembic::AbcGeom::XformSample sample;
			sample.setMatrix(mat);

			jointXforms[i].getSchema().set(sample);
		}
	}
}

void serializeVectorToProperty(const std::string & name, const Eigen::VectorXd& vec, Alembic::Abc::OObject& object)
{
	auto property = Alembic::Abc::ODoubleArrayProperty(object.getProperties(), name);
	property.set(Alembic::Abc::DoubleArraySample(vec.data(), vec.size()));
}

Eigen::VectorXd deserializeVectorFromProperty(const Alembic::Abc::IObject& object, const std::string& name)
{
	if (object)
	{
		Alembic::Abc::ArraySamplePtr sample;
		auto propertyReader = object.getProperties().getPtr()->getArrayProperty(name);

		if (propertyReader)
		{
			propertyReader->asArrayPtr()->getSample(0, sample);

			return Eigen::Map<Eigen::VectorXd>((double*)sample->getData(), sample->size());
		}
	}
	
	return Eigen::VectorXd();
}

Eigen::Matrix<double, 4, 4> deserializeTransformFromProperty(const Alembic::Abc::IObject& object, const std::string& name, const int index = -1)
{
	auto schema = Alembic::AbcGeom::IXform(object, name).getSchema();

	Alembic::AbcCoreAbstract::index_t sampleIndex = (index == -1 ? schema.getNumSamples() - 1 : index);
	auto sampler = Alembic::Abc::ISampleSelector(sampleIndex);

	Alembic::AbcGeom::XformSample lastSample;
	schema.get(lastSample, sampler);

	return Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(lastSample.getMatrix().getValue());
}

void assignJointPose(
	std::shared_ptr<cSimCharacter> character,
	int jointID,
	const Eigen::VectorXd& jointPose,
	Eigen::VectorXd& rootPose)
{
	auto& simJoint = character->GetJoint(jointID);
	int param_offset = character->GetParamOffset(jointID);
	int param_size = character->GetParamSize(jointID);
	rootPose.segment(param_offset, param_size) = jointPose;
}

Eigen::Matrix<double,1,1> toVector(const double& val)
{
	return Eigen::Matrix<double, 1, 1>(val);;
}

Eigen::Vector4d toVector(const Eigen::Quaterniond& q)
{
	return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

Eigen::VectorXd calculatePoseFromArchiveTransforms(
		const Alembic::Abc::IArchive & archive,
		std::shared_ptr<cSimCharacter> simCharacter,
		const int index = -1)
{

	auto initialPose = simCharacter->GetPose0();

	std::cout << " got initial pose" << std::endl;

	for (int i = 0; i < std::size(jointNames); ++i)
	{
		auto& jointName = jointNames[i];
		auto& jointType = jointTypes[i];
		std::cout << " reading joint " << jointName << " as " << jointType << std::endl;
		auto jointXform = deserializeTransformFromProperty(archive.getTop(), bonePrefix + jointNames[i], index);
		auto jointPos = jointXform.block<3, 1>(0, 3);
		auto jointR33 = jointXform.block<3, 3>(0, 0);
		auto jointQ = Eigen::Quaterniond(jointR33);

		if (jointType == "root")
		{
			//std::cout << " root pose: " << std::endl << jointPos << std::endl << toVector(jointQ) << std::endl;

			initialPose.segment(0, 3) = jointPos;
			initialPose.segment(3, 4) = toVector(jointQ);
		}
		else if (jointType == "spherical")
		{
			auto& parentName = jointNames[jointParents[i]];
			auto parentXform = deserializeTransformFromProperty(archive.getTop(), bonePrefix + parentName, index);
			auto parentR33 = parentXform.block<3, 3>(0, 0);
			auto parentQ = Eigen::Quaterniond(parentR33);

			auto diffQ = parentQ.inverse() * jointQ;

			assignJointPose(simCharacter, i, toVector(diffQ), initialPose);

			//std::cout << " " << jointName << " pose: " << std::endl << toVector(diffQ) << std::endl;
		}
		else if (jointType == "revolute")
		{
			auto parentXform = deserializeTransformFromProperty(archive.getTop(), bonePrefix + jointNames[i - 1], index);
			auto childXform = deserializeTransformFromProperty(archive.getTop(), bonePrefix + jointNames[i + 1], index);

			auto parentPos = parentXform.block<3, 1>(0, 3);
			auto childPos = childXform.block<3, 1>(0, 3);

			auto upDir = (parentPos - jointPos).normalized();
			auto downDir = (childPos - jointPos).normalized();

			auto diffAngle = std::acos(std::clamp(upDir.dot(downDir), -1.0, 1.0));

			//std::cout << "DEBUG: " << jointName << std::endl \
								<< " upDir:" << std::endl << upDir << std::endl \
								<< " downDir:" << std::endl << downDir << std::endl \
								<< " dot:" << upDir.dot(downDir) << std::endl << " diffAngle:" << diffAngle << std::endl;

			if (jointName == "right_knee" || jointName == "left_knee")
			{
				diffAngle = diffAngle - M_PI;
			}
			else
			{
				diffAngle = M_PI - diffAngle;
			}

			assignJointPose(simCharacter, i, toVector(diffAngle), initialPose);

			//std::cout << " " << jointName << " pose: " << std::endl << diffAngle << std::endl;
		}
		else
		{
			//std::cout << " " << jointNames[i] << " is fixed" << std::endl;
		}

	}

	return initialPose;
}

Eigen::MatrixXd calculateMotionFromArchiveTransforms(
		const Alembic::Abc::IArchive & archive,
		std::shared_ptr<cSimCharacter> simCharacter)
{
	constexpr int poseVectorDims = 43;
	constexpr float interFrameInterval = 0.016666;
	auto numRootSamples = Alembic::AbcGeom::IXform(archive.getTop(), bonePrefix + "root").getSchema().getNumSamples();

	Eigen::MatrixXd motionMatrix(numRootSamples, poseVectorDims + 1);
	motionMatrix.leftCols(1) = Eigen::MatrixXd::Constant(numRootSamples, 1, interFrameInterval);

	for (int i = 0; i < numRootSamples; ++i)
	{
		auto poseVector = calculatePoseFromArchiveTransforms(archive, simCharacter, i);
		motionMatrix.block<1, poseVectorDims>(i, 1) = poseVector;
	}

	return motionMatrix;
}

void cDeepMimicCore::Reset()
{
	// write out alembic
	if (frames.size() > 10 && !mAlembicOutputPath.empty())
	{
		std::cout << "cDeepMimicCore::Reset " << frames.size() \
			<< " frames in " << timePassed \
			<< " seconds, per frame " << frameDuration() \
			<< " writing to " << mAlembicOutputPath << std::endl;

		auto archive = Alembic::Abc::OArchive(
			Alembic::AbcCoreOgawa::WriteArchive(),
			mAlembicOutputPath,
			Alembic::Abc::ErrorHandler::kThrowPolicy);

		serializeFrameCache(archive);

		auto drawScene = std::dynamic_pointer_cast<cDrawSceneImitate>(mScene);
		if (drawScene)
		{
			auto rlSimScene = dynamic_cast<cRLSceneSimChar*>(drawScene->GetRLScene());
			if (rlSimScene)
			{
				// record data
				auto simChar = rlSimScene->GetCharacter();
	
				auto rootPose = simChar->GetPose();
				auto rootVel = simChar->GetVel();
				auto rootVel0 = simChar->GetVel0();

				auto rootObject = Alembic::Abc::OObject(archive.getTop(), simPrefix + "root");
				serializeVectorToProperty("pose", rootPose, rootObject);
				serializeVectorToProperty("vel", rootVel, rootObject);

				/*
				const auto numJoints = simChar->GetNumJoints();

				for (int i = 1; i < numJoints; ++i)
				{
					const auto& joint = simChar->GetJoint(i);

					Eigen::VectorXd pose(0), velocity(0);

					joint.BuildPose(pose);
					joint.BuildVel(velocity);

					std::cout << "  " << jointNames[i] << " pose: " << std::endl << pose << std::endl;
				}
				*/
			}
		}

	}

	// reset 
	frames.clear();
	timePassed = 0;

	mScene->Reset();
	mUpdatesPerSec = 0;

	// read final pose from alembic
	if (mAlembicInputPath.empty())
	{
		return;
	}

	auto drawScene = std::dynamic_pointer_cast<cDrawSceneImitate>(mScene);
	if (!drawScene)
	{
		std::cout << "cDeepMimicCore::Reset not in a draw scene. ignoring alembic_input_path" << std::endl;
		return;
	}

	auto rlSimScene = dynamic_cast<cRLSceneSimChar*>(drawScene->GetRLScene());
	if (!rlSimScene)
	{
		std::cout << "cDeepMimicCore::Reset not in a RL scene. ignoring alembic_input_path" << std::endl;
		return;
	}

	auto simCharacter = rlSimScene->GetCharacter();

	std::cout << "cDeepMimicCore::Reset reading from " << mAlembicInputPath << std::endl;

	auto archive = Alembic::Abc::IArchive(
			Alembic::AbcCoreOgawa::ReadArchive(),
			mAlembicInputPath,
			Alembic::Abc::ErrorHandler::kThrowPolicy);

	std::cout << " opened archive " << std::endl;

	auto rootObject = Alembic::Abc::IObject(
					archive.getTop(),
					simPrefix + "root",
					Alembic::Abc::ErrorHandler::kQuietNoopPolicy);

	std::cout << " found root " << rootObject << std::endl;

	if (!mMotionOutputPath.empty())
	{
		auto motionMatrix = calculateMotionFromArchiveTransforms(archive, simCharacter);
		std::cout << " motion matrix" << std::endl << motionMatrix << std::endl << std::endl;

		Json::Value output;
		output["Loop"] = "wrap";

		for (int y = 0; y < motionMatrix.rows(); ++y)
		{
			for (int x = 0; x < motionMatrix.cols(); ++x)
			{
				output["Frames"][y][x] = Json::Value(motionMatrix(y, x));
			}
		}

		Json::StyledStreamWriter writer;
		std::ofstream motionFile(mMotionOutputPath);
		writer.write(motionFile, output);
		motionFile.close();

		exit(0);
	}

	auto lastPoses = deserializeVectorFromProperty(rootObject, "pose");
	auto lastVelocities = deserializeVectorFromProperty(rootObject, "vel");

	if (lastPoses.size() && lastVelocities.size())
	{
		std::cout << "set last pose based on poses & velocities" << std::endl;
		simCharacter->SetPose0(lastPoses);
		simCharacter->SetVel0(lastVelocities);
		simCharacter->SetPose(lastPoses);
		simCharacter->SetVel(lastVelocities);
	}
	else
	{
		std::cout << "set initial pose based on alembic transforms alone" << std::endl;

		auto currentPose = simCharacter->GetPose0();
		std::cout << " before " << std::endl << simCharacter->GetPose0() << std::endl;

		auto transformDefinedPose = calculatePoseFromArchiveTransforms(archive, simCharacter);
		std::cout << " after " << std::endl << transformDefinedPose << std::endl;

		simCharacter->SetPose0(transformDefinedPose);
		simCharacter->SetPose(transformDefinedPose);
		simCharacter->SetVel(simCharacter->GetVel0());
	}
}

double cDeepMimicCore::GetTime() const
{
	return mScene->GetTime();
}

std::string cDeepMimicCore::GetName() const
{
	return mScene->GetName();
}

bool cDeepMimicCore::EnableDraw() const
{
	return cDrawUtil::EnableDraw();
}

void cDeepMimicCore::Draw()
{
	if (EnableDraw())
	{
		mDefaultFrameBuffer->BindBuffer();
		mScene->Draw();
		mDefaultFrameBuffer->UnbindBuffer();
	}
}

void cDeepMimicCore::Keyboard(int key, int x, int y)
{
	char c = static_cast<char>(key);
	double device_x = 0;
	double device_y = 0;
	CalcDeviceCoord(x, y, device_x, device_y);
	mScene->Keyboard(c, device_x, device_y);
}

void cDeepMimicCore::MouseClick(int button, int state, int x, int y)
{
	double device_x = 0;
	double device_y = 0;
	CalcDeviceCoord(x, y, device_x, device_y);
	mScene->MouseClick(button, state, device_x, device_y);
}

void cDeepMimicCore::MouseMove(int x, int y)
{
	double device_x = 0;
	double device_y = 0;
	CalcDeviceCoord(x, y, device_x, device_y);
	mScene->MouseMove(device_x, device_y);
}

void cDeepMimicCore::Reshape(int w, int h)
{
	mScene->Reshape(w, h);
	mDefaultFrameBuffer->Reshape(w, h);
	glViewport(0, 0, w, h);
	glutPostRedisplay();
}

void cDeepMimicCore::Shutdown()
{
	mScene->Shutdown();
}

bool cDeepMimicCore::IsDone() const
{
	return mScene->IsDone();
}

cDrawScene* cDeepMimicCore::GetDrawScene() const
{
	return dynamic_cast<cDrawScene*>(mScene.get());
}

void cDeepMimicCore::SetPlaybackSpeed(double speed)
{
	mPlaybackSpeed = speed;
}

void cDeepMimicCore::SetUpdatesPerSec(double updates_per_sec)
{
	mUpdatesPerSec = updates_per_sec;
}

int cDeepMimicCore::GetWinWidth() const
{
	return mDefaultFrameBuffer->GetWidth();
}

int cDeepMimicCore::GetWinHeight() const
{
	return mDefaultFrameBuffer->GetHeight();
}

int cDeepMimicCore::GetNumUpdateSubsteps() const
{
	return mNumUpdateSubsteps;
}

bool cDeepMimicCore::IsRLScene() const
{
	const auto& rl_scene = GetRLScene();
	return rl_scene != nullptr;
}

int cDeepMimicCore::GetNumAgents() const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		return rl_scene->GetNumAgents();
	}
	return 0;
}

bool cDeepMimicCore::NeedNewAction(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		return rl_scene->NeedNewAction(agent_id);
	}
	return false;
}

std::vector<double> cDeepMimicCore::RecordState(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		Eigen::VectorXd state;
		rl_scene->RecordState(agent_id, state);

		std::vector<double> out_state;
		ConvertVector(state, out_state);
		return out_state;
	}
	return std::vector<double>(0);
}

std::vector<double> cDeepMimicCore::RecordGoal(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		Eigen::VectorXd goal;
		rl_scene->RecordGoal(agent_id, goal);

		std::vector<double> out_goal;
		ConvertVector(goal, out_goal);
		return out_goal;
	}
	return std::vector<double>(0);
}

void cDeepMimicCore::SetAction(int agent_id, const std::vector<double>& action)
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		Eigen::VectorXd in_action;
		ConvertVector(action, in_action);
		rl_scene->SetAction(agent_id, in_action);
	}
}

void cDeepMimicCore::LogVal(int agent_id, double val)
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		rl_scene->LogVal(agent_id, val);
	}
}

int cDeepMimicCore::GetActionSpace(int agent_id) const
{
	eActionSpace action_space = eActionSpaceNull;
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		action_space = rl_scene->GetActionSpace(agent_id);
	}
	return static_cast<int>(action_space);
}

int cDeepMimicCore::GetStateSize(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		return rl_scene->GetStateSize(agent_id);
	}
	return 0;
}

int cDeepMimicCore::GetGoalSize(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		return rl_scene->GetGoalSize(agent_id);
	}
	return 0;
}

int cDeepMimicCore::GetActionSize(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		return rl_scene->GetActionSize(agent_id);
	}
	return 0;
}

int cDeepMimicCore::GetNumActions(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		return rl_scene->GetNumActions(agent_id);
	}
	return 0;
}

std::vector<double> cDeepMimicCore::BuildStateOffset(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		Eigen::VectorXd offset;
		Eigen::VectorXd scale;
		rl_scene->BuildStateOffsetScale(agent_id, offset, scale);

		std::vector<double> out_offset;
		ConvertVector(offset, out_offset);
		return out_offset;
	}
	return std::vector<double>(0);
}

std::vector<double> cDeepMimicCore::BuildStateScale(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		Eigen::VectorXd offset;
		Eigen::VectorXd scale;
		rl_scene->BuildStateOffsetScale(agent_id, offset, scale);

		std::vector<double> out_scale;
		ConvertVector(scale, out_scale);
		return out_scale;
	}
	return std::vector<double>(0);
}

std::vector<double> cDeepMimicCore::BuildGoalOffset(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		Eigen::VectorXd offset;
		Eigen::VectorXd scale;
		rl_scene->BuildGoalOffsetScale(agent_id, offset, scale);

		std::vector<double> out_offset;
		ConvertVector(offset, out_offset);
		return out_offset;
	}
	return std::vector<double>(0);
}

std::vector<double> cDeepMimicCore::BuildGoalScale(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		Eigen::VectorXd offset;
		Eigen::VectorXd scale;
		rl_scene->BuildGoalOffsetScale(agent_id, offset, scale);

		std::vector<double> out_scale;
		ConvertVector(scale, out_scale);
		return out_scale;
	}
	return std::vector<double>(0);
}

std::vector<double> cDeepMimicCore::BuildActionOffset(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		Eigen::VectorXd offset;
		Eigen::VectorXd scale;
		rl_scene->BuildActionOffsetScale(agent_id, offset, scale);

		std::vector<double> out_offset;
		ConvertVector(offset, out_offset);
		return out_offset;
	}
	return std::vector<double>(0);
}

std::vector<double> cDeepMimicCore::BuildActionScale(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		Eigen::VectorXd offset;
		Eigen::VectorXd scale;
		rl_scene->BuildActionOffsetScale(agent_id, offset, scale);

		std::vector<double> out_scale;
		ConvertVector(scale, out_scale);
		return out_scale;
	}
	return std::vector<double>(0);
}

std::vector<double> cDeepMimicCore::BuildActionBoundMin(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		Eigen::VectorXd bound_min;
		Eigen::VectorXd bound_max;
		rl_scene->BuildActionBounds(agent_id, bound_min, bound_max);

		std::vector<double> out_min;
		ConvertVector(bound_min, out_min);
		return out_min;
	}
	return std::vector<double>(0);
}

std::vector<double> cDeepMimicCore::BuildActionBoundMax(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		Eigen::VectorXd bound_min;
		Eigen::VectorXd bound_max;
		rl_scene->BuildActionBounds(agent_id, bound_min, bound_max);

		std::vector<double> out_max;
		ConvertVector(bound_max, out_max);
		return out_max;
	}
	return std::vector<double>(0);
}

std::vector<int> cDeepMimicCore::BuildStateNormGroups(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		Eigen::VectorXi groups;
		rl_scene->BuildStateNormGroups(agent_id, groups);

		std::vector<int> out_groups;
		ConvertVector(groups, out_groups);
		return out_groups;
	}
	return std::vector<int>(0);
}

std::vector<int> cDeepMimicCore::BuildGoalNormGroups(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		Eigen::VectorXi groups;
		rl_scene->BuildGoalNormGroups(agent_id, groups);

		std::vector<int> out_groups;
		ConvertVector(groups, out_groups);
		return out_groups;
	}
	return std::vector<int>(0);
}

double cDeepMimicCore::CalcReward(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		return rl_scene->CalcReward(agent_id);
	}
	return 0;
}

double cDeepMimicCore::GetRewardMin(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		return rl_scene->GetRewardMin(agent_id);
	}
	return 0;
}

double cDeepMimicCore::GetRewardMax(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		return rl_scene->GetRewardMax(agent_id);
	}
	return 0;
}

double cDeepMimicCore::GetRewardFail(int agent_id)
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		return rl_scene->GetRewardFail(agent_id);
	}
	return 0;
}

double cDeepMimicCore::GetRewardSucc(int agent_id)
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		return rl_scene->GetRewardSucc(agent_id);
	}
	return 0;
}

bool cDeepMimicCore::IsEpisodeEnd() const
{
	auto drawScene = std::dynamic_pointer_cast<cDrawSceneImitate>(mScene);
	if (drawScene)
	{
		return false;
	}
	else
	{
		return mScene->IsEpisodeEnd();
	}
}

bool cDeepMimicCore::CheckValidEpisode() const
{
	auto drawScene = std::dynamic_pointer_cast<cDrawSceneImitate>(mScene);
	if (false && drawScene)
	{
		return true;
	}
	else
	{
		return mScene->CheckValidEpisode();
	}
}

int cDeepMimicCore::CheckTerminate(int agent_id) const
{
	const auto& rl_scene = GetRLScene();
	cRLScene::eTerminate terminated = cRLScene::eTerminateNull;
	if (rl_scene != nullptr)
	{
		terminated = rl_scene->CheckTerminate(agent_id);
	}
	return static_cast<int>(terminated);
}

void cDeepMimicCore::SetMode(int mode)
{
	assert(mode >= 0 && mode < cRLScene::eModeMax);
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		rl_scene->SetMode(static_cast<cRLScene::eMode>(mode));
	}
}

void cDeepMimicCore::SetSampleCount(int count)
{
	const auto& rl_scene = GetRLScene();
	if (rl_scene != nullptr)
	{
		rl_scene->SetSampleCount(count);
	}
}

void cDeepMimicCore::SetupScene()
{
	ClearScene();

	std::string scene_name = "";
	mArgParser->ParseString("scene", scene_name);

	mScene = nullptr;
	mRLScene = nullptr;
	if (EnableDraw())
	{
		cSceneBuilder::BuildDrawScene(scene_name, mScene);
	}
	else
	{
		cSceneBuilder::BuildScene(scene_name, mScene);
	}

	if (mScene != nullptr)
	{
		mRLScene = std::dynamic_pointer_cast<cRLScene>(mScene);
		mScene->ParseArgs(mArgParser);
		mScene->Init();
		printf("Loaded scene: %s\n", mScene->GetName().c_str());
	}
}

void cDeepMimicCore::ClearScene()
{
	mScene = nullptr;
}

int cDeepMimicCore::GetCurrTime() const
{
	return glutGet(GLUT_ELAPSED_TIME);
}

void cDeepMimicCore::InitFrameBuffer()
{
	mDefaultFrameBuffer = std::unique_ptr<cTextureDesc>(new cTextureDesc(0, 0, 0, 1, 1, 1, GL_RGBA, GL_RGBA));
}

void cDeepMimicCore::CalcDeviceCoord(int pixel_x, int pixel_y, double& out_device_x, double& out_device_y) const
{
	double w = GetWinWidth();
	double h = GetWinHeight();

	out_device_x = static_cast<double>(pixel_x) / w;
	out_device_y = static_cast<double>(pixel_y) / h;
	out_device_x = (out_device_x - 0.5f) * 2.f;
	out_device_y = (out_device_y - 0.5f) * -2.f;
}

double cDeepMimicCore::GetAspectRatio()
{
	double aspect_ratio = static_cast<double>(GetWinWidth()) / GetWinHeight();
	return aspect_ratio;
}

void cDeepMimicCore::CopyFrame(cTextureDesc& src) const
{
	cDrawUtil::CopyTexture(src);
}

const std::shared_ptr<cRLScene>& cDeepMimicCore::GetRLScene() const
{
	return mRLScene;
}

void cDeepMimicCore::ConvertVector(const Eigen::VectorXd& in_vec, std::vector<double>& out_vec) const
{
	int size = static_cast<int>(in_vec.size());
	out_vec.resize(size);
	std::memcpy(out_vec.data(), in_vec.data(), size * sizeof(double));
}

void cDeepMimicCore::ConvertVector(const Eigen::VectorXi& in_vec, std::vector<int>& out_vec) const
{
	int size = static_cast<int>(in_vec.size());
	out_vec.resize(size);
	std::memcpy(out_vec.data(), in_vec.data(), size * sizeof(int));
}

void cDeepMimicCore::ConvertVector(const std::vector<double>& in_vec, Eigen::VectorXd& out_vec) const
{
	int size = static_cast<int>(in_vec.size());
	out_vec.resize(size);
	std::memcpy(out_vec.data(), in_vec.data(), size * sizeof(double));
}

void cDeepMimicCore::ConvertVector(const std::vector<int>& in_vec, Eigen::VectorXi& out_vec) const
{
	int size = static_cast<int>(in_vec.size());
	out_vec.resize(size);
	std::memcpy(out_vec.data(), in_vec.data(), size * sizeof(int));
}
