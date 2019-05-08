#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <sstream>
#include <map>

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.h"
#include "BulletWorldImporter/btMultiBodyWorldImporter.h"

#include "json/json.h"

#include "argparse.h"

struct index_t
{
  std::map<std::string, btCollisionShape*>  collisionShapes;
  std::map<std::string, btRigidBody*>       rigidBodies;
  std::map<std::string, btTypedConstraint*> constraints;

  index_t()
  : collisionShapes(),
    rigidBodies(),
    constraints()
  {
  }
};

std::string rigidBodyNames[] = {
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
  "left_writst"
};

std::string constraintNames[] = {
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
  "left_writst"
};

class OutputBuilder
{
public:
  OutputBuilder(const index_t & index, const Json::Value & preset);
  Json::Value build();

private:
  const index_t & index;
  const Json::Value & preset;

  Json::Value buildJoints();
  Json::Value buildJoint(int i);
  Json::Value buildBodyDefs();
  Json::Value buildBodyDef(int i);
};

OutputBuilder::OutputBuilder(const index_t & index, const Json::Value & preset)
: index(index),
  preset(preset)
{
}

Json::Value OutputBuilder::build()
{
  Json::Value output;
  output["Skeleton"]["Joints"] = buildJoints();
  output["BodyDefs"] = buildBodyDefs();
  return output;
}

Json::Value OutputBuilder::buildJoints()
{
  Json::Value skeleton;

  for (int i = 0; i <= std::size(constraintNames); ++i)
  {
    skeleton[i] = buildJoint(i);
  }

  return skeleton;
}

Json::Value OutputBuilder::buildJoint(int i)
{
  Json::Value joint = preset["Skeleton"]["Joints"][i];

  if (i == 1 || i == 2)
  {
    auto constraintName = constraintNames[i-1];
    auto jointName = joint["Name"].asString();

    std::cout << "Mapping " << constraintName << " to " << jointName << std::endl;

    auto constraint = index.constraints.at(constraintName);
    // TODO: apply constraint properties to joint JSON
    //

  }

  return joint;
}

Json::Value OutputBuilder::buildBodyDefs()
{
  Json::Value skeleton;

  for (int i = 0; i < std::size(rigidBodyNames); ++i)
  {
    skeleton[i] = buildBodyDef(i);
  }

  return skeleton;
}

Json::Value OutputBuilder::buildBodyDef(int i)
{
  Json::Value bodyDef = preset["BodyDefs"][i];

  if (i <= 2)
  {
    auto rigidBodyName = rigidBodyNames[i];
    auto bodyDefName = bodyDef["Name"].asString();

    std::cout << "Mapping " << rigidBodyName << " to " << bodyDefName << std::endl;

    auto rigidBody = index.rigidBodies.at(rigidBodyName);

    auto collision = rigidBody->getCollisionShape();

    if (collision)
    {
      auto type = collision->getShapeType();
      auto scaling = collision->getLocalScaling();

      if (type == BOX_SHAPE_PROXYTYPE)
      {
        auto box = (btBoxShape*) collision;
        auto shapeDims = box->getImplicitShapeDimensions();

        bodyDef["Shape"] = "box";
        bodyDef["Param0"] = shapeDims.x();
        bodyDef["Param1"] = shapeDims.y();
        bodyDef["Param2"] = shapeDims.z();
      }

      if (type == SPHERE_SHAPE_PROXYTYPE)
      {
        auto sphere = (btSphereShape*) collision;
        auto radius = sphere->getRadius();

        bodyDef["Shape"] = "sphere";
        bodyDef["Param0"] = radius;
        bodyDef["Param1"] = radius;
        bodyDef["Param2"] = radius;
      }
    }
    else
    {
      std::cout << "Could not find a collision shape for rigid body " << rigidBodyName << std::endl;
    }

  }

  return bodyDef;
}

std::string stringify(const btVector3 & v)
{
  std::stringstream ss;
  ss << "(" << v.getX() << ", " << v.getY() << ", " << v.getZ() << ")";
  return ss.str();
}

std::string extractSuffix(const std::string & s)
{
  return s.substr(s.rfind(":") + 1);
}

int main(int argc, const char** argv)
{
  ArgumentParser parser;
  parser.addArgument("-i", "--input", 1, false); // .bullet file to convert
  parser.addArgument("-p", "--preset", 1, false); // original humanoid3d.txt to use as reference
  parser.addFinalArgument("output");
  parser.parse(argc, argv);

  auto bulletFilePath = parser.retrieve<std::string>("input");
  auto collisionConfig = new btDefaultCollisionConfiguration();
  auto dispatcher = new btCollisionDispatcher(collisionConfig);
  auto broadphase = new btDbvtBroadphase();
  auto solver = new btMultiBodyConstraintSolver;
  auto world = new btMultiBodyDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
  auto fileLoader = new btMultiBodyWorldImporter(world);

  std::cout << "loading file " << bulletFilePath << std::endl;
  fileLoader->loadFile(bulletFilePath.c_str());
  std::cout <<
      "  collision shapes:\t" << fileLoader->getNumCollisionShapes() << std::endl <<
      "  rigid bodies:\t\t" << fileLoader->getNumRigidBodies() << std::endl <<
      "  constraints:\t\t" << fileLoader->getNumConstraints() << std::endl <<
      "  bounding volumes:\t" << fileLoader->getNumBvhs() << std::endl <<
      "  triangle info maps:\t" << fileLoader->getNumTriangleInfoMaps() << std::endl << std::endl;

  index_t index = index_t();

  for (int i=0; i < fileLoader->getNumCollisionShapes(); i++)
  {
    std::cout << "Collision shape " << i << std::endl;

    auto obj = fileLoader->getCollisionShapeByIndex(i);
    auto name = fileLoader->getNameForPointer(obj);


    if (name)
    {
      std::cout << "  object name: " << name << std::endl;

      auto key = extractSuffix(name);

      std::cout << "  key: " << key << std::endl;
      index.collisionShapes[key] = obj;
    }
  }

  std::cout << std::endl;

  for(int i=0; i < fileLoader->getNumRigidBodies(); i++)
  {
    std::cout << "Rigid Body " << i << std::endl;

    auto obj = fileLoader->getRigidBodyByIndex(i);
    auto name = fileLoader->getNameForPointer(obj);
    auto key = extractSuffix(name);

    std::cout << "  object name: " << name << std::endl;
    std::cout << "  key: " << key << std::endl;

    auto body = btRigidBody::upcast(obj);
    index.rigidBodies[key] = body;

    if (body->getInvMass() == 0)
    {
      std::cout << "  static object" << std::endl;
    }
    else
    {
      std::cout << "  mass: " << 1/body->getInvMass() << std::endl;

      auto collision = body->getCollisionShape();

      if (collision)
      {

        std::cout << "  collision: " << collision->getName() << std::endl;

        auto type = collision->getShapeType();

        std::cout << "    type: " << type << std::endl;

        auto scaling = collision->getLocalScaling();

        std::cout << "    scaling: " << stringify(scaling) << std::endl;


        if (type == BOX_SHAPE_PROXYTYPE)
        {
          auto box = (btBoxShape*) collision;

          auto shapeDims = box->getImplicitShapeDimensions();
          std::cout << "    shape dimensions: " << stringify(shapeDims) << std::endl;

        }

        if (type == SPHERE_SHAPE_PROXYTYPE)
        {
          auto sphere = (btSphereShape*) collision;
          std::cout << "    radius: " << sphere->getRadius() << std::endl;

          auto shapeDims = sphere->getImplicitShapeDimensions();
          std::cout << "    shape dimensions: " << stringify(shapeDims) << std::endl;
        }
      }
    }

    std::cout << std::endl;
  }

  for(int i=0; i < fileLoader->getNumConstraints(); i++)
  {
    std::cout << "Constraint " << i << std::endl;

    auto constraint = fileLoader->getConstraintByIndex(i);
    auto name = fileLoader->getNameForPointer(constraint);
    auto key = extractSuffix(name);
    index.constraints[key] = constraint;

    std::cout << "  object name: " << name << std::endl;
    std::cout << "  key: " << key << std::endl;
    std::cout << "  constraint type: " << constraint->getConstraintType() << std::endl;

    if (constraint->getConstraintType() == 4)
    {
      auto hinge = (btHingeConstraint*)constraint;
      std::cout << "  angle: " << hinge->getHingeAngle() << std::endl;
      std::cout << "  lower limit: " << hinge->getLowerLimit() << std::endl;
      std::cout << "  upper limit: " << hinge->getUpperLimit() << std::endl;

      auto offsetA = hinge->getFrameOffsetA();
      std::cout << "  offsetA: " << stringify(offsetA.getOrigin()) << " [offset -> attach]" << std::endl;

      auto offsetB = hinge->getFrameOffsetB();
      std::cout << "  offsetB: " << stringify(offsetB.getOrigin()) << std::endl;

    }

    if (constraint->getConstraintType() == 5)
    {
      auto coneTwist = (btConeTwistConstraint*)constraint;

      auto offsetA = coneTwist->getFrameOffsetA();
      std::cout << "  offsetA: " << stringify(offsetA.getOrigin()) << " [offset -> attach]" << std::endl;

      auto offsetB = coneTwist->getFrameOffsetB();
      std::cout << "  offsetB: " << stringify(offsetB.getOrigin()) << std::endl;

    }

    std::cout << std::endl;
  }

  std::cout << "built index" << std::endl;
  std::cout <<
      "  collision shapes: " << index.collisionShapes.size() << std::endl <<
      "  rigid bodies: " << index.rigidBodies.size() << std::endl <<
      "  constraints: " << index.constraints.size() << std::endl << std::endl;


  auto presetFilePath = parser.retrieve<std::string>("preset");

  std::cout << "loading preset " << presetFilePath << std::endl;

  std::ifstream presetFile(presetFilePath);
  Json::Value preset;
  presetFile >> preset;

  auto outputBuilder = OutputBuilder(index, preset);
  auto output = outputBuilder.build();
  auto outputFilePath = parser.retrieve<std::string>("output");

  std::cout << "writing to " << outputFilePath << std::endl;

  std::ofstream outFile(outputFilePath);
  outFile << output;
  outFile.close();

  return 0;
}
