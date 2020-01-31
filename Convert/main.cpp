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
  "left_wrist"
};

std::string constraintNames[] = {
  "root",
  "chest",
  "neck",
  "left_hip",
  "left_knee",
  "left_ankle",
  "left_shoulder",
  "left_elbow",
  "left_wrist",
  "right_hip",
  "right_knee",
  "right_ankle",
  "right_shoulder",
  "right_elbow",
  "right_wrist"
};

std::string stringify(const btQuaternion & q)
{
  btScalar x,y,z;
  q.getEulerZYX(z, y, x);
  std::stringstream ss;
  ss << "(" << x << ", " << y << ", " << z << ")";
  return ss.str();

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

template<typename key_t, typename val_t>
bool hasKey(const std::map<key_t,val_t> & m, key_t key)
{
  auto it = m.find(key);
  return it != m.end();
}

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
  Json::Value buildDrawShapeDefs();
  Json::Value buildDrawShapeDef(int i);
  void buildRigidBodyDef(int i, Json::Value & def);
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
  output["DrawShapeDefs"] = buildDrawShapeDefs();
  return output;
}

Json::Value OutputBuilder::buildJoints()
{
  Json::Value skeleton;

  for (int i = 0; i < std::size(constraintNames); ++i)
  {
    skeleton[i] = buildJoint(i);
  }

  return skeleton;
}

Json::Value OutputBuilder::buildJoint(int i)
{
  Json::Value joint = preset["Skeleton"]["Joints"][i];

  if (i > 0)
  {
    auto constraintName = constraintNames[i];
    auto jointName = joint["Name"].asString();

    std::cout << "Mapping Constraint " << constraintName << " to " << jointName << " found? " << hasKey(index.constraints, constraintName) << std::endl;

    auto constraint = index.constraints.at(constraintName);

    if (constraint->getConstraintType() == 4)
    {
      auto hinge = (btHingeConstraint*)constraint;
      auto offsetA = hinge->getFrameOffsetA().getOrigin();

      joint["AttachX"] = offsetA.x();
      joint["AttachY"] = offsetA.y();
      joint["AttachZ"] = offsetA.z();
    }

    if (constraint->getConstraintType() == 5)
    {
      auto coneTwist = (btConeTwistConstraint*)constraint;
      auto offsetB = coneTwist->getFrameOffsetB().getOrigin();

      joint["AttachX"] = offsetB.x();
      joint["AttachY"] = offsetB.y();
      joint["AttachZ"] = offsetB.z();
    }

    auto & rbA = constraint->getRigidBodyA();
    auto & rbB = constraint->getRigidBodyB();

    std::cout << "CenterOfMass[A] " << stringify(rbA.getCenterOfMassPosition()) << std::endl;
    std::cout << "CenterOfMass[B] " << stringify(rbB.getCenterOfMassPosition()) << std::endl;

    auto parentJointID = joint["Parent"].asInt();

    if (parentJointID == 0)
    {
      auto parentRigidBody = index.rigidBodies.at("root");
      auto rootBodyOffset = parentRigidBody->getWorldTransform().getOrigin();

      joint["AttachX"] = joint["AttachX"].asFloat() + rootBodyOffset.x();
      joint["AttachY"] = joint["AttachY"].asFloat() + rootBodyOffset.y();
      joint["AttachZ"] = joint["AttachZ"].asFloat() + rootBodyOffset.z();
    }

    if (parentJointID > 0)
    {
      auto parentJointName = constraintNames[parentJointID];
      auto parentConstraint = index.constraints.at(parentJointName);

      if (parentConstraint->getConstraintType() == 4)
      {
        auto hinge = (btHingeConstraint*)parentConstraint;
        auto offsetB = hinge->getFrameOffsetB().getOrigin();

        joint["AttachX"] = joint["AttachX"].asFloat() - offsetB.x();
        joint["AttachY"] = joint["AttachY"].asFloat() - offsetB.y();
        joint["AttachZ"] = joint["AttachZ"].asFloat() - offsetB.z();
      }

      if (parentConstraint->getConstraintType() == 5)
      {
        auto coneTwist = (btConeTwistConstraint*)parentConstraint;
        auto offsetA = coneTwist->getFrameOffsetA().getOrigin();

        joint["AttachX"] = joint["AttachX"].asFloat() - offsetA.x();
        joint["AttachY"] = joint["AttachY"].asFloat() - offsetA.y();
        joint["AttachZ"] = joint["AttachZ"].asFloat() - offsetA.z();
      }
    }
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

  if (true)
  {
    buildRigidBodyDef(i, bodyDef);
  }

  return bodyDef;
}

Json::Value OutputBuilder::buildDrawShapeDefs()
{
  Json::Value skeleton;

  for (int i = 0; i < std::size(rigidBodyNames); ++i)
  {
    skeleton[i] = buildDrawShapeDef(i);
  }

  return skeleton;
}

Json::Value OutputBuilder::buildDrawShapeDef(int i)
{
  Json::Value bodyDef = preset["DrawShapeDefs"][i];

  if (true)
  {
    buildRigidBodyDef(i, bodyDef);
  }

  return bodyDef;
}


void OutputBuilder::buildRigidBodyDef(int i, Json::Value & bodyDef)
{
  auto rigidBodyName = rigidBodyNames[i];
  auto bodyDefName = bodyDef["Name"].asString();

  std::cout << "Mapping Rigid Body " << rigidBodyName << " to " << bodyDefName << " found? " << hasKey(index.rigidBodies, rigidBodyName) << std::endl;

  auto rigidBody = index.rigidBodies.at(rigidBodyName);
  auto collision = rigidBody->getCollisionShape();

  if (collision)
  {
    auto type = collision->getShapeType();
    auto margin = collision->getMargin();
    auto scaling = collision->getLocalScaling();

    if (type == BOX_SHAPE_PROXYTYPE)
    {
      auto box = (btBoxShape*) collision;
      auto shapeDims = box->getImplicitShapeDimensions();

      bodyDef["Shape"] = "box";
      bodyDef["Param0"] = (margin + shapeDims.x()) * 2;
      bodyDef["Param1"] = (margin + shapeDims.y()) * 2;
      bodyDef["Param2"] = (margin + shapeDims.z()) * 2;
    }

    if (type == SPHERE_SHAPE_PROXYTYPE)
    {
      auto sphere = (btSphereShape*) collision;
      auto radius = sphere->getRadius();

      bodyDef["Shape"] = "sphere";
      bodyDef["Param0"] = radius * 2;
      bodyDef["Param1"] = radius * 2;
      bodyDef["Param2"] = radius * 2;
    }
  }
  else
  {
    std::cout << "Could not find a collision shape for rigid body " << rigidBodyName << std::endl;
    return;
  }

  if (rigidBodyName == "root")
  {
      auto position = rigidBody->getWorldTransform().getOrigin();

      bodyDef["AttachX"] = position.x();
      bodyDef["AttachY"] = position.y();
      bodyDef["AttachZ"] = position.z();
  }

  if (hasKey(index.constraints, rigidBodyName))
  {
    auto constraint = index.constraints.at(rigidBodyName);

    std::cout << "Using parent constraint " << rigidBodyName << " for positioning" << std::endl;

    if (constraint->getConstraintType() == 4)
    {
      auto hinge = (btHingeConstraint*)constraint;
      auto offsetB = hinge->getFrameOffsetB().getOrigin();

      bodyDef["AttachX"] = -offsetB.x();
      bodyDef["AttachY"] = -offsetB.y();
      bodyDef["AttachZ"] = -offsetB.z();
    }

    if (constraint->getConstraintType() == 5)
    {
      auto coneTwist = (btConeTwistConstraint*)constraint;
      auto offsetA = coneTwist->getFrameOffsetA().getOrigin();

      bodyDef["AttachX"] = -offsetA.x();
      bodyDef["AttachY"] = -offsetA.y();
      bodyDef["AttachZ"] = -offsetA.z();
    }

	bodyDef["AttachZ"] = -1 * bodyDef["AttachZ"].asFloat();
  }
  else
  {
    std::cout << "Could not find a parent constraint for rigid body " << rigidBodyName << std::endl;
  }
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

        auto margin = collision->getMargin();

        std::cout << "    margin: " << margin << std::endl;

        auto scaling = collision->getLocalScaling();

        std::cout << "    scaling: " << stringify(scaling) << std::endl;

        auto worldT = body->getWorldTransform();

        std::cout << "    position: " << stringify(worldT.getOrigin()) << std::endl;
        std::cout << "    rotation: " << stringify(worldT.getRotation()) << std::endl;


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
      auto offsetB = hinge->getFrameOffsetB();
      std::cout << "  offsetA: " << stringify(offsetA.getOrigin()) << std::endl;
      std::cout << "  offsetB: " << stringify(offsetB.getOrigin()) << std::endl;
    }

    if (constraint->getConstraintType() == 5)
    {
      auto coneTwist = (btConeTwistConstraint*)constraint;

      auto offsetA = coneTwist->getFrameOffsetA();
      auto offsetB = coneTwist->getFrameOffsetB();
      std::cout << "  offsetA: " << stringify(offsetA.getOrigin()) << std::endl;
      std::cout << "  offsetB: " << stringify(offsetB.getOrigin()) << std::endl;
    }

    auto & rbA = constraint->getRigidBodyA();
    auto & rbB = constraint->getRigidBodyB();

    std::cout << "  centerA " << stringify(rbA.getCenterOfMassPosition()) << std::endl;
    std::cout << "  centerB " << stringify(rbB.getCenterOfMassPosition()) << std::endl;


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
