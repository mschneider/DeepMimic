#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.h"
#include "BulletWorldImporter/btMultiBodyWorldImporter.h"


#include <iostream>

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "usage:" << std::endl;
    std::cerr << "deep_mimic_convert bullet_file" << std::endl;
    return 1;
  }

  auto bulletFilePath = argv[1];
  auto collisionConfig = new btDefaultCollisionConfiguration();
  auto dispatcher = new btCollisionDispatcher(collisionConfig);
  auto broadphase = new btDbvtBroadphase();
  auto solver = new btMultiBodyConstraintSolver;
  auto world = new btMultiBodyDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
  auto fileLoader = new btMultiBodyWorldImporter(world);
  fileLoader->loadFile(bulletFilePath);

  printf("loaded file %s with:\n %i collision shapes\n %i rigid bodies\n %i constraints\n %i bvhs\n %i triangle info maps\n\n",
      bulletFilePath,
      fileLoader->getNumCollisionShapes(),
      fileLoader->getNumRigidBodies(),
      fileLoader->getNumConstraints(),
      fileLoader->getNumBvhs(),
      fileLoader->getNumTriangleInfoMaps());

  for (int i=0; i < fileLoader->getNumCollisionShapes(); i++)
  {
    printf("Collision Shape %i\n", i);
    auto obj = fileLoader->getCollisionShapeByIndex(i);

    printf("  object name = %s\n", fileLoader->getNameForPointer(obj));
  }

  printf("\n");

  for(int i=0; i < fileLoader->getNumRigidBodies(); i++)
  {
    printf("Rigid Body %i\n", i);

    btCollisionObject* obj = fileLoader->getRigidBodyByIndex(i);
    btRigidBody* body = btRigidBody::upcast(obj);

    printf("  object name = %s\n", fileLoader->getNameForPointer(body));
    printf("  get position = (%4.3f,%4.3f,%4.3f)\n",
        body->getCenterOfMassPosition().getX(),
        body->getCenterOfMassPosition().getY(),
        body->getCenterOfMassPosition().getZ());

    printf("  get local scaling = (%4.3f,%4.3f,%4.3f)\n",
        body->getCollisionShape()->getLocalScaling().getX(),
        body->getCollisionShape()->getLocalScaling().getY(),
        body->getCollisionShape()->getLocalScaling().getZ());

    if (body->getInvMass() == 0)
    {
      printf("  static object\n");
    }
    else
    {
      printf("  mass = %4.3f\n", 1/body->getInvMass());
    }

    printf("\n");
  }

  for(int i=0; i < fileLoader->getNumConstraints(); i++)
  {
    printf("Constraint %i\n", i);

    btTypedConstraint* constraint=fileLoader->getConstraintByIndex(i);
    printf("  object name = %s\n", fileLoader->getNameForPointer(constraint));
    printf("  constraint type = %i\n", constraint->getConstraintType());
    printf("  collision shape type = %i\n", constraint->getRigidBodyA().getCollisionShape()->getShapeType());
    printf("  get partA axisVector (part of quaternion) = (%4.3f,%4.3f,%4.3f)\n",
        constraint->getRigidBodyA().getCenterOfMassTransform().getRotation().getX(),
        constraint->getRigidBodyA().getCenterOfMassTransform().getRotation().getY(),
        constraint->getRigidBodyA().getCenterOfMassTransform().getRotation().getZ());

    printf("  get partA CenterOfGeometry = (%4.3f,%4.3f,%4.3f)\n",
        constraint->getRigidBodyA().getCenterOfMassPosition().getX(),
        constraint->getRigidBodyA().getCenterOfMassPosition().getY(),
        constraint->getRigidBodyA().getCenterOfMassPosition().getZ());


    if (constraint->getConstraintType() == 4)
    {
      auto hinge = (btHingeConstraint*)constraint;
      printf("  angle = %f  limit = %f - %f",
          hinge->getHingeAngle(),
          hinge->getLowerLimit(),
          hinge->getUpperLimit());
    }

    printf("\n");
  }


  return 0;
}
