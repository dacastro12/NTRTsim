/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 *
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/

/**
 * @file Hung2ControlTFModel.cpp
 * @brief Contains the definition of the members of the class ControlTFModel.
 *	Model is inspired by Tom Flemmons tensegrity model of the knee and is a working progress.
 * $Id$
 */

// This module
#include "Hung2ControlTFModel.h"
// This library
#include "core/tgRod.h"
#include "core/tgBasicActuator.h"
#include "core/abstractMarker.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgKinematicContactCableInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>
#include <iostream>

/**
 * Anonomous namespace so we don't have to declare the config in
 * the header.
 */
namespace
{
    /**
     * Configuration parameters so they're easily accessable.
     * All parameters must be positive.
     */
    const struct ConfigRod
    {
        double densityA;
        double densityB;
        double radiusA;
	double radiusB;
        double total_height;
        double femur_c;
        double fibia_c;
	double friction;
        double rollFriction;
        double restitution;
        
        
    } cRod =
   {
       0.05,     // density (mass / length^3)
       0.0,     // density
       0.1197,     // radius (length) (should be 0.01197m)
       0.1197,	// radiusB
       17.78,     // total height in decimeters (5'10"= 1.778m)
       0.245,     // femur constant (length)
       0.245,     // fibia constant (length)
       0.99,      // friction (unitless)
       0.01,     // rollFriction (unitless)
       0.2,      // restitution (?)
           	
  };

   const struct ConfigCable 
{
      double elasticity;
      double damping; 
      double stiffness;
      double pretension;
      double knee_pretension;
      bool   history;
      double maxTens;
      double targetVelocity;
      double mRad;
      double motorFriction;
      double motorInertia;
      bool   backDrivable;
} cCable = 
{
      100.0,     // elasticity
      2000.0,      // damping (kg/s)
      3000.0,     // stiffness (kg/sec^2)
      1.0,       // pretension (N = kg*length/sec^2)
      500/10,       // knee pretension (500 N = kg*m/s^2 = 50000 kg*cm/s^2)
      false,      // histroy (boolean)
      100000,     // maxTens
      1.0,        // mRad
      10.0,       // motorFriction
      1.0,        // motorInertia
      false       // backDrivable (boolean)
};
} // namespace

Hung2ControlTFModel::Hung2ControlTFModel() :
tgModel()
{
}

Hung2ControlTFModel::~Hung2ControlTFModel()
{
}

void Hung2ControlTFModel::addNodes(tgStructure& s,
                            double height,
                            double femur,
                            double fibia)
{

//tibia and fibia (Cross Beams)
    //bottom origin
	s.addNode(0,0,0);//0
    // bottom front
    s.addNode(0, 0, (height*fibia*0.25)); // 1
    // bottom left
    s.addNode( (height*fibia*0.25), 0, 0); // 2
    // bottom back
    s.addNode(0, 0, -(height*fibia*0.25)); // 3
    // bottom right
    s.addNode(-(height*fibia*0.25), 0, 0); //4
    //lower knee joint origin
	s.addNode(0, 0.75*height*fibia, 0);//5
    //knee joint left
    s.addNode(0.25*height*fibia, height*fibia, 0); // 6
    //knee joint right
    s.addNode( -0.25*height*fibia, height*fibia, 0); // 7
    


//femur
    // knee joint front (patella)
    s.addNode(0, height*fibia, height*femur*0.25); // 8
    // knee joint left
    s.addNode(height*femur*0.25, height*fibia, -height*femur*0.25); //9
    // knee joint right
    s.addNode(-height*femur*0.25, height*fibia, -height*femur*0.25); //10
    // knee joint origin
    s.addNode(0, height*fibia + 0.25*height*femur, 0); // 11
    // top origin
    s.addNode( 0, height*fibia + height*femur, 0); // 12
    // top front
    s.addNode(0, height*fibia + height*femur, 0.25*height*femur); // 13
    // top front left
    s.addNode(0.125*height*femur, height*fibia + height*femur, 0.125*height*femur);// 14
    //top back left
    s.addNode(0.125*height*femur, height*fibia + height*femur, -0.125*height*femur); //15
    // top back 
    s.addNode(0, height*fibia + height*femur, -0.25*height*femur); // 16
    // top back right
    s.addNode( -0.125*height*femur, height*fibia + height*femur, -0.125*height*femur); // 17
    // top front right
    s.addNode(-0.125*height*femur, height*fibia + height*femur, 0.125*height*femur); // 18
    // top right mid
    s.addNode(-0.125*height*femur, height*fibia + height*femur, 0);//19
    // top left mid
    s.addNode(0.125*height*femur, height*fibia + height*femur, 0);//20

//new point 
   // lower leg attachment point.....
    s.addNode( 0, (height*fibia*(0.7)), 0); //21
    s.addNode(0, (height*fibia*(0.7)), -0.175); //22

  //ee tracker 
    s.addNode(0,-1,0);//23
}

/*void Hung2ControlTFModel::addNodesB(tgStructure& s,
                            double edge,
                            double width,
                            double height)
{
//base to hold tibia and fibula
    //bottom origin
	s.addNode(0,0,0);//0
    // bottom right
    s.addNode(0.6, 0, 0.6); // 1
    // bottom left
    s.addNode( 0.6, 0, -0.6); // 2
    // bottom front
    s.addNode(-0.6, 0, 0.6); // 3
    // bottom back
    s.addNode(-0.6, 0, -0.6); //4
    // bottom right
    s.addNode(0.6, -5, 0.6); // 5
    // bottom left
    s.addNode( 0.6, -5, -0.6); // 6
    // bottom front
    s.addNode(-0.6, -5, 0.6); // 7
    // bottom back
    s.addNode(-0.6, -5, -0.6); //8
}
*/
void Hung2ControlTFModel::addRods(tgStructure& s)
{
//fibula and tibia
	//Bottom Base or Ankle
    s.addPair( 0,  1, "rod");
    s.addPair( 0,  2, "rod");
    s.addPair( 0,  3, "rod");
    s.addPair( 0, 4, "rod");
    s.addPair( 0, 23, "ee endeffector");

    s.addPair(1, 2, "rod");
    s.addPair(2, 3, "rod");
    s.addPair(3, 4, "rod");
    s.addPair(4, 1, "rod");   
	//tibia and fibula structure
    s.addPair(0, 21, "rod");
    s.addPair(21, 5, "rod");
    // attachment point
    s.addPair(21, 22, "rod");
	// lower knee joint
    s.addPair( 5,  6, "rod");
    s.addPair( 5,  7, "rod");

//femur
	//upper knee joint
	s.addPair( 11, 8, "rodB");
	s.addPair( 11, 9, "rodB");
	s.addPair( 11, 10, "rodB");
	//femur structure
	s.addPair( 11, 12, "rodB");

	//Top Base or Hip
	s.addPair(12, 13, "rodB");
	s.addPair(12, 16, "rodB");
	s.addPair(12, 19, "rodB");
	s.addPair(13, 14, "rodB");
        s.addPair(12, 20, "rodB");
        s.addPair(14, 20, "rodB");
	s.addPair(20, 15, "rodB");
	s.addPair(15, 16, "rodB");
	s.addPair(16, 17, "rodB");
        s.addPair(17, 19, "rodB");
        s.addPair(19, 18, "rodB");
	s.addPair(13, 18, "rodB");
	
}

/*void Hung2ControlTFModel::addPairsB(tgStructure& s)
{
//Holding Structure (Massless)
	//Bottom
    s.addPair( 1,  5, "rodB");
    s.addPair( 6,  2, "rodB");
    s.addPair( 7,  3, "rodB");
    s.addPair( 8, 4, "rodB");
    	//Top
    s.addPair( 1,  2, "rodB");
    s.addPair( 2,  4, "rodB");
    s.addPair( 3,  4, "rodB");
    s.addPair( 3,  1, "rodB");
}
*/
void Hung2ControlTFModel::addMuscles(tgStructure& s)
{
//Tibia and Fibia Section
	//Calve
	s.addPair(3, 9, "gastro");//Gastrocnemius (Lateral)
	s.addPair(3, 10, "gastro");//Gastrocnemius (Medial)
	//Shin
	//s.addPair(1, 8, "muscle");//Tibialis Anterior
	//Lower stabilization
	//s.addPair(2, 9, "muscle");//Peroneus Longus
	//s.addPair(4, 10, "muscle");//Plantaris (Incorrect attachment point)
	//********Thought to be Patella Tendons but need feedback *************
	s.addPair(8, 0, "muscle");
	//s.addPair(8, 2, "muscle");

//Knee Joint Ligaments *********May need to rearrange for anatomical correctness************
	s.addPair(8, 6, "joint");
	s.addPair(8, 7, "joint");
	s.addPair(7, 10, "joint");
	s.addPair(6, 9, "joint");
	//s.addPair(10, 9, "joint");//Making a straight line in the back of joint.
//Added to Knee Joint to fully enclose the joint 2/1/2016
	s.addPair(7, 11, "joint"); //Added 2/1/2016
        s.addPair(6, 11, "joint"); //Added 2/1/2016
        s.addPair(8, 5, "joint"); //Added 2/1/2016
        s.addPair(9, 5, "joint"); //Added 2/1/2016
        s.addPair(10, 5, "joint"); //Added 2/1/2016
//Femur Section
	//s.addPair(8, 12, "muscle");//Rectus Femoris
	s.addPair(7, 18, "muscle");//Vastus Medialis
	s.addPair(6, 14, "muscle");//Vastus Lateralis
	s.addPair(7, 17, "muscle");//Semimembranosus (stablization)
	s.addPair(6, 15, "muscle");//Bicep Femoris Long Head (stablization)
	//May need to change geometry of the attachment point 17 and 15 to provide torque to flexion
        s.addPair(17,22, "flexion");//Semimebranosus
        s.addPair(15, 22, "flexion");//Bicep Femoris Long Head
}

void Hung2ControlTFModel::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    // Note that pretension is defined for this string
    const tgRod::Config rodConfigA(cRod.radiusA, cRod.densityA, cRod.friction, cRod.rollFriction, cRod.restitution);
    const tgRod::Config rodConfigB(cRod.radiusB, cRod.densityB, cRod.friction, cRod.rollFriction, cRod.restitution);//Massless rods for base holder
       const tgRod::Config eeConfig(cRod.radiusA, cRod.densityA/4, cRod.friction, cRod.rollFriction, cRod.restitution);

//setting up muscle Configurations
    const tgBasicActuator::Config muscleConfig(cCable.stiffness, cCable.damping, cCable.pretension, cCable.history, cCable.maxTens, cCable.targetVelocity);
   // Joint
    const tgBasicActuator::Config KneeJointMuscleConfig(cCable.stiffness, cCable.damping, cCable.knee_pretension, cCable.history, cCable.maxTens, cCable.targetVelocity);

   //welding holders
   // const tgSphere::Config weldingConfigA(0.25, c.densityA);
    //fixed segment
   /* tgStructure sB;
    addNodesB(sB, c.triangle_length, c.triangle_height, c.Knee_height);
    addPairsB(sB);
    sB.move(btVector3(0,0.15,0));
*/
    // Create a structure that will hold the details of this model
    tgStructure s;

    //child (Taken out for massless femur)
  //  tgStructure* const tB = new tgStructure(sB);
  //  s.addChild(tB);
	
    // Add nodes to the structure
    addNodes(s, cRod.total_height, cRod.femur_c, cRod.fibia_c);

    // Add rods to the structure
    addRods(s);

    // Add muscles to the structure
    addMuscles(s);

    // Move the structure so it doesn't start in the ground
    s.move(btVector3(0, 5, 0));

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rodB", new tgRodInfo(rodConfigB));    
    spec.addBuilder("rod", new tgRodInfo(rodConfigA));
    spec.addBuilder("ee", new tgRodInfo(eeConfig));
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    spec.addBuilder("gastro", new tgBasicActuatorInfo(muscleConfig));
    //spec.addBuilder("GastMedial", new tgBasicActuatorInfo(muscleConfig));
    spec.addBuilder("flexion", new tgBasicActuatorInfo(muscleConfig));
    spec.addBuilder("joint", new tgBasicActuatorInfo(KneeJointMuscleConfig));
    //spec.addBuilder("Bicep Femoris", new tgBasicActuatorInfo(muscleConfig));
    //spec.addBuilder("sphere", new tgSphereInfo(weldingConfigA));

    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control.
    allMuscles = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());

    // Notify controllers that setup has finished.
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);
}

void Hung2ControlTFModel::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        tgModel::step(dt);  // Step any children
    }
}

void Hung2ControlTFModel::onVisit(tgModelVisitor& r)
{
    // Example: m_rod->getRigidBody()->dosomething()...
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& Hung2ControlTFModel::getAllMuscles() const
{
    return allMuscles;
}

void Hung2ControlTFModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
