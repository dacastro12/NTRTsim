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
 * @file SimHipTFModel.cpp
 * @brief Contains the definition of the members of the class ControlTFModel.
 *	Model is inspired by Tom Flemmons tensegrity model of the knee and is a working progress.
 * $Id$
 */

// This module
#include "SimHipTFModel.h"
// This library
#include "core/tgRod.h"
//#include "core/tgSphere.h"
#include "core/tgBasicActuator.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
//#include "tgcreator/tgRigidAutoCompound.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
//#include "tgcreator/tgSphereInfo.h"
//#include "tgcreator/tgUtil.h"
// The Bullet Physics library
#include "btBulletDynamicsCommon.h"
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
    const struct Config
    {
        double densityA;
        double densityB;
        double radiusA;
	double radiusB;
        double stiffness;
        double damping;
        double pretension;
        double triangle_length;//femur_c
        double triangle_height;//fibia_c
        double Knee_height;
	 double friction;
        double rollFriction;
        double restitution;
    } c =
   {
       0.2,     // density (mass / length^3)
       0.0,     // density
       0.25,     // radius (length)
       0.15,	// radiusB
       1000.0,   // stiffness (mass / sec^2)
       10.0,     // damping (mass / sec)
       10.0,     // pretension (mass * length / sec^2) //5001
       10.0,     // triangle_length (length)
       10.0,     // triangle_height (length)
       10.0,     // Knee_height (length)
       0.99,      // friction (unitless)
       0.01,     // rollFriction (unitless)
       0.0,      // restitution (?)	
  };
} // namespace

SimHipTFModel::SimHipTFModel() :
tgModel()
{
}

SimHipTFModel::~SimHipTFModel()
{
}

void SimHipTFModel::addNodes(tgStructure& tetra,
                            double edge,
                            double width,
                            double height)
{

//tibia and fibia (Cross Beams)
    //bottom origin
	tetra.addNode(0,0,0);//0
    // bottom front
    tetra.addNode(0, 0, 1.75); // 1
    // bottom left
    tetra.addNode( 1.75, 0, 0); // 2
    // bottom back
    tetra.addNode(0, 0, -1.75); // 3
    // bottom right
    tetra.addNode(-1.75, 0, 0); //4
    //lower knee joint origin
	tetra.addNode(0, height, 0);//5
    //knee joint left
    tetra.addNode(1.5, height+2, 0); // 6
    //knee joint right
    tetra.addNode( -1.5, height+2, 0); // 7
    


//femur
    // knee joint front (patella)
    tetra.addNode(0, height, 2); // 8
    // knee joint left
    tetra.addNode(1.25, height, -1.25); //9
    // knee joint right
    tetra.addNode(-1.25, height, -1.25); //10
    // knee joint origin
    tetra.addNode(0, height+2, 0); // 11
    // top origin
    tetra.addNode( 0, (height*2)+2, 0); // 12
    // top front
    tetra.addNode(0, (height*2)+2, 2); // 13
    // top front left
    tetra.addNode(1, (height*2)+2, 1);// 14
    //top back left
    tetra.addNode(1, (height*2)+2, -1); //15
    // top back 
    tetra.addNode(0, (height*2)+2, -2); // 16
    // top back right
    tetra.addNode( -1, (height*2)+2, -1); // 17
    // top front right
    tetra.addNode(-1, (height*2)+2, 1); // 18
    // top right mid
    tetra.addNode(-1, (height*2)+2, 0);//19
    // top left mid
    tetra.addNode(1, (height*2)+2, 0);//20


//new point 
   // lower leg attachment point.....
    tetra.addNode( 0, (height*(0.7)), 0); //21
    tetra.addNode(0, (height*(0.7)), -0.175); //22

  //ee tracker 
    //tetra.addNode(0,-1,0);//23


//hip
//Pyramid Design
    // top front left
    tetra.addNode(1.28, (height*2)+2, 0.5);// 23
    //top back left
    tetra.addNode(1.28, (height*2)+2, -0.5); //24
    // top back right
    tetra.addNode( -1.28, (height*2)+2, -0.5); //25
    // top front right
    tetra.addNode(-1.28, (height*2)+2, 0.5); // 26
    // top  front origin
    tetra.addNode(0, (height*2)+4, 0.5);//27
    // top back origin
    tetra.addNode(0, (height*2)+4, -0.5);//28
     // front point
    tetra.addNode(0, (height*2)+4, 1.6);//29
    // back point
    tetra.addNode(0, (height*2)+4, -1.6);//30
    

}

/*void SimHipTFModel::addNodesB(tgStructure& tetra,
                            double edge,
                            double width,
                            double height)
{
//base to hold tibia and fibula
    //bottom origin
	tetra.addNode(0,0,0);//0
    // bottom right
    tetra.addNode(0.6, 0, 0.6); // 1
    // bottom left
    tetra.addNode( 0.6, 0, -0.6); // 2
    // bottom front
    tetra.addNode(-0.6, 0, 0.6); // 3
    // bottom back
    tetra.addNode(-0.6, 0, -0.6); //4
    // bottom right
    tetra.addNode(0.6, -5, 0.6); // 5
    // bottom left
    tetra.addNode( 0.6, -5, -0.6); // 6
    // bottom front
    tetra.addNode(-0.6, -5, 0.6); // 7
    // bottom back
    tetra.addNode(-0.6, -5, -0.6); //8
}
*/
void SimHipTFModel::addPairs(tgStructure& tetra)
{
//fibula and tibia
	//Bottom Base or Ankle
    tetra.addPair( 0,  1, "rod");
    tetra.addPair( 0,  2, "rod");
    tetra.addPair( 0,  3, "rod");
    tetra.addPair( 0, 4, "rod");
    tetra.addPair(1, 2, "rod");
    tetra.addPair(2, 3, "rod");
    tetra.addPair(3, 4, "rod");
    tetra.addPair(4, 1, "rod");   
	//tibia and fibula structure
    tetra.addPair(0, 21,"rod");
    tetra.addPair(0, 5, "rod");
	// attachment lower point
    tetra.addPair(21,22, "rod");
	// lower knee joint
    tetra.addPair( 5,  6, "rod");
    tetra.addPair( 5,  7, "rod");

//femur
	//upper knee joint
	tetra.addPair( 11, 8, "rod");
	tetra.addPair( 11, 9, "rod");
	tetra.addPair( 11, 10, "rod");
	//femur structure
	tetra.addPair( 11, 12, "rod");

	//Top Base or Hip
	tetra.addPair(12, 13, "rod");
	tetra.addPair(12, 16, "rod");
	tetra.addPair(12, 19, "rod");
	tetra.addPair(13, 14, "rod");
        tetra.addPair(12, 20, "rod");
        //tetra.addPair(14, 20, "rodB");
	//tetra.addPair(20, 15, "rodB");
	tetra.addPair(15, 16, "rod");
	tetra.addPair(16, 17, "rod");
        //tetra.addPair(17, 19, "rodB");
        //tetra.addPair(19, 18, "rodB");
	tetra.addPair(13, 18, "rod");
//Hip
	//Pyramid Model
	tetra.addPair(23, 27, "rodB");
	tetra.addPair(26, 27, "rodB");
	tetra.addPair(24, 28, "rodB");
	tetra.addPair(25, 28, "rodB");
        //s.addPair(28, 29, "rodB");
	//s.addPair(28, 30, "rodB");
	tetra.addPair(29, 30, "rodB");
	
	
}

/*void SimHipTFModel::addPairsB(tgStructure& tetra)
{
//Holding Structure (Massless)
	//Bottom
    tetra.addPair( 1,  5, "rodB");
    tetra.addPair( 6,  2, "rodB");
    tetra.addPair( 7,  3, "rodB");
    tetra.addPair( 8, 4, "rodB");
    	//Top
    tetra.addPair( 1,  2, "rodB");
    tetra.addPair( 2,  4, "rodB");
    tetra.addPair( 3,  4, "rodB");
    tetra.addPair( 3,  1, "rodB");
}
*/
void SimHipTFModel::addMuscles(tgStructure& tetra)
{
//Tibia and Fibia Section
	//Calve
	tetra.addPair(3, 9, "gastro");//Gastrocnemius (Lateral)
	tetra.addPair(3, 10, "gastro");//Gastrocnemius (Medial)
	//Shin
	//tetra.addPair(1, 8, "muscle");//Tibialis Anterior
	//Lower stabilization
	//tetra.addPair(2, 9, "muscle");//Peroneus Longus
	//tetra.addPair(4, 10, "muscle");//Plantaris (Incorrect attachment point)
	//********Thought to be Patella Tendons but need feedback *************
	tetra.addPair(8, 4, "muscle");
	//tetra.addPair(8, 2, "muscle");

//Knee Joint Ligaments *********May need to rearrange for anatomical correctness************
	tetra.addPair(8, 6, "muscle");
	tetra.addPair(8, 7, "muscle");
	tetra.addPair(7, 10, "muscle");
	tetra.addPair(6, 9, "muscle");
	//tetra.addPair(10, 9, "muscle");
//Added to Knee Joint to fully enclose the joint 2/1/2016 May need to delete

	tetra.addPair(7, 11, "muscle"); //Added 2/1/2016
        tetra.addPair(6, 11, "muscle"); //Added 2/1/2016
        tetra.addPair(8, 5, "muscle"); //Added 2/1/2016
        tetra.addPair(9, 5, "muscle"); //Added 2/1/2016
        tetra.addPair(10, 5, "muscle"); //Added 2/1/2016
//Femur Section
	//tetra.addPair(8, 12, "muscle");//Rectus Femoris
	tetra.addPair(7, 18, "muscle");//Vastus Medialis
	tetra.addPair(6, 14, "muscle");//Vastus Lateralis
	tetra.addPair(7, 17, "muscle");//Semimembranosus
	tetra.addPair(6, 15, "muscle");//Bicep Femoris Long Head
	//tetra.addPair(5, 16, "flexion"); //Real bicep (3/4)
	//May need to change geometry of the attachment point 17 and 15 to provide torque to flexion
        tetra.addPair(17,22, "flexion");//Semimebranosus
        tetra.addPair(15, 22, "flexion");//Bicep Femoris Long Head

//Hip Section
	//Joint
	tetra.addPair(14, 23, "muscle"); //Left side
	tetra.addPair(20, 23, "muscle");
	tetra.addPair(20, 24, "muscle");
	tetra.addPair(15, 24, "muscle");
	
	tetra.addPair(17, 25, "muscle");// Right side
	tetra.addPair(19, 25, "muscle");
	tetra.addPair(19, 26, "muscle");
	tetra.addPair(18, 26, "muscle");

	tetra.addPair(27, 12, "muscle");//Top
	tetra.addPair(28, 12, "muscle");

	//Movement Muscles (Not yet Implemented)
	tetra.addPair(11, 23, "muscle");
	tetra.addPair(9, 24, "muscle");
	tetra.addPair(10, 25, "muscle");
	tetra.addPair(11, 26, "muscle");
	tetra.addPair(29, 8, "muscle");

}

void SimHipTFModel::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    // Note that pretension is defined for this string
    const tgRod::Config rodConfigA(c.radiusA, c.densityA, c.friction, c.rollFriction, c.restitution);
    const tgRod::Config rodConfigB(c.radiusB, c.densityB, c.friction, c.rollFriction, c.restitution);//Massless rods for base holder
    const tgBasicActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension);
    //welding holders
   // const tgSphere::Config weldingConfigA(0.25, c.densityA);
    //fixed segment
   /* tgStructure tetraB;
    addNodesB(tetraB, c.triangle_length, c.triangle_height, c.Knee_height);
    addPairsB(tetraB);
    tetraB.move(btVector3(0,0.15,0));
*/
    // Create a structure that will hold the details of this model
    tgStructure tetra;

    //child (Taken out for massless femur)
  //  tgStructure* const tB = new tgStructure(tetraB);
  //  tetra.addChild(tB);
	
    // Add nodes to the structure
    addNodes(tetra, c.triangle_length, c.triangle_height, c.Knee_height);

    // Add rods to the structure
    addPairs(tetra);

    // Add muscles to the structure
    addMuscles(tetra);

    // Move the structure so it doesn't start in the ground
    tetra.move(btVector3(0, 10, 0));

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rodB", new tgRodInfo(rodConfigB));    
    spec.addBuilder("rod", new tgRodInfo(rodConfigA));
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    spec.addBuilder("gastro", new tgBasicActuatorInfo(muscleConfig));
    //spec.addBuilder("GastMedial", new tgBasicActuatorInfo(muscleConfig));
    spec.addBuilder("flexion", new tgBasicActuatorInfo(muscleConfig));
    //spec.addBuilder("Bicep Femoris", new tgBasicActuatorInfo(muscleConfig));
    //spec.addBuilder("sphere", new tgSphereInfo(weldingConfigA));

    // Create your structureInfo
    tgStructureInfo structureInfo(tetra, spec);

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

void SimHipTFModel::step(double dt)
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

void SimHipTFModel::onVisit(tgModelVisitor& r)
{
    // Example: m_rod->getRigidBody()->dosomething()...
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& SimHipTFModel::getAllMuscles() const
{
    return allMuscles;
}

void SimHipTFModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}