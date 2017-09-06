// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// A very simple example that can be used as template project for
// a Chrono::Engine simulator with 3D view.
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/assets/ChPointPointDrawing.h"

#include <math.h>



// Use the namespace of Chrono

using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht

using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

//I will clean this up and move the robot into its own class later. For now I don't want to waste that time

//Robot parameters -> robot is oriented with X forward, Y up, and Z right. Origin is at center of left front wheel
//double robotLength = .6096*2;	//length from center front wheel to center rear wheel -> this is a function of other robot parameters
double robotWidth = .9144;		//width from center wheel to center wheel 

double wheelWidth = .15;
double wheelDia = .2286;
double wheelMass = 3;

double chassisW = .6096;
double chassisL = .6096;
double chassisH = .15;
double chassisMass = 30;

//connection points -> all angles are just angles at design configuration. The ride angles will change based on many robot parameters

//tibia is connector to front and rear wheels (larger bone in shin as it supports more direct weight ;) )
double tibiaLength = .3512;
double tibiaAngle = 30 * CH_C_PI / 180.0;	//degrees to radians and from horizontal
double tibiaMass = .25;

//thigh is connector from knee to chassis
double conW = 0.025;		//width of the square tube for thighs and shins -> no need to change this

double thighLength;
//double thighLength = .3244;	//Thigh length is a result of angles and shin lengths
double thighAngle = 30 * CH_C_PI / 180.0;	//degrees to radians and from horizontal
double thighMass = .25;

//tibia is connector to front and rear wheels (larger bone in shin as it supports more direct weight ;) )
double fibulaLength = .3512;
double fibulaAngle = 30 * CH_C_PI / 180.0;	//degrees to radians and from horizontal
double fibulaMass = .25;



//Spring properties
double tibiaSpringPt = 0.15; //distance from knee to spring connection point on fibula
double fibulaSpringPt = 0.15; //distance from knee to spring connection point on tibia
double k = 10000;			//spring constant between tibia and fibula
double c = 1000;			//damping coefficient between tibia and fibula
double restLength = .22;		//rest length of springs

//Torque values at wheels
double torqueLeftSide = 2;
double torqueRightSide = 2;


int main(int argc, char* argv[]) {
    // Set path to Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);
    
    // Create a Chrono physical system
    ChSystemNSC mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"A simple project template", core::dimension2d<u32>(1280,920),
                         false);  // screen dimensions

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(4, 1, -5),
                                 core::vector3df(0, .5, 0));  // to change the position of camera
    // application.AddLightWithShadow(vector3df(1,25,-5), vector3df(0,0,0), 35, 0.2,35, 55, 512, video::SColorf(1,1,1));

    //======================================================================

    // HERE YOU CAN POPULATE THE PHYSICAL SYSTEM WITH BODIES AND LINKS.
    //
    // An example: a pendulum.

    // 1-Create a floor that is fixed (that is used also to represent the absolute reference)
	
	// Create a material that will be used for the floor
	//auto mmaterial = std::make_shared<ChMaterialSurfaceNSC>();
	//mmaterial->SetFriction(0.4f);
	//mmaterial->SetCompliance(0.0000005f);
	//mmaterial->SetComplianceT(0.0000005f);
	//mmaterial->SetDampingF(0.2f);

    auto floorBody = std::make_shared<ChBodyEasyBox>(100, 2, 100,  // x, y, z dimensions
                                                     1000,       // density
                                                     true,      // contact geometry - allow collision
                                                     true        // enable visualization geometry
                                                     );
    floorBody->SetPos(ChVector<>(0, -1.3, 0));
	//floorBody->SetMaterialSurface(mmaterial);
    floorBody->SetBodyFixed(true);

    mphysicalSystem.Add(floorBody);
	// Optionally, attach a RGB color asset to the floor, for better visualization
	auto color = std::make_shared<ChColorAsset>();
	color->SetColor(ChColor(0.2f, 0.25f, 0.25f));
	floorBody->AddAsset(color);



	//-----------------------ROBOT----------------------------------//

	//figure out thigh length
	thighLength = ( tibiaLength*cos(tibiaAngle) ) / cos(thighAngle);
	double robotLength = 2.0*tibiaLength*cos(tibiaAngle) + 2.0*fibulaLength*cos(fibulaAngle);


	// Add chassis
	auto chassis = std::make_shared<ChBodyEasyBox>(chassisL, chassisH, chassisW,	// x,y,z size
		200,													// density
		true,													// collide enable?
		true													// visualization?		
		);													
	chassis->SetMass(chassisMass);
	chassis->SetPos(ChVector<>(-(tibiaLength*cos(tibiaAngle) + thighLength*cos(thighAngle)), //X location
		tibiaLength*sin(tibiaAngle) + thighLength*sin(thighAngle),  //Y location
		robotWidth/2.0));
	mphysicalSystem.Add(chassis);
	chassis->SetBodyFixed(false);
	
	//create the attachments to the body

	//left front thigh
	auto thighLF = std::make_shared<ChBodyEasyBox>(thighLength, conW, conW,	// x,y,z size
		200,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	thighLF->SetMass(thighMass);
	thighLF->SetPos(ChVector<>(-(tibiaLength*cos(tibiaAngle) + .5*thighLength*cos(thighAngle)),		//X direction
		tibiaLength*sin(tibiaAngle)+.5*thighLength*sin(thighAngle),
		wheelWidth/2.0+conW));
	thighLF->SetRot(Q_from_AngZ(-thighAngle));
	mphysicalSystem.Add(thighLF);

	//left rear thigh
	auto thighLR = std::make_shared<ChBodyEasyBox>(thighLength, conW, conW,	// x,y,z size
		200,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	thighLR->SetMass(thighMass);
	thighLR->SetPos(ChVector<>(-(tibiaLength*cos(tibiaAngle) + 1.5*thighLength*cos(thighAngle)),		//X direction
		tibiaLength*sin(tibiaAngle) + .5*thighLength*sin(thighAngle),
		wheelWidth / 2.0 + conW));
	thighLR->SetRot(Q_from_AngZ(thighAngle));
	mphysicalSystem.Add(thighLR);

	//right front thigh
	auto thighRF = std::make_shared<ChBodyEasyBox>(thighLength, conW, conW,	// x,y,z size
		200,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	thighRF->SetMass(thighMass);
	thighRF->SetPos(ChVector<>(-(tibiaLength*cos(tibiaAngle) + .5*thighLength*cos(thighAngle)),		//X direction
		tibiaLength*sin(tibiaAngle) + .5*thighLength*sin(thighAngle),
		robotWidth - (wheelWidth / 2.0 + conW)));
	thighRF->SetRot(Q_from_AngZ(-thighAngle));
	mphysicalSystem.Add(thighRF);

	//right front thigh
	auto thighRR = std::make_shared<ChBodyEasyBox>(thighLength, conW, conW,	// x,y,z size
		200,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	thighRR->SetMass(thighMass);
	thighRR->SetPos(ChVector<>(-(tibiaLength*cos(tibiaAngle) + 1.5*thighLength*cos(thighAngle)),		//X direction
		tibiaLength*sin(tibiaAngle) + .5*thighLength*sin(thighAngle),
		robotWidth - (wheelWidth / 2.0 + conW)));
	thighRR->SetRot(Q_from_AngZ(thighAngle));
	mphysicalSystem.Add(thighRR);

	//connect the thighs to the chassis
	auto thighLFJoint = std::make_shared<ChLinkLockRevolute>();
	thighLFJoint->Initialize(chassis, thighLF, ChCoordsys<>(ChVector<>(-(tibiaLength*cos(tibiaAngle) + thighLength*cos(thighAngle)), //X location
		tibiaLength*sin(tibiaAngle) + thighLength*sin(thighAngle),  //Y location
		thighLF->GetPos().z()),	//Z location
		{ 1,0,0,0 }));	//rotation
	mphysicalSystem.Add(thighLFJoint);

	auto thighLRJoint = std::make_shared<ChLinkLockRevolute>();
	thighLRJoint->Initialize(chassis, thighLR, ChCoordsys<>(ChVector<>(-(tibiaLength*cos(tibiaAngle) + thighLength*cos(thighAngle)), //X location
		tibiaLength*sin(tibiaAngle) + thighLength*sin(thighAngle),  //Y location
		thighLR->GetPos().z()),	//Z location
		{ 1,0,0,0 }));	//rotation
	mphysicalSystem.Add(thighLRJoint);

	auto thighRFJoint = std::make_shared<ChLinkLockRevolute>();
	thighRFJoint->Initialize(chassis, thighRF, ChCoordsys<>(ChVector<>(-(tibiaLength*cos(tibiaAngle) + thighLength*cos(thighAngle)), //X location
		tibiaLength*sin(tibiaAngle) + thighLength*sin(thighAngle),  //Y location
		thighRF->GetPos().z()),	//Z location
		{ 1,0,0,0 }));	//rotation
	mphysicalSystem.Add(thighRFJoint);

	auto thighRRJoint = std::make_shared<ChLinkLockRevolute>();
	thighRRJoint->Initialize(chassis, thighRR, ChCoordsys<>(ChVector<>(-(tibiaLength*cos(tibiaAngle) + thighLength*cos(thighAngle)), //X location
		tibiaLength*sin(tibiaAngle) + thighLength*sin(thighAngle),  //Y location
		thighRR->GetPos().z()),	//Z location
		{ 1,0,0,0 }));	//rotation
	mphysicalSystem.Add(thighRRJoint);




	//Add shins

	//left front tibia
	auto tibiaLF = std::make_shared<ChBodyEasyBox>(tibiaLength, conW, conW,	// x,y,z size
		200,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	tibiaLF->SetMass(tibiaMass);
	tibiaLF->SetPos(ChVector<>(-.5*tibiaLength*cos(tibiaAngle),		//X direction
		.5*tibiaLength*sin(tibiaAngle),
		wheelWidth / 2.0 + conW));
	tibiaLF->SetRot(Q_from_AngZ(-tibiaAngle));
	mphysicalSystem.Add(tibiaLF);

	//left rear tibia
	auto tibiaLR = std::make_shared<ChBodyEasyBox>(tibiaLength, conW, conW,	// x,y,z size
		200,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	tibiaLR->SetMass(tibiaMass);
	tibiaLR->SetPos(ChVector<>(-(1.5*tibiaLength*cos(tibiaAngle) + 2 * thighLength*cos(thighAngle)),		//X direction
		.5*tibiaLength*sin(tibiaAngle),
		wheelWidth / 2.0 + conW));
	tibiaLR->SetRot(Q_from_AngZ(tibiaAngle));
	mphysicalSystem.Add(tibiaLR);

	//right front tibia
	auto tibiaRF = std::make_shared<ChBodyEasyBox>(tibiaLength, conW, conW,	// x,y,z size
		200,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	tibiaRF->SetMass(tibiaMass);
	tibiaRF->SetPos(ChVector<>(-.5*tibiaLength*cos(tibiaAngle),		//X direction
		.5*tibiaLength*sin(tibiaAngle),
		robotWidth - (wheelWidth / 2.0 + conW)));
	tibiaRF->SetRot(Q_from_AngZ(-tibiaAngle));
	mphysicalSystem.Add(tibiaRF);

	//right rear tibia
	auto tibiaRR = std::make_shared<ChBodyEasyBox>(tibiaLength, conW, conW,	// x,y,z size
		200,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	tibiaRR->SetMass(tibiaMass);
	tibiaRR->SetPos(ChVector<>(-(1.5*tibiaLength*cos(tibiaAngle) + 2 * thighLength*cos(thighAngle)),		//X direction
		.5*tibiaLength*sin(tibiaAngle),
		robotWidth - (wheelWidth / 2.0 + conW)));
	tibiaRR->SetRot(Q_from_AngZ(tibiaAngle));
	mphysicalSystem.Add(tibiaRR);

	//connect the thighs to the tibias
	auto tibiaLFJoint = std::make_shared<ChLinkLockRevolute>();
	tibiaLFJoint->Initialize(thighLF, tibiaLF, ChCoordsys<>(ChVector<>(-(tibiaLength*cos(tibiaAngle)), //X location
		tibiaLength*sin(tibiaAngle),  //Y location
		tibiaLF->GetPos().z()),	//Z location
		{ 1,0,0,0 }));	//rotation
	mphysicalSystem.Add(tibiaLFJoint);

	auto tibiaLRJoint = std::make_shared<ChLinkLockRevolute>();
	tibiaLRJoint->Initialize(thighLR, tibiaLR, ChCoordsys<>(ChVector<>(-(2.0*thighLength*cos(thighAngle)+tibiaLength*cos(tibiaAngle)), //X location
		tibiaLength*sin(tibiaAngle),  //Y location
		tibiaLR->GetPos().z()),	//Z location
		{ 1,0,0,0 }));	//rotation
	mphysicalSystem.Add(tibiaLRJoint);

	auto tibiaRFJoint = std::make_shared<ChLinkLockRevolute>();
	tibiaRFJoint->Initialize(thighRF, tibiaRF, ChCoordsys<>(ChVector<>(-(tibiaLength*cos(tibiaAngle)), //X location
		tibiaLength*sin(tibiaAngle),  //Y location
		tibiaRF->GetPos().z()),	//Z location
		{ 1,0,0,0 }));	//rotation
	mphysicalSystem.Add(tibiaRFJoint);

	auto tibiaRRJoint = std::make_shared<ChLinkLockRevolute>();
	tibiaRRJoint->Initialize(thighRR, tibiaRR, ChCoordsys<>(ChVector<>(-(2.0*thighLength*cos(thighAngle) + tibiaLength*cos(tibiaAngle)), //X location
		tibiaLength*sin(tibiaAngle),  //Y location
		tibiaRR->GetPos().z()),	//Z location
		{ 1,0,0,0 }));	//rotation
	mphysicalSystem.Add(tibiaRRJoint);


	//left front fibula
	auto fibulaLF = std::make_shared<ChBodyEasyBox>(fibulaLength, conW, conW,	// x,y,z size
		200,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	fibulaLF->SetMass(fibulaMass);
	fibulaLF->SetPos(ChVector<>(-(tibiaLength*cos(tibiaAngle) + .5*fibulaLength*cos(fibulaAngle)),		//X direction
		.5*fibulaLength*sin(fibulaAngle),
		wheelWidth / 2.0 + conW));
	fibulaLF->SetRot(Q_from_AngZ(fibulaAngle));
	mphysicalSystem.Add(fibulaLF);

	//left front fibula
	auto fibulaLR = std::make_shared<ChBodyEasyBox>(fibulaLength, conW, conW,	// x,y,z size
		200,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	fibulaLR->SetMass(fibulaMass);
	fibulaLR->SetPos(ChVector<>(-(tibiaLength*cos(tibiaAngle) + 1.5*fibulaLength*cos(fibulaAngle)),		//X direction
		.5*fibulaLength*sin(fibulaAngle),
		wheelWidth / 2.0 + conW));
	fibulaLR->SetRot(Q_from_AngZ(-fibulaAngle));
	mphysicalSystem.Add(fibulaLR);

	//left front fibula
	auto fibulaRF = std::make_shared<ChBodyEasyBox>(fibulaLength, conW, conW,	// x,y,z size
		200,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	fibulaRF->SetMass(fibulaMass);
	fibulaRF->SetPos(ChVector<>(-(tibiaLength*cos(tibiaAngle) + .5*fibulaLength*cos(fibulaAngle)),		//X direction
		.5*fibulaLength*sin(fibulaAngle),
		robotWidth - (wheelWidth / 2.0 + conW)));
	fibulaRF->SetRot(Q_from_AngZ(fibulaAngle));
	mphysicalSystem.Add(fibulaRF);

	//right rear fibula
	auto fibulaRR = std::make_shared<ChBodyEasyBox>(fibulaLength, conW, conW,	// x,y,z size
		200,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	fibulaRR->SetMass(fibulaMass);
	fibulaRR->SetPos(ChVector<>(-(tibiaLength*cos(tibiaAngle) + 1.5*fibulaLength*cos(fibulaAngle)),		//X direction
		.5*fibulaLength*sin(fibulaAngle),
		robotWidth - (wheelWidth / 2.0 + conW)));
	fibulaRR->SetRot(Q_from_AngZ(-fibulaAngle));
	mphysicalSystem.Add(fibulaRR);

	//connect fibulas to thighs and each other
	auto fibulaLFJoint = std::make_shared<ChLinkLockRevolute>();
	fibulaLFJoint->Initialize(thighLF, fibulaLF, ChCoordsys<>(ChVector<>(-tibiaLength*cos(tibiaAngle), //X location
		tibiaLength*sin(tibiaAngle),  //Y location
		fibulaLF->GetPos().z()),	//Z location
		{ 1,0,0,0 }));	//rotation
	mphysicalSystem.Add(fibulaLFJoint);

	auto fibulaLRJoint = std::make_shared<ChLinkLockRevolute>();
	fibulaLRJoint->Initialize(thighLR, fibulaLR, ChCoordsys<>(ChVector<>(-(2.0*thighLength*cos(thighAngle) + tibiaLength*cos(tibiaAngle)), //X location
		tibiaLength*sin(tibiaAngle),  //Y location
		fibulaLR->GetPos().z()),	//Z location
		{ 1,0,0,0 }));	//rotation
	mphysicalSystem.Add(fibulaLRJoint);

	auto fibulaRFJoint = std::make_shared<ChLinkLockRevolute>();
	fibulaRFJoint->Initialize(thighRF, fibulaRF, ChCoordsys<>(ChVector<>(-tibiaLength*cos(tibiaAngle), //X location
		tibiaLength*sin(tibiaAngle),  //Y location
		fibulaRF->GetPos().z()),	//Z location
		{ 1,0,0,0 }));	//rotation
	mphysicalSystem.Add(fibulaRFJoint);

	auto fibulaRRJoint = std::make_shared<ChLinkLockRevolute>();
	fibulaRRJoint->Initialize(thighRR, fibulaRR, ChCoordsys<>(ChVector<>(-(2.0*thighLength*cos(thighAngle) + tibiaLength*cos(tibiaAngle)), //X location
		tibiaLength*sin(tibiaAngle),  //Y location
		fibulaRR->GetPos().z()),	//Z location
		{ 1,0,0,0 }));	//rotation
	mphysicalSystem.Add(fibulaRRJoint);


	//to each other
	auto fibulasLeftJoint = std::make_shared<ChLinkLockRevolute>();
	fibulasLeftJoint->Initialize(fibulaLF, fibulaLR, ChCoordsys<>(ChVector<>(-(fibulaLength*cos(fibulaAngle) + tibiaLength*cos(tibiaAngle)), //X location
		tibiaLength*sin(tibiaAngle) - fibulaLength*sin(fibulaAngle),  //Y location
		fibulaLR->GetPos().z()),	//Z location
		{ 1,0,0,0 }));	//rotation
	mphysicalSystem.Add(fibulasLeftJoint);

	auto fibulasRightJoint = std::make_shared<ChLinkLockRevolute>();
	fibulasRightJoint->Initialize(fibulaRF, fibulaRR, ChCoordsys<>(ChVector<>(-(fibulaLength*cos(fibulaAngle) + tibiaLength*cos(tibiaAngle)), //X location
		tibiaLength*sin(tibiaAngle) - fibulaLength*sin(fibulaAngle),  //Y location
		fibulaRR->GetPos().z()),	//Z location
		{ 1,0,0,0 }));	//rotation
	mphysicalSystem.Add(fibulasRightJoint);









	// Add wheels as cylinders
	auto texture = std::make_shared<ChTexture>();
	texture->SetTextureFilename(GetChronoDataFile("redwhite.png"));  // texture in ../data

	auto wheel_0 = std::make_shared<ChBodyEasyCylinder>(wheelDia / 2.0, wheelWidth, 300,// density
		true,// collide
		true// visualization		
		);
	wheel_0->SetMass(wheelMass);
	wheel_0->SetPos(ChVector<>(0, 0, 0));
	wheel_0->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	mphysicalSystem.Add(wheel_0);
	wheel_0->AddAsset(texture);

	auto wheel_1 = std::make_shared<ChBodyEasyCylinder>(wheelDia/2.0, wheelWidth, 300,// density
		true,// collide
		true// visualization		
		);
	wheel_1->SetMass(wheelMass);
	wheel_1->SetPos(ChVector<>(-(tibiaLength*cos(tibiaAngle) + fibulaLength*cos(fibulaAngle)),0,0));
	wheel_1->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	mphysicalSystem.Add(wheel_1);
	wheel_1->AddAsset(texture);

	auto wheel_2 = std::make_shared<ChBodyEasyCylinder>(wheelDia / 2.0, wheelWidth, 300,// density
		true,// collide
		true// visualization		
		);
	wheel_2->SetMass(wheelMass);
	wheel_2->SetPos(ChVector<>(-(2.0*tibiaLength*cos(tibiaAngle) + 2.0*fibulaLength*cos(fibulaAngle)), 0, 0));
	wheel_2->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	mphysicalSystem.Add(wheel_2);
	wheel_2->AddAsset(texture);

	auto wheel_3 = std::make_shared<ChBodyEasyCylinder>(wheelDia / 2.0, wheelWidth, 300,// density
		true,// collide
		true// visualization		
		);
	wheel_3->SetMass(wheelMass);
	wheel_3->SetPos(ChVector<>(0, 0, robotWidth));
	wheel_3->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	mphysicalSystem.Add(wheel_3);
	wheel_3->AddAsset(texture);

	auto wheel_4 = std::make_shared<ChBodyEasyCylinder>(wheelDia / 2.0, wheelWidth, 300,// density
		true,// collide
		true// visualization		
		);
	wheel_4->SetMass(wheelMass);
	wheel_4->SetPos(ChVector<>(-(tibiaLength*cos(tibiaAngle) + fibulaLength*cos(fibulaAngle)), 0, robotWidth));
	wheel_4->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	mphysicalSystem.Add(wheel_4);
	wheel_4->AddAsset(texture);

	auto wheel_5 = std::make_shared<ChBodyEasyCylinder>(wheelDia / 2.0, wheelWidth, 300,// density
		true,// collide
		true// visualization		
		);
	wheel_5->SetMass(wheelMass);
	wheel_5->SetPos(ChVector<>(-(2.0*tibiaLength*cos(tibiaAngle) + 2.0*fibulaLength*cos(fibulaAngle)), 0, robotWidth));
	wheel_5->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	mphysicalSystem.Add(wheel_5);
	wheel_5->AddAsset(texture);

	//connect wheels to shins
	auto wheel0joint = std::make_shared<ChLinkLockRevolute>();
	wheel0joint->Initialize(tibiaLF, wheel_0, ChCoordsys<>(ChVector<>(wheel_0->GetPos().x(), wheel_0->GetPos().y(), .5*wheelWidth), { 1,0,0,0 }));
	mphysicalSystem.Add(wheel0joint);

	auto wheel1joint = std::make_shared<ChLinkLockRevolute>();
	wheel1joint->Initialize(fibulaLF, wheel_1, ChCoordsys<>(ChVector<>(wheel_1->GetPos().x(), wheel_1->GetPos().y(), .5*wheelWidth), { 1,0,0,0 }));
	mphysicalSystem.Add(wheel1joint);

	auto wheel2joint = std::make_shared<ChLinkLockRevolute>();
	wheel2joint->Initialize(tibiaLR, wheel_2, ChCoordsys<>(ChVector<>(wheel_2->GetPos().x(), wheel_2->GetPos().y(), .5*wheelWidth), { 1,0,0,0 }));
	mphysicalSystem.Add(wheel2joint);

	auto wheel3joint = std::make_shared<ChLinkLockRevolute>();
	wheel3joint->Initialize(tibiaRF, wheel_3, ChCoordsys<>(ChVector<>(wheel_3->GetPos().x(), wheel_3->GetPos().y(), robotWidth - .5*wheelWidth), { 1,0,0,0 }));
	mphysicalSystem.Add(wheel3joint);

	auto wheel4joint = std::make_shared<ChLinkLockRevolute>();
	wheel4joint->Initialize(fibulaRF, wheel_4, ChCoordsys<>(ChVector<>(wheel_4->GetPos().x(), wheel_4->GetPos().y(), robotWidth - .5*wheelWidth), { 1,0,0,0 }));
	mphysicalSystem.Add(wheel4joint);

	auto wheel5joint = std::make_shared<ChLinkLockRevolute>();
	wheel5joint->Initialize(tibiaRR, wheel_5, ChCoordsys<>(ChVector<>(wheel_5->GetPos().x(), wheel_5->GetPos().y(), robotWidth - .5*wheelWidth), { 1,0,0,0 }));
	mphysicalSystem.Add(wheel5joint);

	//Add springs between tibia and fibulas
	auto col_1 = std::make_shared<ChColorAsset>();
	col_1->SetColor(ChColor(0.6f, 0, 0));

	// Create right side springs
	// Create a spring between elements 1 and 5 on the right side
	auto springtLF = std::make_shared<ChLinkSpring>();
	springtLF->Initialize(tibiaLF,	// first body to link it with
		fibulaLF,	// second body to link it with
		false,	// pos absolute
		ChVector<>(-((tibiaLength- tibiaSpringPt)*cos(tibiaAngle)) , 
		(tibiaLength - tibiaSpringPt)*sin(tibiaAngle), tibiaLF->GetPos().z()), // position of first end of spring
		ChVector<>(-(tibiaLength*cos(tibiaAngle) + fibulaSpringPt*cos(fibulaAngle)), 
		tibiaLength*sin(tibiaAngle) - fibulaSpringPt*sin(fibulaAngle), tibiaLF->GetPos().z()), // position of second end of spring
		false,	// rest length not original length
		restLength);	// rest length
	mphysicalSystem.Add(springtLF);
	springtLF->Set_SpringK(k);
	springtLF->Set_SpringR(c);
	// Attach a visualization asset.
	springtLF->AddAsset(col_1);
	springtLF->AddAsset(std::make_shared<ChPointPointSpring>(.0125, 20, 10));

	auto springtLR = std::make_shared<ChLinkSpring>();
	springtLR->Initialize(tibiaLR,	// first body to link it with
		fibulaLR,	// second body to link it with
		false,	// pos absolute
		ChVector<>(-(2.0*fibulaLength*cos(fibulaAngle) + (tibiaLength + tibiaSpringPt)*cos(tibiaAngle)), 
		(tibiaLength - tibiaSpringPt)*sin(tibiaAngle), tibiaLR->GetPos().z()), // position of first end of spring
		ChVector<>(-(tibiaLength*cos(tibiaAngle) + (2.0*fibulaLength - fibulaSpringPt)*cos(fibulaAngle)),
			tibiaLength*sin(tibiaAngle) - fibulaSpringPt*sin(fibulaAngle), tibiaLR->GetPos().z()), // position of second end of spring
		false,	// rest length not original length
		restLength);	// rest length
	mphysicalSystem.Add(springtLR);
	springtLR->Set_SpringK(k);
	springtLR->Set_SpringR(c);
	// Attach a visualization asset.
	springtLR->AddAsset(col_1);
	springtLR->AddAsset(std::make_shared<ChPointPointSpring>(.0125, 20, 10));

	auto springtRF = std::make_shared<ChLinkSpring>();
	springtRF->Initialize(tibiaRF,	// first body to link it with
		fibulaRF,	// second body to link it with
		false,	// pos absolute
		ChVector<>(-((tibiaLength - tibiaSpringPt)*cos(tibiaAngle)), 
		(tibiaLength - tibiaSpringPt)*sin(tibiaAngle), tibiaRF->GetPos().z()), // position of first end of spring
		ChVector<>(-(tibiaLength*cos(tibiaAngle) + fibulaSpringPt*cos(fibulaAngle)),
			tibiaLength*sin(tibiaAngle) - fibulaSpringPt*sin(fibulaAngle), tibiaRF->GetPos().z()), // position of second end of spring
		false,	// rest length not original length
		restLength);	// rest length
	mphysicalSystem.Add(springtRF);
	springtRF->Set_SpringK(k);
	springtRF->Set_SpringR(c);
	// Attach a visualization asset.
	springtRF->AddAsset(col_1);
	springtRF->AddAsset(std::make_shared<ChPointPointSpring>(.0125, 20, 10));

	auto springtRR = std::make_shared<ChLinkSpring>();
	springtRR->Initialize(tibiaRR,	// first body to link it with
		fibulaRR,	// second body to link it with
		false,	// pos absolute
		ChVector<>(-(2.0*fibulaLength*cos(fibulaAngle) + (tibiaLength + tibiaSpringPt)*cos(tibiaAngle)), 
		(tibiaLength - tibiaSpringPt)*sin(tibiaAngle), tibiaRR->GetPos().z()), // position of first end of spring
		ChVector<>(-(tibiaLength*cos(tibiaAngle) + (2.0*fibulaLength-fibulaSpringPt)*cos(fibulaAngle)),
			tibiaLength*sin(tibiaAngle) - fibulaSpringPt*sin(fibulaAngle), tibiaRR->GetPos().z()), // position of second end of spring
		false,	// rest length not original length
		restLength);	// rest length
	mphysicalSystem.Add(springtRR);
	springtRR->Set_SpringK(k);
	springtRR->Set_SpringR(c);
	// Attach a visualization asset.
	springtRR->AddAsset(col_1);
	springtRR->AddAsset(std::make_shared<ChPointPointSpring>(.0125, 20, 10));



	auto obstacleBox1 = std::make_shared<ChBodyEasyBox>(.2, .1, 1.22, 1000, true, true);
	obstacleBox1->SetMass(10.0);
	obstacleBox1->SetPos(ChVector<>(2.0, -.3, 0));
	mphysicalSystem.Add(obstacleBox1);
	obstacleBox1->SetBodyFixed(true);
	//auto obstacleColor = std::make_shared<ChColorAsset>();
	//obstacleColor->SetColor(ChColor(0.2f, 0.1f, 0.1f));
	auto obstacleTexture = std::make_shared<ChTexture>();
	obstacleTexture->SetTextureFilename(GetChronoDataFile("cubetexture_wood.png"));  // texture in ../data
	obstacleBox1->AddAsset(obstacleTexture);









	//set torque on the wheels
	wheel0joint->Set_Scr_torque(torqueLeftSide);
	wheel1joint->Set_Scr_torque(torqueLeftSide);
	wheel2joint->Set_Scr_torque(torqueLeftSide);
	wheel3joint->Set_Scr_torque(torqueRightSide);
	wheel4joint->Set_Scr_torque(torqueRightSide);
	wheel5joint->Set_Scr_torque(torqueRightSide);

	
	


    //======================================================================

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    // Adjust some settings:
	double step_size = 0.001;
    application.SetTimestep(0.001);
    application.SetTryRealtime(false);
	mphysicalSystem.SetMaxItersSolverSpeed(5000);

    //
    // THE SOFT-REAL-TIME CYCLE
    //

	//calculate robot mass as given from parameters
	double robotMass = 6 * wheelMass + 4 * tibiaMass + 4 * fibulaMass + 4 * thighMass + chassisMass;

	std::cout << "ROBOT LENGTH: " << robotLength << std::endl;
	std::cout << "ROBOT MASS: " << robotMass << std::endl;

	bool sim = true;
    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        // This performs the integration timestep!
        //application.DoStep();
		if (sim) {
			mphysicalSystem.DoStepDynamics(step_size);
			sim = false;
		}

		mphysicalSystem.DoStepDynamics(step_size);

        application.EndScene();
    }

    return 0;
}
