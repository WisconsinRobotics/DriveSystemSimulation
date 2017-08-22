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
#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono_irrlicht/ChIrrApp.h"


double inTom = 1. / 39.3701;	// converting inches to meters

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
    application.AddTypicalCamera(core::vector3df(4, 2, 5),	// position of camera
                                 core::vector3df(0, 1, 0));  // where camera is looking
    // application.AddLightWithShadow(vector3df(1,25,-5), vector3df(0,0,0), 35, 0.2,35, 55, 512, video::SColorf(1,1,1));

    //======================================================================

    // HERE YOU CAN POPULATE THE PHYSICAL SYSTEM WITH BODIES AND LINKS.

    // 1-Create a floor that is fixed (that is used also to represent the absolute reference)
	
	// Create a material that will be used for the floor
	//auto mmaterial = std::make_shared<ChMaterialSurfaceNSC>();
	//mmaterial->SetFriction(0.4f);
	//mmaterial->SetCompliance(0.0000005f);
	//mmaterial->SetComplianceT(0.0000005f);
	//mmaterial->SetDampingF(0.2f);
	auto wheelTexture = std::make_shared<ChTexture>();
	wheelTexture->SetTextureFilename(GetChronoDataFile("redwhite.png"));  // texture in ../data

    auto floorBody = std::make_shared<ChBodyEasyBox>(100, 1, 100,  // x, y, z dimensions
                                                     1000,       // density
                                                     true,      // contact geometry - allow collision
                                                     true        // enable visualization geometry
                                                     );
    floorBody->SetPos(ChVector<>(0, -.5, 0));
	//floorBody->SetMaterialSurface(mmaterial);
    floorBody->SetBodyFixed(true);
	mphysicalSystem.Add(floorBody);
	
	// Optionally, attach a RGB color asset to the floor, for better visualization
	/*auto color = std::make_shared<ChColorAsset>();
	color->SetColor(ChColor(0.2f, 0.25f, 0.25f));
	floorBody->AddAsset(color);
	*/
	auto floorTexture = std::make_shared<ChTexture>();
	floorTexture->SetTextureFilename(GetChronoDataFile("rock.jpg"));  // texture in ../data
	floorBody->AddAsset(floorTexture);

	
	
	// ===============================
	// Create Chassis
	auto frameBox = std::make_shared<ChBodyEasyBox>(48.*inTom, 2.*inTom, 36.*inTom,	// x,y,z size
		100,													// density
		true,													// collide enable?
		true													// visualization?		
		);													
	frameBox->SetMass(10.0);
	frameBox->SetPos(ChVector<>(0, 30.*inTom, 0));
	mphysicalSystem.Add(frameBox);
	//frameBox->SetBodyFixed(true);	//suspend in the air
	

	// ===============================
	// Create Right Side Drive System
	// Create component 1 (attached to frame) on right side
	auto frameSideRight_1 = std::make_shared<ChBodyEasyBox>(12.*sqrt(2.)*inTom, 1.*inTom, 1.*inTom,	// x,y,z size
		100,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	frameSideRight_1->SetMass(1.0);
	frameSideRight_1->SetPos(ChVector<>(19.757*inTom, 22.652*inTom, (36./2.+.5)*inTom));
	frameSideRight_1->SetRot(Q_from_AngZ(CH_C_PI / 3.));
	mphysicalSystem.Add(frameSideRight_1);
	
	auto frameSideRightPivot_1 = std::make_shared<ChLinkLockRevolute>();
	frameSideRightPivot_1->Initialize(frameSideRight_1, frameBox, ChCoordsys<>(ChVector<>(24.*inTom, 30.*inTom, (36. / 2. + .5)*inTom), Q_from_AngY(0)));
	mphysicalSystem.Add(frameSideRightPivot_1);

	// Create component 2 (attached to frame) on right side
	auto frameSideRight_2 = std::make_shared<ChBodyEasyBox>(12.*sqrt(2.)*inTom, 1. * inTom, 1. * inTom,	// x,y,z size
		100,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	frameSideRight_2->SetMass(1.0);
	frameSideRight_2->SetPos(ChVector<>(-19.757*inTom, 22.652*inTom, (36. / 2. + .5)*inTom));
	frameSideRight_2->SetRot(Q_from_AngZ(-CH_C_PI / 3.));
	mphysicalSystem.Add(frameSideRight_2);

	auto frameSideRightPivot_2 = std::make_shared<ChLinkLockRevolute>();
	frameSideRightPivot_2->Initialize(frameSideRight_2, frameBox, ChCoordsys<>(ChVector<>(-24.*inTom, 30.*inTom, (36. / 2. + .5)*inTom), Q_from_AngY(0)));
	mphysicalSystem.Add(frameSideRightPivot_2);

	// Create component 3 (attached to middle wheel) on right side
	auto frameSideRight_3 = std::make_shared<ChBodyEasyBox>(12.*sqrt(2.)*inTom, 1. * inTom, 1. * inTom,	// x,y,z size
		100,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	frameSideRight_3->SetMass(1.0);
	frameSideRight_3->SetPos(ChVector<>(7.757*inTom, 11.865*inTom, (36./2.+.5)*inTom));
	frameSideRight_3->SetRot(Q_from_AngZ(23.91*CH_C_PI/180.));
	mphysicalSystem.Add(frameSideRight_3);

	auto frameSideRightPivot_3 = std::make_shared<ChLinkLockRevolute>();
	frameSideRightPivot_3->Initialize(frameSideRight_1, frameSideRight_3, ChCoordsys<>(ChVector<>(15.515*inTom, 15.303*inTom, (36./2.+.5)*inTom), Q_from_AngY(0)));
	mphysicalSystem.Add(frameSideRightPivot_3);

	// Create component 4 (attached to middle wheel) on right side
	auto frameSideRight_4 = std::make_shared<ChBodyEasyBox>(12.*sqrt(2.)*inTom, 1. * inTom, 1. * inTom,	// x,y,z size
		100,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	frameSideRight_4->SetMass(1.0);
	frameSideRight_4->SetPos(ChVector<>(-7.757*inTom, 11.865*inTom, (36./2.+.5)*inTom));
	frameSideRight_4->SetRot(Q_from_AngZ(-23.91*CH_C_PI / 180.));
	mphysicalSystem.Add(frameSideRight_4);

	auto frameSideRightPivot_4 = std::make_shared<ChLinkLockRevolute>();
	frameSideRightPivot_4->Initialize(frameSideRight_2, frameSideRight_4, ChCoordsys<>(ChVector<>(-15.515*inTom, 15.303*inTom, (36. / 2. + .5)*inTom), Q_from_AngY(0)));
	mphysicalSystem.Add(frameSideRightPivot_4);

	// Create component 5 (attached to middle wheel) on right side
	auto frameSideRight_5 = std::make_shared<ChBodyEasyBox>(12.*sqrt(2.)*inTom, 1. * inTom, 1. * inTom,	// x,y,z size
		100,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	frameSideRight_5->SetMass(1.0);
	frameSideRight_5->SetPos(ChVector<>(23.272*inTom, 11.865*inTom, (36. / 2. + .5)*inTom));
	frameSideRight_5->SetRot(Q_from_AngZ(-23.91*CH_C_PI / 180.));
	mphysicalSystem.Add(frameSideRight_5);

	auto frameSideRightPivot_5 = std::make_shared<ChLinkLockRevolute>();
	frameSideRightPivot_5->Initialize(frameSideRight_1, frameSideRight_5, ChCoordsys<>(ChVector<>(15.515*inTom, 15.303*inTom, (36. / 2. + .5)*inTom), Q_from_AngY(0)));
	mphysicalSystem.Add(frameSideRightPivot_5);

	// Create component 6 (attached to middle wheel) on right side
	auto frameSideRight_6 = std::make_shared<ChBodyEasyBox>(12.*sqrt(2.)*inTom, 1. * inTom, 1. * inTom,	// x,y,z size
		100,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	frameSideRight_6->SetMass(1.0);
	frameSideRight_6->SetPos(ChVector<>(-23.272*inTom, 11.865*inTom, (36. / 2. + .5)*inTom));
	frameSideRight_6->SetRot(Q_from_AngZ(23.91*CH_C_PI / 180.));
	mphysicalSystem.Add(frameSideRight_6);

	auto frameSideRightPivot_6 = std::make_shared<ChLinkLockRevolute>();
	frameSideRightPivot_6->Initialize(frameSideRight_2, frameSideRight_6, ChCoordsys<>(ChVector<>(-15.515*inTom, 15.303*inTom, (36. / 2. + .5)*inTom), Q_from_AngY(0)));
	mphysicalSystem.Add(frameSideRightPivot_6);


	// add wheels on the right side
	// add wheelRight_1
	auto wheelRight_1 = std::make_shared<ChBodyEasyCylinder>(
		6 * inTom, // radius
		6 * inTom, // height
		100,// density
		true,// collide
		true// visualization		
		);
	wheelRight_1->SetPos(ChVector<>(31.029*inTom, 8.426*inTom, (36. / 2. + .5)*inTom));
	wheelRight_1->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	mphysicalSystem.Add(wheelRight_1);

	wheelRight_1->AddAsset(wheelTexture);
	
	//create a revolute joint for wheel 1 and chassis
	auto wheelRightJoint_1 = std::make_shared<ChLinkLockRevolute>();
	wheelRightJoint_1->Initialize(frameSideRight_5, wheelRight_1, ChCoordsys<>(wheelRight_1->GetPos(), Q_from_AngY(0)));
	mphysicalSystem.Add(wheelRightJoint_1);

	// add wheelRight_2
	auto wheelRight_2 = std::make_shared<ChBodyEasyCylinder>(
		6*inTom, // radius
		6*inTom, // height
		100,// density
		true,// collide
		true// visualization		
		);
	wheelRight_2->SetPos(ChVector<>(0, 8.426*inTom, (36./2.+.5)*inTom));
	wheelRight_2->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	mphysicalSystem.Add(wheelRight_2);

	wheelRight_2->AddAsset(wheelTexture);
	
	//create a revolute joint for wheel 2 and chassis
	auto wheelRightJoint_2 = std::make_shared<ChLinkLockRevolute>();
	wheelRightJoint_2->Initialize(frameSideRight_3, wheelRight_2, ChCoordsys<>(wheelRight_2->GetPos(), Q_from_AngY(0)));
	mphysicalSystem.Add(wheelRightJoint_2);
	auto wheelRightJoint_2_2 = std::make_shared<ChLinkLockRevolute>();
	wheelRightJoint_2_2->Initialize(frameSideRight_4, wheelRight_2, ChCoordsys<>(wheelRight_2->GetPos(), Q_from_AngY(0)));
	mphysicalSystem.Add(wheelRightJoint_2_2);

	// add wheelRight_3
	auto wheelRight_3 = std::make_shared<ChBodyEasyCylinder>(
		6 * inTom, // radius
		6 * inTom, // height
		100,// density
		true,// collide
		true// visualization		
		);
	wheelRight_3->SetPos(ChVector<>(-31.029*inTom, 8.426*inTom, (36. / 2. + .5)*inTom));
	wheelRight_3->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	mphysicalSystem.Add(wheelRight_3);

	wheelRight_3->AddAsset(wheelTexture);

	//create a revolute joint for wheel 1 and chassis
	auto wheelRightJoint_3 = std::make_shared<ChLinkLockRevolute>();
	wheelRightJoint_3->Initialize(frameSideRight_6, wheelRight_3, ChCoordsys<>(wheelRight_3->GetPos(), Q_from_AngY(0)));
	mphysicalSystem.Add(wheelRightJoint_3);


	// ===============================
	// Create Left Side Drive System
	// Create component 1 (attached to frame) on left side
	auto frameSideLeft_1 = std::make_shared<ChBodyEasyBox>(12.*sqrt(2.)*inTom, 1. * inTom, 1. * inTom,	// x,y,z size
		100,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	frameSideLeft_1->SetMass(1.0);
	frameSideLeft_1->SetPos(ChVector<>(19.757*inTom, 22.652*inTom, -(36. / 2. + .5)*inTom));
	frameSideLeft_1->SetRot(Q_from_AngZ(CH_C_PI / 3.));
	mphysicalSystem.Add(frameSideLeft_1);

	auto frameSideLeftPivot_1 = std::make_shared<ChLinkLockRevolute>();
	frameSideLeftPivot_1->Initialize(frameSideLeft_1, frameBox, ChCoordsys<>(ChVector<>(24.*inTom, 30.*inTom, -(36. / 2. + .5)*inTom), Q_from_AngY(0)));
	mphysicalSystem.Add(frameSideLeftPivot_1);

	// Create component 2 (attached to frame) on left side
	auto frameSideLeft_2 = std::make_shared<ChBodyEasyBox>(12.*sqrt(2.)*inTom, 1. * inTom, 1. * inTom,	// x,y,z size
		100,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	frameSideLeft_2->SetMass(1.0);
	frameSideLeft_2->SetPos(ChVector<>(-19.757*inTom, 22.652*inTom, -(36. / 2. + .5)*inTom));
	frameSideLeft_2->SetRot(Q_from_AngZ(-CH_C_PI / 3.));
	mphysicalSystem.Add(frameSideLeft_2);

	auto frameSideLeftPivot_2 = std::make_shared<ChLinkLockRevolute>();
	frameSideLeftPivot_2->Initialize(frameSideLeft_2, frameBox, ChCoordsys<>(ChVector<>(-24.*inTom, 30.*inTom, -(36. / 2. + .5)*inTom), Q_from_AngY(0)));
	mphysicalSystem.Add(frameSideLeftPivot_2);

	// Create component 3 (attached to middle wheel) on left side
	auto frameSideLeft_3 = std::make_shared<ChBodyEasyBox>(12.*sqrt(2.)*inTom, 1. * inTom, 1. * inTom,	// x,y,z size
		100,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	frameSideLeft_3->SetMass(1.0);
	frameSideLeft_3->SetPos(ChVector<>(7.757*inTom, 11.865*inTom, -(36. / 2. + .5)*inTom));
	frameSideLeft_3->SetRot(Q_from_AngZ(23.91*CH_C_PI / 180));
	mphysicalSystem.Add(frameSideLeft_3);

	auto frameSideLeftPivot_3 = std::make_shared<ChLinkLockRevolute>();
	frameSideLeftPivot_3->Initialize(frameSideLeft_1, frameSideLeft_3, ChCoordsys<>(ChVector<>(15.515*inTom, 15.303*inTom, -(36. / 2. + .5)*inTom), Q_from_AngY(0)));
	mphysicalSystem.Add(frameSideLeftPivot_3);

	// Create component 4 (attached to middle wheel) on left side
	auto frameSideLeft_4 = std::make_shared<ChBodyEasyBox>(12.*sqrt(2.)*inTom, 1. * inTom, 1. * inTom,	// x,y,z size
		100,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	frameSideLeft_4->SetMass(1.0);
	frameSideLeft_4->SetPos(ChVector<>(-7.757*inTom, 11.865*inTom, -(36. / 2. + .5)*inTom));
	frameSideLeft_4->SetRot(Q_from_AngZ(-23.91*CH_C_PI / 180.));
	mphysicalSystem.Add(frameSideLeft_4);

	auto frameSideLeftPivot_4 = std::make_shared<ChLinkLockRevolute>();
	frameSideLeftPivot_4->Initialize(frameSideLeft_2, frameSideLeft_4, ChCoordsys<>(ChVector<>(-15.515*inTom, 15.303*inTom, -(36. / 2. + .5)*inTom), Q_from_AngY(0)));
	mphysicalSystem.Add(frameSideLeftPivot_4);

	// Create component 5 (attached to middle wheel) on left side
	auto frameSideLeft_5 = std::make_shared<ChBodyEasyBox>(12.*sqrt(2.)*inTom, 1. * inTom, 1. * inTom,	// x,y,z size
		100,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	frameSideLeft_5->SetMass(1.0);
	frameSideLeft_5->SetPos(ChVector<>(23.272*inTom, 11.865*inTom, -(36. / 2. + .5)*inTom));
	frameSideLeft_5->SetRot(Q_from_AngZ(-23.91*CH_C_PI / 180));
	mphysicalSystem.Add(frameSideLeft_5);

	auto frameSideLeftPivot_5 = std::make_shared<ChLinkLockRevolute>();
	frameSideLeftPivot_5->Initialize(frameSideLeft_1, frameSideLeft_5, ChCoordsys<>(ChVector<>(15.515*inTom, 15.303*inTom, -(36. / 2. + .5)*inTom), Q_from_AngY(0)));
	mphysicalSystem.Add(frameSideLeftPivot_5);

	// Create component 6 (attached to middle wheel) on left side
	auto frameSideLeft_6 = std::make_shared<ChBodyEasyBox>(12.*sqrt(2.)*inTom, 1. * inTom, 1. * inTom,	// x,y,z size
		100,													// density
		false,													// collide enable?
		true													// visualization?		
		);
	frameSideLeft_6->SetMass(1.0);
	frameSideLeft_6->SetPos(ChVector<>(-23.272*inTom, 11.865*inTom, -(36. / 2. + .5)*inTom));
	frameSideLeft_6->SetRot(Q_from_AngZ(23.91*CH_C_PI / 180));
	mphysicalSystem.Add(frameSideLeft_6);

	auto frameSideLeftPivot_6 = std::make_shared<ChLinkLockRevolute>();
	frameSideLeftPivot_6->Initialize(frameSideLeft_2, frameSideLeft_6, ChCoordsys<>(ChVector<>(-15.515*inTom, 15.303*inTom, -(36. / 2. + .5)*inTom), Q_from_AngY(0)));
	mphysicalSystem.Add(frameSideLeftPivot_6);


	// add wheels on the left side
	// add wheelLeft_1
	auto wheelLeft_1 = std::make_shared<ChBodyEasyCylinder>(
		6 * inTom, // radius
		6 * inTom, // height
		100,// density
		true,// collide
		true// visualization		
		);
	wheelLeft_1->SetPos(ChVector<>(31.029*inTom, 8.426*inTom, -(36. / 2. + .5)*inTom));
	wheelLeft_1->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	mphysicalSystem.Add(wheelLeft_1);

	wheelLeft_1->AddAsset(wheelTexture);

	//create a revolute joint for wheel 1 and chassis
	auto wheelLeftJoint_1 = std::make_shared<ChLinkLockRevolute>();
	wheelLeftJoint_1->Initialize(frameSideLeft_5, wheelLeft_1, ChCoordsys<>(wheelLeft_1->GetPos(), Q_from_AngY(0)));
	mphysicalSystem.Add(wheelLeftJoint_1);

	// add wheelLeft_2
	auto wheelLeft_2 = std::make_shared<ChBodyEasyCylinder>(
		6 * inTom, // radius
		6 * inTom, // height
		100,// density
		true,// collide
		true// visualization		
		);
	wheelLeft_2->SetPos(ChVector<>(0, 8.426*inTom, -(36. / 2. + .5)*inTom));
	wheelLeft_2->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	mphysicalSystem.Add(wheelLeft_2);

	wheelLeft_2->AddAsset(wheelTexture);

	//create a revolute joint for wheel 2 and chassis
	auto wheelLeftJoint_2 = std::make_shared<ChLinkLockRevolute>();
	wheelLeftJoint_2->Initialize(frameSideLeft_3, wheelLeft_2, ChCoordsys<>(wheelLeft_2->GetPos(), Q_from_AngY(0)));
	mphysicalSystem.Add(wheelLeftJoint_2);
	auto wheelLeftJoint_2_2 = std::make_shared<ChLinkLockRevolute>();
	wheelLeftJoint_2_2->Initialize(frameSideLeft_4, wheelLeft_2, ChCoordsys<>(wheelLeft_2->GetPos(), Q_from_AngY(0)));
	mphysicalSystem.Add(wheelLeftJoint_2_2);

	// add wheelLeft_3
	auto wheelLeft_3 = std::make_shared<ChBodyEasyCylinder>(
		6 * inTom, // radius
		6 * inTom, // height
		100,// density
		true,// collide
		true// visualization		
		);
	wheelLeft_3->SetPos(ChVector<>(-31.029*inTom, 8.426*inTom, -(36. / 2. + .5)*inTom));
	wheelLeft_3->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	mphysicalSystem.Add(wheelLeft_3);

	wheelLeft_3->AddAsset(wheelTexture);

	//create a revolute joint for wheel 1 and chassis
	auto wheelLeftJoint_3 = std::make_shared<ChLinkLockRevolute>();
	wheelLeftJoint_3->Initialize(frameSideLeft_6, wheelLeft_3, ChCoordsys<>(wheelLeft_3->GetPos(), Q_from_AngY(0)));
	mphysicalSystem.Add(wheelLeftJoint_3);

	
	// ===============================
	double springCoefOutside = 700;
	double springCoefInside = 3000;
	double damping_coef = 80;
	double restLengthOutside = 11 * inTom;	//11.345 is original length
	double restLengthInside = 26 * inTom;	//21.356 is original length
	
	auto col_1 = std::make_shared<ChColorAsset>();
	col_1->SetColor(ChColor(0.6f, 0, 0));
	
	// Create right side springs
	// Create a spring between elements 1 and 5 on the right side
	auto springRight1 = std::make_shared<ChLinkSpring>();
	springRight1->Initialize(frameSideRight_1,	// first body to link it with
		frameSideRight_5,	// second body to link it with
		false,	// pos absolute
		ChVector<>(19.757*inTom, 20.625*inTom, (36. / 2. + .5)*inTom), // position of first end of spring
		ChVector<>(23.272*inTom, 11.865*inTom, (36. / 2. + .5)*inTom), // position of second end of spring
		false,	// rest length not original length
		restLengthOutside);	// rest length
	mphysicalSystem.Add(springRight1);
	springRight1->Set_SpringK(springCoefOutside);
	springRight1->Set_SpringR(damping_coef);
	// Attach a visualization asset.
	springRight1->AddAsset(col_1);
	springRight1->AddAsset(std::make_shared<ChPointPointSpring>(.75*inTom, 20, 5));

	// Create a spring between elements 2 and 6 on the right side
	auto springRight2 = std::make_shared<ChLinkSpring>();
	springRight2->Initialize(frameSideRight_2,	// first body to link it with
		frameSideRight_6,	// second body to link it with
		false,	// pos absolute
		ChVector<>(-19.757*inTom, 20.625*inTom, (36. / 2. + .5)*inTom), // position of first end of spring
		ChVector<>(-23.272*inTom, 11.865*inTom, (36. / 2. + .5)*inTom), // position of second end of spring
		false,	// rest length not original length
		restLengthOutside);	// rest length
	mphysicalSystem.Add(springRight2);
	springRight2->Set_SpringK(springCoefOutside);
	springRight2->Set_SpringR(damping_coef);
	// Attach a visualization asset.
	springRight2->AddAsset(col_1);
	springRight2->AddAsset(std::make_shared<ChPointPointSpring>(.75*inTom, 20, 5));

	// Create a spring between frame and element 3 on the right side
	auto springRight3 = std::make_shared<ChLinkSpring>();
	springRight3->Initialize(frameBox,	// first body to link it with
		frameSideRight_3,	// second body to link it with
		false,	// pos absolute
		ChVector<>(-8.*inTom, 30.*inTom, (36. / 2. + .5)*inTom), // position of first end of spring
		ChVector<>(8.*inTom, 11.972*inTom, (36. / 2. + .5)*inTom), // position of second end of spring
		false,	// rest length not original length
		restLengthInside);	// rest length
	mphysicalSystem.Add(springRight3);
	springRight3->Set_SpringK(springCoefInside);
	springRight3->Set_SpringR(damping_coef);
	// Attach a visualization asset.
	springRight3->AddAsset(col_1);
	springRight3->AddAsset(std::make_shared<ChPointPointSpring>(.75*inTom, 40, 15));
	
	// Create a spring between frame and element 4 on the right side
	auto springRight4 = std::make_shared<ChLinkSpring>();
	springRight4->Initialize(frameBox,	// first body to link it with
		frameSideRight_4,	// second body to link it with
		false,	// pos absolute
		ChVector<>(8.*inTom, 30.*inTom, (36. / 2. + .5)*inTom), // position of first end of spring
		ChVector<>(-8.*inTom, 11.972*inTom, (36. / 2. + .5)*inTom), // position of second end of spring
		false,	// rest length not original length
		restLengthInside);	// rest length
	mphysicalSystem.Add(springRight4);
	springRight4->Set_SpringK(springCoefInside);
	springRight4->Set_SpringR(damping_coef);
	// Attach a visualization asset.
	springRight4->AddAsset(col_1);
	springRight4->AddAsset(std::make_shared<ChPointPointSpring>(.75*inTom, 40, 15));


	// Create left side springs
	// Create a spring between elements 1 and 5 on the left side
	auto springLeft1 = std::make_shared<ChLinkSpring>();
	springLeft1->Initialize(frameSideLeft_1,	// first body to link it with
		frameSideLeft_5,	// second body to link it with
		false,	// pos absolute
		ChVector<>(19.757*inTom, 20.625*inTom, -(36. / 2. + .5)*inTom), // position of first end of spring
		ChVector<>(23.272*inTom, 11.865*inTom, -(36. / 2. + .5)*inTom), // position of second end of spring
		false,	// rest length not original length
		restLengthOutside);	// rest length
	mphysicalSystem.Add(springLeft1);
	springLeft1->Set_SpringK(springCoefOutside);
	springLeft1->Set_SpringR(damping_coef);
	// Attach a visualization asset.
	springLeft1->AddAsset(col_1);
	springLeft1->AddAsset(std::make_shared<ChPointPointSpring>(.75*inTom, 20, 5));

	// Create a spring between elements 2 and 6 on the left side
	auto springLeft2 = std::make_shared<ChLinkSpring>();
	springLeft2->Initialize(frameSideLeft_2,	// first body to link it with
		frameSideLeft_6,	// second body to link it with
		false,	// pos absolute
		ChVector<>(-19.757*inTom, 20.625*inTom, -(36. / 2. + .5)*inTom), // position of first end of spring
		ChVector<>(-23.272*inTom, 11.865*inTom, -(36. / 2. + .5)*inTom), // position of second end of spring
		false,	// rest length not original length
		restLengthOutside);	// rest length
	mphysicalSystem.Add(springLeft2);
	springLeft2->Set_SpringK(springCoefOutside);
	springLeft2->Set_SpringR(damping_coef);
	// Attach a visualization asset.
	springLeft2->AddAsset(col_1);
	springLeft2->AddAsset(std::make_shared<ChPointPointSpring>(.75*inTom, 20, 5));

	// Create a spring between frame and element 3 on the left side
	auto springLeft3 = std::make_shared<ChLinkSpring>();
	springLeft3->Initialize(frameBox,	// first body to link it with
		frameSideLeft_3,	// second body to link it with
		false,	// pos absolute
		ChVector<>(-8.*inTom, 30.*inTom, -(36. / 2. + .5)*inTom), // position of first end of spring
		ChVector<>(8.*inTom, 11.972*inTom, -(36. / 2. + .5)*inTom), // position of second end of spring
		false,	// rest length not original length
		restLengthInside);	// rest length
	mphysicalSystem.Add(springLeft3);
	springLeft3->Set_SpringK(springCoefInside);
	springLeft3->Set_SpringR(damping_coef);
	// Attach a visualization asset.
	springLeft3->AddAsset(col_1);
	springLeft3->AddAsset(std::make_shared<ChPointPointSpring>(.75*inTom, 40, 15));

	// Create a spring between frame and element 4 on the left side
	auto springLeft4 = std::make_shared<ChLinkSpring>();
	springLeft4->Initialize(frameBox,	// first body to link it with
		frameSideLeft_4,	// second body to link it with
		false,	// pos absolute
		ChVector<>(8.*inTom, 30.*inTom, -(36. / 2. + .5)*inTom), // position of first end of spring
		ChVector<>(-8.*inTom, 11.972*inTom, -(36. / 2. + .5)*inTom), // position of second end of spring
		false,	// rest length not original length
		restLengthInside);	// rest length
	mphysicalSystem.Add(springLeft4);
	springLeft4->Set_SpringK(springCoefInside);
	springLeft4->Set_SpringR(damping_coef);
	// Attach a visualization asset.
	springLeft4->AddAsset(col_1);
	springLeft4->AddAsset(std::make_shared<ChPointPointSpring>(.75*inTom, 40, 15));


	// ===============================
	// Add motors to wheels
	double torqueRightSide = 1.;
	double torqueLeftSide = 1.;
	wheelRightJoint_1->Set_Scr_torque(torqueRightSide);
	wheelRightJoint_2->Set_Scr_torque(torqueRightSide);
	wheelRightJoint_2_2->Set_Scr_torque(torqueRightSide);
	wheelRightJoint_3->Set_Scr_torque(torqueRightSide);
	wheelLeftJoint_1->Set_Scr_torque(torqueLeftSide);
	wheelLeftJoint_2->Set_Scr_torque(torqueLeftSide);	
	wheelLeftJoint_2_2->Set_Scr_torque(torqueLeftSide);
	wheelLeftJoint_3->Set_Scr_torque(torqueLeftSide);

	// ===============================
		/* combined into entire side?
	auto frameSideRight_1 = std::make_shared<ChBody>();
	mphysicalSystem.AddBody(frameSideRight_1);
	frameSideRight_1->SetIdentifier(1);
	frameSideRight_1->SetBodyFixed(false);
	frameSideRight_1->SetCollide(false);
	frameSideRight_1->SetMass(1);
	//frameSideRight_1->SetInertiaXX(ChVector<>(0.2, 1, 1));
	auto frameSideRightComp_1 = std::make_shared<ChBodyEasyBox>();
	*/
	
	// ===============================
	// Add obstacles
	auto obstacleBox1 = std::make_shared<ChBodyEasyBox>(8.*inTom, 4.*inTom, 48.*inTom, 100, true, true);
	obstacleBox1->SetMass(10.0);
	obstacleBox1->SetPos(ChVector<>(60.*inTom, 2.*inTom, 0));
	mphysicalSystem.Add(obstacleBox1);
	obstacleBox1->SetBodyFixed(true);
	//auto obstacleColor = std::make_shared<ChColorAsset>();
	//obstacleColor->SetColor(ChColor(0.2f, 0.1f, 0.1f));
	auto obstacleTexture = std::make_shared<ChTexture>();
	obstacleTexture->SetTextureFilename(GetChronoDataFile("cubetexture_wood.png"));  // texture in ../data
	obstacleBox1->AddAsset(obstacleTexture);



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
	mphysicalSystem.SetMaxItersSolverSpeed(1000);

    //
    // THE SOFT-REAL-TIME CYCLE
    //
	int i = 0;
    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        // This performs the integration timestep!
        //application.DoStep();
		mphysicalSystem.DoStepDynamics(step_size);
		std::cout << "Step number: " << i << std::endl;

		i++;
        application.EndScene();
    }

    return 0;
}
