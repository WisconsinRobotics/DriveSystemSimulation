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


double ftTom = 1 / 3.3;

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

double in2m = .0254;

//wheel parameters
std::vector<ChVector<>> wheelPos = { {24*in2m, 0, -16*in2m},{-24*in2m, 0, -16*in2m} ,{24*in2m, 0, 16*in2m} ,{-24*in2m, 0, 16*in2m} };
double wheelMass = 2;
double wheelWidth = 5 * in2m;
double wheelRadius = 4 * in2m;

//leg parameters
double legMass = 1;
double legVis = 1 * in2m;

//body parameters
std::vector<double> bodyDims = { 24 * in2m, 6 * in2m, 16 * in2m };
ChVector<> bodyPos = { 0,12 * in2m,0 };
double bodyMass = 10;

//spring parameters
double k = 5000;
double c = 500;
ChVector<> springStartLeft = {wheelPos[0].x()/2.0,bodyPos.y()/2.0,wheelPos[0].z()};
ChVector<> springEndLeft = { wheelPos[1].x() / 2.0,bodyPos.y() / 2.0,wheelPos[1].z() };
ChVector<> springStartRight = { wheelPos[2].x() / 2.0,bodyPos.y() / 2.0,wheelPos[2].z() };
ChVector<> springEndRight = { wheelPos[3].x() / 2.0,bodyPos.y() / 2.0,wheelPos[3].z() };
double restLength = 21.5 * in2m;

//torques
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
    application.AddTypicalCamera(core::vector3df(4, 2, -5),
                                 core::vector3df(0, 1, 0));  // to change the position of camera
    // application.AddLightWithShadow(vector3df(1,25,-5), vector3df(0,0,0), 35, 0.2,35, 55, 512, video::SColorf(1,1,1));

    //======================================================================

    auto floorBody = std::make_shared<ChBodyEasyBox>(100, 2, 100,  // x, y, z dimensions
                                                     1000,       // density
                                                     true,      // contact geometry - allow collision
                                                     true        // enable visualization geometry
                                                     );
    floorBody->SetPos(ChVector<>(0, -1.5, 0));
    floorBody->SetBodyFixed(true);
    mphysicalSystem.Add(floorBody);
	// Optionally, attach a RGB color asset to the floor, for better visualization
	auto color = std::make_shared<ChColorAsset>();
	color->SetColor(ChColor(0.2f, 0.25f, 0.25f));
	floorBody->AddAsset(color);


	//------------------------START 4 WHEELED ROVER-------------------------------------//


	// Add Frame
	auto frameBox = std::make_shared<ChBodyEasyBox>(bodyDims[0],bodyDims[1],bodyDims[2],	// x,y,z size
		1000,													// density
		true,													// collide enable?
		true													// visualization?		
		);													
	frameBox->SetMass(bodyMass);
	frameBox->SetPos(bodyPos);
	mphysicalSystem.Add(frameBox);
	

	//add legs
	double legLength = sqrt(wheelPos[0].x()*wheelPos[0].x() + wheelPos[0].y()*wheelPos[0].y());

	auto legColor = std::make_shared<ChColorAsset>();
	legColor->SetColor(ChColor(0.2f, 0.25f, 0.25f));
	floorBody->AddAsset(legColor);

	auto leg0 = std::make_shared<ChBodyEasyBox>(legLength, legVis, legVis, 1000, false, true);
	leg0->SetMass(legMass);
	leg0->SetPos(ChVector<>((wheelPos[0].x()+bodyPos.x()) / 2.0, (wheelPos[0].y() + bodyPos.y()) / 2.0,wheelPos[0].z()));
	leg0->SetRot(Q_from_AngZ(-atan2(wheelPos[0].y() + bodyPos.y(), wheelPos[0].x() + bodyPos.x())));
	mphysicalSystem.Add(leg0);
	leg0->AddAsset(legColor);

	auto leg0Joint = std::make_shared<ChLinkLockRevolute>();
	leg0Joint->Initialize(frameBox, leg0, ChCoordsys<>(ChVector<>(bodyPos.x(),bodyPos.y(),wheelPos[0].z()), Q_from_AngY(0)));
	mphysicalSystem.Add(leg0Joint);

	auto leg1 = std::make_shared<ChBodyEasyBox>(legLength, legVis, legVis, 1000, false, true);
	leg1->SetMass(legMass);
	leg1->SetPos(ChVector<>((wheelPos[1].x() + bodyPos.x()) / 2.0, (wheelPos[1].y() + bodyPos.y()) / 2.0, wheelPos[1].z()));
	leg1->SetRot(Q_from_AngZ(-atan2(wheelPos[1].y() + bodyPos.y(), wheelPos[1].x() + bodyPos.x())));
	mphysicalSystem.Add(leg1);
	leg1->AddAsset(legColor);

	auto leg1Joint = std::make_shared<ChLinkLockRevolute>();
	leg1Joint->Initialize(frameBox, leg1, ChCoordsys<>(ChVector<>(bodyPos.x(), bodyPos.y(), wheelPos[1].z()), Q_from_AngY(0)));
	mphysicalSystem.Add(leg1Joint);

	auto leg2 = std::make_shared<ChBodyEasyBox>(legLength, legVis, legVis, 1000, false, true);
	leg2->SetMass(legMass);
	leg2->SetPos(ChVector<>((wheelPos[2].x() + bodyPos.x()) / 2.0, (wheelPos[2].y() + bodyPos.y()) / 2.0, wheelPos[2].z()));
	leg2->SetRot(Q_from_AngZ(-atan2(wheelPos[2].y() + bodyPos.y(), wheelPos[2].x() + bodyPos.x())));
	mphysicalSystem.Add(leg2);
	leg0->AddAsset(legColor);

	auto leg2Joint = std::make_shared<ChLinkLockRevolute>();
	leg2Joint->Initialize(frameBox, leg2, ChCoordsys<>(ChVector<>(bodyPos.x(), bodyPos.y(), wheelPos[2].z()), Q_from_AngY(0)));
	mphysicalSystem.Add(leg2Joint);

	auto leg3 = std::make_shared<ChBodyEasyBox>(legLength, legVis, legVis, 1000, false, true);
	leg3->SetMass(legMass);
	leg3->SetPos(ChVector<>((wheelPos[3].x() + bodyPos.x()) / 2.0, (wheelPos[3].y() + bodyPos.y()) / 2.0, wheelPos[3].z()));
	leg3->SetRot(Q_from_AngZ(-atan2(wheelPos[3].y()+bodyPos.y(), wheelPos[3].x()+bodyPos.x())));
	mphysicalSystem.Add(leg3);
	leg0->AddAsset(legColor);

	auto leg3Joint = std::make_shared<ChLinkLockRevolute>();
	leg3Joint->Initialize(frameBox, leg3, ChCoordsys<>(ChVector<>(bodyPos.x(), bodyPos.y(), wheelPos[3].z()), Q_from_AngY(0)));
	mphysicalSystem.Add(leg3Joint);







	//add wheels
	auto wheel_texture = std::make_shared<ChTexture>();
	wheel_texture->SetTextureFilename(GetChronoDataFile("redwhite.png"));  // texture in ../data
	

	auto wheel_0 = std::make_shared<ChBodyEasyCylinder>(wheelRadius, wheelWidth, 1000, true, true);
	wheel_0->SetPos(wheelPos[0]);
	wheel_0->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	wheel_0->SetMass(wheelMass);
	mphysicalSystem.Add(wheel_0);
	wheel_0->AddAsset(wheel_texture);

	//create a revolute joint for wheel 1 and chassis
	auto wheel0joint = std::make_shared<ChLinkLockRevolute>();
	wheel0joint->Initialize(leg0, wheel_0, ChCoordsys<>(wheelPos[0],Q_from_AngY(0)));
	mphysicalSystem.Add(wheel0joint);


	auto wheel_1 = std::make_shared<ChBodyEasyCylinder>(wheelRadius, wheelWidth, 1000, true, true);
	wheel_1->SetPos(wheelPos[1]);
	wheel_1->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	wheel_1->SetMass(wheelMass);
	mphysicalSystem.Add(wheel_1);
	wheel_1->AddAsset(wheel_texture);

	//create a revolute joint for wheel 1 and chassis
	auto wheel1joint = std::make_shared<ChLinkLockRevolute>();
	wheel1joint->Initialize(leg1, wheel_1, ChCoordsys<>(wheelPos[1], Q_from_AngY(0)));
	mphysicalSystem.Add(wheel1joint);

	auto wheel_2 = std::make_shared<ChBodyEasyCylinder>(wheelRadius, wheelWidth, 1000, true, true);
	wheel_2->SetPos(wheelPos[2]);
	wheel_2->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	wheel_2->SetMass(wheelMass);
	mphysicalSystem.Add(wheel_2);
	wheel_2->AddAsset(wheel_texture);

	//create a revolute joint for wheel 1 and chassis
	auto wheel2joint = std::make_shared<ChLinkLockRevolute>();
	wheel2joint->Initialize(leg2, wheel_2, ChCoordsys<>(wheelPos[2], Q_from_AngY(0)));
	mphysicalSystem.Add(wheel2joint);

	auto wheel_3 = std::make_shared<ChBodyEasyCylinder>(wheelRadius, wheelWidth, 1000, true, true);
	wheel_3->SetPos(wheelPos[3]);
	wheel_3->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	wheel_3->SetMass(wheelMass);
	mphysicalSystem.Add(wheel_3);
	wheel_3->AddAsset(wheel_texture);

	//create a revolute joint for wheel 1 and chassis
	auto wheel3joint = std::make_shared<ChLinkLockRevolute>();
	wheel3joint->Initialize(leg3, wheel_3, ChCoordsys<>(wheelPos[3], Q_from_AngY(0)));
	mphysicalSystem.Add(wheel3joint);



	//add springs
	auto springtLeft = std::make_shared<ChLinkSpring>();
	springtLeft->Initialize(leg0,	// first body to link it with
		leg1,	// second body to link it with
		false,	// pos absolute
		springStartLeft, // position of first end of spring
		springEndLeft, // position of second end of spring
		false,	// rest length not original length
		restLength);	// rest length
	mphysicalSystem.Add(springtLeft);
	springtLeft->Set_SpringK(k);
	springtLeft->Set_SpringR(c);
	// Attach a visualization asset.
	springtLeft->AddAsset(color);
	springtLeft->AddAsset(std::make_shared<ChPointPointSpring>(.0125, 20, 10));

	auto springtRight = std::make_shared<ChLinkSpring>();
	springtRight->Initialize(leg2,	// first body to link it with
		leg3,	// second body to link it with
		false,	// pos absolute
		springStartRight, // position of first end of spring
		springEndRight, // position of second end of spring
		false,	// rest length not original length
		restLength);	// rest length
	mphysicalSystem.Add(springtRight);
	springtRight->Set_SpringK(k);
	springtRight->Set_SpringR(c);
	// Attach a visualization asset.
	springtRight->AddAsset(color);
	springtRight->AddAsset(std::make_shared<ChPointPointSpring>(.0125, 20, 10));


	//Add differential bar (assumed to be on top of chassis frame












	//Add obstacles
	auto obstacleTexture = std::make_shared<ChTexture>();
	obstacleTexture->SetTextureFilename(GetChronoDataFile("cubetexture_wood.png"));  // texture in ../data

	auto obstacleBox1 = std::make_shared<ChBodyEasyBox>(.2, .1, 1.22, 1000, true, true);
	obstacleBox1->SetPos(ChVector<>(2.0, -.5, 0));
	mphysicalSystem.Add(obstacleBox1);
	obstacleBox1->SetBodyFixed(true);
	obstacleBox1->AddAsset(obstacleTexture);

	auto obsCyl = std::make_shared<ChBodyEasyCylinder>(12 * in2m, 3, 1000, true, true);
	obsCyl->SetPos(ChVector<>(3.0, -.5, 0));
	obsCyl->SetBodyFixed(true);
	obsCyl->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	obsCyl->AddAsset(obstacleTexture);
	mphysicalSystem.Add(obsCyl);


	


	//set torque on the wheels
	wheel0joint->Set_Scr_torque(torqueLeftSide);
	wheel1joint->Set_Scr_torque(torqueLeftSide);
	wheel2joint->Set_Scr_torque(torqueRightSide);
	wheel3joint->Set_Scr_torque(torqueRightSide);
	


	


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
	int i = 0;
    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        // This performs the integration timestep!
        //application.DoStep();
		mphysicalSystem.DoStepDynamics(step_size);

        application.EndScene();
    }

    return 0;
}
