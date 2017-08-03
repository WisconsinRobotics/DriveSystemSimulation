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
    floorBody->SetPos(ChVector<>(0, -1, 0));
	//floorBody->SetMaterialSurface(mmaterial);
    floorBody->SetBodyFixed(true);

    mphysicalSystem.Add(floorBody);
	// Optionally, attach a RGB color asset to the floor, for better visualization
	auto color = std::make_shared<ChColorAsset>();
	color->SetColor(ChColor(0.2f, 0.25f, 0.25f));
	floorBody->AddAsset(color);

	// Add Frame
	auto frameBox = std::make_shared<ChBodyEasyBox>(.5, .08, .4,	// x,y,z size
		100,													// density
		true,													// collide enable?
		true													// visualization?		
		);													
	frameBox->SetMass(10.0);
	frameBox->SetPos(ChVector<>(0, 3, 0));
	
	mphysicalSystem.Add(frameBox);
	

	// Add wheels
	// attempt to use cylinders
	auto wheel_1 = std::make_shared<ChBodyEasyCylinder>(
		.1, // radius
		.08, // height
		100,// density
		true,// collide
		true// visualization		
		);
	
	wheel_1->SetPos(ChVector<>(.25, 3, .25));
	wheel_1->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	mphysicalSystem.Add(wheel_1);

	auto texture = std::make_shared<ChTexture>();
	texture->SetTextureFilename(GetChronoDataFile("redwhite.png"));  // texture in ../data
	wheel_1->AddAsset(texture);

	//create a revolute joint for wheel 1 and chassis
	auto wheel1joint = std::make_shared<ChLinkLockRevolute>();
	wheel1joint->Initialize(frameBox, wheel_1, ChCoordsys<>(ChVector<>(.25,3,.25),Q_from_AngY(0)));
	mphysicalSystem.Add(wheel1joint);


	// attempt to use cylinders
	auto wheel_2 = std::make_shared<ChBodyEasyCylinder>(
		.1, // radius
		.08, // height
		100,// density
		true,// collide
		true// visualization		
		);

	wheel_2->SetPos(ChVector<>(.25, 3, -.25));
	wheel_2->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	mphysicalSystem.Add(wheel_2);

	wheel_2->AddAsset(texture);

	//create a revolute joint for wheel 1 and chassis
	auto wheel2joint = std::make_shared<ChLinkLockRevolute>();
	wheel2joint->Initialize(frameBox, wheel_2, ChCoordsys<>(wheel_2->GetPos(), Q_from_AngY(0)));
	mphysicalSystem.Add(wheel2joint);


	// attempt to use cylinders
	auto wheel_3 = std::make_shared<ChBodyEasyCylinder>(
		.1, // radius
		.08, // height
		100,// density
		true,// collide
		true// visualization		
		);

	wheel_3->SetPos(ChVector<>(-.25, 3, .25));
	wheel_3->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	mphysicalSystem.Add(wheel_3);

	wheel_3->AddAsset(texture);

	//create a revolute joint for wheel 1 and chassis
	auto wheel3joint = std::make_shared<ChLinkLockRevolute>();
	wheel3joint->Initialize(frameBox, wheel_3, ChCoordsys<>(wheel_3->GetPos(), Q_from_AngY(0)));
	mphysicalSystem.Add(wheel3joint);


	// attempt to use cylinders
	auto wheel_4 = std::make_shared<ChBodyEasyCylinder>(
		.1, // radius
		.08, // height
		100,// density
		true,// collide
		true// visualization		
		);

	wheel_4->SetPos(ChVector<>(-.25, 3, -.25));
	wheel_4->SetRot(Q_from_AngX(CH_C_PI / 2.0));
	mphysicalSystem.Add(wheel_4);

	wheel_4->AddAsset(texture);

	//create a revolute joint for wheel 1 and chassis
	auto wheel4joint = std::make_shared<ChLinkLockRevolute>();
	wheel4joint->Initialize(frameBox, wheel_4, ChCoordsys<>(wheel_4->GetPos(), Q_from_AngY(0)));
	mphysicalSystem.Add(wheel4joint);

	
	


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
