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
    ChIrrApp application(&mphysicalSystem, L"A simple project template", core::dimension2d<u32>(1920, 1080),
                         false);  // screen dimensions

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(10, 6, -15),
                                 core::vector3df(0, 1, 0));  // to change the position of camera
    // application.AddLightWithShadow(vector3df(1,25,-5), vector3df(0,0,0), 35, 0.2,35, 55, 512, video::SColorf(1,1,1));

    //======================================================================

    // HERE YOU CAN POPULATE THE PHYSICAL SYSTEM WITH BODIES AND LINKS.
    //
    // An example: a pendulum.

    // 1-Create a floor that is fixed (that is used also to represent the absolute reference)
	
	// Create a material that will be used for the floor
	auto mmaterial = std::make_shared<ChMaterialSurfaceNSC>();
	//mmaterial->SetFriction(0.4f);
	//mmaterial->SetCompliance(0.0000005f);
	//mmaterial->SetComplianceT(0.0000005f);
	//mmaterial->SetDampingF(0.2f);

    auto floorBody = std::make_shared<ChBodyEasyBox>(30, 2, 30,  // x, y, z dimensions
                                                     1000,       // density
                                                     true,      // contact geometry - allow collision
                                                     true        // enable visualization geometry
                                                     );
    floorBody->SetPos(ChVector<>(0, -2, 0));
	floorBody->SetMaterialSurface(mmaterial);
    floorBody->SetBodyFixed(true);

    mphysicalSystem.Add(floorBody);
	
	
	/* comment out pendulum for possible future use
    // 2-Create a pendulum

    auto pendulumBody = std::make_shared<ChBodyEasyBox>(0.5, 2, 0.5,  // x, y, z dimensions
                                                        3000,         // density
                                                        false,        // no contact geometry
                                                        true          // enable visualization geometry
                                                        );
    pendulumBody->SetPos(ChVector<>(0, 3, 0));
    pendulumBody->SetPos_dt(ChVector<>(1, 0, 0));

    mphysicalSystem.Add(pendulumBody);

    // 3-Create a spherical constraint.
    //   Here we'll use a ChLinkMateGeneric, but we could also use ChLinkLockSpherical

    auto sphericalLink =
        std::make_shared<ChLinkMateGeneric>(true, true, true, false, false, false);  // x,y,z,Rx,Ry,Rz constrains
    ChFrame<> link_position_abs(ChVector<>(0, 4, 0));

    sphericalLink->Initialize(pendulumBody,        // the 1st body to connect
                              floorBody,           // the 2nd body to connect
                              false,               // the two following frames are in absolute, not relative, coords.
                              link_position_abs,   // the link reference attached to 1st body
                              link_position_abs);  // the link reference attached to 2nd body

    mphysicalSystem.Add(sphericalLink);

    // Optionally, attach a RGB color asset to the floor, for better visualization
    auto color = std::make_shared<ChColorAsset>();
    color->SetColor(ChColor(0.2f, 0.25f, 0.25f));
    floorBody->AddAsset(color);

    // Optionally, attach a texture to the pendulum, for better visualization
    auto texture = std::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("rock.jpg"));  // texture in ../data
    pendulumBody->AddAsset(texture);
	*/

	/*attempt to make wheels (template from demo_IRR_aux_ref)
	// Physical Rover body
	auto rover = std::make_shared<ChBody>();
	mphysicalSystem.AddBody(rover);
	rover->SetIdentifier(1);
	rover->SetBodyFixed(false);
	rover->SetCollide(false);
	rover->SetMass(1);

	// Add Frame
	auto frameBox = std::make_shared<ChBodyEasyBox>(10, 2, 15,	// x,y,z size
		100,													// density
		true,													// collide enable?
		true													// visualization?		
		);

	rover->AddAsset(frameBox);

	// Attach a visualization asset. Note that the cylinder is defined with
	// respect to the centroidal reference frame (which is the body reference
	// frame for a ChBody).
	auto wheel_1 = std::make_shared<ChCylinderShape>();
	wheel_1->GetCylinderGeometry().p1 = ChVector<>(-2, 0, 0);
	wheel_1->GetCylinderGeometry().p2 = ChVector<>(5, 0, 0);
	wheel_1->GetCylinderGeometry().rad = 3;
	rover->AddAsset(wheel_1);
	//auto col_1 = std::make_shared<ChColorAsset>();
	//col_1->SetColor(ChColor(0.6f, 0, 0));
	//rover->AddAsset(col_1);
	
	rover->SetPos(ChVector<>(0, 3, 0));
	mphysicalSystem.Add(rover);
	*/

	// Add Frame
	auto frameBox = std::make_shared<ChBodyEasyBox>(10, 2, 15,	// x,y,z size
		100,													// density
		true,													// collide enable?
		true													// visualization?		
		);													
	
	frameBox->SetPos(ChVector<>(0, 3, 0));
	
	mphysicalSystem.Add(frameBox);

	// Add wheels
	/* attempt to use cylinders
	auto wheel_1 = std::make_shared<ChBodyEasyCylinder>(
		3, // radius
		2, // height
		100,// density
		true,// collide
		true// visualization		
		);
	*/
	auto wheel_1 = std::make_shared<ChBodyEasyBox>(2, 6, 6,	// x,y,z size
		100,													// density
		true,													// collide enable?
		true													// visualization?		
		);
	wheel_1->SetPos(ChVector<>(6, 3, 6));
	mphysicalSystem.Add(wheel_1);
	
	auto wheel_2 = std::make_shared<ChBodyEasyBox>(2, 6, 6,	// x,y,z size
		100,													// density
		true,													// collide enable?
		true													// visualization?		
		);
	wheel_2->SetPos(ChVector<>(-6, 3, 6));
	mphysicalSystem.Add(wheel_2);

	auto wheel_3 = std::make_shared<ChBodyEasyBox>(2, 6, 6,	// x,y,z size
		100,													// density
		true,													// collide enable?
		true													// visualization?		
		);
	wheel_3->SetPos(ChVector<>(6, 3, -6));
	mphysicalSystem.Add(wheel_3);

	auto wheel_4 = std::make_shared<ChBodyEasyBox>(2, 6, 6,	// x,y,z size
		100,													// density
		true,													// collide enable?
		true													// visualization?		
		);
	wheel_4->SetPos(ChVector<>(-6, 3, -6));

	mphysicalSystem.Add(wheel_4);


    //======================================================================

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    // Adjust some settings:
    application.SetTimestep(0.005);
    application.SetTryRealtime(false);

    //
    // THE SOFT-REAL-TIME CYCLE
    //
	int i = 0;
    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        // This performs the integration timestep!
        application.DoStep();
		std::cout << "Step number" << i << std::endl;

		i++;
        application.EndScene();
    }

    return 0;
}
