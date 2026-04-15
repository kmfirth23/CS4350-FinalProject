#include "GLViewAssignment_FinalProject.h"

#include "WorldList.h" //This is where we place all of our WOs
#include "ManagerOpenGLState.h" //We can change OpenGL State attributes with this
#include "Axes.h" //We can set Axes to on/off with this
#include "PhysicsEngineODE.h"

//Different WO used by this module
#include "WO.h"
#include "WOStatic.h"
#include "WOStaticPlane.h"
#include "WOStaticTrimesh.h"
#include "WOTrimesh.h"
#include "WOHumanCyborg.h"
#include "WOHumanCal3DPaladin.h"
#include "WOWayPointSpherical.h"
#include "WOLight.h"
#include "WOSkyBox.h"
#include "WOCar1970sBeater.h"
#include "Camera.h"
#include "CameraStandard.h"
#include "CameraChaseActorSmooth.h"
#include "CameraChaseActorAbsNormal.h"
#include "CameraChaseActorRelNormal.h"
#include "Model.h"
#include "ModelDataShared.h"
#include "ModelMesh.h"
#include "ModelMeshDataShared.h"
#include "ModelMeshSkin.h"
#include "WONVStaticPlane.h"
#include "WONVPhysX.h"
#include "WONVDynSphere.h"
#include "WOImGui.h" //GUI Demos also need to #include "AftrImGuiIncludes.h"
#include "AftrImGuiIncludes.h"
#include "AftrGLRendererBase.h"
#include "MGLIndexedGeometry.h"
#include "IndexedGeometrySphereTriStrip.h"
#include "WOAxesTubes.h"
#include "AftrTimer.h"

#include "NetMessengerClient.h"
#include "NetMsgCreateWO.h"

#include <chrono>

using namespace Aftr;


NetMsgMacroDefinition(NetMsgControlObjects);

NetMsgControlObjects::NetMsgControlObjects()
{
    size = 0;


    for (int i = 0; i < 16; i++)
        m[i] = 0.0;

    ID = -1;
}
NetMsgControlObjects::~NetMsgControlObjects() {}
bool NetMsgControlObjects::toStream(NetMessengerStreamBuffer& os) const {

    os << this->size;
    os << ID;
    for (int i = 0; i < 16; i++)
        os << m[i];
    return true;
}
bool NetMsgControlObjects::fromStream(NetMessengerStreamBuffer& is)
{
    is >> this->size;
    is >> ID;
    for (int i = 0; i < 16; i++)
        is >> m[i];
    return true;
     
}
void NetMsgControlObjects::onMessageArrived()
{
    //update cube position and rotation
    GLViewAssignment_FinalProject* glv = ManagerGLView::getGLViewT<GLViewAssignment_FinalProject>();
    glv->updateObj(ID, m);


}

std::string NetMsgControlObjects::toString()
{
    std::stringstream ss;
    ss << NetMsg::toString();
    ss << "  Payload: " << this->size << "...\n";
    return ss.str();
}


GLViewAssignment_FinalProject* GLViewAssignment_FinalProject::New( const std::vector< std::string >& args )
{
   GLViewAssignment_FinalProject* glv = new GLViewAssignment_FinalProject( args );
   if (playerNum == 0)
   {
       glv->init(Aftr::GRAVITY, Vector(0, 0, -1.0f), "aftr.conf", PHYSICS_ENGINE_TYPE::petODE);
   }
   else
   {
       glv->init(Aftr::GRAVITY, Vector(0, 0, -1.0f), "aftr.conf", PHYSICS_ENGINE_TYPE::petODE);
   }
   //glv->init( Aftr::GRAVITY, Vector( 0, 0, -1.0f ), "player1.conf", PHYSICS_ENGINE_TYPE::petODE );
   glv->onCreate();
   return glv;
}


GLViewAssignment_FinalProject::GLViewAssignment_FinalProject( const std::vector< std::string >& args ) : GLView( args )
{
   //Initialize any member variables that need to be used inside of LoadMap() here.
   //Note: At this point, the Managers are not yet initialized. The Engine initialization
   //occurs immediately after this method returns (see GLViewAssignment_FinalProject::New() for
   //reference). Then the engine invoke's GLView::loadMap() for this module.
   //After loadMap() returns, GLView::onCreate is finally invoked.

   //The order of execution of a module startup:
   //GLView::New() is invoked:
   //    calls GLView::init()
   //       calls GLView::loadMap() (as well as initializing the engine's Managers)
   //    calls GLView::onCreate()

   //GLViewAssignment_FinalProject::onCreate() is invoked after this module's LoadMap() is completed.
}


void GLViewAssignment_FinalProject::onCreate()
{
   //GLViewAssignment_FinalProject::onCreate() is invoked after this module's LoadMap() is completed.
   //At this point, all the managers are initialized. That is, the engine is fully initialized.

   if( this->pe != NULL )
   {
      //optionally, change gravity direction and magnitude here
      //The user could load these values from the module's aftr.conf
      this->pe->setGravityNormalizedVector( Vector( 0,0,-1.0f ) );
      this->pe->setGravityScalar( Aftr::GRAVITY );
   }
   this->setActorChaseType( STANDARDEZNAV ); //Default is STANDARDEZNAV mode
   //this->setNumPhysicsStepsPerRender( 0 ); //pause physics engine on start up; will remain paused till set to 1

   // find all available controllers
   if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) < 0) {
       std::cerr << "SDL could not initialize! SDL Error: " << SDL_GetError() << std::endl;
   }
   for (int i = 0; i < SDL_NumJoysticks(); i++)
   {
       if (SDL_IsGameController(i))
       {
           SDL_GameController* tempControl = SDL_GameControllerOpen(i);
           if (tempControl)
           {
               //if a controller is connected push it to the controller list
               std::cout << "Controller connected" << std::endl;
               controllerList.push_back(tempControl);
           }
       }
   }

   //based on which player it is set the controller
   if (playerNum < controllerList.size())
   {
       control = controllerList[playerNum];
   }

   if (playerNum == 0)
   {
       client = NetMessengerClient::New("127.0.0.1", "12684");
   }
   else
   {
       client = NetMessengerClient::New("127.0.0.1", "12683");
   }


}


GLViewAssignment_FinalProject::~GLViewAssignment_FinalProject()
{
   //Implicitly calls GLView::~GLView()
}


void GLViewAssignment_FinalProject::updateWorld()
{
   GLView::updateWorld(); //Just call the parent's update world first.
                          //If you want to add additional functionality, do it after
                          //this call.

   //Customize the world's behavior. Typically one would call a function to keep this short
   //but for the sake of a minimial example, we'll do something mildly interesting here

   if( this->gulfstream != nullptr && this->moon != nullptr )
      this->moon->setPose( 
         this->orbit_gui.compute_pose( this->gulfstream->getModel()->getPose() ) );

   controllerMove(); //updates location from controller inputs
   controllerPerspective(); //updates camera angle from controller inputs

   NetMsgControlObjects msag;
    //set ID
    msag.ID = this->cam->getID();

    Mat4 m2 = this->cam->getPose();

    for (int i = 0; i < 16; i++)
    {
        msag.m[i] = m2[i];
    }

    msag.size = sizeof(msag);

    //send the infromation
    if (client)
        client->sendNetMsgSynchronousTCP(msag);

}


void GLViewAssignment_FinalProject::onResizeWindow( GLsizei width, GLsizei height )
{
   GLView::onResizeWindow( width, height ); //call parent's resize method.
}


void GLViewAssignment_FinalProject::onMouseDown( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseDown( e );
}


void GLViewAssignment_FinalProject::onMouseUp( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseUp( e );
}


void GLViewAssignment_FinalProject::onMouseMove( const SDL_MouseMotionEvent& e )
{
   GLView::onMouseMove( e );
}


void GLViewAssignment_FinalProject::onKeyDown( const SDL_KeyboardEvent& key )
{
   GLView::onKeyDown( key );
   if( key.keysym.sym == SDLK_0 )
      this->setNumPhysicsStepsPerRender( 1 );

   if( key.keysym.sym == SDLK_1 )
   {

   }
}


void GLViewAssignment_FinalProject::onKeyUp( const SDL_KeyboardEvent& key )
{
   GLView::onKeyUp( key );
}


void GLViewAssignment_FinalProject::updateObj(int id, float m[16]) // , float xl, float yl, float zl)
{
    //transverse the worldLst to find the corresponding object ID
    /*for (int i = 0; i < worldLst->size(); i++)
    {
        if (id == worldLst->at(i)->getID())
        {
            Mat4 m2;
            for (int i = 0; i < 16; i++)
            {
                m2[i] = m[i];
            }


            worldLst->at(i)->setPose(m2);
        }
    }*/

    Mat4 m2;
    for (int i = 0; i < 16; i++)
    {
        m2[i] = m[i];
    }
    playerModel->setPose(m2);
}

void GLViewAssignment_FinalProject::controllerMove()
{
    //ensure there is a controller
    if (!control)
    {
        return;
    }

    //get values from left joystick
    int xleft = SDL_GameControllerGetAxis(control, SDL_CONTROLLER_AXIS_LEFTX);
    int yleft = SDL_GameControllerGetAxis(control, SDL_CONTROLLER_AXIS_LEFTY);

    //floats to hold either positive or negative directions from controller movement
    float xDir = 0.0f;
    float yDir = 0.0f;

    //compare to dead zone to ensure enough movement occurred (prevent drifting)
    //identify the direction it moved in
    if (xleft < -JOYSTICK_DEAD_ZONE)
    {
        xDir = -1.0;
    }
    else if (xleft > JOYSTICK_DEAD_ZONE)
    {
        xDir = 1.0;
    }

    if (yleft < -JOYSTICK_DEAD_ZONE)
    {
        yDir = -1.0;
    }
    else if (yleft > JOYSTICK_DEAD_ZONE)
    {
        yDir = 1.0;
    }


    // if value is zero do not move (stop movement if joystick is not actively being touched)
    if (xDir == 0.0f && yDir == 0.0f)
        return;

    float currentZ = this->cam->getPosition().z;
    //move camera position accordingly
    if (yDir < 0)
        this->cam->moveInLookDirection();
    else if (yDir > 0)
        this->cam->moveOppositeLookDirection();

    if (xDir < 0)
        this->cam->moveLeft();
    else if (xDir > 0)
        this->cam->moveRight();

   Vector currentPos = this->cam->getPosition();
   currentPos.z = currentZ;
   this->cam->setPosition(currentPos);

}

void GLViewAssignment_FinalProject::controllerPerspective()
{
    //get values from right joystick
    int xright = SDL_GameControllerGetAxis(control, SDL_CONTROLLER_AXIS_RIGHTX);
    int yright = SDL_GameControllerGetAxis(control, SDL_CONTROLLER_AXIS_RIGHTY);

    //holds either positive or negative directions from joystick movement
    int xCam = 0;
    int yCam = 0;

    //compare to dead zone to ensure enough movement occurred (prevent drifting
    //dentify the direction joystick moved in)
    if (xright < -JOYSTICK_DEAD_ZONE)
    {
        xCam = -1;
    }
    else if (xright > JOYSTICK_DEAD_ZONE)
    {
        xCam = 1;
    }

    if (yright < -JOYSTICK_DEAD_ZONE)
    {
        yCam = -1;
    }
    else if (yright > JOYSTICK_DEAD_ZONE)
    {
        yCam = 1;
    }


    // if value is zero do not move (stops movement if joystick is not actively being touched)
    if (xCam == 0 && yCam == 0)
        return;

    //speed up camera angle movement
    int quicker = 5;
    xCam = xCam * quicker;
    yCam = yCam * quicker;

    //move camera angle accordingly
    this->cam->changeLookAtViaMouse(xCam, yCam);

}





void Aftr::GLViewAssignment_FinalProject::loadMap()
{
   this->worldLst = new WorldList(); //WorldList is a 'smart' vector that is used to store WO*'s
   this->actorLst = new WorldList();
   this->netLst = new WorldList();

   ManagerOpenGLState::GL_CLIPPING_PLANE( 1000.0 );
   ManagerOpenGLState::GL_NEAR_PLANE( 0.1f );
   ManagerOpenGLState::enableFrustumCulling( false );
   Axes::isVisible = true;
   this->glRenderer->isUsingShadowMapping( false ); //set to TRUE to enable shadow mapping, must be using GL 3.2+

   this->cam->setPosition( 15,15,5 );

   std::string shinyRedPlasticCube( ManagerEnvironmentConfiguration::getSMM() + "/models/cube4x4x4redShinyPlastic_pp.wrl" );
   std::string wheeledCar( ManagerEnvironmentConfiguration::getSMM() + "/models/rcx_treads.wrl" );
   std::string grass( ManagerEnvironmentConfiguration::getSMM() + "/models/grassFloor400x400_pp.wrl" );
   std::string human( ManagerEnvironmentConfiguration::getSMM() + "/models/human_chest.wrl" );
   
   //SkyBox Textures readily available
   std::vector< std::string > skyBoxImageNames; //vector to store texture paths
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_water+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_dust+6.jpg" );
   skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_mountains+6.jpg" );


   {
      //Create a light
      float ga = 0.1f; //Global Ambient Light level for this module
      ManagerLight::setGlobalAmbientLight( aftrColor4f( ga, ga, ga, 1.0f ) );
      WOLight* light = WOLight::New();
      light->isDirectionalLight( true );
      light->setPosition( Vector( 0, 0, 100 ) );
      //Set the light's display matrix such that it casts light in a direction parallel to the -z axis (ie, downwards as though it was "high noon")
      //for shadow mapping to work, this->glRenderer->isUsingShadowMapping( true ), must be invoked.
      light->getModel()->setDisplayMatrix( Mat4::rotateIdentityMat( { 0, 1, 0 }, 90.0f * Aftr::DEGtoRAD ) );
      light->setLabel( "Light" );
      worldLst->push_back( light );
   }

   {
      //Create the SkyBox
      WO* wo = WOSkyBox::New( skyBoxImageNames.at( 0 ), this->getCameraPtrPtr() );
      wo->setPosition( Vector( 0, 0, 0 ) );
      wo->setLabel( "Sky Box" );
      wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
      worldLst->push_back( wo );
   }

   { 
      ////Create the infinite grass plane (the floor)
      WO* wo = WO::New( grass, Vector( 1, 1, 1 ), MESH_SHADING_TYPE::mstFLAT );
      wo->setPosition( Vector( 0, 0, 0 ) );
      wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
      wo->upon_async_model_loaded( [wo]()
         {
            ModelMeshSkin& grassSkin = wo->getModel()->getModelDataShared()->getModelMeshes().at( 0 )->getSkins().at( 0 );
            grassSkin.getMultiTextureSet().at( 0 ).setTexRepeats( 5.0f );
            grassSkin.setAmbient( aftrColor4f( 0.4f, 0.4f, 0.4f, 1.0f ) ); //Color of object when it is not in any light
            grassSkin.setDiffuse( aftrColor4f( 1.0f, 1.0f, 1.0f, 1.0f ) ); //Diffuse color components (ie, matte shading color of this object)
            grassSkin.setSpecular( aftrColor4f( 0.4f, 0.4f, 0.4f, 1.0f ) ); //Specular color component (ie, how "shiney" it is)
            grassSkin.setSpecularCoefficient( 10 ); // How "sharp" are the specular highlights (bigger is sharper, 1000 is very sharp, 10 is very dull)
         } );
      wo->setLabel( "Grass" );
      worldLst->push_back( wo );
   }

   {
      this->gulfstream = WO::New( ManagerEnvironmentConfiguration::getSMM() + "/models/Aircraft/Gulfstream3/G3.obj", Vector(1.0f, 1.0f, 1.0f ), MESH_SHADING_TYPE::mstAUTO );
      this->gulfstream->setPosition( Vector( 0, 0, 10 ) );
      this->gulfstream->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
      this->gulfstream->upon_async_model_loaded( [this]()
         {
            ModelMeshSkin& skin = this->gulfstream->getModel()->getModelDataShared()->getModelMeshes().at( 0 )->getSkins().at( 0 );
            skin.setAmbient( aftrColor4f( 0.1f, 0.1f, 0.1f, 1.0f ) ); //Color of object when it is not in any light
            //skin.setDiffuse( aftrColor4f( .1f, .1f, .5f, 1.0f ) ); //Diffuse color components (ie, matte shading color of this object) // Make it blue? Why not?
            skin.setSpecular( aftrColor4f( 0.4f, 0.4f, 0.4f, 1.0f ) ); //Specular color component (ie, how "shiney" it is)
            skin.setSpecularCoefficient( 10 ); // How "sharp" are the specular highlights (bigger is sharper, 1000 is very sharp, 10 is very dull)
         } );
      gulfstream->setLabel( "Gulfstream GIII" );
      worldLst->push_back( this->gulfstream );
   }

   //{
   //    this->playerModel = WO::New(ManagerEnvironmentConfiguration::getLMM() + "/models/metalPlayer1.obj", Vector(1.0f, 1.0f, 1.0f), MESH_SHADING_TYPE::mstAUTO);
   //    this->playerModel->setPosition(Vector(0, 40, 2));

   //    this->playerModel->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;

   //    this->playerModel->upon_async_model_loaded([this]()
   //        {
   //            ModelMeshSkin& skin = this->playerModel->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0);

   //            //gray character model

   //            //skin.setAmbient(aftrColor4f(0.25f, 0.25f, 0.25f, 1.0f));
   //            skin.setAmbient(aftrColor4f(1.0f, 1.0f, 1.0f, 1.0f));
   //            //skin.setDiffuse(aftrColor4f(0.85f, 0.85f, 0.85f, 1.0f));
   //            skin.setDiffuse(aftrColor4f(1.0f, 1.0f, 1.0f, 1.0f));
   //            skin.setSpecular(aftrColor4f(0.2f, 0.2f, 0.2f, 1.0f));
   //            skin.setSpecularCoefficient(10);

   //            //skin.setAmbient(aftrColor4f(0.4f, 0.2f, 0.95f, 1.0f)); //Color of object when it is not in any light
   //            //skin.setDiffuse(aftrColor4f(.1f, .1f, .5f, 1.0f)); //Diffuse color components (ie, matte shading color of this object) // Make it blue? Why not?
   //            //skin.setSpecular(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Specular color component (ie, how "shiney" it is)
   //            //skin.setSpecularCoefficient(1000); // How "sharp" are the specular highlights (bigger is sharper, 1000 is very sharp, 10 is very dull)
   //        });

   //    this->playerModel->setLabel("Player1");
   //    worldLst->push_back(this->playerModel);
   // }
   {
       //if player 2, load player1 model (grey)
       if (playerNum == 1)
       {
           this->playerModel = WO::New(ManagerEnvironmentConfiguration::getLMM() + "/models/metalPlayer1.obj", Vector(1.0f, 1.0f, 1.0f), MESH_SHADING_TYPE::mstAUTO);
           this->playerModel->setPosition(Vector(0, 40, 5));

           this->playerModel->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;

           this->playerModel->upon_async_model_loaded([this]()
               {
                   ModelMeshSkin& skin = this->playerModel->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0);

                   //gray character model

                   //skin.setAmbient(aftrColor4f(0.25f, 0.25f, 0.25f, 1.0f));
                   skin.setAmbient(aftrColor4f(1.0f, 1.0f, 1.0f, 1.0f));
                   //skin.setDiffuse(aftrColor4f(0.85f, 0.85f, 0.85f, 1.0f));
                   skin.setDiffuse(aftrColor4f(1.0f, 1.0f, 1.0f, 1.0f));
                   skin.setSpecular(aftrColor4f(0.2f, 0.2f, 0.2f, 1.0f));
                   skin.setSpecularCoefficient(10);

                   //skin.setAmbient(aftrColor4f(0.4f, 0.2f, 0.95f, 1.0f)); //Color of object when it is not in any light
                   //skin.setDiffuse(aftrColor4f(.1f, .1f, .5f, 1.0f)); //Diffuse color components (ie, matte shading color of this object) // Make it blue? Why not?
                   //skin.setSpecular(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Specular color component (ie, how "shiney" it is)
                   //skin.setSpecularCoefficient(1000); // How "sharp" are the specular highlights (bigger is sharper, 1000 is very sharp, 10 is very dull)
               });


           this->playerModel->setLabel("Player1");
           worldLst->push_back(this->playerModel);
       }
       //if player 1, load player2 model (yellow)
       else
       {
           this->playerModel = WO::New(ManagerEnvironmentConfiguration::getLMM() + "/models/metalPlayer1.obj", Vector(1.0f, 1.0f, 1.0f), MESH_SHADING_TYPE::mstAUTO);
           this->playerModel->setPosition(Vector(0, 50, 5));

           this->playerModel->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;

           this->playerModel->upon_async_model_loaded([this]()
               {
                   ModelMeshSkin& skin = this->playerModel->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0);

                   //yellow character model
                   skin.setAmbient(aftrColor4f(0.60f, 0.60f, 0.0f, 1.0f));
                   skin.setDiffuse(aftrColor4f(1.0f, 1.0f, 0.0f, 1.0f));
                   skin.setSpecular(aftrColor4f(0.2f, 0.2f, 0.2f, 1.0f));
                   skin.setSpecularCoefficient(10);

               });


           this->playerModel->setLabel("Player2");
           worldLst->push_back(this->playerModel);
       }
       

   }

   {
       this->ball = WO::New(ManagerEnvironmentConfiguration::getLMM() + "/models/beachBall.obj", Vector(1.0f, 1.0f, 1.0f), MESH_SHADING_TYPE::mstAUTO);
       this->ball->setPosition(Vector(0, 0, 0.5));

       this->ball->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;

       this->ball->upon_async_model_loaded([this]()
           {
               ModelMeshSkin& skin = this->ball->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0);

               //gray character model

               //skin.setAmbient(aftrColor4f(0.25f, 0.25f, 0.25f, 1.0f));
               skin.setAmbient(aftrColor4f(1.0f, 1.0f, 1.0f, 1.0f));
               //skin.setDiffuse(aftrColor4f(0.85f, 0.85f, 0.85f, 1.0f));
               skin.setDiffuse(aftrColor4f(1.0f, 1.0f, 1.0f, 1.0f));
               skin.setSpecular(aftrColor4f(0.2f, 0.2f, 0.2f, 1.0f));
               skin.setSpecularCoefficient(10);

               //skin.setAmbient(aftrColor4f(0.4f, 0.2f, 0.95f, 1.0f)); //Color of object when it is not in any light
               //skin.setDiffuse(aftrColor4f(.1f, .1f, .5f, 1.0f)); //Diffuse color components (ie, matte shading color of this object) // Make it blue? Why not?
               //skin.setSpecular(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Specular color component (ie, how "shiney" it is)
               //skin.setSpecularCoefficient(1000); // How "sharp" are the specular highlights (bigger is sharper, 1000 is very sharp, 10 is very dull)
           });


       this->playerModel->setLabel("Ball");
       worldLst->push_back(this->ball);
   }

   {
      //Make a sphere
      this->moon = WO::New();
      MGLIndexedGeometry* mglSphere = MGLIndexedGeometry::New( this->moon );
      IndexedGeometrySphereTriStrip* geoSphere = IndexedGeometrySphereTriStrip::New( 3.0f, 12, 12, true, true );
      mglSphere->setIndexedGeometry( geoSphere );
      this->moon->setModel( mglSphere );
      this->moon->setLabel( "Moon" );
      this->moon->setPosition( { 15,15,15 } );
      this->moon->renderOrderType = RENDER_ORDER_TYPE::roTRANSPARENT;
      this->worldLst->push_back( this->moon );

      //Place a texture on the sphere, now its a moon
      fmt::print( "To the moon...\n" );
      Tex tex = *ManagerTex::loadTexAsync( ManagerEnvironmentConfiguration::getSMM() + "/images/moonMap.jpg" );
      this->moon->getModel()->getSkin().getMultiTextureSet().at( 0 ) = tex;
      this->moon->setPosition( {15,2,10});

      //We always want some axes, too!
      WO* axes = WOAxesTubes::New( { 15.0f,15.0f,15.0f }, .2f );
      axes->setParentWorldObject( this->moon );
      axes->setPosition( this->moon->getPosition() ); //match parent position before locking
      axes->lockWRTparent(); //makes a joint that "welds" this child rigidly to parent
      this->moon->getChildren().push_back( axes );     
   }
   
   // Let's make a GUI. We create a WOImGui instance, and then use the strategy pattern to
   // submit/subscribe lambdas/std::functions/callbacks (~same thing) to draw our desired widgets.
   // We nestle the callbacks inside a menu to keep everything organized. The menu uses an on/off
   // toggle -- when on, that menu item's corresponding callback will be invoked each frame, when
   // off that callback is not invoked and therefore the corresponding window is not drawn.

   {
      this->gui = WOImGui::New( nullptr );
      gui->setLabel( "My Gui" );
      //callbacks -- When the user toggles the checkbox from the menu, call these callbacks:
      
      //This callback shows the WOEditor window. It will be visible when the user
      //selected Menu -> Edit -> Show WO Editor (as linked up below).
      auto woEditFunc = [this]() { this->wo_editor.draw( this->getLastSelectionQuery(), *this->getWorldContainer(), this->getCamera_functor() ); };

      //We will put these demo items under the "Demo" menu
      auto showDemoWindow_ImGui     = [this]() { ImGui::ShowDemoWindow(); };
      auto showDemoWindow_AftrDemo  = [this]() { WOImGui::draw_AftrImGui_Demo(this->gui); };
      auto showDemoWindow_ImGuiPlot = [this]() { ImPlot::ShowDemoWindow(); };
      auto show_moon_orbit_params   = [this]() { this->orbit_gui.draw(); };

      this->gui->subscribe_drawImGuiWidget(
         [=,this]() //this is a lambda, the capture clause is in [], the input argument list is in (), and the body is in {}
         {
            //We defined the callbacks above, now hook them into the menu labels
            menu.attach( "Edit", "Show WO Editor", woEditFunc );
            menu.attach( "Demos", "Show Default ImGui Demo", showDemoWindow_ImGui );
            menu.attach( "Demos", "Show Default ImPlot Demo", showDemoWindow_ImGuiPlot );
            menu.attach( "Demos", "Show Aftr ImGui w/ Markdown & File Dialogs", showDemoWindow_AftrDemo );
            menu.attach( "Orbit Gui", "Show Orbit", show_moon_orbit_params, true );
            menu.draw(); //The menu.draw() is the entry point for your gui. It is called once per frame to draw the GUI.
         } );
      this->worldLst->push_back( this->gui );
   }

   createAssignment_FinalProjectWayPoints();
}


void GLViewAssignment_FinalProject::createAssignment_FinalProjectWayPoints()
{
   // Create a waypoint with a radius of 3, a frequency of 5 seconds, activated by GLView's camera, and is visible.
   WayPointParametersBase params(this);
   params.frequency = 5000;
   params.useCamera = true;
   params.visible = true;
   WOWayPointSpherical* wayPt = WOWayPointSpherical::New( params, 3 );
   wayPt->setPosition( Vector( 50, 0, 3 ) );
   worldLst->push_back( wayPt );
}
