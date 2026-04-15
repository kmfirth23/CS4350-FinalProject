#pragma once

#include "GLView.h"
#include "AftrImGui_MenuBar.h"
#include "AftrImGui_WO_Editor.h"
#include "AftrImGui_Assignment_FinalProject.h"

#include "NetMsg.h"

#include "NetMessengerClient.h"
#include "NetMsgCreateWO.h"

namespace Aftr
{
   class Camera;
   class WOImGui;


   class NetMsgControlObjects : public NetMsg
   {
   public:
       NetMsgMacroDeclaration(NetMsgControlObjects);

       NetMsgControlObjects();
       virtual ~NetMsgControlObjects();
       /// <summary>
       /// Send out the message
       /// </summary>
       /// <param name="os"></param>
       /// <returns></returns>
       virtual bool toStream(NetMessengerStreamBuffer& os) const override;
       /// <summary>
       /// Read in the message
       /// </summary>
       /// <param name="is"></param>
       /// <returns></returns>
       virtual bool fromStream(NetMessengerStreamBuffer& is) override;
       /// <summary>
       /// Once the message is recieved react accordingly
       /// </summary>
       virtual void onMessageArrived() override;
       /// <summary>
       /// Payload size
       /// </summary>
       /// <returns></returns>
       virtual std::string toString();

       unsigned int size;
       //std::string objectName;
       unsigned int ID;
       float m[16];

   protected:
   };

/**
   \class GLViewAssignment_FinalProject
   \author Scott Nykl 
   \brief A child of an abstract GLView. This class is the top-most manager of the module.

   Read \see GLView for important constructor and init information.

   \see GLView

    \{
*/

class GLViewAssignment_FinalProject : public GLView
{
public:
   static GLViewAssignment_FinalProject* New( const std::vector< std::string >& outArgs );
   virtual ~GLViewAssignment_FinalProject();
   virtual void updateWorld() override; ///< Called once per frame
   virtual void loadMap() override; ///< Called once at startup to build this module's scene
   virtual void createAssignment_FinalProjectWayPoints();
   virtual void onResizeWindow( GLsizei width, GLsizei height ) override;
   virtual void onMouseDown( const SDL_MouseButtonEvent& e ) override;
   virtual void onMouseUp( const SDL_MouseButtonEvent& e ) override;
   virtual void onMouseMove( const SDL_MouseMotionEvent& e ) override;
   virtual void onKeyDown( const SDL_KeyboardEvent& key ) override;
   virtual void onKeyUp( const SDL_KeyboardEvent& key ) override;

   /// <summary>
   /// Moves camera based on controller left joystick
   /// </summary>
   virtual void controllerMove();
   /// <summary>
   /// Moves the camera perspeactive based on controller right joystick
   /// </summary>
   virtual void controllerPerspective();


   /// <summary>
   /// takes the position and rotations of the cube and updates them accordingly
   /// takes the values recieved from the other instance and updates the cubes position
   /// </summary>
   /// <param name="id"></param> object id
   /// <param name="m"></param> pose 
   virtual void updateObj(int id, float m[16]);

   std::shared_ptr<NetMessengerClient> client = nullptr;

protected:
   GLViewAssignment_FinalProject( const std::vector< std::string >& args );
   virtual void onCreate();

   WOImGui* gui = nullptr; //The GUI which contains all ImGui widgets
   AftrImGui_MenuBar menu;      //The Menu bar at the top of the GUI window
   AftrImGui_WO_Editor wo_editor;//The WO Editor to mutate a selected WO
   AftrImGui_Assignment_FinalProject orbit_gui;
   WO* moon = nullptr;
   WO* gulfstream = nullptr;

   std::vector<SDL_GameController*> controllerList; //list to store different controllers in

   SDL_GameController* control = nullptr; //this windows controller
   const int JOYSTICK_DEAD_ZONE = 8000; //value that axis movement needs to be larger than


   //WO* player1 = nullptr;
   //WO* player2 = nullptr;

   WO* playerModel = nullptr;
   WO* ball = nullptr;


   //playerNum = 0 (Player1) and playerNum = 1 (Player2)
   static int const playerNum = 0; 
   //int playerNum = 1;
};

/** \} */

} //namespace Aftr
