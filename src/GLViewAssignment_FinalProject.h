#pragma once

#include "GLView.h"
#include "AftrImGui_MenuBar.h"
#include "AftrImGui_WO_Editor.h"
#include "AftrImGui_Assignment_FinalProject.h"

namespace Aftr
{
   class Camera;
   class WOImGui;

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


   WO* player1 = nullptr;
   WO* player2 = nullptr;

   int playerNum = 0; 
   //int playerNum = 1;
};

/** \} */

} //namespace Aftr
