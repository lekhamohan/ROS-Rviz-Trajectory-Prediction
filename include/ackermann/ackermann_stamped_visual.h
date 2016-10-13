/* See the file "LICENSE" for the full license and copyrights governing this code. */

#ifndef ACKERMANN_STAMPED_VISUAL_H
#define ACKERMANN_STAMPED_VISUAL_H

#include <tf/transform_listener.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>
#include <rviz/message_filter_display.h>
#include <ros/ros.h>
#include <rviz/ogre_helpers/line.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <rviz/ogre_helpers/line.h>
#include <tf/transform_listener.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>
#include <string.h>
#include <ros/ros.h>
#include <math.h>
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "rviz/ogre_helpers/movable_text.h"



namespace Ogre
{
class Vector3;
class Quaternion;
}

namespace rviz
{
class Line;
class Shape;
class MovableText;

}

namespace path_manager_plugins 
{

class AckermannStampedVisual
{
public:
  // Constructor.  
	AckermannStampedVisual( Ogre::SceneManager*, Ogre::SceneNode*);

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~AckermannStampedVisual();

  // Configure the visual to show the data in the message.
  void setMessage(const ackermann_msgs::AckermannDriveStamped::ConstPtr&,int,float,std::string);

  //Position
  void setFramePosition(const Ogre::Vector3& );
  void setFrameOrientation( const Ogre::Quaternion& );
  
  //Color of the Arrow or Line
  void setColor(float r, float g,float b);

  //variable declaration
  Ogre::Vector3 initPos, destPos,temp;


private:
  
  boost::shared_ptr<rviz::Line> lineVisual;
  Ogre::SceneNode* sceneNode;
  Ogre::SceneManager* sceneManager;
  tf::TransformListener tfListener ;//, listenerDest;
  float prevTime;
  boost::shared_ptr<rviz::BillboardLine> billBoardVisual;
  float startTheta, endTheta;

 
};




}

#endif
