/* See the file "LICENSE" for the full license and copyrights governing this code. */

#include "../include/node_states/twist_stamped_visual.h"

namespace path_manager_plugins

{

TwistStampedVisual::TwistStampedVisual(Ogre::SceneManager* sceneManagerNode, Ogre::SceneNode* parentNode)
{
  sceneManager = sceneManagerNode;
  sceneNode = parentNode->createChildSceneNode();
  billBoardVisual.reset(new rviz::BillboardLine( sceneManager, sceneNode ));
  startTheta = 0.0;
}

TwistStampedVisual::~TwistStampedVisual()
{
  sceneManager->destroySceneNode( sceneNode );
}

void TwistStampedVisual::setMessage(const geometry_msgs::TwistStamped::ConstPtr& msg, int timeLength, float width,std::string frame )
{
  float Vx, Vy, Vz, Vmag, Vang, radius;

	////ROS_INFO_STREAM("INSIDE SET MESSAGE");
  Vx = msg->twist.linear.x;
  Vy = msg->twist.linear.y;
  Vz = msg->twist.linear.z;
  Vang = msg->twist.angular.z;

  Vmag = sqrt(pow(Vx, 2) + pow(Vy, 2) + pow(Vz,2));
  radius = Vmag/fabs(Vang);

  startTheta = 0;
  float t = startTheta;
  endTheta = fabs(Vang) * timeLength;

  if(endTheta-startTheta > 2*M_PI) endTheta = startTheta + 2*M_PI;
  if(endTheta-startTheta < -2*M_PI) endTheta = startTheta - 2*M_PI;
  float fradius = fabs(radius);
  float logRadius = log(fradius); 
  if(logRadius < 1) logRadius = 1;
  int steps = fabs(endTheta-startTheta)* 6 * logRadius;
  if(steps < 3) steps = 3;
  if(steps > 64) steps = 64;

  float dt = (endTheta-startTheta)/(float)(steps);

  Ogre::Vector3 pointList;
  billBoardVisual->clear();

  tf::StampedTransform transform;
  try {
        tfListener.waitForTransform(frame, "base_footprint",ros::Time(0), ros::Duration(10.0) );
  	tfListener.lookupTransform(frame,"base_footprint",  ros::Time(0), transform);
      }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }

  tf::Vector3 newPointInRobotFrame(0, 0, 0);
  tf::Vector3 newPointInMapFrame = transform * newPointInRobotFrame;
  Ogre::Vector3 pointToPush(newPointInMapFrame.getX(), newPointInMapFrame.getY(), newPointInMapFrame.getZ());


  billBoardVisual->addPoint(pointToPush);
  billBoardVisual->setLineWidth(width);


  for(int i = 0; i < steps; i++)
  {
    tf::Vector3 newPointInRobotFrame;
    newPointInRobotFrame.setX(fradius * (sin(t)));
    if(Vang > 0)
      newPointInRobotFrame.setY( (fradius - (fradius*cos(t))));
    if(Vang  < 0)
      newPointInRobotFrame.setY(- (fradius -(fradius *(cos(t)))));

    newPointInRobotFrame.setZ(0.0); //transform.getOrigin().z();
    tf::Vector3 newPointInMapFrame = transform * newPointInRobotFrame;
    Ogre::Vector3 pointToPush(newPointInMapFrame.getX(), newPointInMapFrame.getY(), newPointInMapFrame.getZ());
    billBoardVisual->addPoint(pointToPush);
    t += dt;
  }
//	//ROS_INFO_STREAM("OUTSIDE SET MESSAGE");
}

void TwistStampedVisual::setFramePosition( const Ogre::Vector3& position )
{
	//ROS_INFO("INSIDE SET_FRAME_POSITION FUNCTION");
  sceneNode->setPosition( position );
  billBoardVisual->setPosition(position);
}

void TwistStampedVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{ 
  sceneNode->setOrientation( orientation );
  billBoardVisual->setOrientation(orientation);
}

// Color is passed through to the Arrow object.
void TwistStampedVisual::setColor( float r, float g, float b )
{
  billBoardVisual->setColor(r,g,b,1);
}

} // NS




