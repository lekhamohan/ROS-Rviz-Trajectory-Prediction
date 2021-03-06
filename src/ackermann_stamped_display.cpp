//Visualization of Ackermann_msgs published by the robot on Rviz simulator

#include "../include/ackermann/ackermann_stamped_display.h"
#include "../include/ackermann/ackermann_stamped_visual.h"
namespace path_manager_plugins
{

AckermannStampedDisplay::AckermannStampedDisplay()
{
  length = new rviz::IntProperty("Time",1,"Time in seconds to visualize the turn rate",this,SLOT(updateLengthAndColor() ));
  lineWidth = new rviz::FloatProperty("Line Width",0.1,"set the line thickness",this,SLOT(updateLengthAndColor() ));
  color_property_ = new rviz::ColorProperty( "Color", QColor( 204, 1, 04 ),
                                                 "Color to visualize the trajectory",
                                                 this, SLOT( updateLengthAndColor() ));
}

void AckermannStampedDisplay::onInitialize()
{
	//ROS_INFO("INSIDE ACK ONINITIALIZE");
  MFDClass::onInitialize();
  updateLengthAndColor();
  reset();
}

AckermannStampedDisplay::~AckermannStampedDisplay()
{
}

void AckermannStampedDisplay::updateLengthAndColor()
{
  color = color_property_->getOgreColor();
  timeLength = length->getInt();
  width = lineWidth->getFloat();
}

void AckermannStampedDisplay::reset()
{
  //ROS_INFO("ACK RESET");
  MFDClass::reset();
  visuals.clear();
}

void AckermannStampedDisplay::processMessage(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
{
  //ROS_INFO_STREAM(std::endl<<std::endl<<"ACK PROCESS MESSAGE");
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  std::string frame = context_->getFrameManager()->getFixedFrame();
  if( !context_->getFrameManager()->getTransform( frame,
			msg->header.stamp,
			position, orientation ))
  {
    ROS_ERROR( "Rviz Node State Plugin:Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }


  if( visuals.size()<1)
  {
    visual.reset(new AckermannStampedVisual( context_->getSceneManager(), scene_node_ ));
  }

  else
  {
  //ROS_INFO_STREAM("MEMORY RETAINED AND ALLOCATED");
    visual = visuals.at(0);
  }
  //ROS_INFO_STREAM("TIME LENGTH BEING PASSED ON TO THE SETMESSAGE IS"<<timeLength);
  visual->setMessage(msg,timeLength,width,frame);
  visual->setFramePosition( position );
  visual->setFrameOrientation( orientation );
  visual->setColor(color.r, color.g, color.b);
  visuals.push_back(visual);
//ROS_INFO_STREAM("END OF PROCESS MESSAGE");
}

}//end of NS

PLUGINLIB_EXPORT_CLASS(path_manager_plugins::AckermannStampedDisplay,rviz::Display)
