/* See the file "LICENSE" for the full license and copyrights governing this code. */

#ifndef ACKERMANN_STAMPED_DISPLAY_H
#define  ACKERMANN_STAMPED_DISPLAY_H

#ifndef Q_MOC_RUN

#include <rviz/message_filter_display.h>
#include <OGRE/OgreSceneManager.h>
#include <tf/transform_listener.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/frame_manager.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <string.h>
#include <deque>
#include <boost/circular_buffer.hpp>
#include "ackermann_msgs/AckermannDriveStamped.h"
#endif

namespace Ogre
{
class SceneManager;
}

namespace rviz
{
class FloatProperty;
class IntProperty;
class ColorProperty;
}

namespace path_manager_plugins
{

//class TwistStampedVisual;
//
class AckermannStampedVisual;
class AckermannStampedDisplay: public rviz::MessageFilterDisplay<ackermann_msgs::AckermannDriveStamped>

{
	Q_OBJECT
public:
	// Constructor.  pluginlib::ClassLoader creates instances by calling the default constructor
	AckermannStampedDisplay();
virtual ~AckermannStampedDisplay();

protected:
	virtual void onInitialize();
	// clear display back to the initial state.
	virtual void reset();
//
//	//  Qt slots
	private Q_SLOTS:
	void updateLengthAndColor();
	void enable();
//
//	// handle incoming ROS message.
	private:
//
	std::deque<boost::shared_ptr<AckermannStampedVisual> > visuals;
	boost::shared_ptr<AckermannStampedVisual> visual;
	rviz::IntProperty* historyLength;
	rviz::IntProperty* length;
	rviz::FloatProperty* lineWidth;
	rviz::ColorProperty* color_property_;
        Ogre::ColourValue color;

//
	int timeLength; float width;
//
	void processMessage( const ackermann_msgs::AckermannDriveStamped::ConstPtr&);
//
};


} // end namespace 

#endif
