/* See the file "LICENSE" for the full license and copyrights governing this code. */

#ifndef TWIST_STAMPED_DISPLAY_H
#define  TWIST_STAMPED_DISPLAY_H

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
#include "nav_msgs/Odometry.h"

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

class TwistStampedVisual;

class TwistStampedDisplay: public rviz::MessageFilterDisplay<geometry_msgs::TwistStamped>
{
	Q_OBJECT
public:
	// Constructor.  pluginlib::ClassLoader creates instances by calling the default constructor
	TwistStampedDisplay();
	virtual ~TwistStampedDisplay();

protected:
	virtual void onInitialize();
	// clear display back to the initial state.
	virtual void reset();

	//  Qt slots
	private Q_SLOTS:
	void updateLengthAndColor();
	void enable();

	// handle incoming ROS message.
	private:

	std::deque<boost::shared_ptr<TwistStampedVisual> > visuals;
	boost::shared_ptr<TwistStampedVisual> visual;
	rviz::IntProperty* length;
	rviz::FloatProperty* lineWidth;
	rviz::ColorProperty* color_property_;
	Ogre::ColourValue color;

	int timeLength; float width;

	void processMessage( const geometry_msgs::TwistStamped::ConstPtr&);

};


} // end namespace 

#endif // NODE_STATE_DISPLAY_H

