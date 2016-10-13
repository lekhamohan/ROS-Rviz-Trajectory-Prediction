///* See the file "LICENSE" for the full license and copyrights governing this code. */
//
#include "../include/ackermann/ackermann_stamped_visual.h"

//Start of NAmespace
namespace path_manager_plugins
//
{
  AckermannStampedVisual::AckermannStampedVisual(Ogre::SceneManager* sceneManagerNode, Ogre::SceneNode* parentNode)
  {
    sceneManager = sceneManagerNode;
    sceneNode = parentNode->createChildSceneNode();
    lineVisual.reset(new rviz::Line( sceneManager, sceneNode ));
    billBoardVisual.reset(new rviz::BillboardLine( sceneManager, sceneNode ));
    initPos = Ogre::Vector3(0.0,0.0,0.0);
    startTheta = 0.0;
  }
//
  AckermannStampedVisual::~AckermannStampedVisual()
  {
    sceneManager->destroySceneNode( sceneNode );
  }
//
void AckermannStampedVisual::setMessage(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg, int timeLength, float width,std::string frame)
{
  float Vx, Vy, Vz, Vmag, Vang, radius, V, B, wheelLength;

  wheelLength = 1.0;
  B = msg->drive.steering_angle/2;
	//ROS_INFO_STREAM("ACK SET MESSAGE");

  radius = (wheelLength/(cos(B)*-tan(msg->drive.steering_angle)));
  Vang = msg->drive.speed/radius;

	//ROS_INFO_STREAM("Radius is"<<radius<<"  Vangular is"<<Vang<<"Velocity "<<msg->drive.speed<< "Steering angle "<<msg->drive.steering_angle);

  startTheta = 0;
  endTheta = fabs(Vang) * timeLength; //time - 15 seconds
  float t = startTheta;

  if(endTheta-startTheta > 2*M_PI)
    endTheta = 2*M_PI;
  float fradius = fabs(radius);
  float logRadius = log(fradius);
  
  if(logRadius < 1) logRadius = 1;
    int steps = fabs(endTheta-startTheta)* 6 * logRadius;
  if(steps < 3) steps = 3;
  if(steps > 64) steps = 64;

  float dt = (endTheta-startTheta)/(float)(steps);
	//	ROS_INFO_STREAM("Log Radius is "<<logRadius<<" steps are "<<steps<<"  DT is "<<dt );

  Ogre::Vector3 pointList;
  billBoardVisual->clear();

  tf::StampedTransform transform;
	try {
		tfListener.waitForTransform(frame, "base_footprint",ros::Time(0), ros::Duration(15.0) );
		tfListener.lookupTransform(frame,"base_footprint",  ros::Time(0), transform);
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
	}

	tf::Vector3 newPointInRobotFrame(0, 0, 0);
	//ROS_INFO_STREAM("The pointinRobot  "<< newPointInRobotFrame.getX()<<","<<newPointInRobotFrame.getY()<<","<<newPointInRobotFrame.getZ());

	tf::Vector3 newPointInMapFrame = transform * newPointInRobotFrame;

	//ROS_INFO_STREAM("The transform  "<< transform.getOrigin().getX()<<"  ,"<<transform.getOrigin().getY()<<",  "<<transform.getOrigin().getZ());

	Ogre::Vector3 pointToPush(newPointInMapFrame.getX(), newPointInMapFrame.getY(), newPointInMapFrame.getZ());

	//ROS_INFO_STREAM("The pointinMap   "<< newPointInMapFrame.getX()<<","<<newPointInMapFrame.getY()<<","<< newPointInMapFrame.getZ());

	billBoardVisual->addPoint(pointToPush);
	billBoardVisual->setLineWidth(width);


	for(int i = 0; i < steps; i++)
	{
		tf::Vector3 newPointInRobotFrame;
		newPointInRobotFrame.setX(fradius * (sin(t)));
		if(Vang < 0)
			newPointInRobotFrame.setY( (fradius - (fradius*cos(t))));
		if(Vang  > 0)
			newPointInRobotFrame.setY(- (fradius -(fradius *(cos(t)))));

		newPointInRobotFrame.setZ(0.0); //transform.getOrigin().z();
		//ROS_INFO_STREAM("iteration " << i << ", The pointinRobot  "<< newPointInRobotFrame.getX()<<","<<newPointInRobotFrame.getY()<<","<<newPointInRobotFrame.getZ());

		tf::Vector3 newPointInMapFrame = transform * newPointInRobotFrame;

		//ROS_INFO_STREAM("The transform  "<< transform.getOrigin().getX()<<"  ,"<<transform.getOrigin().getY()<<",  "<<transform.getOrigin().getZ());

		Ogre::Vector3 pointToPush(newPointInMapFrame.getX(), newPointInMapFrame.getY(), newPointInMapFrame.getZ());

		//ROS_INFO_STREAM("The pointinMap   "<< newPointInMapFrame.getX()<<","<<newPointInMapFrame.getY()<<","<< newPointInMapFrame.getZ());

		billBoardVisual->addPoint(pointToPush);
		t += dt;
	}

	//	ROS_INFO_STREAM("OUTSIDE SET MESSAGE");

}


void AckermannStampedVisual::setFramePosition( const Ogre::Vector3& position )
{
	//ROS_INFO("INSIDE SET_FRAME_POSITION FUNCTION");
	sceneNode->setPosition( position );
	lineVisual->setPosition(position);
}

void AckermannStampedVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
	//	ROS_INFO("INSIDE SET_FRAME_ORIENTATTION FUNCTION");
	sceneNode->setOrientation( orientation );
	lineVisual->setOrientation(orientation);
}

// Color is passed through to the Arrow object.
void AckermannStampedVisual::setColor( float r, float g, float b )
{
	billBoardVisual->setColor(r,g,b,1);

}

} // end namespace
