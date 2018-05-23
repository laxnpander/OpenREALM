/************************************************************************
*  Autors:   Hannes Harms 5.2015         *       		        *
*  E-Mail:   hannes.harms@tu-bs.de       *                              *
************************************************************************/
#ifndef RVIZ_MAP_DISPLAY_H
#define RVIZ_MAP_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>

#include <OgreTexture.h>
#include <OgreMaterial.h>
#include <OgreVector3.h>
#include <OgreSharedPtr.h>
#endif

#include <ros/time.h>
#include <tf/transform_listener.h>

#include <geodesy/wgs84.h>
#include <geodesy/utm.h>

#include "rviz/display.h"
#include "realm_msgs/GroundImageCompressed.h"

namespace Ogre
{
class ManualObject;
}

namespace rviz {

class EnumProperty;
class FloatProperty;
class IntProperty;
class Property;
class QuaternionProperty;
class RosTopicProperty;
class VectorProperty;

}
namespace realm_rviz_plugin
{

/**
 *  Displays a image
 */
class GroundImageDisplay: public rviz::Display
{
Q_OBJECT
public:
  GroundImageDisplay();
  virtual ~GroundImageDisplay() override;

  // Overrides from Display
  virtual void onInitialize() override;
  virtual void fixedFrameChanged() override;
  virtual void reset() override;

  float getResolution() { return resolution_; }
  int getWidth() { return width_; }
  int getHeight() { return height_; }

  virtual void setTopic( const QString &topic, const QString &datatype) override;


protected Q_SLOTS:
  void updateTopic();

protected:
  // overrides from Display
  virtual void onEnable() override;
  virtual void onDisable() override;

  virtual void subscribe();
  virtual void unsubscribe();
  virtual void update( float wall_dt, float ros_dt) override;

  /** @brief Copy msg into current_map_ and call showMap(). */ 
  void incomingImage(const realm_msgs::GroundImageCompressed::ConstPtr& msg);


  void clear();

  void transformImage();
  std::vector<Ogre::ManualObject*> manual_object_;
  std::vector<Ogre::TexturePtr> texture_;
  std::vector<Ogre::MaterialPtr> material_;
  bool loaded_;

  bool has_transform;
  tf::StampedTransform transform;

  std::string topic_;
  float resolution_;
  int width_;
  int height_;
  std::string frame_;
  realm_msgs::GroundImageCompressed image;

  ros::Subscriber image_sub_;

  rviz::RosTopicProperty* topic_property_;
  std::vector<rviz::VectorProperty*> position_property_;
  std::vector<rviz::QuaternionProperty*> orientation_property_;
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  std::vector<realm_msgs::GroundImageCompressed::ConstPtr> image_vec;
  void add_new_Image();
  static int mcounter;
  Ogre::MaterialPtr material_local;
  Ogre::ManualObject* manual_object_local;

};

} // namespace rviz

 #endif
