#include <boost/bind.hpp>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgreSharedPtr.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/custom_parameter_indices.h"
#include "rviz/ogre_helpers/grid.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h"
#include "ground_image_display.h"
#include <algorithm>
#include <iterator>
#include <sensor_msgs/image_encodings.h>
#include <rviz/view_controller.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>

#define BASE_STATION_FRAMEID "/realm_base"
#define WORLD_FRAMEID "/utm"

namespace realm_rviz_plugin
{

// helper class to set alpha parameter on all renderables.
class AlphaSetter : public Ogre::Renderable::Visitor
{
  public:
    AlphaSetter(float alpha) :
        alpha_vec_(alpha, alpha, alpha, alpha)
    {
    }

    void visit(Ogre::Renderable *rend, ushort lodIndex, bool isDebug, Ogre::Any *pAny = 0)
    {
      rend->setCustomParameter(ALPHA_PARAMETER, alpha_vec_);
    }
  private:
    Ogre::Vector4 alpha_vec_;
};

int GroundImageDisplay::mcounter = 0;

GroundImageDisplay::GroundImageDisplay() :
    Display(), loaded_(false), resolution_(0.0f), width_(0), height_(0)
{

  topic_property_ = new rviz::RosTopicProperty("Topic",
                                               "",
                                               QString::fromStdString(ros::message_traits::datatype<realm_msgs::GroundImageCompressed>()),
                                               "realm_msgs::GroundImageCompressed topic to subscribe to.",
                                               this,
                                               SLOT(updateTopic()));

  has_transform = false;
  material_local = Ogre::MaterialManager::getSingleton()
      .create("plane_material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
}

GroundImageDisplay::~GroundImageDisplay()
{
  unsubscribe();
  clear();
}

void GroundImageDisplay::add_new_Image()
{

  //get utm to utm_griound_base only once
  if (!has_transform)
  {
    static tf::TransformListener listener;
    if (!listener.waitForTransform(WORLD_FRAMEID, BASE_STATION_FRAMEID, ros::Time(0), ros::Duration(0.5)))
    {
      ROS_WARN("waitForTransform WORLD_FRAMEID to BASE_STATION_FRAMEID failed!");
      setStatus(rviz::StatusProperty::Warn, "Transform", "No Transform from /utm to /ground_base_utm received");
      return;
    }
    else
    {
      listener.lookupTransform(WORLD_FRAMEID, BASE_STATION_FRAMEID, ros::Time(0), transform);
      setStatus(rviz::StatusProperty::Ok, "Transform", "Transform OK");
      has_transform = true;
    }
  }

  std::stringstream ss;
  ss << mcounter;
  std::string text = "plane_material" + ss.str();
  material_local = material_local->clone(text);
  material_local->setReceiveShadows(false);
  material_local->getTechnique(0)->setLightingEnabled(false);
  material_local->setDepthBias(-16.0f, 0.0f);
  material_local->setCullingMode(Ogre::CULL_NONE);
  material_local->setDepthWriteEnabled(false);

  //cv::Mat matrix = cv::imdecode(cv::Mat(image_vec.back()->imagedata.data), 4);

  cv_bridge::CvImagePtr img_ptr;
  try
  {
    img_ptr = cv_bridge::toCvCopy(image_vec.back()->imagedata, "bgra8");
  }
  catch(...)
  {
    throw(cv_bridge::Exception("Error converting compressed image!"));
  }

  sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(image_vec.back()->imagedata.header, sensor_msgs::image_encodings::RGBA8, img_ptr->image).toImageMsg();

  unsigned int numBytes = msg->height * msg->step;
  bool image_status_set = false;
  Ogre::DataStreamPtr pixel_stream;
  Ogre::PixelFormat format = Ogre::PF_R8G8B8;
  setStatus(rviz::StatusProperty::Ok, "Message", "Image received");

  if (msg->encoding == sensor_msgs::image_encodings::RGB8)
  {
    format = Ogre::PF_R8G8B8;
  }
  else if (msg->encoding == sensor_msgs::image_encodings::RGBA8)
  {
    format = Ogre::PF_BYTE_RGBA;
  }
  else if (msg->encoding == sensor_msgs::image_encodings::TYPE_8UC4
      || msg->encoding == sensor_msgs::image_encodings::TYPE_8SC4
      || msg->encoding == sensor_msgs::image_encodings::BGRA8)
  {
    format = Ogre::PF_BYTE_BGRA;
  }
  else if (msg->encoding == sensor_msgs::image_encodings::TYPE_8UC3
      || msg->encoding == sensor_msgs::image_encodings::TYPE_8SC3
      || msg->encoding == sensor_msgs::image_encodings::BGR8)
  {
    format = Ogre::PF_BYTE_BGR;
  }
  else if (msg->encoding == sensor_msgs::image_encodings::TYPE_8UC1
      || msg->encoding == sensor_msgs::image_encodings::TYPE_8SC1
      || msg->encoding == sensor_msgs::image_encodings::MONO8)
  {
    format = Ogre::PF_BYTE_L;
  }
  else
  {
    // throw UnsupportedImageEncoding(image.image.encoding);
    std::cerr << "unsupported encoding" << std::endl;
    return;
  }

  pixel_stream.bind(new Ogre::MemoryDataStream((uint8_t *) &(msg->data[0]), numBytes));
  Ogre::TexturePtr texture_local = Ogre::TextureManager::getSingleton()
      .loadRawData(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                   pixel_stream, msg->width, msg->height, format, Ogre::TEX_TYPE_2D, 0);

  if (!image_status_set)
  {
    setStatus(rviz::StatusProperty::Ok, "Image", "Image OK");
  }

  Ogre::Pass *pass = material_local->getTechnique(0)->getPass(0);

  Ogre::TextureUnitState *tex_unit = NULL;
  if (pass->getNumTextureUnitStates() > 0)
  {
    tex_unit = pass->getTextureUnitState(0);
  }
  else
  {
    tex_unit = pass->createTextureUnitState();
  }

  tex_unit->setTextureName(texture_local->getName());
  tex_unit->setTextureFiltering(Ogre::TFO_NONE);

  std::string text2 = "manual_object_local_" + ss.str();
  Ogre::ManualObject *manual_object_local = scene_manager_->createManualObject(text2);

  scene_node_->attachObject(manual_object_local);
  manual_object_local->begin(material_local->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

  geographic_msgs::GeoPose p;

  p.position.latitude = image_vec.back()->gpsdata.latitude;
  p.position.altitude = image_vec.back()->gpsdata.altitude;
  p.position.longitude = image_vec.back()->gpsdata.longitude;
  p.orientation.x = 0;
  p.orientation.y = 0;
  p.orientation.z = 0;
  p.orientation.w = 1;

  geodesy::UTMPoint position_umt;
  geodesy::convert(p.position, position_umt);

  //Berechnung der Relativposition zwischen Basesstatioin und Image

  double x = (position_umt.easting - transform.getOrigin().getX()) * 1.0;//position_umt.easting;
  double y = (position_umt.northing - transform.getOrigin().getY()) * 1.0;//position_umt.northing;
  int z = 0;
  double w = msg->width;
  double h = msg->height;

  double resolution = image_vec.back()->scale;

  double wr = resolution * w;
  double hr = resolution * h;

  // NOTE: Coordinates were chosen to be 0.01 and 0.99 because in RVIZ 0 and 1 resulted in a weird 1 pixel frame around
  //       the displayed image.

  // left top
  manual_object_local->position(Ogre::Vector3(x - wr / 2.0, y + hr / 2.0, z)); //(x, y, z));
  manual_object_local->textureCoord(0.01, 0.01);

  // right top
  manual_object_local->position(Ogre::Vector3(x + wr / 2.0, y + hr / 2.0, z)); //(x + wr, y, z));
  manual_object_local->textureCoord(0.99, 0.01);

  // right bottom
  manual_object_local->position(Ogre::Vector3(x + wr / 2.0, y - hr / 2.0, z)); //(x + wr, y - hr, z));
  manual_object_local->textureCoord(0.99, 0.99);

  // left bottom
  manual_object_local->position(Ogre::Vector3(x - wr / 2.0, y - hr / 2.0, z)); //(x, y - hr, z));
  manual_object_local->textureCoord(0.01, 0.99);

  manual_object_local->triangle(2, 1, 0);
  manual_object_local->triangle(3, 2, 0);

  manual_object_local->end();

  {
    //float alpha = alpha_property_->getFloat();

    //if (alpha < 0.9998) {
    material_local->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material_local->setDepthWriteEnabled(false);
    //} else {
    //	material_local->setSceneBlending(Ogre::SBT_REPLACE);
    //    material_local->setDepthWriteEnabled(false);
    //}

    AlphaSetter alpha_setter(0);
    if (manual_object_local)
    {
      manual_object_local->visitRenderables(&alpha_setter);
    }

  }

  //for(int i = 0; i < image_vec.size();i++)
  for (int i = 0; i < mcounter; i++)
  {
    std::stringstream ss;
    ss << i + 1;
    std::string text2 = "manual_object_local_" + ss.str();
    scene_manager_->getManualObject(text2)->setRenderQueueGroup(Ogre::RENDER_QUEUE_8);
  }
  manual_object_local->setRenderQueueGroup(Ogre::RENDER_QUEUE_9);

  context_->queueRender();

  mcounter++;
}

void GroundImageDisplay::onInitialize()
{
}

void GroundImageDisplay::onEnable()
{
  subscribe();
}

void GroundImageDisplay::onDisable()
{
  unsubscribe();
  clear();
}

void GroundImageDisplay::subscribe()
{
  if (!isEnabled())
  {
    return;
  }

  if (!topic_property_->getTopic().isEmpty())
  {
    try
    {
      image_sub_ = update_nh_.subscribe(topic_property_->getTopicStd(), 1, &GroundImageDisplay::incomingImage, this);
      setStatus(rviz::StatusProperty::Ok, "Topic", "OK");
    } catch (ros::Exception &e)
    {
      setStatus(rviz::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
    }
  }
}

void GroundImageDisplay::unsubscribe()
{
  image_vec.clear();
  image_sub_.shutdown();
}

void GroundImageDisplay::updateTopic()
{
  unsubscribe();
  subscribe();
  clear();
}

void GroundImageDisplay::clear()
{
  setStatus(rviz::StatusProperty::Warn, "Message", "No image received");
}

void GroundImageDisplay::incomingImage(const realm_msgs::GroundImageCompressed::ConstPtr &msg)
{
  //ROS_INFO_STREAM("incomingImage: " << msg->imagedata.data.size());
  image_vec
      .push_back(msg); //eventuell kann der Speicher wieder freigegeben werden nach add_new_image (noch nicht getestet)
  add_new_Image();
}

void GroundImageDisplay::transformImage()
{

  if (!loaded_)
  {
    return;
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

}

void GroundImageDisplay::fixedFrameChanged()
{
  transformImage();
}

void GroundImageDisplay::reset()
{
  Display::reset();

  clear();
  // Force resubscription so that the image will be re-sent
  updateTopic();
}

void GroundImageDisplay::setTopic(const QString &topic, const QString &datatype)
{
  topic_property_->setString(topic);
}

void GroundImageDisplay::update(float wall_dt, float ros_dt)
{
  transformImage();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(realm_rviz_plugin::GroundImageDisplay, rviz::Display)