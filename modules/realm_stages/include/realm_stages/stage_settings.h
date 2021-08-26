

#ifndef PROJECT_STAGE_SETTINGS_H
#define PROJECT_STAGE_SETTINGS_H

#include <realm_core/settings_base.h>

namespace realm
{

class StageSettings : public SettingsBase
{
  public:
    using Ptr = std::shared_ptr<StageSettings>;
    using ConstPtr = std::shared_ptr<const StageSettings>;
  public:
    StageSettings()
    {
      add("type", Parameter_t<std::string>{"", "Stage type, e.g. pose_estimation, densification, ..."});
      add("queue_size", Parameter_t<int>{5, "Size of the measurement input queue, implemented as ringbuffer"});
      add("path_output", Parameter_t<std::string>{"", "Path to output folder."});
      add("log_to_file", Parameter_t<int>{1, "Write log to stage directory"});
    }
};

class PoseEstimationSettings : public StageSettings
{
  public:
    PoseEstimationSettings()
    {
      add("use_vslam", Parameter_t<int>{0, "Flag can be set to 'false', then this stage only works as image stream throttle depending on the max overlap."});
      add("use_imu", Parameter_t<int>{0, "Flag to activate usage of IMU. Settings file for IMU parameters must be provided in the profile."});
      add("set_all_frames_keyframes", Parameter_t<int>{0, "Flag to set all tracked frames to being keyframes."});
      add("fallback_strategy", Parameter_t<int>{0, "Strategy when to use the projection only fallback: 1 - Always, 2 - once orientation is calibrated, 3 - never"});
      add("use_initial_guess", Parameter_t<int>{0, "Flag can be set to 'false', then the initial guess of the pose is not being considered in the visual SLAM."});
      add("update_georef", Parameter_t<int>{0, "Flag can be set to 'false', then georeference will only be computed at initialization."});
      add("do_delay_keyframes", Parameter_t<int>{0, "Flag to delay publishing of keyframes by the duration it takes for the georeference to initialize."
                                                                           "This ensures higher pose and map point quality, as the frames are refined with consecutive frames."});
      add("suppress_outdated_pose_pub", Parameter_t<int>{0, "Flag to suppress publish of outdated poses after georeference computation."});
      add("th_error_georef", Parameter_t<double>{1.0, "Threshold of error for georeference until initialization is performed."});
      add("init_lost_frames_reset_count", Parameter_t<int>{15, "The number of lost frames allowed during georeferencing before a VSLAM reset is issued."});
      add("min_nrof_frames_georef", Parameter_t<int>{5, "Minimum number of unique frames required before georeference is initialized."});
      add("overlap_max", Parameter_t<double>{0.0, "Maximum overlap for all publishes, even keyframes"});
      add("overlap_max_fallback", Parameter_t<double>{0.0, "Maximum overlap for fallback publishes, e.g. GNSS only imgs"});
      add("save_trajectory_gnss", Parameter_t<int>{0, "Save gnss trajectory of receiver"});
      add("save_trajectory_visual", Parameter_t<int>{0, "Save visual camera trajectory"});
      add("save_frames", Parameter_t<int>{0, "Save all processed frames"});
      add("save_keyframes", Parameter_t<int>{0, "Save all processed keyframes in working resolution"});
      add("save_keyframes_full", Parameter_t<int>{0, "Save all processed keyframes in full resolution"});
    }
};

class DensificationSettings : public StageSettings
{
  public:
    DensificationSettings()
    {
      add("do_drop_planar", Parameter_t<int>{0, "Flag to drop frames that can not be stereo reconstructed."});
      add("use_filter_bilat", Parameter_t<int>{0, "Flag to use bilateral filter for disparity map."});
      add("use_filter_guided", Parameter_t<int>{0, "Flag to use guided filter. Only possible with stereo reconstruction."});
      add("compute_normals", Parameter_t<int>{0, "Flag to compute surface normals from disparity map."});
      add("save_bilat", Parameter_t<int>{0, "Save disparity map after bilateral filtering (if processed)"});
      add("save_dense", Parameter_t<int>{0, "Save map produced by stereo reconstruction (if processed)"});
      add("save_guided", Parameter_t<int>{0, "Save disparity map after guided filtering (if processed)"});
      add("save_imgs", Parameter_t<int>{0, "Save processed, resized images"});
      add("save_sparse", Parameter_t<int>{0, "Save sparse disparity map produced through inpainting (if processed)"});
      add("save_thumb", Parameter_t<int>{0, "Save sparse disparity map before inpainting (if processed)"});
      add("save_normals", Parameter_t<int>{0, "Save surface normals as color map from disparity"});
    }
};

class SurfaceGenerationSettings : public StageSettings
{
  public:
    SurfaceGenerationSettings()
    {
      add("try_use_elevation", Parameter_t<int>{0, "Flag for trying to use surface points for elevation map generation"});
      add("compute_all_frames", Parameter_t<int>{0, "When in PLANAR mode, estimate the elevation of every frame using sparse data rather than locking on the first"});
      add("knn_max_iter", Parameter_t<int>{5, "Maximum number of iterations for each cell to find the closest 3D point in the dense cloud"});
      add("mode_surface_normals", Parameter_t<int>{0, "0 - None, 1 - Random neighbours, 2 - Furthest neighbours, 3 - Best-fit"});
      add("save_valid", Parameter_t<int>{0, "Save valid elevation grid element mask"});
      add("save_elevation", Parameter_t<int>{0, "Save elevation map as colored PNG image file"});
      add("save_normals", Parameter_t<int>{0, "Save surface normals as colored PNG image file"});
    }
};

class OrthoRectificationSettings : public StageSettings
{
  public:
    OrthoRectificationSettings()
    {
      add("GSD", Parameter_t<double>{0.0, "Ground sampling distance in [m/px]"});
      add("publish_pointcloud", Parameter_t<int>{0, "Publishing the point cloud requires additional resources for data conversion."});
      add("save_valid", Parameter_t<int>{0, "Save valid incremental map grid elements"});
      add("save_ortho_rgb", Parameter_t<int>{0, "Save incremental map ortho foto as PNG image file"});
      add("save_ortho_gtiff", Parameter_t<int>{0, "Save global map ortho foto as one GeoTIFF image file"});
      add("save_elevation", Parameter_t<int>{0, "Save incremental elevation map as PNG image file"});
      add("save_elevation_angle", Parameter_t<int>{0, "Save incremental elevation angle of map as PNG image file"});
    }
};

class MosaicingSettings : public StageSettings
{
  public:
    MosaicingSettings()
    {
      add("th_elevation_min_nobs", Parameter_t<int>{0, "Threshold for minimum number of observations for elevation point"});
      add("th_elevation_variance", Parameter_t<double>{0.0, "Threshold for elevation variance marking outlier"});
      add("publish_mesh_every_nth_kf", Parameter_t<int>{0, "Activate global map publish every n keyframes as mesh"});
      add("publish_mesh_at_finish", Parameter_t<int>{0, "Activate global map publish as mesh at finishCallback call"});
      add("downsample_publish_mesh", Parameter_t<double>{0.0, "Downsample published mesh to lower GSD for performance. Unit: [m/pix]"});
      add("split_gtiff_channels", Parameter_t<int>{0, "Splits all channels of an OpenCV matrix into separate tif files while saving"});
      add("save_valid", Parameter_t<int>{0, "Save valid global map grid elements"});
      add("save_ortho_rgb_one", Parameter_t<int>{0, "Save global map ortho foto as one PNG image file"});
      add("save_ortho_rgb_all", Parameter_t<int>{0, "Save global map ortho foto as incremental PNG image files"});
      add("save_ortho_gtiff_one", Parameter_t<int>{0, "Save global map ortho foto as one GeoTIFF image file"});
      add("save_ortho_gtiff_all", Parameter_t<int>{0, "Save global map ortho foto as incremental GeoTIFF image files"});
      add("save_elevation_one", Parameter_t<int>{0, "Save global elevation map as one PNG image file"});
      add("save_elevation_all", Parameter_t<int>{0, "Save global elevation map as incremental PNG image files"});
      add("save_elevation_var_one", Parameter_t<int>{0, "Save global standard deviation map as one PNG image file"});
      add("save_elevation_var_all", Parameter_t<int>{0, "Save global standard deviation map as incremental PNG image files"});
      add("save_elevation_obs_angle_one", Parameter_t<int>{0, "Save global map elevation observation angles as one PNG image file"});
      add("save_elevation_obs_angle_all", Parameter_t<int>{0, "Save global map elevation observation angles as incremental PNG image files"});
      add("save_elevation_mesh_one", Parameter_t<int>{0, "Save global map elevation mesh as .ply file."});
      add("save_num_obs_one", Parameter_t<int>{0, "Save global map number of observations per grid element as one PNG image file"});
      add("save_num_obs_all", Parameter_t<int>{0, "Save global map number of observations per grid element as incremental PNG image files"});
      add("save_dense_ply", Parameter_t<int>{0, "Save dense cloud as .ply file"});
    }
};

class TileingSettings : public StageSettings
{
public:
  TileingSettings()
  {

  }
};

} // namespace realm

#endif //PROJECT_STAGE_SETTINGS_H
