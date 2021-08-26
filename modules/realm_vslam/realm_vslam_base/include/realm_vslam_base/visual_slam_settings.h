

#ifndef PROJECT_VISUAL_SLAM_SETTINGS_H
#define PROJECT_VISUAL_SLAM_SETTINGS_H

#include <realm_core/settings_base.h>

namespace realm
{

class VisualSlamSettings : public SettingsBase
{
  public:
    using Ptr = std::shared_ptr<VisualSlamSettings>;
    using ConstPtr = std::shared_ptr<const VisualSlamSettings>;
  public:
    VisualSlamSettings()
    {
      add("type", Parameter_t<std::string>{"", "Framework for visual SLAM."});
      add("resizing", Parameter_t<double>{0.0, "Resize factor of input images."});
    }
};

class OrbSlamSettings : public VisualSlamSettings
{
  public:
    OrbSlamSettings()
    {
      add("nrof_features", Parameter_t<int>{0, "ORB Extractor: Number of features per image."});
      add("scale_factor", Parameter_t<double>{0, "ORB Extractor: Scale factor between levels in the scale pyramid."});
      add("n_pyr_levels", Parameter_t<int>{0, "ORB Extractor: Number of levels in the scale pyramid."});
      add("ini_th_FAST", Parameter_t<int>{0, "Initial FAST features threshold for detection."});
      add("min_th_FAST", Parameter_t<int>{0, "Minimum response of FAST features."});
      add("path_vocabulary", Parameter_t<std::string>{"", "Path to ORB_SLAM2 vocabulary file."});
    }
};

class SvoSettings : public VisualSlamSettings
{
  public:
    SvoSettings()
    {
    }
};

class Svo2Settings : public VisualSlamSettings
{
  public:
    Svo2Settings()
    {
    }
};

class DsoSettings : public VisualSlamSettings
{
  public:
    DsoSettings()
    {
      // TODO: Add help
      add("desiredImmatureDensity", Parameter_t<double>{0, ""});
      add("desiredPointDensity", Parameter_t<double>{0, ""});
      add("minFrames", Parameter_t<int>{0, ""});
      add("maxFrames", Parameter_t<int>{0, ""});
      add("maxOptIterations", Parameter_t<int>{0, ""});
      add("minOptIterations", Parameter_t<int>{0, ""});
      add("logStuff", Parameter_t<int>{0, ""});
      add("kfGlobalWeight", Parameter_t<double>{0, ""});
      add("photometricCalibration", Parameter_t<int>{0, ""});
      add("affineOptModeA", Parameter_t<double>{0, ""});
      add("affineOptModeB", Parameter_t<double>{0, ""});
    }
};

class OpenVslamSettings : public VisualSlamSettings
{
public:
  OpenVslamSettings()
  {
    add("nrof_features", Parameter_t<int>{0, "ORB Extractor: Number of features per image."});
    add("scale_factor", Parameter_t<double>{0, "ORB Extractor: Scale factor between levels in the scale pyramid."});
    add("n_pyr_levels", Parameter_t<int>{0, "ORB Extractor: Number of levels in the scale pyramid."});
    add("ini_th_FAST", Parameter_t<int>{0, "Initial FAST features threshold for detection."});
    add("min_th_FAST", Parameter_t<int>{0, "Minimum response of FAST features."});
    add("path_vocabulary", Parameter_t<std::string>{"", "Path to ORB_SLAM2 vocabulary file."});
  }
};

class Ov2SlamSettings : public VisualSlamSettings
{
public:
  Ov2SlamSettings()
  {
    add("force_realtime", Parameter_t<int>{0, ""});
    add("buse_loop_closer", Parameter_t<int>{0, ""});
    add("use_shi_tomasi", Parameter_t<int>{0, ""});
    add("use_brief", Parameter_t<int>{0, ""});
    add("use_fast", Parameter_t<int>{0, ""});
    add("use_singlescale_detector", Parameter_t<int>{0, ""});
    add("nmax_dist", Parameter_t<int>{0, ""});
    add("nfast_th", Parameter_t<int>{0, ""});
    add("use_clahe", Parameter_t<int>{0, ""});
    add("do_klt", Parameter_t<int>{0, ""});
    add("klt_use_prior", Parameter_t<int>{0, ""});
    add("btrack_keyframetoframe", Parameter_t<int>{0, ""});
    add("nklt_win_size", Parameter_t<int>{0, ""});
    add("nklt_pyr_lvl", Parameter_t<int>{0, ""});
    add("nmax_iter", Parameter_t<int>{0, ""});
    add("nklt_err", Parameter_t<int>{0, ""});
    add("bdo_track_localmap", Parameter_t<int>{0, ""});
    add("do_epipolar", Parameter_t<int>{0, ""});
    add("do_p3p", Parameter_t<int>{0, ""});
    add("do_random", Parameter_t<int>{0, ""});
    add("nransac_iter", Parameter_t<int>{0, ""});
    add("use_sparse_schur", Parameter_t<int>{0, ""});
    add("use_dogleg", Parameter_t<int>{0, ""});
    add("use_subspace_dogleg", Parameter_t<int>{0, ""});
    add("use_nonmonotic_step", Parameter_t<int>{0, ""});
    add("apply_l2_after_robust", Parameter_t<int>{0, ""});
    add("nmin_covscore", Parameter_t<int>{0, ""});
    add("do_full_ba", Parameter_t<int>{0, ""});
    add("alpha", Parameter_t<double>{0.0, ""});
    add("dmax_quality", Parameter_t<double>{0.0, ""});
    add("finit_parallax", Parameter_t<double>{0.0, ""});
    add("fclahe_val", Parameter_t<double>{0.0, ""});
    add("fmax_px_precision", Parameter_t<double>{0.0, ""});
    add("fmax_fbklt_dist", Parameter_t<double>{0.0, ""});
    add("fmax_desc_dist", Parameter_t<double>{0.0, ""});
    add("fmax_proj_pxdist", Parameter_t<double>{0.0, ""});
    add("fmax_reproj_err", Parameter_t<double>{0.0, ""});
    add("robust_mono_th", Parameter_t<double>{0.0, ""});
    add("fkf_filtering_ratio", Parameter_t<double>{0.0, ""});
    add("fransac_err", Parameter_t<double>{0.0, ""});
  }
};

} // namespace realm

#endif //PROJECT_VISUAL_SLAM_SETTINGS_H
