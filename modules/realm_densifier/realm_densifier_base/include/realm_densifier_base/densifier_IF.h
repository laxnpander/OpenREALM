

#ifndef PROJECT_DENSIFIER_IF_H
#define PROJECT_DENSIFIER_IF_H

#include <iostream>
#include <memory>
#include <deque>

#include <opencv2/core.hpp>

#include <realm_core/loguru.h>
#include <realm_core/frame.h>
#include <realm_core/depthmap.h>

namespace realm
{

class DensifierIF
{
  public:
    using Ptr = std::shared_ptr<DensifierIF>;
    using ConstPtr = std::shared_ptr<const DensifierIF>;
  public:
    /*!
     * @brief Basic densification function to process multiple frames
     * @param frames vector of frames used for SFM, for stereo implementations should be two
     * @param ref_idx index of vector element that should be used as reference -> output disparity map will be defined
     * using reference image
     * @return disparity ma
     */
    virtual Depthmap::Ptr densify(const std::deque<Frame::Ptr> &frames, uint8_t ref_idx) = 0;

    /*!
     * @brief Getter for nrof frames for SFM
     * @return number of frames to be used for densification, for stereo implementations usually two
     */
    virtual uint8_t getNrofInputFrames() = 0;

    /*!
     * @brief Getter for resize factor of densifier implementation
     * @return Factor applied to image width and height for size reduction
     */
    virtual double getResizeFactor() = 0;

    /*!
     * @brief Function to print densifier settings to the log. Must be implemented by the derived class.
     */
    virtual void printSettingsToLog() = 0;
  private:

};

} // namespace realm

#endif //PROJECT_DENSIFIER_IF_H
