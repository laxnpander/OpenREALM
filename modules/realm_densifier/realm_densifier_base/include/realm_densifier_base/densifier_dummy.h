

#ifndef PROJECT_DENSIFIER_DUMMY_H
#define PROJECT_DENSIFIER_DUMMY_H

#include <iostream>

#include <realm_densifier_base/densifier_IF.h>
#include <realm_densifier_base/densifier_settings.h>

namespace realm
{
namespace densifier
{
/*!
 * @brief This class does nothing, can nothing and is only there to be the dummy.
 */
class Dummy : public DensifierIF
{
  public:
    /*!
     * @brief Explicit one argument constructor for densifier factory
     * @param settings Settings file with type: dummy and resizing of your choice
     */
    explicit Dummy(const DensifierSettings::Ptr &settings);

    /*!
     * @brief Overriden core function for densification of input frames. Should be provided with the correct number of
     *        frames and the id for which one the depth map should be computed.
     * @param frames Vector of frames to be densified
     * @param ref_idx Index of param frames for that the depth map should be computed
     * @return Depth map for reference frames[ref_idx]
     */
    Depthmap::Ptr densify(const std::deque<Frame::Ptr> &frames, uint8_t ref_idx) override;

    /*!
     * @brief Overriden getter for number of input frames. Dummy densifies zero frames
     * @return Size of frame input vector
     */
    uint8_t getNrofInputFrames() override;

    /*!
     * @brief Overriden getter for resize factor of the framework. Will typically be set in the settings file.
     * @return Resize factor for input images. Usually full resolution images are too big to be densified
     */
    double getResizeFactor() override;

    /*!
     * @brief Overriden base function to print settings to log.
     */
    void printSettingsToLog() override;
  private:
    double m_resizing;
};

} // namespace densifier
} // namespace realm

#endif //PROJECT_DENSIFIER_DUMMY_H
