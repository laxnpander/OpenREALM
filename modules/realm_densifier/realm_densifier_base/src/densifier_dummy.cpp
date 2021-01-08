

#include <realm_densifier_base/densifier_dummy.h>

using namespace realm;

densifier::Dummy::Dummy(const DensifierSettings::Ptr &settings)
: m_resizing((*settings)["resizing"].toDouble())
{

}

Depthmap::Ptr densifier::Dummy::densify(const std::deque<Frame::Ptr> &frames, uint8_t ref_idx)
{
  return nullptr;
}

uint8_t densifier::Dummy::getNrofInputFrames()
{
  return 0;
}

double densifier::Dummy::getResizeFactor()
{
  return m_resizing;
}

void densifier::Dummy::printSettingsToLog()
{
  LOG_F(INFO, "### Dummy settings ###");
  LOG_F(INFO, "- no settings");
}