

#include <realm_densifier_base/densifier_factory.h>

using namespace realm;

DensifierIF::Ptr densifier::DensifierFactory::create(const DensifierSettings::Ptr &settings)
{
  if ((*settings)["type"].toString() == "DUMMY")
    return std::make_shared<densifier::Dummy>(settings);
#ifdef USE_CUDA
  if ((*settings)["type"].toString() == "PSL")
    return std::make_shared<densifier::PlaneSweep>(settings);
#endif
  throw std::invalid_argument("Error: Densifier framework '" + (*settings)["type"].toString() + "' not found");
}