#pragma once
// Redirect: migrated to fsd_common. This header will be removed in a future release.
#include <fsd_common/topic_contract.hpp>
// Re-export into old namespace for backward compatibility
namespace autodrive_msgs {
  namespace topic_contract { using namespace fsd_common::topic_contract; }
  namespace frame_contract { using namespace fsd_common::frame_contract; }
}
