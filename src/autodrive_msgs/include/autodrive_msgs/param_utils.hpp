#pragma once
// Redirect: migrated to fsd_common. This header will be removed in a future release.
#include <fsd_common/param_utils.hpp>
// Re-export into old namespace for backward compatibility
namespace autodrive_msgs {
  using fsd_common::LoadParam;
  using fsd_common::LoadVector;
}