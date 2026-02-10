#pragma once

#include <localization_core/factor_graph_types.hpp>
#include <localization_core/types.hpp>

#include <vector>

namespace localization_core {

/// Descriptor-based relocalization using polar histograms of local landmarks.
///
/// Workflow:
///   1. BuildDescriptor() from local cone observations
///   2. AddToDatabase() to store keyframe descriptors
///   3. TryRelocalize() to find best match via cosine distance + SE(2) RANSAC
class DescriptorRelocator {
 public:
  explicit DescriptorRelocator(const RelocConfig& cfg = {});

  /// Build a polar histogram descriptor from local landmarks.
  /// @param landmarks  landmarks in global frame
  /// @param pose       current pose (used to transform to local frame)
  /// @return descriptor vector (n_rings * n_sectors * n_channels floats)
  std::vector<float> BuildDescriptor(
      const std::vector<FgLandmark>& landmarks,
      const Pose2& pose) const;

  /// Add a descriptor to the database.
  void AddToDatabase(const SubMapDescriptor& desc);

  /// Attempt relocalization against the database.
  /// @param current_desc  descriptor built from current observations
  /// @param current_landmarks  current local landmarks (for RANSAC)
  /// @param current_pose  current dead-reckoned pose
  /// @return RelocResult with success flag and relative transform
  RelocResult TryRelocalize(
      const std::vector<float>& current_desc,
      const std::vector<FgLandmark>& current_landmarks,
      const Pose2& current_pose) const;

  /// Number of descriptors in the database.
  size_t DatabaseSize() const { return database_.size(); }

  /// Clear the database.
  void ClearDatabase() { database_.clear(); }

 private:
  RelocConfig cfg_;
  std::vector<SubMapDescriptor> database_;

  int descriptorSize() const;
  double cosineDistance(const std::vector<float>& a,
                       const std::vector<float>& b) const;
};

}  // namespace localization_core
