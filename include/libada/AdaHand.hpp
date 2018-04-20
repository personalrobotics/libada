#ifndef LIBADA_ADAHAND_HPP_
#define LIBADA_ADAHAND_HPP_

#include <memory>
#include <unordered_map>
#include <Eigen/Core>
#include <aikido/common/pointers.hpp>
#include <aikido/control/PositionCommandExecutor.hpp>
#include <aikido/robot/GrabMetadata.hpp>
#include <aikido/robot/Hand.hpp>
#include <boost/optional.hpp>
#include <dart/dart.hpp>
#include <ros/ros.h>
#include "libada/AdaHandKinematicSimulationPositionCommandExecutor.hpp"

namespace ada {

/// URI to retrieve preshapes from.
/// Currently specified as a \c package://libada URI.
extern const dart::common::Uri preshapesUri;

/// URI to retrieve TSR end-effector transforms from.
/// Currently specified as a \c package://libada URI.
extern const dart::common::Uri tsrEndEffectorTransformsUri;

AIKIDO_DECLARE_POINTERS(AdaHand)

class AdaHand : public aikido::robot::Hand
{
public:
  /// Creates an instance of an AdaHand.
  ///
  /// When simulation is false, uses \c RosPositionCommandExecutor to send
  /// commands to an \c actionlib server in the namespace \c
  /// <name>_hand_controller/set_position.
  ///
  /// When simulation is true, this class uses \c
  /// AdaHandKinematicSimulationPositionCommandExecutor to simulate a
  /// AdaHand consisting of the \c BodyNodes named \c
  /// /<name>/finger<finger_index>_<joint_index> where \c finger_index is the
  /// zero-indexed finger and \c joint_index is the zero-indexed phalange within
  /// the finger.
  ///
  /// \param[in] name Name of the hand, either "left" or "right" (ADA has one hand). 
  /// \param[in] simulation True if running in simulation mode
  /// \param[in] endEffectorBodyNode End-effector body node. Must be the link
  ///            that represents the palm of an AdaHand (i.e. \c hand_base).
  /// \param[in] selfCollisionFilter CollisionFilter used for self-collision
  ///            checking for the whole robot.
  /// \param[in] node ROS node. Required for running in real.
  ///            May be nullptr if simulation is true
  /// \param[in] retriever Resource retriever for retrieving preshapes and
  ///            end-effector transforms, specified as \c package://libada
  ///            URIs.
  AdaHand(
      const std::string& name,
      bool simulation,
      dart::dynamics::BodyNodePtr endEffectorBodyNode,
      std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
          selfCollisionFilter,
      const ::ros::NodeHandle* node,
      const dart::common::ResourceRetrieverPtr& retriever);

  virtual ~AdaHand() = default;

  // Documentation inherited.
  virtual void grab(const dart::dynamics::SkeletonPtr& bodyToGrab) override;

  // Documentation inherited.
  virtual void ungrab() override;

  // Documentation inherited.
  virtual std::future<void> executePreshape(
      const std::string& preshapeName) override;

  // Documentation inherited.
  virtual void step(
      const std::chrono::system_clock::time_point& timepoint) override;

  // Documentation inherited.
  virtual dart::dynamics::ConstMetaSkeletonPtr getMetaSkeleton() const override;

  // Documentation inherited.
  virtual dart::dynamics::MetaSkeletonPtr getMetaSkeleton() override;

  // Documentation inherited.
  virtual dart::dynamics::BodyNode* getBodyNode() const override;

  /// Load preshapes from a YAML file.
  ///
  /// \param[in] preshapesUri Preshapes in YAML file
  /// \param[in] retriever Resource retriever to resolve URIs
  void loadPreshapes(
      const dart::common::Uri& preshapesUri,
      const dart::common::ResourceRetrieverPtr& retriever);

  /// Load TSR transforms for an end-effector from a YAML file.
  ///
  /// \param[in] tsrTransformsUri TSR transforms in YAML file
  /// \param[in] retriever Resource retriever to resolve URIs
  void loadTSRTransforms(
      const dart::common::Uri& tsrTransformsUri,
      const dart::common::ResourceRetrieverPtr& retriever);
 
  boost::optional<Eigen::Isometry3d> getEndEffectorTransform(
          const std::string& objectType) const;


private:
  /// Schema description for preshapes YAML file.
  ///
  /// Maps a preshape name (string) to a configuration
  using PreshapeMap = std::unordered_map<std::string, Eigen::VectorXd>;

  /// Schema description for end-effector transform YAML file.
  ///
  /// Maps an end-effector transform name (string) to a transform.
  using EndEffectorTransformMap = std::
      unordered_map<std::string,
                    Eigen::Isometry3d,
                    std::hash<std::string>,
                    std::equal_to<std::string>,
                    Eigen::aligned_allocator<std::pair<const std::string,
                                                       Eigen::Isometry3d>>>;

  /// Create controllers in real or simulation.
  ///
  /// \param[in] robot Robot to construct executor for
  std::shared_ptr<aikido::control::PositionCommandExecutor>
  createAdaHandPositionExecutor(const dart::dynamics::SkeletonPtr& robot);

  /// Loads preshapes from YAML file (retrieved from \c preshapesUri).
  ///
  /// \param[in] node The YAML node to read from
  PreshapeMap parseYAMLToPreshapes(const YAML::Node& node);

  /// Loads end-effector transforms from YAML file (retrieved from
  /// \c tsrEndEffectorTransformsUri).
  ///
  /// \param[in] node The YAML node to read from
  EndEffectorTransformMap parseYAMLToEndEffectorTransforms(
      const YAML::Node& node);

  /// Returns the corresponding preshape (from \c preshapesUri).
  ///
  /// \param[in] preshapeName Name of preshape (e.g. "open")
  /// \return preshape if it exists, boost::none if not
  boost::optional<Eigen::VectorXd> getPreshape(const std::string& preshapeName);

  /// Name of the hand, either "left" or "right"
  const std::string mName;

  /// Hand MetaSkeleton consisting of the nodes rooted at \c
  /// mEndEffectorBodyNode
  dart::dynamics::BranchPtr mHand;

  /// Whether hand is running in simulation mode
  const bool mSimulation;

  /// End-effector body node
  dart::dynamics::BodyNodePtr mEndEffectorBodyNode;

  /// CollisionFilter used for self-collision checking for the whole robot
  std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
      mSelfCollisionFilter;

  /// Hand position command executor (may be real if simulation is False)
  std::shared_ptr<aikido::control::PositionCommandExecutor> mExecutor;

  /// Hand position command executor (always in simulation)
  std::shared_ptr<aikido::control::PositionCommandExecutor> mSimExecutor;

  /// Maps a preshape name (string) to a configuration
  PreshapeMap mPreshapeConfigurations;

  /// Maps an end-effector transform name (string) to a transform.
  EndEffectorTransformMap mEndEffectorTransforms;

  /// Metadata about the object currently being grabbed
  // TODO: change this to grab multiple objects
  std::unique_ptr<aikido::robot::GrabMetadata> mGrabMetadata;

  /// ROS node. Required for running in real.
  std::unique_ptr<::ros::NodeHandle> mNode;

  /// Maps a finger joint name (string) to its index
  static const std::unordered_map<std::string, size_t>
      adaFingerJointNameToPositionIndexMap;
};

} // namespace ada

#endif
