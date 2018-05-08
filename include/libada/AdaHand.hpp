#ifndef LIBADA_ADAHAND_HPP_
#define LIBADA_ADAHAND_HPP_

#include "libada/AdaHandKinematicSimulationPositionCommandExecutor.hpp"
#include <Eigen/Core>
#include <aikido/common/RNG.hpp>
#include <aikido/common/pointers.hpp>
#include <aikido/control/PositionCommandExecutor.hpp>
#include <aikido/control/TrajectoryExecutor.hpp>
#include <aikido/robot/GrabMetadata.hpp>
#include <aikido/robot/Hand.hpp>
#include <aikido/robot/Robot.hpp>
#include <boost/optional.hpp>
#include <dart/dart.hpp>
#include <memory>
#include <ros/ros.h>
#include <unordered_map>

namespace ada {

/// URI to retrieve preshapes from.
/// Currently specified as a \c package://libada URI.
extern const dart::common::Uri preshapesUri;

/// URI to retrieve TSR end-effector transforms from.
/// Currently specified as a \c package://libada URI.
extern const dart::common::Uri tsrEndEffectorTransformsUri;

AIKIDO_DECLARE_POINTERS(AdaHand)

class AdaHand : public aikido::robot::Hand {
public:
  const double rosTrajectoryInterpolationTimestep = 0.1;
  const double rosTrajectoryGoalTimeTolerance = 5.0;

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
  /// \param[in] name Name of the hand, either "left" or "right" (ADA has one
  /// hand).
  /// \param[in] simulation True if running in simulation mode
  /// \param[in] handBaseBodyNode Body node which is the root of all fingers.
  /// \param[in] endEffectorBodyNode End-effector body node. Must be the link
  ///            that represents the palm of an AdaHand (i.e. \c hand_base),
  ///            for which inverse kinematics is solved.
  /// \param[in] selfCollisionFilter CollisionFilter used for self-collision
  ///            checking for the whole robot.
  /// \param[in] node ROS node. Required for running in real.
  ///            May be nullptr if simulation is true
  /// \param[in] retriever Resource retriever for retrieving preshapes and
  ///            end-effector transforms, specified as \c package://libada
  ///            URIs.
  AdaHand(const std::string &name, bool simulation,
          dart::dynamics::BodyNodePtr handBaseBodyNode,
          dart::dynamics::BodyNodePtr endEffectorBodyNode,
          std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
              selfCollisionFilter,
          const ::ros::NodeHandle *node,
          const dart::common::ResourceRetrieverPtr &retriever);

  virtual ~AdaHand() = default;

  // Documentation inherited.
  void grab(const dart::dynamics::SkeletonPtr &bodyToGrab) override;

  // Documentation inherited.
  void ungrab() override;

  // Documentation inherited.
  std::future<void> executePreshape(const std::string &preshapeName) override;

  // Documentation inherited.
  void step(const std::chrono::system_clock::time_point &timepoint) override;

  // Documentation inherited.
  dart::dynamics::ConstMetaSkeletonPtr getMetaSkeleton() const override;

  // Documentation inherited.
  dart::dynamics::MetaSkeletonPtr getMetaSkeleton() override;

  /// Get the end-effector body node for which IK can be created.
  /// \return DART body node of end-effector
  dart::dynamics::BodyNode *getEndEffectorBodyNode() const override;

  /// Get the body node which is the root of the hand, containing
  /// all fingers.
  /// \return DART body node at the root of the hand
  dart::dynamics::BodyNode *getHandBaseBodyNode() const override;

  /// Load preshapes from a YAML file.
  ///
  /// \param[in] preshapesUri Preshapes in YAML file
  /// \param[in] retriever Resource retriever to resolve URIs
  void loadPreshapes(const dart::common::Uri &preshapesUri,
                     const dart::common::ResourceRetrieverPtr &retriever);

  /// Load TSR transforms for an end-effector from a YAML file.
  ///
  /// \param[in] tsrTransformsUri TSR transforms in YAML file
  /// \param[in] retriever Resource retriever to resolve URIs
  void loadTSRTransforms(const dart::common::Uri &tsrTransformsUri,
                         const dart::common::ResourceRetrieverPtr &retriever);

  boost::optional<Eigen::Isometry3d>
  getEndEffectorTransform(const std::string &objectType) const;

private:
  /// Schema description for preshapes YAML file.
  ///
  /// Maps a preshape name (string) to a configuration
  using PreshapeMap = std::unordered_map<std::string, Eigen::VectorXd>;

  /// Schema description for end-effector transform YAML file.
  ///
  /// Maps an end-effector transform name (string) to a transform.
  using EndEffectorTransformMap =
      std::unordered_map<std::string, Eigen::Isometry3d, std::hash<std::string>,
                         std::equal_to<std::string>,
                         Eigen::aligned_allocator<
                             std::pair<const std::string, Eigen::Isometry3d>>>;

  /// Create controllers in real or simulation.
  ///
  /// \param[in] robot Robot to construct executor for
  std::shared_ptr<aikido::control::TrajectoryExecutor>
  createTrajectoryExecutor(const dart::dynamics::SkeletonPtr &robot);

  /// Create a controller for simulated hand movement.
  ///
  /// \param[in] robot Robot to construct executor for
  std::shared_ptr<aikido::control::PositionCommandExecutor>
  createSimPositionCommandExecutor(const dart::dynamics::SkeletonPtr &robot);

  /// Loads preshapes from YAML file (retrieved from \c preshapesUri).
  ///
  /// \param[in] node The YAML node to read from
  PreshapeMap parseYAMLToPreshapes(const YAML::Node &node);

  /// Loads end-effector transforms from YAML file (retrieved from
  /// \c tsrEndEffectorTransformsUri).
  ///
  /// \param[in] node The YAML node to read from
  EndEffectorTransformMap
  parseYAMLToEndEffectorTransforms(const YAML::Node &node);

  /// Returns the corresponding preshape (from \c preshapesUri).
  ///
  /// \param[in] preshapeName Name of preshape (e.g. "open")
  /// \return preshape if it exists, boost::none if not
  boost::optional<Eigen::VectorXd> getPreshape(const std::string &preshapeName);

  /// Name of the hand, either "left" or "right"
  const std::string mName;

  /// Parent robot
  aikido::robot::RobotPtr mRobot;

  /// Hand metaskeleton consisting of the nodes rooted at \c
  /// mHandBodyNode
  dart::dynamics::GroupPtr mHand;
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mSpace;

  /// Whether hand is running in simulation mode
  const bool mSimulation;

  /// Body node which is the root link for all fingers
  dart::dynamics::BodyNodePtr mHandBaseBodyNode;

  /// End-effector body node, for which IK is created
  dart::dynamics::BodyNodePtr mEndEffectorBodyNode;

  /// CollisionFilter used for self-collision checking for the whole robot
  std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
      mSelfCollisionFilter;

  /// Hand position command executor (may be real if simulation is False)
  std::shared_ptr<aikido::control::TrajectoryExecutor> mExecutor;

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
