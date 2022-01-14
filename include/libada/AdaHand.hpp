#ifndef LIBADA_ADAHAND_HPP_
#define LIBADA_ADAHAND_HPP_

#include <memory>
#include <unordered_map>

#include <Eigen/Core>
#include <aikido/common/pointers.hpp>
#include <aikido/control/TrajectoryExecutor.hpp>
#include <aikido/robot/GrabMetadata.hpp>
#include <aikido/robot/Hand.hpp>
#include <aikido/robot/Robot.hpp>
#include <boost/optional.hpp>
#include <dart/dart.hpp>
#include <ros/ros.h>

#include "libada/Ada.hpp"

namespace ada {

class Ada::AdaHand : public aikido::robot::Hand
{
public:
  /// Creates an instance of an AdaHand.
  ///
  /// Wraps parent Ada object in Aikido's Hand interface.
  ///
  /// \param[in] ada Parent Ada object.
  /// \param[in] handBaseBodyNode Body node which is the root of all fingers.
  /// \param[in] endEffectorBodyNode End-effector body node. Must be the link
  ///            that represents the palm of an AdaHand (i.e. \c hand_base),
  ///            for which inverse kinematics is solved.
  AdaHand(
      Ada* ada,
      dart::dynamics::BodyNodePtr handBaseBodyNode,
      dart::dynamics::BodyNodePtr endEffectorBodyNode);

  virtual ~AdaHand() = default;

  // Documentation inherited.
  void grab(const dart::dynamics::SkeletonPtr& bodyToGrab) override;

  // Documentation inherited.
  void ungrab() override;

  // Documentation inherited.
  std::future<void> executePreshape(const std::string& preshapeName) override;

  /// Sets the hand to the given preshape configuration.
  /// \param[in] preshape Configuration of the hand.
  std::future<void> executePreshape(const Eigen::VectorXd& preshape);

  // Documentation inherited.
  void step(const std::chrono::system_clock::time_point& timepoint) override;

  // Documentation inherited.
  dart::dynamics::ConstMetaSkeletonPtr getMetaSkeleton() const override;

  // Documentation inherited.
  dart::dynamics::MetaSkeletonPtr getMetaSkeleton() override;

  // Documentation inherited.
  dart::dynamics::BodyNode* getEndEffectorBodyNode() const override;

  // Documentation inherited.
  dart::dynamics::BodyNode* getHandBaseBodyNode() const override;

private:
  /// Parent Ada Object
  Ada* mAda;

  /// Body node which is the root link for all fingers
  dart::dynamics::BodyNodePtr mHandBaseBodyNode;

  /// End-effector body node, for which IK is created
  dart::dynamics::BodyNodePtr mEndEffectorBodyNode;

  /// Metadata about the object currently being grabbed
  // TODO: change this to grab multiple objects
  std::unique_ptr<aikido::robot::GrabMetadata> mGrabMetadata;
};

} // namespace ada

#endif // LIBADA_ADAHAND_HPP_
