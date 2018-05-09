#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <gtest/gtest.h>
#include <libada/Ada.hpp>

using namespace dart;

TEST(IkFast, TestIkSolver)
{
  // TODO : Verify if IK is working
  // 1. Select a set of angles A.
  // 2. Call the FK function from ikfast cpp -> gives SE(3) pose.
  // 3. Feed that into IK solver and see if one of solution matches initial set
  // A.
}

//==============================================================================
TEST(IkFast, VerifyGeneratedAdaIkFast)
{
  utils::DartLoader urdfParser;
  auto retreiver = std::make_shared<aikido::io::CatkinResourceRetriever>();
  auto ada = urdfParser.parseSkeleton(
      "package://ada_description/robots/ada_with_camera_forque.urdf",
      retreiver);
  EXPECT_NE(ada, nullptr);

  auto eeBodyNode = ada->getBodyNode("j2n6s200_end_effector");
  auto ee = eeBodyNode->createEndEffector("ee");
  auto ik = ee->createIK();
  auto targetFrame
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  targetFrame->setRotation(Eigen::Matrix3d::Identity());

  ik->setTarget(targetFrame);
  ik->setHierarchyLevel(1);

  std::string libName = "devel/lib/libadaIk";
#if DART_OS_LINUX
  libName += ".so";
#elif DART_OS_MACOS
  libName += ".dylib";
#elif DART_OS_WINDOWS
  libName += ".dll";
#endif
  std::vector<std::size_t> ikFastDofs{0, 1, 2, 3, 4, 5};

  ik->setGradientMethod<dynamics::SharedLibraryIkFast>(
      libName, ikFastDofs, std::vector<std::size_t>());
  auto analytical = ik->getAnalytical();
  EXPECT_NE(analytical, nullptr);
  EXPECT_EQ(analytical->getDofs().size(), 6);

  auto ikfast = dynamic_cast<dynamics::SharedLibraryIkFast*>(analytical);
  EXPECT_NE(ikfast, nullptr);

  // TODO
  // 1. Set a specific TSR in our script
  // 2. Use that to solve for IK using DART and ikfast
  // 3. See if solution is "close" to atleast one of IKs from ikfast.

  // Set up a target for IKFast
  targetFrame->setTranslation(Eigen::Vector3d(0, 0, 0.5));
  auto solutions = ikfast->getSolutions(targetFrame->getTransform());
  EXPECT_TRUE(!solutions.empty());

  const auto dofs = ikfast->getDofs();

  for (const auto& solution : solutions)
  {
    EXPECT_EQ(solution.mConfig.size(), 6);

    if (solution.mValidity != dynamics::InverseKinematics::Analytical::VALID)
      continue;

    ada->setPositions(dofs, solution.mConfig);
    Eigen::Isometry3d newTf = ee->getTransform();
    EXPECT_TRUE(
        targetFrame->getTransform().matrix().isApprox(newTf.matrix(), 1e-2));
  }
}

//==============================================================================
TEST(IkFast, VerifyLibadaLoadIkFast)
{
  const bool adaReal = false;
  aikido::planner::WorldPtr env(new aikido::planner::World("test"));
  dart::common::Uri adaUrdfUri{
      "package://ada_description/robots/ada_with_camera_forque.urdf"};
  dart::common::Uri adaSrdfUri{
      "package://ada_description/robots/ada_with_camera_forque.srdf"};
  std::string endEffectorName = "j2n6s200_end_effector";
  ada::Ada robot(env, !adaReal, adaUrdfUri, adaSrdfUri, endEffectorName);
  EXPECT_NE(robot.getEndEffectorIkSolver(), nullptr);
}
