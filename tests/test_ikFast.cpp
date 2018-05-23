#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <gtest/gtest.h>
#include <libada/Ada.hpp>

using namespace dart;

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

//==============================================================================
TEST(IkFast, IKTestGivenPose)
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
  ik->setHierarchyLevel(0);

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

  // Set up a target for IKFast
  targetFrame->setTranslation(Eigen::Vector3d(0.4, -0.142525, 0.102));
  auto targetTF = targetFrame->getTransform().matrix();


  auto solutions = ikfast->getSolutions(targetFrame->getTransform());
  EXPECT_TRUE(!solutions.empty());
  std::cout << solutions.size() << std::endl;

  const auto dofs = ikfast->getDofs();

  for (const auto& solution : solutions)
  {
    EXPECT_EQ(solution.mConfig.size(), 6);

    if (solution.mValidity != dynamics::InverseKinematics::Analytical::VALID)
      continue;

    ada->setPositions(dofs, solution.mConfig);
    auto newTf = ee->getTransform().matrix();
    EXPECT_TRUE(targetTF.isApprox(newTf, 1e-2));
  }
}


//==============================================================================
TEST(IkFast, IKTestGivenPositions)
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

  ik->setHierarchyLevel(0);

  std::vector<std::size_t> ikFastDofs{0, 1, 2, 3, 4, 5};

  ik->setGradientMethod<AdaIkFast>(ikFastDofs, std::vector<std::size_t>());
  auto analytical = ik->getAnalytical();
  EXPECT_NE(analytical, nullptr);
  EXPECT_EQ(analytical->getDofs().size(), 6);

  auto ikfast = dynamic_cast<dynamics::SharedLibraryIkFast*>(analytical);
  EXPECT_NE(ikfast, nullptr);

  const auto dofs = ikfast->getDofs();

  Eigen::VectorXd armRelaxedHome(Eigen::VectorXd::Ones(6));
  armRelaxedHome << 1.09007, -2.97579, -0.563162, -0.907691, 1.09752, -1.47537;
  ada->setPositions(dofs, armRelaxedHome);

  auto eePose = ee->getTransform().matrix();
  auto solutions = ikfast->getSolutions(ee->getTransform());
  EXPECT_TRUE(!solutions.empty());

  bool correctness = false;
  for (const auto& solution : solutions)
  {
    EXPECT_EQ(solution.mConfig.size(), 6);

    if (solution.mValidity != dynamics::InverseKinematics::Analytical::VALID)
      continue;

    ada->setPositions(dofs, solution.mConfig);
    EXPECT_FALSE(eePose.isApprox(ee->getTransform().matrix(), 1e-2));
    if (solution.mConfig.isApprox(armRelaxedHome))
    {
      correctness = true;
      break;
    }
  }
  EXPECT_FALSE(correctness);
}
