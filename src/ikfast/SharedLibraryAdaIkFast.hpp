#ifndef LIBADA_SHAREDLIBRARYADAIKFAST_HPP_
#define LIBADA_SHAREDLIBRARYADAIKFAST_HPP_

#include "dart/common/SharedLibrary.hpp"
#include "dart/dynamics/IkFast.hpp"

class SharedLibraryAdaIkFast : public dart::dynamics::IkFast
{
public:
  SharedLibraryAdaIkFast(
      dart::dynamics::InverseKinematics* ik,
      const std::vector<std::size_t>& dofMap,
      const std::vector<std::size_t>& freeDofMap,
      const std::string& methodName = "IKFast",
      const Analytical::Properties& properties = Analytical::Properties());

  // Documentation inherited.
  auto clone(dart::dynamics::InverseKinematics* newIK) const
      -> std::unique_ptr<GradientMethod>;

protected:
  // Documentation inherited.
  int getNumFreeParameters() const override;

  // Documentation inherited.
  int* getFreeParameters() const override;

  // Documentation inherited.
  int getNumJoints() const override;

  // Documentation inherited.
  int getIkRealSize() const override;

  // Documentation inherited.
  int getIkType() const override;

  // Documentation inherited.
  bool computeIk(
      const IkReal* targetTranspose,
      const IkReal* targetRotation,
      const IkReal* freeParams,
      ikfast::IkSolutionListBase<IkReal>& solutions) override;

  // Documentation inherited.
  const char* getKinematicsHash() override;

  // Documentation inherited.
  const char* getIkFastVersion() override;
};

#endif // LIBADA_SHAREDLIBRARYADAIKFAST_HPP_