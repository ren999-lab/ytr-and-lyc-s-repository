#ifndef ROBOCON2027_GAZEBO_RC_PLANAR_MOVE_HPP_
#define ROBOCON2027_GAZEBO_RC_PLANAR_MOVE_HPP_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <memory>

namespace rc_simulation
{
class RcPlanarMovePrivate;

class RcPlanarMove : public gazebo::ModelPlugin
{
public:
  RcPlanarMove();
  ~RcPlanarMove() override;
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
  void Reset() override;

private:
  std::unique_ptr<RcPlanarMovePrivate> impl_;
};
}

#endif
