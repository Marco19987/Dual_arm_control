#include <uclv_systems_lib/controllers/pi.hpp>
#include <uclv_systems_lib/sim/state_space_system_simulator.hpp>
#include <uclv_systems_lib/ss/linear_state_space.hpp>
#include <uclv_systems_lib/continuous_time/continuous_time_linear_state_space.hpp>
#include <uclv_systems_lib/discretization/forward_euler.hpp>
#include <uclv_systems_lib/observers/ekf.hpp>
#include "../include/robots_object_system.hpp"

int main()
{
  uclv::systems::controllers::PI pi(0.1, 1.0, 1.0);
  uclv::systems::StateSpaceSystemSimulator<2, 3, 4, 5, 6, 7> simulator1;
  auto system = std::make_shared<uclv::systems::LinearStateSpace<2, 3, 4, 5>>();
  uclv::systems::StateSpaceSystemSimulator<2, 3, 4, 5, 5, 5> simulator2(system);
  auto continuous_time_linear_state_space_system = std::make_shared<uclv::systems::ContinuousTimeLinearStateSpace<3,2,1,1>>();
  continuous_time_linear_state_space_system->display();
  uclv::systems::ForwardEuler<3,2,1,1,1,1> forward_euler(continuous_time_linear_state_space_system,0.1);
  forward_euler.display();
  std::cout << "Hello World!" << std::endl;
  auto robots_object_system = std::make_shared<uclv::systems::RobotsObjectSystem<3,2,1,1>>();
  robots_object_system->display();

  return 0;
}
