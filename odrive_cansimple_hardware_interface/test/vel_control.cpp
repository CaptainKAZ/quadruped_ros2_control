#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include "odrive_cansimple_hardware_interface/cansimple_hardware_interface.hpp"
#include "odrive_cansimple_hardware_interface/cansimple_protocol.hpp"
#include "odrive_cansimple_hardware_interface/socketcan.hpp"

// expect the motor to turn once and then turn back

int main()
{
  std::shared_ptr<SocketCan> can_device = std::make_shared<SocketCan>();
  std::cout << "can device name?" << std::endl;
  std::string can_name;
  std::cin >> can_name;
  can_device->open(can_name, 1e6);
  odrive_cansimple_hardware_interface::CanSimpleAxis axis;
  axis.configure(10, 0.05);
  int id;
  std::cout << "can id?" << std::endl;
  std::cin >> id;
  axis.attach(can_device, id);
  axis.setControllerMode(0x03,0x01);
  axis.setAxisRequestedState(0x08);
  // NOTE:may need to adjust
  //*axis.getCommandKpPtr() = 20;
  //*axis.getCommandKpPtr() = 5;
  axis.setGains(10, 0.01);
  while (1)
  {
    printf("%f ", *axis.getStatePosPtr());
    printf("%f\n", *axis.getStateVelPtr());
    //expect the motor turn at 1 rps
    *axis.getCommandVelPtr()=2*M_PI;
    axis.writeCommand();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}