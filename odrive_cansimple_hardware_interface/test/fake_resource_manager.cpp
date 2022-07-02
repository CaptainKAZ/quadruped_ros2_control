#include <memory>
#include "odrive_cansimple_hardware_interface/cansimple_hardware_interface.hpp"
#include "odrive_cansimple_hardware_interface/cansimple_protocol.hpp"
#include "odrive_cansimple_hardware_interface/socketcan.hpp"

int main(){
    std::shared_ptr<SocketCan> can_device = std::make_shared<SocketCan>();
    can_device->open("can0",1e6);
    odrive_cansimple_hardware_interface::CanSimpleAxis axis;
    axis.attach(can_device,0x02);
    while (1){
        printf("%f\n",*axis.getStatePosPtr());
        printf("%f\n",*axis.getStateVelPtr());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}