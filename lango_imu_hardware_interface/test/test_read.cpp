#include <boost/asio.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/streambuf.hpp>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <iterator>

int main() {
  boost::asio::io_service service{};
  boost::asio::serial_port serial(service, "/dev/ttyACM0");
  //serial.set_option(boost::asio::serial_port::baud_rate(1000000000));
  uint8_t buffer[100]={0};
  
  while (1) {
    std::memset(buffer,0,100);
    auto readed=serial.read_some(boost::asio::buffer(buffer,100));
    size_t index=0;
    while (index<readed){
    if(buffer[index]==0x0D&&buffer[index+1]==0x0A&&buffer[index]&&buffer[index+15]==0x0a&&buffer[index+16]==0x0d){
        if(buffer[index+2]=='A'){
            float accel[3]{};
            std::memcpy(accel,&buffer[index+3],12);
            printf("acc: %f %f %f \n",accel[0],accel[1],accel[2]);
        }
        else if(buffer[index+2]=='G'){
            float gyro[3]{};
            std::memcpy(gyro,&buffer[index+3],12);
            printf("gyo: %f %f %f \n",gyro[0],gyro[1],gyro[2]);
        }
        else if(buffer[index+2]=='M'){
            float mag[3]{};
            std::memcpy(mag,&buffer[index+3],12);
            printf("mag: %f %f %f \n",mag[0],mag[1],mag[2]);
        }
        index+=17;
        continue;
    }
    index++;
    }
    printf("\n");
  }
}
