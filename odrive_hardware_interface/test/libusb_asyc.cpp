#include <cstdint>
#include <libusb-1.0/libusb.h>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>

#include "odrive_hardware_interface/odrive_endpoints.hpp"
#include "odrive_hardware_interface/odrive_usb.hpp"

libusb_context* ctx;
static struct libusb_transfer* out_xfr;
static struct libusb_transfer* in_xfr;
static struct libusb_device_handle* devh;
static std::chrono::time_point<std::chrono::high_resolution_clock> last_time;
static std::chrono::time_point<std::chrono::high_resolution_clock> loop_time;
static std::vector<uint8_t> req_payload;
static double total=0;
static double avage_loop;
static int sample=0;
unsigned short seq_num = 0;
uint8_t* outbuf;
uint8_t* inbuf;

int get_seq_num()
{
  seq_num = (seq_num + 1) & 0x7fff;
  seq_num |= LIBUSB_ENDPOINT_IN;
  return seq_num;
}

int get_ep_id(bool MSB, short endpoint_id)
{
  if (MSB)
  {
    endpoint_id |= 0x8000;
  }
  return endpoint_id;
}

size_t encode(bool MSB, short endpoint_id, short response_size, const std::vector<uint8_t>& payload, uint8_t* outbuf)
{
  int seq = get_seq_num();
  int ep = get_ep_id(MSB, endpoint_id);
  uint8_t index = 0;
  outbuf[index++] = (seq & 0xFF);
  outbuf[index++] = ((seq >> 8) & 0xFF);
  outbuf[index++] = (ep & 0xFF);
  outbuf[index++] = ((ep >> 8) & 0xFF);
  outbuf[index++] = (response_size & 0xFF);
  outbuf[index++] = ((response_size >> 8) & 0xFF);
  for (size_t i = 0; i < payload.size(); i++)
  {
    outbuf[index++] = payload[i];
  }
  short crc = ((endpoint_id & 0x7fff) == 0) ? ODRIVE_PROTOCOL_VERSION : odrive::json_crc;
  outbuf[index++] = (crc & 0xFF);
  outbuf[index++] = ((crc >> 8) & 0xFF);
  return index;
}

float decode()
{
  union
  {
    float ret;
    uint8_t buf[4];
  } convert;
  convert.buf[0] = inbuf[2];
  convert.buf[1] = inbuf[3];
  convert.buf[2] = inbuf[4];
  convert.buf[3] = inbuf[5];
  return convert.ret;
}

void out_cplt(struct libusb_transfer* out_xfr)
{
  (void)out_xfr;
  
  if (out_xfr->status != LIBUSB_TRANSFER_COMPLETED) {
		fprintf(stderr, "transfer status %d\n", out_xfr->status);
		libusb_free_transfer(out_xfr);
		exit(3);
	}
  double dt =
      std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - last_time)
          .count();
  //printf("out packet took %f ms\n", dt * 1000);
  last_time = std::chrono::high_resolution_clock::now();
  (void)encode(true, odrive::VBUS_VOLTAGE, sizeof(float), req_payload, outbuf);
  libusb_submit_transfer(in_xfr);
}

void in_cplt(struct libusb_transfer* in_xfr)
{
  (void)in_xfr;
  libusb_submit_transfer(out_xfr);
  double dt =
      std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - last_time)
          .count();
  //printf("in packet took %f ms\n", dt * 1000);
  last_time = std::chrono::high_resolution_clock::now();
  float vbus = decode();
  //printf("readed vbus = %f\n", vbus);
  double dt2 =
      std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - loop_time)
          .count();
  sample++;
  total=total+dt2;
  avage_loop=total/(double)sample;
  //printf("avage_loop : %fms\n",avage_loop*1000);
  printf("loop time :%fms\n",dt2*1000);
  loop_time = std::chrono::high_resolution_clock::now();
}

[[noreturn]] void handle_thread()
{
  while (1)
  {
    libusb_handle_events(ctx);
  }
}

int main()
{
  auto ret = libusb_init(&ctx);
  if(ret<0){
    fprintf(stderr, "Error initializing libusb: %s\n", libusb_error_name(ret));
    exit(0);
  }
  printf("inited!!%d\n",ret);

  devh = libusb_open_device_with_vid_pid(ctx, ODRIVE_USB_VENDORID, ODRIVE_USB_PRODUCTID);
  if (!devh) {
		fprintf(stderr, "Error finding USB device\n");
		exit(0);
	}
  printf("fined dev %d\n",devh);
  if ((libusb_kernel_driver_active(devh, 2) != LIBUSB_SUCCESS) &&
      (libusb_detach_kernel_driver(devh, 2) != LIBUSB_SUCCESS))
  {
    libusb_close(devh);
  }
  printf("detach!!\n");
  if ((libusb_claim_interface(devh, 2)) != LIBUSB_SUCCESS)
  {
    libusb_close(devh);
  }
  out_xfr = libusb_alloc_transfer(0);
  in_xfr = libusb_alloc_transfer(0);
  inbuf = libusb_dev_mem_alloc(devh, 64);
  outbuf = libusb_dev_mem_alloc(devh, 64);

  int outsize = encode(true, odrive::VBUS_VOLTAGE, sizeof(float), req_payload, outbuf);

  libusb_fill_bulk_transfer(out_xfr, devh, ODRIVE_OUT_ENDPOINT, outbuf, outsize, out_cplt, NULL, 0);
  libusb_fill_bulk_transfer(in_xfr, devh, ODRIVE_IN_ENDPOINT, inbuf, ODRIVE_MAX_PACKET_SIZE, in_cplt, NULL, 0);

  last_time = std::chrono::high_resolution_clock::now();
  loop_time = std::chrono::high_resolution_clock::now();
  libusb_submit_transfer(out_xfr);
  std::thread thread(handle_thread);
  thread.detach();
  while (42)
  {
  };
}