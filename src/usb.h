#ifndef USB_H
#define USB_H

#include <libusb.h>
#include "helper.h"

class USB : public Poller{
  libusb_context* usb_context = NULL;
  libusb_hotplug_callback_handle hotplugHandle;
public:
  void Init();
  void DeInit();
  int getFds(poll_fd* fds);
  void OnFDData(poll_fd fd);
  void ScanDevices();
  libusb_context* GetContext();
private:
  static void OnPollFDRemoved(int fd, void *user_data);
  static void OnPollFDAdded(int fd, short events, void *user_data);
  static int OnHotPlug(libusb_context *ctx, libusb_device *dev, libusb_hotplug_event event, void *user_data);
};

class USBDevice{
  uint8_t portNumber = 0;
  libusb_device *dev = NULL;
  libusb_context* context = NULL;
  libusb_device_handle* device_handle = NULL;
public:
  const char* name = NULL;
  void Init(libusb_device *dev, libusb_context* context);
  void DeInit();
  bool Open();
  bool Close();
  bool IsOpen();
  bool ClaimIface(uint8_t idx);
  bool ReleaseIface(uint8_t idx);
  libusb_device* GetDevice();
  libusb_device_handle* GetHandle();
  libusb_context* GetContext();
  bool GetTransferInfo(uint8_t endpointAddr, uint8_t& ttype, uint16_t& maxPktSize);
};

class USBTransfer;

class TransferCB{
public:
  virtual void OnComplete(USBTransfer* transfer, uint8_t* data, uint16_t dataLen) = 0;
};

class USBTransfer{
  uint16_t maxPktSize = 0;
  USBDevice* device = NULL;
  libusb_transfer* transfer = NULL;
  bool txActive = true;
  int stopFlag = 1;
public:
  TransferCB* cb = NULL;
  bool DeInit();
  bool Init(TransferCB* cb, USBDevice* device, uint8_t endpoint);
  bool Submit(uint8_t* data, uint16_t dataLen);
private:
  static void OnTransferComplete(struct libusb_transfer *transfer);
};

#endif
