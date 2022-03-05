#include "usb.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

// #define USB_DEBUG 1

#ifndef USB_DEBUG
  #define debug(...)
#else
  #define debug(fmt, ...) printf("[usb]" fmt "\r\n", __VA_ARGS__)
#endif

static struct timeval zero_tv = {0, 0};

static void printDev(libusb_device* dev){
  libusb_device_descriptor dd = {};
  libusb_get_device_descriptor(dev, &dd);
  debug("device: %04x, cls: %02x, scls: %02x, proto: %02x, vid: %04x, pid: %04x, confs: %hhu, bcdDevice: %hu, portno: %hhu",
    dd.bcdUSB, dd.bDeviceClass, dd.bDeviceSubClass, dd.bDeviceProtocol, dd.idVendor, dd.idProduct, dd.bNumConfigurations, dd.bcdDevice, libusb_get_port_number(dev)
  );
}

static void printDesc(libusb_device_handle *device_handle, libusb_config_descriptor* cdesc){
  debug("cfg desLen: %hhu, type: %hhu, dataLen: %hu, numIface: %hhu, confVal %hhu, idxConf: %hhu, numAttr: %hhu, mpow: %hhu",
    cdesc->bLength, cdesc->bDescriptorType, cdesc->wTotalLength, cdesc->bNumInterfaces,
    cdesc->bConfigurationValue, cdesc->iConfiguration, cdesc->bmAttributes, cdesc->MaxPower
  );

  libusb_config_descriptor* conf_desc = cdesc;
  const struct libusb_endpoint_descriptor *endpoint;

  int i, j, k;
  unsigned char string_desc[256];
  uint8_t endpoint_in = 0, endpoint_out = 0;  // default IN and OUT endpoints

  for(i = 0; i < cdesc->bNumInterfaces; i++){
    printf("              interface[%d]: id = %d\n", i, conf_desc->interface[i].altsetting[0].bInterfaceNumber);
    for(j=0; j<conf_desc->interface[i].num_altsetting; j++){
      printf("interface[%d].altsetting[%d]: num endpoints = %d\n", i, j, conf_desc->interface[i].altsetting[j].bNumEndpoints);
      printf("   Class.SubClass.Protocol: %02X.%02X.%02X\n",
        conf_desc->interface[i].altsetting[j].bInterfaceClass,
        conf_desc->interface[i].altsetting[j].bInterfaceSubClass,
        conf_desc->interface[i].altsetting[j].bInterfaceProtocol);

      if(libusb_get_string_descriptor_ascii(device_handle, conf_desc->interface[i].altsetting[j].iInterface, string_desc, sizeof(string_desc)) >= 0)
      printf("               string_desc: %s\n", string_desc);

      for(k=0; k<conf_desc->interface[i].altsetting[j].bNumEndpoints; k++){
        struct libusb_ss_endpoint_companion_descriptor *ep_comp = NULL;
        endpoint = &conf_desc->interface[i].altsetting[j].endpoint[k];
        printf("       endpoint[%d].address: %02X\n", k, endpoint->bEndpointAddress);
        // Use the first interrupt or bulk IN/OUT endpoints as default for testing
        if((endpoint->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) & (LIBUSB_TRANSFER_TYPE_BULK | LIBUSB_TRANSFER_TYPE_INTERRUPT)){
          if(endpoint->bEndpointAddress & LIBUSB_ENDPOINT_IN){
            if(!endpoint_in) endpoint_in = endpoint->bEndpointAddress;
          }
          else{
            if(!endpoint_out) endpoint_out = endpoint->bEndpointAddress;
          }
        }
        printf("           descriptor type: %02X\n", endpoint->bDescriptorType);
        printf("             transfer type: %02X\n", endpoint->bmAttributes & 0x03);
        printf("           max packet size: %04X\n", endpoint->wMaxPacketSize);
        printf("          polling interval: %02X\n", endpoint->bInterval);
        libusb_get_ss_endpoint_companion_descriptor(NULL, endpoint, &ep_comp);
        if(ep_comp){
          printf("                 max burst: %02X   (USB 3.0)\n", ep_comp->bMaxBurst);
          printf("        bytes per interval: %04X (USB 3.0)\n", ep_comp->wBytesPerInterval);
          libusb_free_ss_endpoint_companion_descriptor(ep_comp);
        }
      }
    }
  }
}

static char* uuid_to_string(const uint8_t* uuid){
  static char uuid_string[40];
  if (uuid == NULL) return NULL;
  sprintf(uuid_string, "{%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x}",
    uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5], uuid[6], uuid[7],
    uuid[8], uuid[9], uuid[10], uuid[11], uuid[12], uuid[13], uuid[14], uuid[15]);
  return uuid_string;
}

static void print_device_cap(libusb_bos_dev_capability_descriptor *dev_cap){
  switch(dev_cap->bDevCapabilityType) {
  case LIBUSB_BT_USB_2_0_EXTENSION: {
    struct libusb_usb_2_0_extension_descriptor *usb_2_0_ext = NULL;
    libusb_get_usb_2_0_extension_descriptor(NULL, dev_cap, &usb_2_0_ext);
    if (usb_2_0_ext) {
      printf("    USB 2.0 extension:\n");
      printf("      attributes             : %02X\n", usb_2_0_ext->bmAttributes);
      libusb_free_usb_2_0_extension_descriptor(usb_2_0_ext);
    }
    break;
  }
  case LIBUSB_BT_SS_USB_DEVICE_CAPABILITY: {
    struct libusb_ss_usb_device_capability_descriptor *ss_usb_device_cap = NULL;
    libusb_get_ss_usb_device_capability_descriptor(NULL, dev_cap, &ss_usb_device_cap);
    if (ss_usb_device_cap) {
      printf("    USB 3.0 capabilities:\n");
      printf("      attributes             : %02X\n", ss_usb_device_cap->bmAttributes);
      printf("      supported speeds       : %04X\n", ss_usb_device_cap->wSpeedSupported);
      printf("      supported functionality: %02X\n", ss_usb_device_cap->bFunctionalitySupport);
      libusb_free_ss_usb_device_capability_descriptor(ss_usb_device_cap);
    }
    break;
  }
  case LIBUSB_BT_CONTAINER_ID: {
    struct libusb_container_id_descriptor *container_id = NULL;
    libusb_get_container_id_descriptor(NULL, dev_cap, &container_id);
    if (container_id) {
      printf("    Container ID:\n      %s\n", uuid_to_string(container_id->ContainerID));
      libusb_free_container_id_descriptor(container_id);
    }
    break;
  }
  default:
    printf("    Unknown BOS device capability %02x:\n", dev_cap->bDevCapabilityType);
  }
}

__attribute__ ((weak)) void OnUSBDevice(USB* usb, libusb_device *dev, bool isOnline, uint16_t vid, uint16_t pid, uint8_t bus, uint8_t addr);

void USB::Init(){
  if(usb_context) return;
  int rc;
  rc = libusb_init(&usb_context);
  debug("libusb_init rc: %d", rc);

  rc = libusb_pollfds_handle_timeouts(usb_context);
  debug("libusb_pollfds_handle_timeouts rc: %d", rc);

  rc = libusb_hotplug_register_callback(usb_context,
    (libusb_hotplug_event)(LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT),
    (libusb_hotplug_flag)0, LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY,
    OnHotPlug, this, &hotplugHandle);
  debug("libusb_hotplug_register_callback rc: %d", rc);

  libusb_set_pollfd_notifiers(usb_context, OnPollFDAdded, OnPollFDRemoved, this);
}

void USB::DeInit(){
  if(!usb_context) return;
  libusb_hotplug_deregister_callback(usb_context, hotplugHandle);
  libusb_exit(usb_context);
}

int USB::getFds(poll_fd* fds){
  if(!usb_context) return 0;
  int rc = 0;
  const struct libusb_pollfd** pollfds = libusb_get_pollfds(usb_context);
  assert(pollfds);
  for(const struct libusb_pollfd** i=pollfds; *i; i++){
    rc += 1;
    *fds++ = {.fd = (*i)->fd, .event_mask = (*i)->events};
    // printf("fd: %d, event_mask: %d\r\n", (*i)->fd, (*i)->events);
  }
  free(pollfds);
  return rc;
}

libusb_context* USB::GetContext(){
  return usb_context;
}

void USB::ScanDevices(){
  if(!usb_context) return;
  libusb_device* dev = NULL;
  libusb_device **devs = NULL;
  debug("ScanDevices: %p", usb_context);
  int rc = libusb_get_device_list(usb_context, &devs);
  debug("found dev count: %d", rc);
  for(uint8_t i = 0; i < rc; i++) {
    dev = devs[i];
    libusb_device_descriptor dd = {};
    uint8_t bus = libusb_get_bus_number(dev);
    uint8_t addr = libusb_get_device_address(dev);
    libusb_get_device_descriptor(dev, &dd);
    if(OnUSBDevice) OnUSBDevice(this, dev, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED, dd.idVendor, dd.idProduct, bus, addr);
  }
  if(devs) libusb_free_device_list(devs, true);
}

void USB::OnFDData(poll_fd fd){
  if(!usb_context) return;
  libusb_handle_events_timeout(usb_context, &zero_tv);
}

void USB::OnPollFDRemoved(int fd, void *user_data){
  debug("OnPollFDRemoved fd: %d", fd);
}

void USB::OnPollFDAdded(int fd, short events, void *user_data){
  debug("OnPollFDAdded fd: %d", fd);
}

int USB::OnHotPlug(libusb_context *ctx, libusb_device *dev, libusb_hotplug_event event, void *user_data){
  uint8_t bus = libusb_get_bus_number(dev);
  uint8_t addr = libusb_get_device_address(dev);
  libusb_device_descriptor dd = {};
  libusb_get_device_descriptor(dev, &dd);
  debug("device %s: bus: %hhu, addr: %hhu, vid: %04x, pid: %04x, bus: %hhu, addr: %hhu",
    event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED ? "added" : "removed", bus, addr, dd.idVendor, dd.idProduct, bus, addr);
  // printDev(dev);

  if(OnUSBDevice) OnUSBDevice((USB*)user_data, dev, event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED, dd.idVendor, dd.idProduct, bus, addr);

  return 0;
}

void USBDevice::Init(libusb_device *dev, libusb_context* context){
  if(this->dev) return;
  this->dev = dev;
  this->context = context;
  libusb_ref_device(dev);
  Open();
}

void USBDevice::DeInit(){
  Close();
  libusb_unref_device(dev);
  dev = NULL;
}

libusb_device* USBDevice::GetDevice(){
  return dev;
}

libusb_device_handle* USBDevice::GetHandle(){
  return device_handle;
}

libusb_context* USBDevice::GetContext(){
  return context;
}

bool USBDevice::Close(){
  if(!IsOpen()) return false;
  if(device_handle) libusb_close(device_handle);
  device_handle = NULL;
  return true;
}

bool USBDevice::IsOpen(){
  return dev && device_handle;
}

bool USBDevice::Open(){
  if(IsOpen()) return false;

  printDev(dev);

  portNumber = libusb_get_port_number(dev);

  int rc = libusb_open(dev, &device_handle);
  debug("libusb_open rc: %d", rc);

  rc = libusb_set_auto_detach_kernel_driver(device_handle, 1);
  debug("libusb_set_auto_detach_kernel_driver rc: %d", rc);

  #ifndef USB_DEBUG
  return rc == 0;
  #endif

  libusb_bos_descriptor *bos_desc = NULL;
  if(libusb_get_bos_descriptor(device_handle, &bos_desc) == LIBUSB_SUCCESS){
    debug("%d caps", bos_desc->bNumDeviceCaps);
    for(int i = 0; i < bos_desc->bNumDeviceCaps; i++) print_device_cap(bos_desc->dev_capability[i]);
    libusb_free_bos_descriptor(bos_desc);
  }
  else{
    debug("%s", "no bos descriptor");
  }

  libusb_device_descriptor dd = {};
  libusb_config_descriptor* cdesc = NULL;

  // rc = libusb_get_active_config_descriptor(dev, &cdesc);
  // debug("libusb_get_active_config_descriptor rc: %d", rc);
  // if(cdesc) printDesc(cdesc), libusb_free_config_descriptor(cdesc);

  // uint8_t nb_ifaces = 0;
  // rc = libusb_get_config_descriptor(dev, 0, &cdesc);
  // if(cdesc){
  //   nb_ifaces = cdesc->bNumInterfaces;
  //   printDesc(cdesc);
  //   libusb_free_config_descriptor(cdesc);
  // }

  libusb_get_device_descriptor(dev, &dd);

  unsigned char string[256];
  if (dd.iManufacturer) {
    if (libusb_get_string_descriptor_ascii(device_handle, dd.iManufacturer, string, sizeof(string)) > 0)
      debug("Manufacturer: %s", (char *)string);
  }

  if (dd.iProduct) {
    if (libusb_get_string_descriptor_ascii(device_handle, dd.iProduct, string, sizeof(string)) > 0)
      debug("Product: %s", (char *)string);
  }

  if (dd.iSerialNumber) {
    if (libusb_get_string_descriptor_ascii(device_handle, dd.iSerialNumber, string, sizeof(string)) > 0)
      debug("Serial Number: %s", (char *)string);
  }

  for(uint8_t i = 0; i < dd.bNumConfigurations; i++){
    cdesc = NULL;
    rc = libusb_get_config_descriptor(dev, i, &cdesc);
    debug("libusb_get_config_descriptor idx: %hhu, rc: %d", i, rc);
    if(cdesc) printDesc(device_handle, cdesc), libusb_free_config_descriptor(cdesc);
  }

  // uint8_t string_index[3];
  // string_index[0] = dd.iManufacturer;
  // string_index[1] = dd.iProduct;
  // string_index[2] = dd.iSerialNumber;
  // for(int i=0; i<3; i++){
  //   if(string_index[i] == 0) continue;
  //   if(libusb_get_string_descriptor_ascii(device_handle, string_index[i], (unsigned char*)string, 128) >= 0)
  //     debug("   String (0x%02X): \"%s\"", string_index[i], string);
  // }

  // for(int i = 0; i < nb_ifaces; i++){
  //   int rc = libusb_claim_interface(device_handle, i);
  //   debug("libusb_claim_interface %hhu rc: %d", i, rc);
  // }

  // for(int i = 0; i < nb_ifaces; i++){
  //   rc = libusb_release_interface(device_handle, i);
  //   debug("libusb_release_interface %hhu rc: %d", i, rc);
  // }

  return rc == 0;
}

bool USBDevice::GetTransferInfo(uint8_t endpointAddr, uint8_t& ttype, uint16_t& maxPktSize){
  libusb_config_descriptor* conf_desc = NULL;
  int rc = libusb_get_active_config_descriptor(dev, &conf_desc);
  if(conf_desc){
    rc = -1;
    for(uint8_t i = 0; i < conf_desc->bNumInterfaces; i++){
      for(uint8_t j = 0; j < conf_desc->interface[i].num_altsetting; j++){
        for(uint8_t k = 0; k < conf_desc->interface[i].altsetting[j].bNumEndpoints; k++){
          const libusb_endpoint_descriptor* endpoint = &conf_desc->interface[i].altsetting[j].endpoint[k];
          if(endpointAddr == endpoint->bEndpointAddress){
            ttype = endpoint->bmAttributes & 0x03;
            maxPktSize = endpoint->wMaxPacketSize;
            rc = 0;
            goto done;
          }
        }
      }
    }
    done:
    libusb_free_config_descriptor(conf_desc);
  }

  debug("GetTransferInfo for ep %02X, rc: %d, ttype: %hhu, maxPktSize: %hu", endpointAddr, rc, ttype, maxPktSize);
  return rc == 0;
}

bool USBDevice::ClaimIface(uint8_t idx){
  if(!IsOpen()) return false;
  int rc;
  // rc = libusb_detach_kernel_driver(device_handle, idx);
  // debug("libusb_detach_kernel_driver %hhu rc: %d", idx, rc);
  rc = libusb_claim_interface(device_handle, idx);
  debug("libusb_claim_interface %hhu rc: %d", idx, rc);
  return rc == 0;
}

bool USBDevice::ReleaseIface(uint8_t idx){
  if(!IsOpen()) return false;
  int rc = libusb_release_interface(device_handle, idx);
  debug("libusb_release_interface %hhu rc: %d", idx, rc);
  return rc == 0;
}

bool USBTransfer::Init(TransferCB* cb, USBDevice* device, uint8_t endpoint){
  if(maxPktSize) return false;
  this->cb = cb;
  this->device = device;
  if(!transfer) transfer = libusb_alloc_transfer(0);
  transfer->dev_handle = device->GetHandle();
  transfer->callback = OnTransferComplete;
  transfer->user_data = this;
  transfer->timeout = 0;
  transfer->endpoint = endpoint;
  transfer->flags = 0;
  transfer->num_iso_packets = 0;

  if(transfer->endpoint != 0) return device->GetTransferInfo(transfer->endpoint, transfer->type, maxPktSize);

  maxPktSize = 64;
  transfer->timeout = 1000;
  transfer->type = LIBUSB_TRANSFER_TYPE_CONTROL;
  return true;
}

bool USBTransfer::DeInit(){
  if(maxPktSize == 0) return false;
  maxPktSize = 0;
  if(transfer){
    debug("DeInit transfer ep: %02x, status: %u, active: %hhu", transfer->endpoint, transfer->status, txActive);
    if(txActive && transfer->endpoint & LIBUSB_ENDPOINT_IN){
      txActive = false, stopFlag = 0;
      int rc = libusb_cancel_transfer(transfer);
      debug("libusb_cancel_transfer rc: %d, %s", rc, libusb_strerror((libusb_error)rc));
      if(rc == 0){
        rc = libusb_handle_events_completed(device->GetContext(), &stopFlag);
        debug("libusb_handle_events_completed rc: %d", rc);
      }
      else{
        rc = libusb_handle_events_timeout(device->GetContext(), &zero_tv);
        debug("libusb_handle_events_timeout rc: %d", rc);
      }
    }
    libusb_free_transfer(transfer);
  }
  transfer = NULL;
  return true;
}

bool USBTransfer::Submit(uint8_t* data, uint16_t dataLen){
  int rc = -1;
  int outLen = 0;
  if((transfer->endpoint & LIBUSB_ENDPOINT_IN)){
    transfer->buffer = data;
    transfer->length = dataLen;
    rc = libusb_submit_transfer(transfer);
    txActive = rc == 0;
  }
  else if(transfer->type == LIBUSB_TRANSFER_TYPE_CONTROL)
    rc = libusb_control_transfer(transfer->dev_handle, LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE, 0, 0, 0, data, dataLen, transfer->timeout);
  else if(transfer->type == LIBUSB_TRANSFER_TYPE_BULK)
    rc = libusb_bulk_transfer(transfer->dev_handle, transfer->endpoint, data, dataLen, &outLen, transfer->timeout);
  else if(transfer->type == LIBUSB_TRANSFER_TYPE_INTERRUPT)
    rc = libusb_interrupt_transfer(transfer->dev_handle, transfer->endpoint, data, dataLen, &outLen, transfer->timeout);

  debug("libusb_submit_transfer ep: %02X, rc: %d", transfer->endpoint, rc);
  return rc >= 0;
}

void USBTransfer::OnTransferComplete(struct libusb_transfer *transfer){
  USBTransfer* usbTransfer = (USBTransfer*)transfer->user_data;

  if(usbTransfer->txActive){
    if(usbTransfer->cb) usbTransfer->cb->OnComplete(usbTransfer, transfer->buffer, transfer->actual_length);
    if(transfer->endpoint & LIBUSB_ENDPOINT_IN) usbTransfer->txActive = libusb_submit_transfer(transfer) == 0;
  }

  debug("OnTransferComplete ep: %02x, status: %u, active: %hhu", transfer->endpoint, transfer->status, usbTransfer->txActive);

  usbTransfer->stopFlag = 1;
}
