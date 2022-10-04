#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>

#ifdef __linux__
#include <pty.h>
#else
#include <util.h>
#endif

#include "usb.h"

#define countof(x) (sizeof(x)/sizeof(x[0]))

typedef enum{
  IF_CDC,
  IF_HCI,
} usb_interface_type;

typedef struct{
  usb_interface_type type;
  const char* label;
  uint8_t if_no;
  union{
    struct{
      uint8_t tx_ep, rx_ep;
    } cdc;
    struct{
      uint8_t cmd_tx_ep, evt_rx_ep, acl_tx_ep, acl_rx_ep;
    } hci;
  };
} InterfaceInfo;

typedef struct{
  const char* name;
  uint16_t vid, pid;
  uint8_t ifaceCount;
  const InterfaceInfo* ifaces;
} USBDeviceInfo;

#define USE_CSR
// #define USE_BRCM
// #define USE_INTEL
#define USE_TPLINK

#define USB_DEV_IFACE(_name) _name##_iface

#define REG_DEV(_name, _vid, _pid, ...) static const InterfaceInfo USB_DEV_IFACE(_name)[] = {__VA_ARGS__};
#define REG_IFACE(_type,_label, _if_no) .type = _type, .label = #_label, .if_no = _if_no

#define REG_CDC_IFACE(_label, _if_no, _tx_ep, _rx_ep) \
  {REG_IFACE(IF_CDC, _label, _if_no), {.cdc = {.tx_ep = _tx_ep, .rx_ep = _rx_ep}}},

#define REG_HCI_IFACE(_label, _if_no, _cmd_tx_ep, _evt_rx_ep, _acl_tx_ep, _acl_rx_ep) \
  {REG_IFACE(IF_HCI, _label, _if_no), {.hci = {.cmd_tx_ep = _cmd_tx_ep, .evt_rx_ep = _evt_rx_ep, .acl_tx_ep = _acl_tx_ep, .acl_rx_ep = _acl_rx_ep}}},

#include "dev_defs.h"

static const USBDeviceInfo kUSBDevices[] = {  
  #define REG_DEV(_name, _vid, _pid, ...) {.name = #_name, .vid = _vid, .pid = _pid, \
    .ifaceCount = countof(USB_DEV_IFACE(_name)), .ifaces = USB_DEV_IFACE(_name)},
  #include "dev_defs.h"
};

static Node* pollers = NULL, *devices = NULL;

#define LOG_PTY(fmt, ...) printf("[%llu|%s|%s]" fmt "\r\n", sys_now(), usbDevice->name, info->label, ## __VA_ARGS__)

static uint64_t start_ms = 0;

static uint64_t sys_now(){
  struct timespec uptime;
  clock_gettime(CLOCK_MONOTONIC_RAW, &uptime);
  uint64_t now = (uptime.tv_sec * 1000) + (uptime.tv_nsec / (1000 * 1000));
  if(!start_ms) start_ms = now;
  return now - start_ms;
}

class PTYInterface : public Poller, public TransferCB{
  char ptyname[64];
  int ptyfd = 0, ttyfd = 0;
  uint8_t tty_recv_buff[512];
protected:
  USBDevice* usbDevice = NULL;
  const InterfaceInfo* info = NULL;

  PTYInterface(USBDevice* usbDevice, const InterfaceInfo* info){
    this->info = info;
    int rc = usbDevice->ClaimIface(info->if_no);
    LOG_PTY("ClaimIface 0x%02x: %d", info->if_no, rc);

    if(!rc) return;
    this->usbDevice = usbDevice;

    struct termios termp;
    // memset(&termp, 0, sizeof(termp));
    cfmakeraw(&termp);
    cfsetispeed(&termp, B115200);
    cfsetospeed(&termp, B115200);
    termp.c_oflag = 0;
    termp.c_cflag |= CLOCAL | CREAD;
    termp.c_lflag &= ~ICANON;
    // termp.c_lflag = 0;
    termp.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    termp.c_cc[VMIN] = 0;
    termp.c_cc[VTIME] = 0;

    rc = openpty(&ptyfd, &ttyfd, ptyname, NULL, NULL);
    if(rc) LOG_PTY("openpty() failed (%s)", strerror(errno));
    else LOG_PTY("if: 0x%02x -> %s, ptyfd: %d, ttyfd: %d", info->if_no, ptyname, ptyfd, ttyfd);

    if(ptyfd > 0) fcntl(ptyfd, F_SETFL, fcntl(ptyfd, F_GETFL) | O_NONBLOCK);
    // if(ttyfd > 0) fcntl(ttyfd, F_SETFL, fcntl(ttyfd, F_GETFL) | O_NONBLOCK);
    // if(ttyfd > 0) close(ttyfd), ttyfd = 0;

    this->name = info->label;
    pollers = addNode(pollers, this);
  }

  int getFds(poll_fd* fds){
    fds[0] = {
      .fd = ptyfd,
      .event_mask = FD_READ,
    };
    return ptyfd > 0 ? 1 : 0;
  }

  void OnFDData(poll_fd fd){
    int rc = read(ptyfd, tty_recv_buff, sizeof(tty_recv_buff));
    if(rc > 0) PTYRecv(tty_recv_buff, rc);
    else if(rc < 0) LOG_PTY("ptyfd read rc: %d, err: %d -> %s", rc, errno, strerror(errno));
  }

  virtual void PTYRecv(uint8_t* data, int length) = 0;

  int PTYSend(uint8_t* data, int length){
    int n = 0, written = 0, prevWritten = -1;
    while(written < length){
      n = write(ptyfd, data + written, length - written);
      // printf("write: %d\r\n", n);
      if(n < 0){
        if(errno == EAGAIN || errno == EINTR) goto retry;
        LOG_PTY("write() failed (%s)", strerror(errno));
        break;
      }
      else if(n > 0) written += n;
      else{
        retry:
        if(prevWritten == written){
          LOG_PTY("dropping %d bytes...", length - written);
          break;
        }
        if(prevWritten == -1) prevWritten = 0;
        LOG_PTY("sleeping, written: %d", written - prevWritten);
        prevWritten = written;
        usleep(2000);
      }
    }
    return written;
  }

  virtual ~PTYInterface(){
    if(!this->usbDevice) return;

    bool rc = usbDevice->ReleaseIface(info->if_no);
    LOG_PTY("ReleaseIface: %d", rc);

    pollers = removeNode(pollers, this);
    if(ptyfd > 0) close(ptyfd);
    if(ttyfd > 0) close(ttyfd);

    ptyfd = ttyfd = 0;
    this->usbDevice = NULL;
  }
};

#define XFER_INIT(xfer, cb, ep) rc = xfer.Init(cb, usbDevice, ep); LOG_PTY(#xfer ".Init %02x: %d", ep, rc);
#define XFER_SUBMIT(xfer, buff, size) rc = xfer.Submit(buff, size); LOG_PTY(#xfer ".Submit: %d", rc);
#define XFER_DEINIT(xfer, ep) rc = xfer.DeInit(); LOG_PTY(#xfer ".DeInit %02x: %d", ep, rc);

class CDCInterface : public PTYInterface{
  uint8_t rxBuff[512];
  USBTransfer tx, rx;

  void OnComplete(USBTransfer* transfer, uint8_t* data, uint16_t dataLen){
    PTYSend(data, dataLen);
  }

  void PTYRecv(uint8_t* data, int length){
    tx.Submit(data, length);
  }

  public:
  bool removed = false;
  CDCInterface(USBDevice* usbDevice, const InterfaceInfo* info):PTYInterface(usbDevice, info){
    if(!this->usbDevice) return;
    int rc;
    XFER_INIT(tx, NULL, info->cdc.tx_ep);
    XFER_INIT(rx, this, info->cdc.rx_ep);
    XFER_SUBMIT(rx, rxBuff, sizeof(rxBuff));
  }

  ~CDCInterface(){
    if(!this->usbDevice) return;
    bool rc;
    XFER_DEINIT(tx, info->cdc.tx_ep);
    XFER_DEINIT(rx, info->cdc.rx_ep);
  }
};

#define HCI_EVT_BUFF_SIZE (64)
#define MAX_BLE_HCI_PKT_SIZE (256)

#define __start_packed
#define __end_packed __attribute__((__packed__))

typedef enum{
  HCI_UNK = 0x00,
  HCI_CMD = 0x01,
  HCI_ACL = 0x02,
  HCI_SCO = 0x03,
  HCI_EVT = 0x04,
  HCI_ECMD = 0x09,
} HCIPktType;

typedef struct __start_packed{
  uint8_t type;
  uint8_t data[];
} __end_packed HCIPkt;

typedef struct __start_packed{
  uint16_t opcode;
  uint8_t length;
} __end_packed HCICmd;

typedef struct __start_packed{
  uint16_t handle : 12;
  uint8_t pbflag  : 2;
  uint8_t bcflag  : 2;
  uint16_t length;
} __end_packed HCIAcl;

typedef struct __start_packed{
  uint16_t opcode  : 12;
  uint8_t reserved : 4;
  uint8_t length;
} __end_packed HCISco;

typedef struct __start_packed{
  uint8_t opcode;
  uint8_t length;
} __end_packed HCIEvt;

class HCIInterface : public PTYInterface{
  USBTransfer cmd_tx, evt_rx, acl_tx, acl_rx;

  uint8_t evt_rx_buff[HCI_EVT_BUFF_SIZE + 1];
  uint8_t acl_rx_buff[MAX_BLE_HCI_PKT_SIZE + 1];

  size_t pty_rx_idx = 0;
  uint8_t opty_rx_buff[1024 + 4];

  size_t rem_acl_bytes = 0, rem_evt_bytes = 0;

  void OnComplete(USBTransfer* transfer, uint8_t* data, uint16_t dataLen){
    if(!dataLen) return;
    // data -= 1, dataLen += 1;

    switch(data[-1]){
      case HCI_ACL:{
        HCIAcl* pkt = (HCIAcl*)(data);
        if(rem_acl_bytes == 0){
          rem_acl_bytes = sizeof(HCIAcl) + pkt->length - dataLen;
          data -= 1, dataLen += 1;
        }
        else rem_acl_bytes -= dataLen;
        break;
      }
      case HCI_EVT:{
        HCIEvt* pkt = (HCIEvt*)(data);
        if(rem_evt_bytes == 0){
          rem_evt_bytes = sizeof(HCIEvt) + pkt->length - dataLen;
          data -= 1, dataLen += 1;
        }
        else rem_evt_bytes -= dataLen;
        break;
      }
      default: return;
    }

    int rc = PTYSend(data, dataLen);
    // hexdump(data, dataLen, "OnComplete");
    // LOG_PTY("got usb hci pkt: %hhu, length: %hu, sent: %hu", data[0], dataLen, rc);
  }

  #define ELIF_PKT else IF_PKT
  #define IF_PKT(typ, pktStruct, xyz, ...) if(pkt->type == typ){      \
    if(pty_rx_idx >= sizeof(HCIPkt) + sizeof(pktStruct)){             \
      pktStruct* xyz = (pktStruct*)(pkt->data);                       \
      pktLength = sizeof(HCIPkt) + sizeof(pktStruct) + xyz->length;   \
      if(pty_rx_idx >= pktLength){                                    \
        __VA_ARGS__                                                   \
      }                                                               \
    }                                                                 \
  }

  #define min(a,b)             \
  ({                           \
      __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
      _a < _b ? _a : _b;       \
  })

  void PTYRecv(uint8_t* data, int length){
    // LOG_PTY("PTYRecv %d bytes", length);
    uint8_t* pty_rx_buff = opty_rx_buff;
    while(length > 0){
      int bytes_to_read = sizeof(opty_rx_buff) - pty_rx_idx;
      if(bytes_to_read > length) bytes_to_read = length;
      else if(bytes_to_read <= 0){
        pty_rx_idx = 0;
        continue;
      }
      memcpy(pty_rx_buff + pty_rx_idx, data, bytes_to_read);
      pty_rx_idx += bytes_to_read, data += bytes_to_read, length -= bytes_to_read;

      // hexdump(pty_rx_buff, pty_rx_idx, "PTYRecv");

      checkAgain:
      if(pty_rx_idx >= sizeof(HCIPkt)){
        HCIPkt* pkt = (HCIPkt*)(pty_rx_buff);
        size_t pktLength = 0;

        IF_PKT(HCI_CMD, HCICmd, cmd, {})
        ELIF_PKT(HCI_EVT, HCIEvt, evt, {})
        ELIF_PKT(HCI_ACL, HCIAcl, acl, {})
        ELIF_PKT(HCI_SCO, HCISco, sco, {})
        else goto resetRead;

        if(!pktLength) goto readAgain;
        else if(pktLength > 1024) goto resetRead;

        if(pty_rx_idx >= pktLength){
          // hexdump(pty_rx_buff, pktLength, "got pkt");
          // LOG_PTY("got pty hci pkt: %hhu, length: %lu", pkt->type, pktLength);
          if(pkt->type == HCI_CMD) cmd_tx.Submit(pkt->data, pktLength - sizeof(HCIPkt));
          else if(pkt->type == HCI_ACL){
            size_t sent = 0, pkt_size = pktLength - sizeof(HCIPkt);
            while(sent < pkt_size){
              size_t to_send = min(255, pkt_size - sent);
              bool rc = acl_tx.Submit(pkt->data + sent, to_send);
              sent += to_send;
              // LOG_PTY("submit rc: %u, to_send: %zu, sent: %zu, pkt_size: %zu", rc, to_send, sent, pkt_size);
            }
          }
          else LOG_PTY("unknown hci pkt");

          pty_rx_idx -= pktLength;
          if(pty_rx_idx > 0){
            pty_rx_buff += pktLength;
            goto checkAgain;
          }
        }
      }

      readAgain:
      if(pty_rx_buff != opty_rx_buff){
        memmove(opty_rx_buff, pty_rx_buff, pty_rx_idx);
        pty_rx_buff = opty_rx_buff;
      }
      continue;

      resetRead:
      // hexdump(pty_rx_buff, pty_rx_idx, "hci: ignoring rx data");
      pty_rx_idx = 0;
    }
  }

  public:
  bool removed = false;
  HCIInterface(USBDevice* usbDevice, const InterfaceInfo* info):PTYInterface(usbDevice, info){
    if(!this->usbDevice) return;
    int rc;
    evt_rx_buff[0] = HCI_EVT, acl_rx_buff[0] = HCI_ACL;
    XFER_INIT(cmd_tx, NULL, info->hci.cmd_tx_ep);
    XFER_INIT(acl_tx, NULL, info->hci.acl_tx_ep);
    XFER_INIT(evt_rx, this, info->hci.evt_rx_ep);
    XFER_INIT(acl_rx, this, info->hci.acl_rx_ep);
    XFER_SUBMIT(evt_rx, evt_rx_buff + 1, HCI_EVT_BUFF_SIZE);
    XFER_SUBMIT(acl_rx, acl_rx_buff + 1, MAX_BLE_HCI_PKT_SIZE);
  }

  ~HCIInterface(){
    if(!this->usbDevice) return;
    bool rc;
    XFER_DEINIT(cmd_tx, info->hci.cmd_tx_ep);
    XFER_DEINIT(evt_rx, info->hci.evt_rx_ep);
    XFER_DEINIT(acl_tx, info->hci.acl_tx_ep);
    XFER_DEINIT(acl_rx, info->hci.acl_rx_ep);
  }
};

class USBDeviceWrapper : public Node{
  uint8_t bus, addr;
  USBDevice device;
  const USBDeviceInfo* info;
  PTYInterface** ifaces = NULL;
public:
  bool removed = false;
  USBDeviceWrapper(libusb_device *dev, libusb_context* context, const USBDeviceInfo* info, uint8_t bus, uint8_t addr){
    device.name = this->name = info->name;
    devices = addNode(devices, this);

    this->bus = bus;
    this->addr = addr;
    this->info = info;
    device.Init(dev, context);
    ifaces = new PTYInterface*[info->ifaceCount]();
    for(uint8_t i = 0; i < info->ifaceCount; i++){
      PTYInterface** iface = &ifaces[i];
      const InterfaceInfo* if_info = &info->ifaces[i];
      switch(if_info->type){
        case IF_CDC: *iface = new CDCInterface(&device, if_info); break;
        case IF_HCI: *iface = new HCIInterface(&device, if_info); break;
        default: *iface = NULL;
      }
    }
  }
  ~USBDeviceWrapper(){
    if(ifaces){
      for(uint8_t i = 0; i < info->ifaceCount; i++){
        PTYInterface* iface = ifaces[i];
        const InterfaceInfo* if_info = &info->ifaces[i];
        if(!iface) continue;
        switch(if_info->type){
          case IF_CDC: delete (CDCInterface*)iface; break;
          case IF_HCI: delete (HCIInterface*)iface; break;
        }
      }
      delete ifaces;
      ifaces = NULL;
    }
    device.DeInit();
    devices = removeNode(devices, this);
  }

  bool IsDevice(uint8_t bus, uint8_t addr){
    return this->bus == bus && this->addr == addr;
  }
};

void OnUSBDevice(USB* usb, libusb_device *dev, bool isOnline, uint16_t vid, uint16_t pid, uint8_t bus, uint8_t addr){
  printf("OnUSBDevice %s: vid: %04x, pid: %04x, bus: %hhu, addr: %hhu\r\n", isOnline ? "connected" : "disconnected", vid, pid, bus, addr);

  if(isOnline){
    for(size_t i = 0; i < countof(kUSBDevices); i++){
      const USBDeviceInfo* info = &kUSBDevices[i];
      if(info->vid == vid && info->pid == pid){
        new USBDeviceWrapper(dev, usb->GetContext(), info, bus, addr);
        break;
      }
    }
  }
  else{
    USBDeviceWrapper* tail = (USBDeviceWrapper*)devices;
    while(tail){
      if(tail->IsDevice(bus, addr)){
        tail->removed = true;
        break;
      }
      tail = (USBDeviceWrapper*)tail->next;
    }
  }
}

static volatile bool keepRunning = true;
void intHandler(int signal){
  keepRunning = false;
  printf("\r\nexiting on signal %d\r\n", signal);
}

extern "C"
int main(int argc, char *argv[]){
  USB usb;
  poll_fd fds[10];
  fd_set rfds, wfds;
  struct timeval tv;
  struct timeval* tv_ptr = &tv;
  int fd, maxfd, event_mask;

  signal(SIGINT, intHandler);

  tv_ptr = NULL;

  usb.Init();
  usb.name = "USB";
  pollers = addNode(pollers, &usb);

  // usb.ScanDevices();

  while(keepRunning){
    maxfd = -1;
    FD_ZERO(&rfds);
    FD_ZERO(&wfds);
    Poller* tail = (Poller*)pollers;

    while(tail){
      int rc = tail->getFds(fds);
      for(int i = 0; i < rc; i++){
        fd = fds[i].fd;
        event_mask = fds[i].event_mask;
        if(fd > maxfd) maxfd = fd;
        if(event_mask & FD_READ) FD_SET(fd, &rfds);
        if(event_mask & FD_WRITE) FD_SET(fd, &wfds);
      }
      tail = (Poller*)tail->next;
    }

    if(maxfd >= 0){
      if(tv_ptr){
        tv_ptr->tv_sec = 1;
        tv_ptr->tv_usec = 0;
      }
      int rc = select(maxfd + 1, &rfds, &wfds, NULL, tv_ptr);
      // printf("select: %d\r\n", rc);
      if(rc > 0){
        tail = (Poller*)pollers;
        while(tail){
          int rc = tail->getFds(fds);
          for(int i = 0; i < rc; i++){
            fd = fds[i].fd;
            event_mask = 0;
            if(fds[i].event_mask & FD_READ && FD_ISSET(fd, &rfds)) event_mask |= FD_READ;
            if(fds[i].event_mask & FD_WRITE && FD_ISSET(fd, &wfds)) event_mask |= FD_WRITE;
            if(event_mask){
              // printf("on %s fd: %d\r\n", tail->name, fd);
              tail->OnFDData({.fd = fd, .event_mask = event_mask});
            }
          }
          tail = (Poller*)tail->next;
        }
      }
      else if(rc < 0){
        if(keepRunning) fprintf(stderr, "select() failed (%d|%s)\r\n", errno, strerror(errno));
        keepRunning = false;
      }
    }
    else{
      fprintf(stderr, "exiting nothing to do\r\n");
      break;
    }

    USBDeviceWrapper* device = (USBDeviceWrapper*)devices;
    while(device){
      if(device->removed){
        USBDeviceWrapper* toRemove = device;
        device = (USBDeviceWrapper*)device->next;
        delete toRemove;
        continue;
      }
      device = (USBDeviceWrapper*)device->next;
    }
  }

  while(devices) delete (USBDeviceWrapper*)devices;
  pollers = removeNode(pollers, &usb);
  usb.DeInit();

  if(pollers || devices) printf("WARNING!!! pollers %p and devices: %p should be null for graceful exit\r\n", pollers, devices);
  else printf("bye :)\r\n");

  return 0;
}
