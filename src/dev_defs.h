#ifndef REG_DEV
#define REG_DEV(name, vid, pid, ifaces)
#endif
#ifndef REG_CDC_IFACE
#define REG_CDC_IFACE(label, if_no, tx_ep, rx_ep)
#endif

#ifndef REG_HCI_IFACE
#define REG_HCI_IFACE(label, if_no, cmd_tx_ep, evt_rx_ep, acl_tx_ep, acl_rx_ep)
#endif

#ifdef USE_INTEL
REG_DEV(AX200, 0x8087, 0x0029,
  REG_HCI_IFACE(hci0, 0x00, 0x00, 0x81, 0x02, 0x82)
)
#endif

#ifdef USE_BRCM
REG_DEV(BCM2070, 0x0a5c, 0x22be,
  REG_HCI_IFACE(hci0, 0x00, 0x00, 0x81, 0x02, 0x82)
)
#endif

REG_DEV(a7672x, 0x1e0e, 0x9011,
  /*
  REG_CDC_IFACE(rndis, 0x01, 0x0c, 0x83)
  REG_CDC_IFACE(diag1, 0x02, 0x0b, 0x82)
  REG_CDC_IFACE(cmd1, 0x03, 0x0f, 0x86)
  REG_CDC_IFACE(cmd2, 0x04, 0x0a, 0x81)
  REG_CDC_IFACE(diag2, 0x05, 0x0e, 0x85)
  */
  REG_CDC_IFACE(cmd1, 0x04, 0x0f, 0x86)
  REG_CDC_IFACE(cmd2, 0x05, 0x0a, 0x81)
)

REG_DEV(asr1603, 0x2ecc, 0x3004,
  REG_CDC_IFACE(aboot, 0x01, 0x02, 0x81)
)

/*
REG_DEV(ec20, 0x2c7c, 0x0125,
  REG_CDC_IFACE(nmea, 0x01, 0x02, 0x82)
  REG_CDC_IFACE(cmd1, 0x02, 0x03, 0x84)
  REG_CDC_IFACE(cmd2, 0x03, 0x04, 0x86)
)
*/

/*
REG_DEV(ec200u, 0x2c7c, 0x0901,
  REG_CDC_IFACE(cmd1, 0x02, 0x02, 0x83) // Quectel USB AT Port
  REG_CDC_IFACE(nmea, 0x03, 0x03, 0x84) // Quectel USB Diag Port
  REG_CDC_IFACE(dbg1, 0x04, 0x04, 0x85) // Quectel USB MOS Port
  REG_CDC_IFACE(dbg2, 0x05, 0x05, 0x86) // Quectel USB CP Log Port
  REG_CDC_IFACE(dbg3, 0x06, 0x06, 0x87) // Quectel USB AP Log Port
  REG_CDC_IFACE(cmd2, 0x07, 0x07, 0x88) // Quectel Modem
  REG_CDC_IFACE(cmd3, 0x08, 0x08, 0x89) // Quectel USB Serial-1 Port
  REG_CDC_IFACE(mgrp, 0x09, 0x09, 0x8a) // Quectel USB Serial-2 Port
)
*/

#undef REG_DEV
#undef REG_CDC_IFACE
#undef REG_HCI_IFACE
