#ifndef USB_H
#define USB_H
    
typedef enum {
    USB_STATUS_READY = 0,
    USB_STATUS_PROBE_HIT,// 1
    USB_STATUS_PROBING,//2
    USB_STATUS_PROBE_ERROR,//3
    USB_STATUS_PROBE_FLUSH_CMD,
    USB_STATUS_PROBE_WAIT_RESUME,
    USB_STATUS_ERROR
} usb_status_t;

typedef enum {
    USB_CMD_NOOP = 0,           /* no-operation */
    USB_CMD_ABORT,              /* abort current command */
    USB_CMD_PROBE_HIGH,         /* probing for probe.input changes from 0->1 */
    USB_CMD_PROBE_LOW,          /* probing for probe.input changes from 1->0 */
    USB_CMD_STATUS_ACK          /* ack to usb ater receiving USB_STATUS */
} usb_cmd_t;


#endif	/* USB_H */
