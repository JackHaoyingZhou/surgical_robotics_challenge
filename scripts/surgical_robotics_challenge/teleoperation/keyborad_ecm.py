import os
import sys
import numpy as np
import usb.core
import usb.util
import time

dynamic_path = os.path.abspath(__file__+"/../../../")
print(dynamic_path)
sys.path.append(dynamic_path)


if __name__ == '__main__':
    USB_IF = 0  # Interface
    USB_TIMEOUT = 5  # Timeout in MS

    USB_VENDOR = 0x046d  # Rii
    # USB_PRODUCT = 0xc33e  # Mini Wireless Keyboard
    USB_PRODUCT = 0xc541  # Mini Wireless Keyboard

    dev = usb.core.find(idVendor=USB_VENDOR, idProduct=USB_PRODUCT)

    # cfg = dev.get_active_configuration()

    endpoint = dev[0][(0, 0)][0]

    if dev.is_kernel_driver_active(USB_IF) is True:
        dev.detach_kernel_driver(USB_IF)

    usb.util.claim_interface(dev, USB_IF)

    while True:
        control = None
        try:
            control = dev.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize, USB_TIMEOUT)
            print(control)
        except:
            pass
    time.sleep(0.01)  # Let CTRL+C actually exit