#!/usr/bin/env python

# Copyright (c) 2019, Plus One Robotics, Inc. All rights reserved.

import datetime
import logging
import re
import subprocess
## These usb.* depend on python-usb package on Ubuntu, which is not installed
## by default on Ubuntu.
#import usb.core
#import usb.util
import time


class RealsenseAccessor():
    ID_VENDOR = "8086"
    ID_PRODUCT = "0ad3"
    MSG_ = ""

    def __init__(self):
        self.devs = None
        logging.basicConfig(filename='realsense.log', level=logging.DEBUG)
        logging.info("---- Realsense status check ran at {} ----".format(datetime.datetime.fromtimestamp(time.time()).strftime('%Y%m%d%H%M%S')))

    def _device_notfound_subproc(self, id_manufacturer, id_product):
        """
        @rtype: [dict]
        @return: Example:
    
                            [{'device': '/dev/bus/usb/002/004', 'tag': 'Lenovo ', 'id': '17ef:305a'},
                             {'device': '/dev/bus/usb/002/001', 'tag': 'Linux Foundation 3.0 root hub', 'id': '1d6b:0003'},
                             {'device': '/dev/bus/usb/001/006', 'tag': 'Validity Sensors, Inc. ', 'id': '138a:0090'},,,]
    
        @note: This method doesn't depend on special library and should run on generic
            Linux installation (although tested on Ubuntu only as of Apr 2019). This method
            depends on Linux command (via subprocess), which makes this command
            platform-dependent. Ubuntu Xenial onward, a Python module that encapsulate
            platform operation becomes available so this method can be wiped out.
            See https://github.com/ros-drivers/openni2_camera/pull/80#discussion_r193295442
        """
        device_re = re.compile("Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+).+ID\s(?P<id>\w+:\w+)\s(?P<tag>.+)$", re.I)
        df = subprocess.check_output("lsusb")
        devices = []
        for i in df.split('\n'):
            if i:
                info = device_re.match(i)
                if info:
                    dinfo = info.groupdict()
                    logging.debug("dinfo: {}, dinfo.id: {}".format(dinfo, dinfo["id"]))
                    if dinfo["id"] == "{}:{}".format(id_manufacturer, id_product):
                        dinfo['device'] = "/dev/bus/usb/{}/{}".format(dinfo.pop('bus'), dinfo.pop('device'))
                        devices.append(dinfo)
        logging.info("#devices: {}\ndevices: {}".format(len(devices), devices))
        return devices

    def _find_usb_device(self):
        """
        @deprecated: Needs tested.
        @summary: Search Realsense in USB level. Equivalent to 'lsusb' command
            on Linux. This method is meant to be called by other public methods everytime
            they get called, as USB status can change during runtime.
        """
        dev = usb.core.find(idVendor=self.ID_VENDOR, idProduct=self.ID_PRODUCT)
        if dev:
            logging.info("USB device found: ", dev)
            self.dev = dev
        else:
            raise IOError("USB connection to Realsense is not found. Make sure it is plugged to the USB port.")

    def _find_usb_device_nopyusb(self):
        devices = self._device_notfound_subproc(self.ID_VENDOR, self.ID_PRODUCT)
        if devices:
            print("Num of USB device(s) found: {}".format(len(devices)))
            self.devs = devices
        else:
            raise IOError("USB connection to Realsense is not found. Make sure it is plugged to the USB port.")

    def status_usb(self):
        try:
            self._find_usb_device_nopyusb()
            return True
        except Exception as e:
            logging.error(str(e))
            return False

    def status_serial_number(self):
        """
        @summary: See if serial number(s) are found using Realsense' utility 
            called 'rs-enumerate-devices'.
        """
        ps = subprocess.Popen(("rs-enumerate-devices"), stdout=subprocess.PIPE)
        output = subprocess.check_output(('grep', "-i", "serial"), stdin=ps.stdout)
        ps.wait()
        logging.info(output)

    def status_usb_type(self):
        ps = subprocess.Popen(("rs-enumerate-devices"), stdout=subprocess.PIPE)
        output = subprocess.check_output(('grep', "Usb Type Descriptor"), stdin=ps.stdout)
        ps.wait()
        logging.info(output)

    def main(self):
        self.status_usb()
        self.status_serial_number()
        self.status_usb_type()
        
if __name__ == '__main__':
    rsa = RealsenseAccessor()
    rsa.main()
