#! /usr/bin/env python

import navio.adc
import navio.ms5611
import navio.util
import navio.ublox

import rospy
from yqb_car.msg import GPS


class GPSReader(object):
    def __init__(self):
        self.pub = rospy.Publisher('/sensor/gps', GPS, queue_size=1)
        self.ubl = navio.ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)
        self.initialize()

    def get_data(self):
        msg = self.ubl.receive_message()
        if msg is None:
            if opts.reopen:
                self.ubl.close()
                self.ubl = navio.ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)

        #print(msg.name())
        if msg.name() == "NAV_POSLLH":
            outstr = str(msg).split(",")[1:]
            print outstr
	    outstr = "".join(outstr)
            print outstr
        if msg.name() == "NAV_STATUS":
            outstr = str(msg).split(",")[1:2]
            print outstr
            outstr = "".join(outstr)
            print outstr
        #print(str(msg))

    def initialize(self):
        self.ubl.configure_poll_port()
        self.ubl.configure_poll(navio.ublox.CLASS_CFG, navio.ublox.MSG_CFG_USB)
        #ubl.configure_poll(navio.ublox.CLASS_MON, navio.ublox.MSG_MON_HW)

        self.ubl.configure_port(port=navio.ublox.PORT_SERIAL1, inMask=1, outMask=0)
        self.ubl.configure_port(port=navio.ublox.PORT_USB, inMask=1, outMask=1)
        self.ubl.configure_port(port=navio.ublox.PORT_SERIAL2, inMask=1, outMask=0)
        self.ubl.configure_poll_port()
        self.ubl.configure_poll_port(navio.ublox.PORT_SERIAL1)
        self.ubl.configure_poll_port(navio.ublox.PORT_SERIAL2)
        self.ubl.configure_poll_port(navio.ublox.PORT_USB)
        self.ubl.configure_solution_rate(rate_ms=1000)

        self.ubl.set_preferred_dynamic_model(None)
        self.ubl.set_preferred_usePPP(None)

        self.ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_POSLLH, 1)
        self.ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_PVT, 1)
        self.ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_STATUS, 1)
        self.ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_SOL, 1)
        self.ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_VELNED, 1)
        self.ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_SVINFO, 1)
        self.ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_VELECEF, 1)
        self.ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_POSECEF, 1)
        self.ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_RAW, 1)
        self.ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_SFRB, 1)
        self.ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_SVSI, 1)
        self.ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_ALM, 1)
        self.ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_EPH, 1)
        self.ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_TIMEGPS, 5)
        self.ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_CLOCK, 5)
        #ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_DGPS, 5)


if __name__ == "__main__":
    rospy.init_node('gps')
    gps_reader_object = GPSReader()

    rate = rospy.Rate(1000)

    while not rospy.is_shutdown():
        gps_reader_object.get_data()
        rate.sleep()
