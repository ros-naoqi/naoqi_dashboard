# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# Copyright (c) 2014, Aldebaran Robotics (c)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import dbus, gobject, dbus.glib

from python_qt_binding.QtGui import QComboBox

import collections

NetworkRobot = collections.namedtuple('NetworkRobot', ['name', 'address', 'port'])

#import avahi
class avahi:
    DBUS_NAME = "org.freedesktop.Avahi"
    DBUS_INTERFACE_SERVER = DBUS_NAME + ".Server"
    DBUS_PATH_SERVER = "/"
    DBUS_INTERFACE_ENTRY_GROUP = DBUS_NAME + ".EntryGroup"
    DBUS_INTERFACE_DOMAIN_BROWSER = DBUS_NAME + ".DomainBrowser"
    DBUS_INTERFACE_SERVICE_TYPE_BROWSER = DBUS_NAME + ".ServiceTypeBrowser"
    DBUS_INTERFACE_SERVICE_BROWSER = DBUS_NAME + ".ServiceBrowser"
    DBUS_INTERFACE_ADDRESS_RESOLVER = DBUS_NAME + ".AddressResolver"
    DBUS_INTERFACE_HOST_NAME_RESOLVER = DBUS_NAME + ".HostNameResolver"
    DBUS_INTERFACE_SERVICE_RESOLVER = DBUS_NAME + ".ServiceResolver"
    DBUS_INTERFACE_RECORD_BROWSER = DBUS_NAME + ".RecordBrowser"    
    PROTO_UNSPEC, PROTO_INET, PROTO_INET6  = -1, 0, 1
    IF_UNSPEC = -1
    LOOKUP_RESULT_CACHED = 1
    LOOKUP_RESULT_WIDE_AREA = 2
    LOOKUP_RESULT_MULTICAST = 4
    LOOKUP_RESULT_LOCAL = 8
    LOOKUP_RESULT_OUR_OWN = 16
    LOOKUP_RESULT_STATIC = 32


class AvahiWidget(QComboBox):
    def __init__(self):
        super(AvahiWidget, self).__init__()
        self.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.setInsertPolicy(QComboBox.InsertAlphabetically)
        self.setEditable(True)

        gobject.threads_init()
        dbus.glib.threads_init()
        self.robots = set()
        self.sys_bus = dbus.SystemBus()
        self.avahi_server = dbus.Interface(self.sys_bus.get_object(avahi.DBUS_NAME, '/'), avahi.DBUS_INTERFACE_SERVER)

        self.sbrowser = dbus.Interface(self.sys_bus.get_object(avahi.DBUS_NAME, self.avahi_server.ServiceBrowserNew(avahi.IF_UNSPEC,
            avahi.PROTO_INET, '_naoqi._tcp', 'local', dbus.UInt32(0))), avahi.DBUS_INTERFACE_SERVICE_BROWSER)

        self.sbrowser.connect_to_signal("ItemNew", self.avahiNewItem)
        self.sbrowser.connect_to_signal("ItemRemove", self.avahiItemRemove)

    def avahiNewItem(self, interface, protocol, name, stype, domain, flags):
        self.avahi_server.ResolveService(interface, protocol, name, stype, 
            domain, avahi.PROTO_INET, dbus.UInt32(0), 
            reply_handler=self.service_resolved, error_handler=self.print_error)

    def avahiItemRemove(self, interface, protocol, name, stype, domain, flags):
        tup = NetworkRobot(name, address, port)
        if tup in self.robots:
            self.robots.remove(tup)
        updateRobotCombobox();

    def service_resolved(self, interface, protocol, name, type, domain, host, aprotocol, address, port, txt, flags):
        self.robots.add(NetworkRobot(name, address, port))
        self.updateRobotCombobox()

    def updateRobotCombobox(self):
        selected = self.currentText()
        for i in range(self.count()):
            self.removeItem(0)
        id = -1
        texts = []
        for robot in self.robots:
            text = '%s:%s' % (robot.address, robot.port)
            text_full = "%s (%s)" % (robot.name, text)
            texts.append((text_full, text))
        texts = sorted(texts, key=lambda s: s[0].lower())
        for text in texts:
            self.addItem(text[0], text[1])
            if(text[0] == selected):
                id = self.count()-1;

        if(self.count() == 1):
            self.setCurrentIndex(0)
        elif(id > -1):
            self.setCurrentIndex(id)

    def print_error(self, *args):
        print('error_handler')
        print(args)
