#!/usr/bin/env python

# import gi
# from gi.repository import gtk

from __future__ import print_function

import gtk
import pango
import ConfigParser

import sys, os, subprocess
from optparse import Option, OptionParser

options = [ Option( '-x', dest='parent', type=int, metavar='XID'
                  , help="Reparent into an existing window XID instead of creating a new top level window")
          ]

class Utility:
    def __init__(self):
        
        usage = "usage: %prog [options]"
        parser = OptionParser(usage=usage)
        parser.disable_interspersed_args()
        parser.add_options(options)
        self.set_theme_font()
        (opts, args) = parser.parse_args()

        self.window = gtk.Window()
        self.window.set_default_size(1024, 600)
        
        xid = None
        if opts.parent:
            xid = int(opts.parent)
            plug = gtk.Plug(xid)
            for c in self.window.get_children():
                self.window.remove(c)
                plug.add(c)
            self.window = plug

        self.window.connect('destroy', self.quit)
        
        self.table = gtk.Table(10,5)
        
        self.btn_hal= gtk.Button(label="HAL Configuration")
        self.btn_hal.set_border_width(4)
        self.btn_hal.connect("clicked", self.halshow_action)

        self.btn_halscope= gtk.Button(label="HAL Scope")
        self.btn_halscope.set_border_width(4)
        self.btn_halscope.connect("clicked", self.halscope_action)

        self.btn_ladder = gtk.Button(label="Ladder Logic")
        self.btn_ladder.set_border_width(4)
        self.btn_ladder.connect("clicked", self.ladder_action)

        self.btn_teamviewer = gtk.Button(label="Teamviewer")
        self.btn_teamviewer.set_border_width(4)
        self.btn_teamviewer.connect("clicked", self.teamviewer_action)
        
        self.btn_nautilus = gtk.Button(label="File Manager")
        self.btn_nautilus.set_border_width(4)
        self.btn_nautilus.connect("clicked", self.nautilus_action)

        self.btn_linuxcncop = gtk.Button(label="Linuxcnc Status")
        self.btn_linuxcncop.set_border_width(4)
        self.btn_linuxcncop.connect("clicked", self.linuxcnc_status)
        
        self.table.attach(self.btn_hal,         0, 1, 0, 1)
        self.table.attach(self.btn_ladder,      0, 1, 1, 2)
        self.table.attach(self.btn_linuxcncop,  0, 1, 2, 3)
        self.table.attach(self.btn_halscope,    1, 2, 0, 1)
        self.table.attach(self.btn_teamviewer,  1, 2, 1, 2)
        self.table.attach(self.btn_nautilus,    1, 2, 2, 3)
        self.window.add(self.table)
  
    def teamviewer_action(self, btn):
        if (os.path.exists("/usr/bin/teamviewer") == True):
            os.system("/usr/bin/teamviewer &")
    
    def nautilus_action(self, btn):
        if (os.path.exists("/usr/bin/nautilus") == True):
            os.system("/usr/bin/nautilus &")
    
    def ladder_action(self, btn):
        os.system('halcmd loadusr -w classicladder &')
        return

    def linuxcnc_status(self, btn):
        os.system('linuxcnctop &')
        return
    
    def halshow_action(self, btn):
        tcl_home =  os.environ['LINUXCNC_TCL_DIR']
        os.system('tclsh ' + tcl_home + '/bin/halshow.tcl &')
        return

    def halscope_action(self, btn):
        os.system('halscope &')
        return

    def run(self):
        self.window.show_all()
        gtk.main()

    def quit(self, window):
        gtk.main_quit()
        
    def set_theme_font(self):
        # set ui theme
        config = ConfigParser.ConfigParser()
        fn = os.path.expanduser(".touchy_preferences")
        config.read(fn)
        theme_name = config.get("DEFAULT", 'gtk_theme')
        settings = gtk.settings_get_default()
        settings.set_string_property("gtk-theme-name", theme_name, "")
        
utility = Utility()
utility.run()
