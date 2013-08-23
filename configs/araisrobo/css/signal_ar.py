#!/usr/bin/env python
# -*- coding: UTF-8 -*
# vim: sts=4 sw=4 et
import os,sys
import hal
import glib
import hal_glib
import pygtk
pygtk.require('2.0')
import gtk
import numpy as np
import datetime

from gladevcp.persistence import IniFile,widget_defaults,set_debug,select_widgets

import ConfigParser
import pango
import linuxcnc
import arhmi.numpad as pad
import arhmi.emc_control as emc_control
        

class HandlerClass:
    '''
    class with gladevcp callback handlers
    '''

    def set_theme_font(self):
        # set ui theme
        config = ConfigParser.ConfigParser()
        fn = os.path.expanduser(".touchy_preferences")
        config.read(fn)
        theme_name = config.get("DEFAULT", 'gtk_theme')
        settings = gtk.settings_get_default()
        settings.set_string_property("gtk-theme-name", theme_name, "")
        
    def __init__(self, halcomp, builder, useropts):
        '''
        Handler classes are instantiated in the following state:
        - the widget tree is created, but not yet realized (no toplevel window.show() executed yet)
        - the halcomp HAL component is set up and the widhget tree's HAL pins have already been added to it
        - it is safe to add more hal pins because halcomp.ready() has not yet been called at this point.

        after all handlers are instantiated in command line and get_handlers() order, callbacks will be
        connected with connect_signals()/signal_autoconnect()

        The builder may be either of libglade or GtkBuilder type depending on the glade file format.
        '''
        self.halcomp = halcomp
        self.builder = builder
        self.set_theme_font()

def get_handlers(halcomp,builder,useropts):
    '''
    this function is called by gladevcp at import time (when this module is passed with '-u <modname>.py')

    return a list of object instances whose methods should be connected as callback handlers
    any method whose name does not begin with an underscore ('_') is a  callback candidate

    the 'get_handlers' name is reserved - gladevcp expects it, so do not change
    '''
    return [HandlerClass(halcomp,builder,useropts)]

