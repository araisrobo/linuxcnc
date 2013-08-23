#/usr/bin/env python
# -*- coding: UTF-8 -*
# vim: sts=4 sw=4 et
import os,sys
from gladevcp.persistence import IniFile,widget_defaults,set_debug,select_widgets
import hal
import hal_glib
import gtk
import glib
import time

import linuxcnc
import ConfigParser
import zmq
import pango

debug = 0

class EmcInterface(object):

    def __init__(self):
        try:
            # emcIniFile = linuxcnc.ini(os.environ['INI_FILE_NAME'])
            # linuxcnc.nmlfile = os.path.join(os.path.dirname(os.environ['INI_FILE_NAME']), emcIniFile.find("EMC", "NML_FILE"))
            self.s = linuxcnc.stat();
            self.c = linuxcnc.command()
        except Exception, msg:
            print "cant initialize EmcInterface: %s - EMC not running?" %(msg)

    def running(self,do_poll=True):
        if do_poll: self.s.poll()
        return self.s.task_mode == linuxcnc.MODE_AUTO and self.s.interp_state != linuxcnc.INTERP_IDLE

    def manual_ok(self,do_poll=True):
        if do_poll: self.s.poll()
        if self.s.task_state != linuxcnc.STATE_ON: return False
        return self.s.interp_state == linuxcnc.INTERP_IDLE

    def ensure_mode(self, m, *p):
        '''
        If emc is not already in one of the modes given, switch it to the first mode
        example:
        ensure_mode(linuxcnc.MODE_MDI)
        ensure_mode(linuxcnc.MODE_AUTO, linuxcnc.MODE_MDI)
        '''
        self.s.poll()
        if self.s.task_mode == m or self.s.task_mode in p: return True
        if self.running(do_poll=False): return False
        self.c.mode(m)
        self.c.wait_complete()
        return True

    def active_codes(self):
        self.s.poll()
        return self.s.gcodes
    
    def spindle_speed(self):
        self.s.poll()
        return self.s.spindle_speed
    
    def get_current_system(self):
        for i in self.active_codes():
                if i >= 540 and i <= 590:
                        return i/10 - 53
                elif i >= 590 and i <= 593:
                        return i - 584
        return 1

    def mdi_command(self, command, wait=True):
        if (not self.ensure_mode(linuxcnc.MODE_MDI)):
            print "cannot switch to MODE_MDI"
            return
        self.c.mdi(command)
        if wait: self.c.wait_complete()

    def emc_status(self):
        '''
        return tuple (task mode, task state, exec state, interp state) as strings
        '''
        self.s.poll()
        task_mode = ['invalid', 'MANUAL', 'AUTO', 'MDI'][self.s.task_mode]
        task_state = ['invalid', 'ESTOP', 'ESTOP_RESET', 'OFF', 'ON'][self.s.task_state]
        exec_state = ['invalid', 'ERROR', 'DONE',
                      'WAITING_FOR_MOTION',
                      'WAITING_FOR_MOTION_QUEUE',
                      'WAITING_FOR_IO',
                      'WAITING_FOR_PAUSE',
                      'WAITING_FOR_MOTION_AND_IO',
                      'WAITING_FOR_DELAY',
                      'WAITING_FOR_SYSTEM_CMD'][self.s.exec_state]
        interp_state = ['invalid', 'IDLE', 'READING', 'PAUSED', 'WAITING'][self.s.interp_state]
        return (task_mode, task_state, exec_state, interp_state)


class HandlerClass:
    '''
    class with gladevcp callback handlers
    '''
    
    def _query_emc_status(self,data=None):
        (task_mode, task_state, exec_state, interp_state) = self.e.emc_status()
        self.builder.get_object('task_mode').set_label("Task mode: " + task_mode)
        self.builder.get_object('task_state').set_label("Task state: " + task_state)
        self.builder.get_object('exec_state').set_label("Exec state: " + exec_state)
        self.builder.get_object('interp_state').set_label("Interp state: " + interp_state)
        # 為了左右邊的 mdi command 訊號可以同步
#        if(self.e.spindle_speed() > 0):
#            self.builder.get_object('do1').set_active(True)     # M3, SPINDLE.FWD
#            self.builder.get_object('do2').set_active(False)    # M4, SPINDLE.REV
#        elif(self.e.spindle_speed() < 0):
#            self.builder.get_object('do1').set_active(False)    # M3, SPINDLE.FWD
#            self.builder.get_object('do2').set_active(True)     # M4, SPINDLE.REV
#        else:
#            self.builder.get_object('do1').set_active(False)
#            self.builder.get_object('do2').set_active(False)

#        print self.e.active_modes()
        # looping: if (task_mode == "MANUAL") and (task_state == "ON") and (exec_state == "DONE") and (interp_state == "IDLE"):
        # looping:     # print ("task_mode: manual...")
        # looping:     # print ("about to cycle-start...")
        # looping:     # if self.emcstat.interp_state == self.emc.INTERP_IDLE:
        # looping:     self.e.c.mode(linuxcnc.MODE_AUTO)
        # looping:     self.e.c.wait_complete()    # linuxcnc.command
        # looping:     self.e.c.auto(linuxcnc.AUTO_RUN, 0)
        return True

#    def on_button_press(self,widget,data=None):
#        '''
#        a callback method
#        parameters are:
#            the generating object instance, likte a GtkButton instance
#            user data passed if any - this is currently unused but
#            the convention should be retained just in case
#        '''
#        print "on_button_press called"
#        self.nhits += 1
#        self.builder.get_object('hits').set_label("Hits: %d" % (self.nhits))
    
    def on_destroy(self,obj,data=None):
        self.ini.save_state(self)
    
#    def on_do7_toggled(self, widget, data=None):
#        if (not self.e.manual_ok(do_poll=True)):
#            # bypass issuing MDI when program is running
#            return        
#        label = gtk.Label("Click OK to TOOL-RELEASE")
#        dialog = gtk.Dialog("TOOL-RELEASE",
#                           None,
#                           gtk.DIALOG_MODAL | gtk.DIALOG_DESTROY_WITH_PARENT,
#                           (gtk.STOCK_CANCEL, gtk.RESPONSE_REJECT,
#                            gtk.STOCK_OK, gtk.RESPONSE_ACCEPT))
#        dialog.vbox.pack_start(label)
#        label.show()
#        
#        response = dialog.run()
#        if response == gtk.RESPONSE_ACCEPT:
#            print 'on_do7_toggled'
#            dialog.destroy()
#            if widget.get_active() == True:
#                self.e.mdi_command('M64 P8', True)  # release tool
#            else:
#                self.e.mdi_command('M65 P8', True)  # clamp tool
#        else:
#            dialog.destroy()
#    
#    def on_do1_toggled(self, widget, data=None):
#        # print 'debug: on_do1_toggled'
#        if (not self.e.manual_ok(do_poll=True)):
#            # bypass issuing MDI when program is running
#            return
#        if widget.get_active() == True:
#            self.e.mdi_command('M3', True)
#        else:
#            if(self.e.spindle_speed() != 0):
#                self.e.mdi_command('M5', True)
#            
#    def on_do2_toggled(self, widget, data=None):
#        # print 'debug: on_do2_toggled'
#        if (not self.e.manual_ok(do_poll=True)):
#            # bypass issuing MDI when program is running
#            return
#        if widget.get_active() == True:
#            self.e.mdi_command('M4', True)
#        else:
#            if(self.e.spindle_speed() != 0):
#                self.e.mdi_command('M5', True)
#            
#            
#    def on_spindle_brake_toggled(self, widget, data=None):
#        if (not self.e.manual_ok(do_poll=True)):
#            # bypass issuing MDI when program is running
#            return        
#
#        if (self.halcomp['spindle.brake'] == 0):
#            self.halcomp['spindle.brake'] = 1
##            self.e.mdi_command('M64 P9', True)  
#        else:
#            self.halcomp['spindle.brake'] = 0
##            self.e.mdi_command('M65 P9', True)  

    def on_restore_defaults(self,button,data=None):
        '''
        example callback for 'Reset to defaults' button
        currently unused
        '''
        self.ini.create_default_ini()
        self.ini.restore_state(self)

    def set_theme_font(self):
        # set ui theme
        config = ConfigParser.ConfigParser()
        fn = os.path.expanduser(".touchy_preferences")
        config.read(fn)
        
        o = config.get("DEFAULT", 'control_font')
        l = config.get("DEFAULT", 'listing_font')

        control_font = pango.FontDescription(o)

        #control_font = pango.FontDescription("San 24")

#        # set spin button font
#        for i in ["btn_scan_barcode"]:
#            w = self.builder.get_object(i)
#            if w:
#                w = w.child
#                w.modify_font(control_font)
        '''set label font'''
        self.listing_font = pango.FontDescription(l)
        for i in ["task_mode","task_state","exec_state","interp_state",
                  "label3","j4-enc_pos-label",
                  "label6","current-tool","label7","prepared-tool","label8",
                  "label9","label10","label11","label12","label13","label14","label15",
                  "label16","label17","label18","label19"]:
            w = self.builder.get_object(i)
            if w:
                w.modify_font(self.listing_font)        


        for i in ["prepared-tool","current-tool","j4-enc_pos-label"]:
            w = self.builder.get_object(i)
            if w:
                w.modify_font(control_font)        
                        
        theme_name = config.get("DEFAULT", 'gtk_theme')
        settings = gtk.settings_get_default()
        settings.set_string_property("gtk-theme-name", theme_name, "")

    def _on_update_status(self):
        evts = self.zmq_poller.poll(0)
        if evts:
            msg = self.zmq_server.recv()
            '''TODO: 偵測訊息內容'''
            if msg:
                pass
            self.zmq_server.send("message from vbc.py")
        return True
    def _delayed_sys_init_(self):
        self.builder.get_object("task_mode").hide()
        self.builder.get_object("task_state").hide()
        self.builder.get_object("exec_state").hide()
        self.builder.get_object("interp_state").hide()
        return False
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
        
        # TODO: add a signal to check if the relay for spindle-pump is ON
        halcomp.newpin("spindle.fwd", hal.HAL_BIT, hal.HAL_IN)
        halcomp.newpin("spindle.rev", hal.HAL_BIT, hal.HAL_IN)
        halcomp.newpin("spindle.jog-fwd", hal.HAL_BIT, hal.HAL_IN)
        halcomp.newpin("spindle.jog-rev", hal.HAL_BIT, hal.HAL_IN)
        halcomp.newpin("spindle.pump", hal.HAL_BIT, hal.HAL_OUT)
        halcomp.newpin("spindle.brake", hal.HAL_BIT, hal.HAL_OUT)
        

        self.halcomp = halcomp
        self.builder = builder
        self.nhits = 0
        self.set_theme_font()
        
        self.context = zmq.Context()
        self.zmq_server = self.context.socket(zmq.PAIR)
        self.zmq_server.bind('tcp://127.0.0.1:5556')
        self.zmq_poller = zmq.Poller()
        
        self.zmq_poller.register(self.zmq_server, zmq.POLLIN)                
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.PUSH)
        self.zmq_socket.connect('tcp://127.0.0.1:5555')
        
        self.ini_filename = __name__ + '.ini'
        self.defaults = {  IniFile.vars: dict(),
                           IniFile.widgets : widget_defaults(select_widgets(self.builder.get_objects(), hal_only=False,output_only = True))
                        }
        self.ini = IniFile(self.ini_filename,self.defaults,self.builder)
        self.ini.restore_state(self)

        self.e = EmcInterface()
        # program startup
        self.timer_id = glib.timeout_add(100, self._delayed_sys_init_) 
        glib.timeout_add_seconds(1, self._query_emc_status)
        glib.timeout_add(100, self._on_update_status)


def get_handlers(halcomp,builder,useropts):
    '''
    this function is called by gladevcp at import time (when this module is passed with '-u <modname>.py')

    return a list of object instances whose methods should be connected as callback handlers
    any method whose name does not begin with an underscore ('_') is a  callback candidate

    the 'get_handlers' name is reserved - gladevcp expects it, so do not change
    '''
    return [HandlerClass(halcomp,builder,useropts)]
