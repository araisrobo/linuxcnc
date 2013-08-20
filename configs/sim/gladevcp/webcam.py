#!/usr/bin/python3

import gi
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst, Gtk

# Needed for window.get_xid(), xvimagesink.set_window_handle(), respectively:
from gi.repository import GdkX11, GstVideo

import sys, os, subprocess
from optparse import Option, OptionParser

options = [ Option( '-x', dest='parent', type=int, metavar='XID'
                  , help="Reparent webcam into an existing window XID instead of creating a new top level window")
          ]

GObject.threads_init()
Gst.init(None)

class Webcam:
    def __init__(self):
        
        usage = "usage: %prog [options] /dev/video_device_name"
        parser = OptionParser(usage=usage)
        parser.disable_interspersed_args()
        parser.add_options(options)
        
        (opts, args) = parser.parse_args()

        if not args:
            parser.print_help()
            sys.exit(1)
        
        self.window = Gtk.Window()
        self.window.set_default_size(1024, 600)
        
        xid = None
        if opts.parent:
            xid = int(opts.parent)
            plug = Gtk.Plug.new(xid)
            for c in self.window.get_children():
                self.window.remove(c)
                plug.add(c)
            self.window = plug

        self.drawingarea = Gtk.DrawingArea()
        self.window.add(self.drawingarea)
        self.window.connect('destroy', self.quit)

        # Create GStreamer pipeline
        self.pipeline = Gst.Pipeline()

        # Create bus to get events from GStreamer pipeline
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect('message::error', self.on_error)

        # This is needed to make the video output in our DrawingArea:
        self.bus.enable_sync_message_emission()
        self.bus.connect('sync-message::element', self.on_sync_message)

        # Create GStreamer elements

        # testing: self.src = Gst.ElementFactory.make('videotestsrc', None)
        self.src = Gst.ElementFactory.make('v4l2src', None)
        self.src.set_property("device", args[0])
        
        # cameracaps = gst.caps_from_string("video/x-raw, format=YV12 width=1280, height=720, framerate=30/1")
        cameracaps = Gst.Caps.from_string("image/jpeg, width=1280, height=720, framerate=30/1")
        self.camerafilter = Gst.ElementFactory.make("capsfilter", "filter") 
        self.camerafilter.set_property("caps", cameracaps)
        
        self.jpegdec = Gst.ElementFactory.make('jpegdec', 'jpegdec')

        # self.sink = Gst.ElementFactory.make('autovideosink', None)
        self.sink = Gst.ElementFactory.make('xvimagesink', None)
        self.sink.set_property("sync", "false")

        # Add elements to the pipeline
        self.pipeline.add(self.src)
        self.pipeline.add(self.camerafilter)
        self.pipeline.add(self.jpegdec)
        self.pipeline.add(self.sink)

        self.src.link(self.camerafilter)
        self.camerafilter.link(self.jpegdec)
        self.jpegdec.link(self.sink)
        
    def run(self):
        self.window.show_all()
        # You need to get the XID after window.show_all().  You shouldn't get it
        # in the on_sync_message() handler because threading issues will cause
        # segfaults there.
        self.xid = self.drawingarea.get_property('window').get_xid()
        self.pipeline.set_state(Gst.State.PLAYING)
        Gtk.main()

    def quit(self, window):
        self.pipeline.set_state(Gst.State.NULL)
        Gtk.main_quit()

    def on_sync_message(self, bus, msg):
        if msg.get_structure().get_name() == 'prepare-window-handle':
            print('prepare-window-handle')
            msg.src.set_property('force-aspect-ratio', True)
            msg.src.set_window_handle(self.xid)

    def on_error(self, bus, msg):
        print('on_error():', msg.parse_error())


webcam = Webcam()
webcam.run()
