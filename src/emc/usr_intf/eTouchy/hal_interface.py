# Touchy is Copyright (c) 2009  Chris Radek <chris@timeguy.com>
#
# Touchy is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# Touchy is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.



import hal

class hal_interface:
    def __init__(self, gui, emc_control, mdi_control):
        print "touchy.hal init ++++"
        self.gui = gui
        self.emc_control = emc_control
        self.mdi_control = mdi_control
        self.c = hal.component("touchy")
        self.c.newpin("jog.active", hal.HAL_BIT, hal.HAL_OUT)
        self.c.newpin("jog.wheel.x", hal.HAL_BIT, hal.HAL_OUT)
        self.c.newpin("jog.wheel.y", hal.HAL_BIT, hal.HAL_OUT)
        self.c.newpin("jog.wheel.z", hal.HAL_BIT, hal.HAL_OUT)
        self.c.newpin("jog.wheel.a", hal.HAL_BIT, hal.HAL_OUT)
        self.c.newpin("jog.wheel.b", hal.HAL_BIT, hal.HAL_OUT)
        self.c.newpin("jog.wheel.c", hal.HAL_BIT, hal.HAL_OUT)
        self.c.newpin("jog.wheel.u", hal.HAL_BIT, hal.HAL_OUT)
        self.c.newpin("jog.wheel.v", hal.HAL_BIT, hal.HAL_OUT)
        self.c.newpin("jog.wheel.w", hal.HAL_BIT, hal.HAL_OUT)
        self.c.newpin("jog.wheel.increment", hal.HAL_FLOAT, hal.HAL_OUT)

        self.c.newpin("jog.continuous.x.positive", hal.HAL_BIT, hal.HAL_IN)
        self.xp = 0
        self.c.newpin("jog.continuous.x.negative", hal.HAL_BIT, hal.HAL_IN)
        self.xn = 0
        self.c.newpin("jog.continuous.y.positive", hal.HAL_BIT, hal.HAL_IN)
        self.yp = 0
        self.c.newpin("jog.continuous.y.negative", hal.HAL_BIT, hal.HAL_IN)
        self.yn = 0
        self.c.newpin("jog.continuous.z.positive", hal.HAL_BIT, hal.HAL_IN)
        self.zp = 0
        self.c.newpin("jog.continuous.z.negative", hal.HAL_BIT, hal.HAL_IN)
        self.zn = 0

        self.c.newpin("jog.continuous.a.positive", hal.HAL_BIT, hal.HAL_IN)
        self.ap = 0
        self.c.newpin("jog.continuous.a.negative", hal.HAL_BIT, hal.HAL_IN)
        self.an = 0
        self.c.newpin("jog.continuous.b.positive", hal.HAL_BIT, hal.HAL_IN)
        self.bp = 0
        self.c.newpin("jog.continuous.b.negative", hal.HAL_BIT, hal.HAL_IN)
        self.bn = 0
        self.c.newpin("jog.continuous.c.positive", hal.HAL_BIT, hal.HAL_IN)
        self.cp = 0
        self.c.newpin("jog.continuous.c.negative", hal.HAL_BIT, hal.HAL_IN)
        self.cn = 0

        self.c.newpin("jog.continuous.u.positive", hal.HAL_BIT, hal.HAL_IN)
        self.up = 0
        self.c.newpin("jog.continuous.u.negative", hal.HAL_BIT, hal.HAL_IN)
        self.un = 0
        self.c.newpin("jog.continuous.v.positive", hal.HAL_BIT, hal.HAL_IN)
        self.vp = 0
        self.c.newpin("jog.continuous.v.negative", hal.HAL_BIT, hal.HAL_IN)
        self.vn = 0
        self.c.newpin("jog.continuous.w.positive", hal.HAL_BIT, hal.HAL_IN)
        self.wp = 0
        self.c.newpin("jog.continuous.w.negative", hal.HAL_BIT, hal.HAL_IN)
        self.wn = 0

        self.xp_sw = 0
        self.xn_sw = 0
        self.yp_sw = 0
        self.yn_sw = 0
        self.zp_sw = 0
        self.zn_sw = 0
        self.ap_sw = 0
        self.an_sw = 0
        self.bp_sw = 0
        self.bn_sw = 0
        self.cp_sw = 0
        self.cn_sw = 0
        self.up_sw = 0
        self.un_sw = 0
        self.vp_sw = 0
        self.vn_sw = 0
        self.wp_sw = 0
        self.wn_sw = 0
        self.sw_button_presented = 0
        self.cycle_start_stop = 0
        self.cycle_button_trigged = 0

        self.c.newpin("quill-up", hal.HAL_BIT, hal.HAL_IN)
        self.quillup = 0
        self.c.newpin("cycle-start", hal.HAL_BIT, hal.HAL_IN)
        self.cyclestart = 0
        self.c.newpin("abort", hal.HAL_BIT, hal.HAL_IN)
        self.abort = 0
        self.c.newpin("single-block", hal.HAL_BIT, hal.HAL_IN)
        self.singleblock = 0
        self.c.newpin("wheel-counts", hal.HAL_S32, hal.HAL_IN)
        self.counts = 0
        self.jog_velocity = 1
        self.c.ready()
        self.active = 0
        self.jogaxis(0)
        print "touchy.hal init -----"

    def wheel(self):# read the difference between now and last wheel count
        counts = self.c["wheel-counts"]/4
        ret = counts - self.counts
        self.counts = counts
        return ret

    def jogaxis(self, n):
        # n =0 and self.active => self.c["jog.wheel.x"] = TRUE
        self.c["jog.wheel.x"] = n == 0 and self.active
        self.c["jog.wheel.y"] = n == 1 and self.active
        self.c["jog.wheel.z"] = n == 2 and self.active
        self.c["jog.wheel.a"] = n == 3 and self.active
        self.c["jog.wheel.b"] = n == 4 and self.active
        self.c["jog.wheel.c"] = n == 5 and self.active
        self.c["jog.wheel.u"] = n == 6 and self.active
        self.c["jog.wheel.v"] = n == 7 and self.active
        self.c["jog.wheel.w"] = n == 8 and self.active
       
        
    def jogincrement(self, inc):
        incs = [0.01, 0.001, 0.0001]
        self.c["jog.wheel.increment"] = incs[inc]

    def jogactive(self, active):
        self.active = active
        self.c["jog.active"] = active;
        if(active==0):
            self.stopjog()
    def setjogplus(self,n):
        self.xp_sw = n == 0 
        self.yp_sw = n == 1 
        self.zp_sw = n == 2 
        self.ap_sw = n == 3 
        self.bp_sw = n == 4 
        self.cp_sw = n == 5 
        self.up_sw = n == 6 
        self.vp_sw = n == 7 
        self.wp_sw = n == 8 
        self.sw_button_presented = 1
    def stopjog(self):
        self.xp_sw = 0
        self.yp_sw = 0 
        self.zp_sw = 0
        self.ap_sw = 0 
        self.bp_sw = 0 
        self.cp_sw = 0
        self.up_sw = 0
        self.vp_sw = 0 
        self.wp_sw = 0
        self.xn_sw = 0 
        self.yn_sw = 0  
        self.zn_sw = 0 
        self.an_sw = 0 
        self.bn_sw = 0 
        self.cn_sw = 0 
        self.un_sw = 0 
        self.vn_sw = 0 
        self.wn_sw = 0   
        self.sw_button_presented = 0  
    def setjogminus(self,n):
        self.xn_sw = n == 0  
        self.yn_sw = n == 1  
        self.zn_sw = n == 2  
        self.an_sw = n == 3  
        self.bn_sw = n == 4  
        self.cn_sw = n == 5  
        self.un_sw = n == 6 
        self.vn_sw = n == 7  
        self.wn_sw = n == 8 
        self.sw_button_presented = 1
    def set_cycle_start_stop(self,n):
        print "set_cycle_start_stop",n
        self.cycle_start_stop = n
        self.cycle_button_trigged = 1
        
    def periodic(self, mdi_mode):
        # edge detection
        # print "hal_interface periodic +++++"
        xp = self.c["jog.continuous.x.positive"]
        if self.sw_button_presented == 1:
            xp = self.xp_sw
        if xp ^ self.xp: 
            self.emc_control.continuous_jog(0, xp)
        self.xp = xp

        xn = self.c["jog.continuous.x.negative"]
        if self.sw_button_presented == 1:
            xn = self.xn_sw
        if xn ^ self.xn:
            self.emc_control.continuous_jog(0, -xn)
        self.xn = xn

        yp = self.c["jog.continuous.y.positive"]
        if self.sw_button_presented == 1:
            yp = self.yp_sw
        if yp ^ self.yp: self.emc_control.continuous_jog(1, yp)
        self.yp = yp

        yn = self.c["jog.continuous.y.negative"]
        if self.sw_button_presented == 1:
            yn = self.yn_sw
        if yn ^ self.yn: self.emc_control.continuous_jog(1, -yn)
        self.yn = yn

        zp = self.c["jog.continuous.z.positive"]
        if self.sw_button_presented == 1:
            zp = self.zp_sw
        if zp ^ self.zp: self.emc_control.continuous_jog(2, zp)
        self.zp = zp

        zn = self.c["jog.continuous.z.negative"]
        if self.sw_button_presented == 1:
            zn = self.zn_sw
        if zn ^ self.zn: self.emc_control.continuous_jog(2, -zn)
        self.zn = zn

        ap = self.c["jog.continuous.a.positive"]
        if self.sw_button_presented == 1:
            ap = self.ap_sw
        if ap ^ self.ap: self.emc_control.continuous_jog(3, ap)
        self.ap = ap

        an = self.c["jog.continuous.a.negative"]
        if self.sw_button_presented == 1:
            an = self.an_sw
        if an ^ self.an: self.emc_control.continuous_jog(3, -an)
        self.an = an

        bp = self.c["jog.continuous.b.positive"]
        if self.sw_button_presented == 1:
            bp = self.bp_sw
        if bp ^ self.bp: self.emc_control.continuous_jog(4, bp)
        self.bp = bp
   
        bn = self.c["jog.continuous.b.negative"]
        if self.sw_button_presented == 1:
            bn = self.bn_sw
        if bn ^ self.bn: self.emc_control.continuous_jog(4, -bn)
        self.bn = bn

        cp = self.c["jog.continuous.c.positive"]
        if self.sw_button_presented == 1:
            cp = self.cp_sw
        if cp ^ self.cp: self.emc_control.continuous_jog(5, cp)
        self.cp = cp

        cn = self.c["jog.continuous.c.negative"]
        if self.sw_button_presented == 1:
            cn = self.cn_sw
        if cn ^ self.cn: self.emc_control.continuous_jog(5, -cn)
        self.cn = cn

        up = self.c["jog.continuous.u.positive"]
        if self.sw_button_presented == 1:
            up = self.up_sw
        if up ^ self.up: self.emc_control.continuous_jog(6, up)
        self.up = up

        un = self.c["jog.continuous.u.negative"]
        if self.sw_button_presented == 1:
            un = self.un_sw
        if un ^ self.un: self.emc_control.continuous_jog(6, -un)
        self.un = un

        vp = self.c["jog.continuous.v.positive"]
        if self.sw_button_presented == 1:
            vp = self.vp_sw
        if vp ^ self.vp: self.emc_control.continuous_jog(7, vp)
        self.vp = vp

        vn = self.c["jog.continuous.v.negative"]
        if self.sw_button_presented == 1:
            vn = self.vn_sw
        if vn ^ self.vn: self.emc_control.continuous_jog(7, -vn)
        self.vn = vn

        wp = self.c["jog.continuous.w.positive"]
        if self.sw_button_presented == 1:
            wp = self.wp_sw
        if wp ^ self.wp: self.emc_control.continuous_jog(8, wp)
        self.wp = wp

        wn = self.c["jog.continuous.w.negative"]
        if self.sw_button_presented == 1:
            wn = self.wn_sw
        if wn ^ self.wn: self.emc_control.continuous_jog(8, -wn)
        self.wn = wn
        
        quillup = self.c["quill-up"]
        if quillup and not self.quillup: 
            self.emc_control.quill_up()
        self.quillup = quillup

        cyclestart = self.c["cycle-start"]
        if self.cycle_button_trigged == 1:
            cyclestart = self.cycle_start_stop
            self.cycle_button_trigged = 0
            self.cycle_start_stop = 0
            if self.cycle_start_stop == 0:
                self.emc_control.abort()
        if cyclestart and not self.cyclestart:
            print "cyclestart"
            if self.gui.wheel == "jogging":
                print "set gui wheel mv" 
                self.gui.wheel = "mv"
            self.gui.jogsettings_activate(0)
            if mdi_mode:
                print "in mdi_mode"
                self.mdi_control.ok(0)
            else:
                print "cycle_start"
                self.emc_control.cycle_start()
        self.cyclestart = cyclestart

        abort = self.c["abort"]
        if abort and not self.abort:
            print "allow abort" 
            self.emc_control.abort()
        self.abort = abort

        singleblock = self.c["single-block"]
        if singleblock ^ self.singleblock: self.emc_control.single_block(singleblock)
        self.singleblock = singleblock
        #print "hal_interface periodic -----"
        
