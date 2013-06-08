#    This is a component of AXIS, a front-end for emc
#    Copyright 2004, 2005, 2006 Jeff Epler <jepler@unpythonic.net>
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 2 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program; if not, write to the Free Software
#    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

from rs274 import Translated, ArcsToSegmentsMixin, OpenGLTk
from minigl import *
import math
import glnav
import hershey
import linuxcnc
import array
import gcode
import gettext
_=gettext.gettext

from scipy import mat
from scipy import linalg
from numpy import cross
from numpy import linalg as LA

def minmax(*args):
    return min(*args), max(*args)

homeicon = array.array('B',
        [0x2, 0x00,   0x02, 0x00,   0x02, 0x00,   0x0f, 0x80,
        0x1e, 0x40,   0x3e, 0x20,   0x3e, 0x20,   0x3e, 0x20,
        0xff, 0xf8,   0x23, 0xe0,   0x23, 0xe0,   0x23, 0xe0,
        0x13, 0xc0,   0x0f, 0x80,   0x02, 0x00,   0x02, 0x00])

limiticon = array.array('B',
        [  0,   0,  128, 0,  134, 0,  140, 0,  152, 0,  176, 0,  255, 255,
         255, 255,  176, 0,  152, 0,  140, 0,  134, 0,  128, 0,    0,   0,
           0,   0,    0, 0])

class GLCanon(Translated, ArcsToSegmentsMixin):
    lineno = -1
    def __init__(self, colors, geometry, is_foam=0):
        self.path = []
        # traverse list - [line number, [start position], [end position], [tlo x, tlo y, tlo z]]
        self.traverse = []; self.traverse_append = self.traverse.append
        # all_traverse list - [line number , [start position], [end position], feedrate, length]
        self.all_traverse = []; self.all_traverse_append = self.all_traverse.append
        # feed list -     [line number, [start position], [end position], feedrate, [tlo x, tlo y, tlo z]]
        self.feed = []; self.feed_append = self.feed.append
        # feed info list - [line number, [start position], [end position], length]
        self.feed_info = []; self.feed_info_append = self.feed_info.append
        # arcfeed list -  [line number, [start position], [end position], feedrate, [tlo x, tlo y, tlo z]]
        self.arcfeed = []; self.arcfeed_append = self.arcfeed.append
        # arc info list - [line number, [c_x,c_y], [s_x, s_y], [e_x, e_y], length, cw]
        self.arc_info = []; self.arc_info_append = self.arc_info.append
        # dwell list - [line number, color, pos x, pos y, pos z, plane]
        self.dwells = []; self.dwells_append = self.dwells.append
        # block path list - [start line, end line, [start position], feedrate]
        self.blocks = []; self.blocks_append = self.blocks.append
        self.block_start = None
        self.highlight_mode = 'line'
        self.selected_block = None
        self.block_pos = []
        self.block_feed = 0
        self.choice = None
        self.feedrate = 1
        self.lo = self.rotate_and_translate(x=0,y=0,z=0,a=0,b=0,c=0,u=0,v=0,w=0)
        self.first_move = True
        self.geometry = geometry
        self.min_extents = [9e99,9e99,9e99]
        self.max_extents = [-9e99,-9e99,-9e99]
        self.min_extents_notool = [9e99,9e99,9e99]
        self.max_extents_notool = [-9e99,-9e99,-9e99]
        self.colors = colors
        self.in_arc = 0
        self.xo = self.yo = self.zo = self.ao = self.bo = self.co = self.uo = self.vo = self.wo = 0
        self.dwell_time = 0
        self.suppress = 0
        self.g92_offset_x = 0.0
        self.g92_offset_y = 0.0
        self.g92_offset_z = 0.0
        self.g92_offset_a = 0.0
        self.g92_offset_b = 0.0
        self.g92_offset_c = 0.0
        self.g92_offset_u = 0.0
        self.g92_offset_v = 0.0
        self.g92_offset_w = 0.0
        self.g5x_index = 1
        self.g5x_offset_x = 0.0
        self.g5x_offset_y = 0.0
        self.g5x_offset_z = 0.0
        self.g5x_offset_a = 0.0
        self.g5x_offset_b = 0.0
        self.g5x_offset_c = 0.0
        self.g5x_offset_u = 0.0
        self.g5x_offset_v = 0.0
        self.g5x_offset_w = 0.0
        self.diff = [0.0, 0.0, 0.0, 0.0]
        self.prev_diff = [0.0, 0.0, 0.0, 0.0]
        self.tool_scale = 1.0
        self.is_foam = is_foam
        self.foam_z = 0
        self.foam_w = 1.5
        self.notify = 0
        self.notify_message = ""

    def comment(self, arg):
        if arg.startswith("AXIS,"):
            parts = arg.split(",")
            command = parts[1]
            # if command == "stop": raise KeyboardInterrupt
            if command == "hide": self.suppress += 1
            if command == "show": self.suppress -= 1
            if command == "XY_Z_POS": 
                if len(parts) > 2 :
                    try:
                        self.foam_z = float(parts[2])
                        if 210 in self.state.gcodes:
                            self.foam_z = self.foam_z / 25.4
                    except:
                        self.foam_z = 5.0/25.4
            if command == "UV_Z_POS": 
                if len(parts) > 2 :
                    try:
                        self.foam_w = float(parts[2])
                        if 210 in self.state.gcodes:
                            self.foam_w = self.foam_w / 25.4
                    except:
                        self.foam_w = 30.0
            if command == "notify":
                self.notify = self.notify + 1
                self.notify_message = "(AXIS,notify):" + str(self.notify)
                if len(parts) > 2:
                    if len(parts[2]): self.notify_message = parts[2]

    def message(self, message): pass

    def check_abort(self): pass

    def next_line(self, st):
        self.state = st
        self.lineno = self.state.sequence_number

    def draw_lines(self, lines, for_selection, j=0, geometry=None):
        return linuxcnc.draw_lines(geometry or self.geometry, lines, for_selection)

    def colored_lines(self, color, lines, for_selection, j=0):
        if self.is_foam:
            if not for_selection:
                self.color_with_alpha(color + "_xy")
            glPushMatrix()
            glTranslatef(0, 0, self.foam_z)
            self.draw_lines(lines, for_selection, 2*j, 'XY')
            glPopMatrix()
            if not for_selection:
                self.color_with_alpha(color + "_uv")
            glPushMatrix()
            glTranslatef(0, 0, self.foam_w)
            self.draw_lines(lines, for_selection, 2*j+len(lines), 'UV')
            glPopMatrix()
        else:
            if not for_selection:
                self.color_with_alpha(color)
            self.draw_lines(lines, for_selection, j)

    def draw_dwells(self, dwells, alpha, for_selection, j0=0):
        return linuxcnc.draw_dwells(self.geometry, dwells, alpha, for_selection, self.is_lathe())

    def calc_extents(self):
        self.min_extents, self.max_extents, self.min_extents_notool, self.max_extents_notool = gcode.calc_extents(self.arcfeed, self.feed, self.traverse)
        if self.is_foam:
            min_z = min(self.foam_z, self.foam_w)
            max_z = max(self.foam_z, self.foam_w)
            self.min_extents = self.min_extents[0], self.min_extents[1], min_z
            self.max_extents = self.max_extents[0], self.max_extents[1], max_z
            self.min_extents_notool = \
                self.min_extents_notool[0], self.min_extents_notool[1], min_z
            self.max_extents_notool = \
                self.max_extents_notool[0], self.max_extents_notool[1], max_z
    def tool_offset(self, xo, yo, zo, ao, bo, co, uo, vo, wo):
        self.first_move = True
        x, y, z, a, b, c, u, v, w = self.lo
        self.lo = (x - xo + self.xo, y - yo + self.yo, z - zo + self.zo, a - ao + self.ao, b - bo + self.bo, c - bo + self.bo,
          u - uo + self.uo, v - vo + self.vo, w - wo + self.wo)
        self.xo = xo
        self.yo = yo
        self.zo = zo
        self.so = ao
        self.bo = bo
        self.co = co
        self.uo = uo
        self.vo = vo
        self.wo = wo

    def set_spindle_rate(self, arg): pass
    def set_feed_rate(self, arg): self.feedrate = arg / 60.
    def select_plane(self, arg): pass

    def change_tool(self, arg):
        self.first_move = True

    def straight_traverse(self, x,y,z, a,b,c, u, v, w):
        if self.suppress > 0: return
        l = self.rotate_and_translate(x,y,z,a,b,c,u,v,w)
        # calculate length
        length_vector = []
        length = 0.0
        for i in range (0, (len(l)-1)):
            length_vector.append(l[i] - self.lo[i])
        length = LA.norm(length_vector)

        if not self.first_move:
            self.traverse_append((self.lineno, self.lo, l, [self.xo, self.yo, self.zo]))
            
        self.all_traverse_append([self.lineno, self.lo, l, self.feedrate, length])
        self.path.append(('traverse', self.lineno, self.lo, l, self.feedrate, length))
        
        self.lo = l
        
    def spindle_sync_motion(self, x, y, z, ssm_mode):
        if self.suppress > 0: return
        self.first_move = False
        l = self.rotate_and_translate(x,y,z,0,0,0,0,0,0)[:3]
        l += [self.lo[3], self.lo[4], self.lo[5],
               self.lo[6], self.lo[7], self.lo[8]]
        self.feed_append((self.lineno, self.lo, l, self.feedrate, [self.xo, self.yo, self.zo]))
        # calculate length
        length_vector = []
        length = 0.0
        for i in range (0, (len(l)-1)):
            length_vector.append(l[i] - self.lo[i])
        length = LA.norm(length_vector)
        self.feed_info_append((self.lineno, self.lo, l, self.feedrate, [self.xo, self.yo, self.zo], length))
        self.path.append(('feed', self.lineno, self.lo, l, self.feedrate, [self.xo, self.yo, self.zo], length))
        
        if (ssm_mode == 0):
            """ G33 """
            self.lo = l
        elif (ssm_mode == 1):
            """ G33.1 RIGID_TAP """
            self.feed_append((self.lineno, l, self.lo, self.feedrate, [self.xo, self.yo, self.zo]))
            self.feed_info_append((self.lineno, l, self.lo, self.feedrate, [self.xo, self.yo, self.zo], length))
            self.path.append(('feed', self.lineno, l, self.lo, self.feedrate, [self.xo, self.yo, self.zo], length))
        else:
            assert(0)
            
    def arc_feed(self, *args):
        if self.suppress > 0: return
        self.first_move = False
        self.in_arc = True
        try:
            ArcsToSegmentsMixin.arc_feed(self, *args)
        finally:
            self.in_arc = False

    def straight_arcsegments(self, segs):
        self.first_move = False
        lo = self.lo
        lineno = self.lineno
        feedrate = self.feedrate
        to = [self.xo, self.yo, self.zo]
        append = self.arcfeed_append
        points = []
        length = 0.0
        length_vector = []
        vector = []
        vector2 = []
        cw = 0 # 0 is clockwise, 1 is counter-clockwise

        for l in segs:
            # if self.block_start != None and self.block_pos == None:
            #     block_pos = self.l
            append((lineno, lo, l, feedrate, to))
            
            # calculate the length from lo to l
            for i in range (0, (len(l)-1)):
                length_vector.append(l[i] - lo[i])
            length = length + LA.norm(length_vector)
            length_vector = []
            lo = l
        # print "arc length is", length  
        self.lo = lo
        
        for i in range (0, 2):
            vector.append(segs[len(segs)/3][i] - segs[0][i])
            vector2.append(segs[len(segs)*2/3][i] - segs[len(segs)/3][i])
        if cross(vector, vector2) >= 0:
            cw = 1
        else:
            cw = 0

        points.append(segs[0])
        points.append(segs[len(segs)/3])
        points.append(segs[len(segs)/3*2])

        A = mat([[ points[0][0],     points[0][1], 1], \
                 [ points[1][0],     points[1][1], 1], \
                 [ points[2][0],     points[2][1], 1]])
        b = mat([[-points[0][0]**2 - points[0][1]**2], \
                 [-points[1][0]**2 - points[1][1]**2], \
                 [-points[2][0]**2 - points[2][1]**2]])
        d, e, f = linalg.solve(A,b)
        self.arc_info_append([lineno,[-d/2,-e/2],segs[0],segs[len(segs)-1], length, cw])
        self.path.append(('arc', lineno, [-d/2,-e/2], segs[0], segs[len(segs)-1], length, cw))
        # print "self.arc_info", self.arc_info

    def straight_feed(self, x,y,z, a,b,c, u,v,w):
        if self.suppress > 0: return
        self.first_move = False
        l = self.rotate_and_translate(x,y,z,a,b,c,u,v,w)
        self.feed_append((self.lineno, self.lo, l, self.feedrate, [self.xo, self.yo, self.zo]))

        # calculate length
        length_vector = []
        length = 0.0
        for i in range (0, (len(l)-1)):
            length_vector.append(l[i] - self.lo[i])
        length = LA.norm(length_vector)
        # print "line length is", length
        self.feed_info_append((self.lineno, self.lo, l, self.feedrate, [self.xo, self.yo, self.zo], length))
        self.path.append(('feed', self.lineno, self.lo, l, self.feedrate, [self.xo, self.yo, self.zo], length))
        # print "self.feed_info", self.feed_info

        self.lo = l
        
    straight_probe = straight_feed

    def user_defined_function(self, i, p, q):
        if self.suppress > 0: return
        color = self.colors['m1xx']
        self.dwells_append((self.lineno, color, self.lo[0], self.lo[1], self.lo[2], self.state.plane/10-17))

    def dwell(self, arg):
        if self.suppress > 0: return
        self.dwell_time += arg
        color = self.colors['dwell']
        self.dwells_append((self.lineno, color, self.lo[0], self.lo[1], self.lo[2], self.state.plane/10-17))
    
    def start_spindle_clockwise(self, arg):
        # M3
        if self.suppress > 0: return
        color = self.colors['dwell']
        self.dwells_append((self.lineno, color, self.lo[0], self.lo[1], self.lo[2], self.state.plane/10-17))
        if self.block_start != None:
            self.blocks_append((self.block_start, self.lineno, self.block_pos,self.block_feed))
        self.block_start = None
        self.block_pos = []
        self.path.append(('M3', self.lineno))
        
    def start_spindle_counterclockwise(self, arg):
        # M4
        if self.suppress > 0: return
        color = self.colors['dwell']
        self.dwells_append((self.lineno, color, self.lo[0], self.lo[1], self.lo[2], self.state.plane/10-17))
        self.block_start = self.lineno 
        self.block_pos = self.lo # None # self.lo # we should record next feed (arcfeed or traverse) 
        self.block_feed = self.feedrate
        self.path.append(('M4', self.lineno))
        
    def program_end(self):
        # M2/M30
        self.path.append(('M2', self.lineno))

    def clear_motion_output_bit(self, arg):
        # M63 P-
        self.path.append(('M63', self.lineno, arg))


    def set_motion_output_bit(self, arg):
        # M62 P-
        self.path.append(('M62', self.lineno, arg))
  
    def highlight2(self, lineno, geometry):
        glLineWidth(3)
        c = self.colors['selected']
        glColor3f(*c)
        glBegin(GL_LINES)
        coords = []
        for i in range(0,len(self.blocks)):
            if lineno >= self.blocks[i][0]  and lineno <= self.blocks[i][1]:
                # print 'block(%d) selected start(%d) end(%d)' % (i, self.blocks[i][0], self.blocks[i][1])
                self.selected_block = i
                for j in range(self.blocks[i][0], self.blocks[i][1]):
                    for line in self.traverse:
                        if line[0] != j: continue
                        # print 'append traverse'
                        coords.append(line[1][:3])
                        coords.append(line[2][:3])
                    for line in self.arcfeed:
                        if line[0] != j: continue
                        linuxcnc.line9(geometry, line[1], line[2])
                        coords.append(line[1][:3])
                        coords.append(line[2][:3])
                    for line in self.arcfeed:
                        if line[0] != j: continue
                        linuxcnc.line9(geometry, line[1], line[2])
                        coords.append(line[1][:3])
                        coords.append(line[2][:3])
                    for line in self.feed:
                        if line[0] != j: continue
                        linuxcnc.line9(geometry, line[1], line[2])
                        coords.append(line[1][:3])
                        coords.append(line[2][:3])
                glEnd()
                for j in range(self.blocks[i][0], self.blocks[i][1]):
                    for line in self.dwells: 
                        if line[0] != j: continue
                        self.draw_dwells([(line[0], c) + line[2:]], 2, 0)
                        coords.append(line[2:5])
                glLineWidth(1)
                if coords:
                    x = reduce(lambda x,y:x+y, [c[0] for c in coords]) / len(coords)
                    y = reduce(lambda x,y:x+y, [c[1] for c in coords]) / len(coords)
                    z = reduce(lambda x,y:x+y, [c[2] for c in coords]) / len(coords)
                else:
                    x = (self.min_extents[0] + self.max_extents[0])/2
                    y = (self.min_extents[1] + self.max_extents[1])/2
                    z = (self.min_extents[2] + self.max_extents[2])/2
                return x, y, z
        glEnd()
        for line in self.dwells: 
            if line[0] != lineno: continue
            self.draw_dwells([(line[0], c) + line[2:]], 2, 0)
            coords.append(line[2:5])
        glLineWidth(1)
        if coords:
            x = reduce(lambda x,y:x+y, [c[0] for c in coords]) / len(coords)
            y = reduce(lambda x,y:x+y, [c[1] for c in coords]) / len(coords)
            z = reduce(lambda x,y:x+y, [c[2] for c in coords]) / len(coords)
        else:
            x = (self.min_extents[0] + self.max_extents[0])/2
            y = (self.min_extents[1] + self.max_extents[1])/2
            z = (self.min_extents[2] + self.max_extents[2])/2
        return x, y, z
    # TODO: move to injector?
    def get_first_pos_of_prog(self):
        if len(self.blocks) > 0:
            block_line = self.blocks[0][0]
        else:
            block_line = None
        
        # if len(self.traverse) > 0:
        #     traverse_line = self.traverse[0][0]
        # else:
        #     traverse_line = None
        if len(self.all_traverse) > 0:
            first_traverse = self.all_traverse[0]
            traverse_line = first_traverse[0]
        else:
            traverse_line = None 
            first_traverse = None
        if len(self.feed) > 0:
            feed_line = self.feed[0][0]
        else:
            feed_line = None
        if len(self.arcfeed) > 0:
            arcfeed_line = self.arcfeed[0][0]
        else:
            arcfeed_line = None

        # DEBUG: print 'block-line',block_line
        # DEBUG: print 'traverse-line',traverse_line
        # DEBUG: print 'feed-line',feed_line
        # DEBUG: print 'arcfeed-line', arcfeed_line

        if arcfeed_line is None:
            arcfeed_line = max((block_line,arcfeed_line,feed_line,traverse_line))
        if feed_line is None:
            feed_line = max((block_line,arcfeed_line,feed_line,traverse_line))
        if block_line is None:
            block_line = max((block_line,arcfeed_line,feed_line,traverse_line))
        if traverse_line is None:
            traverse_line = max((block_line,arcfeed_line,feed_line,traverse_line))

        if block_line <= min((traverse_line, feed_line, arcfeed_line)):
        #    print 'min is block',block_line
            if len(self.blocks) > 0:
                return self.blocks[0][2][:3], self.blocks[0][3]
            else: 
                return (0,0,0),1000
        if arcfeed_line <= min((traverse_line, feed_line, block_line)):
        #    print 'min is arcfeed', arcfeed_line
            if len(self.arcfeed) > 0:
                return self.arcfeed[0][2][:3],self.arcfeed[0][3]
            else:
                return (0,0,0),1000
        if feed_line <= min((traverse_line, block_line, feed_line)):
            if len(self.feed) > 0:
                return self.feed[0][2][:3],self.feed[0][3]
            else:
                return (0,0,0),1000 
        if traverse_line <= min((block_line, arcfeed_line, feed_line)):
        #    print 'min is traverse', traverse_line 
            if len(self.all_traverse) > 0:
                feedrate = self.all_traverse[0][3]
                return first_traverse[2][:3], feedrate
            else:
                return (0,0,0),1000
    def get_last_line_of_path(self):
        if len(self.all_traverse) > 0: 
            traverse_line = self.all_traverse[len(self.all_traverse)-1][0]
        else:
            traverse_line = 0 
        if len(self.feed) > 0:
            feed_line = self.feed[len(self.feed)-1][0]
        else:
            feed_line = 0 
        if len(self.arcfeed) > 0:
            arcfeed_line = self.arcfeed[len(self.arcfeed)-1][0]
        else:
            arcfeed_line = 0
        # print 'max line = %f' % (max(traverse_line, feed_line, arcfeed_line))
        return max(traverse_line, feed_line, arcfeed_line)
    def get_last_pos_of_prog(self):
        # if len(self.blocks) > 0:
        #     block_line = self.blocks[len(self.blocks)-1][0]
        # else:
        #     block_line = None
        
        #if len(self.traverse) > 0:
        #    traverse_line = self.traverse[len(self.traverse)-1][0]
        #else:
        #    traverse_line = None
        if len(self.all_traverse) > 0: 
            last_traverse = self.all_traverse[len(self.all_traverse)-1]
            traverse_line = last_traverse[0] 
        else:
            traverse_line = None 
            last_traverse = None
        if len(self.feed) > 0:
            feed_line = self.feed[len(self.feed)-1][0]
        else:
            feed_line = None
        if len(self.arcfeed) > 0:
            arcfeed_line = self.arcfeed[len(self.arcfeed)-1][0]
        else:
            arcfeed_line = None

        # DEBUG: print 'block-line',block_line
        # DEBUG: print 'traverse-line',traverse_line
        # DEBUG: print 'feed-line',feed_line
        # DEBUG: print 'arcfeed-line', arcfeed_line

        if arcfeed_line is None:
            arcfeed_line = min((arcfeed_line,feed_line,traverse_line))
        if feed_line is None:
            feed_line = min((arcfeed_line,feed_line,traverse_line))
        if traverse_line is None:
            traverse_line = min((arcfeed_line,feed_line,traverse_line))
        print arcfeed_line, feed_line, traverse_line
        if feed_line >= max((traverse_line, arcfeed_line)):
            # print 'max is feed', feed_line
            index = len(self.feed) - 1
            print 'last pos is feed', self.feed[index][2][:3]
            print 'line', feed_line
            return self.feed[index][2][:3],self.feed[index][3]
        if arcfeed_line >= max((traverse_line, feed_line)):
            # print 'max is arcfeed', arcfeed_line
            index = len(self.arcfeed)- 1
            print 'last pos is arc', self.arcfeed[index]
            return self.arcfeed[index][2][:3],self.arcfeed[index][3]
        if traverse_line >= max((arcfeed_line, feed_line)):
            # print 'max is traverse', traverse_line 
            print 'last pos is traverse', last_traverse[2][:3]
            feedrate = last_traverse[3]
            return last_traverse[2][:3], feedrate
    def get_start_line_of_block(self, lineno = None):
        for i in range(0, len(self.blocks)):
            if lineno >= self.blocks[i][0] and lineno <= self.blocks[i][1]:
                return self.blocks[i][0] # return start line of block
        return lineno
    def set_highlight_mode(self, mode=None):
        if mode is None:
            self.highlight_mode = 'line'
            return
        mode = mode.lower()
        if mode == "block":
            self.highlight_mode = 'block'
        else:
            self.highlight_mode = 'line'
    def highlight(self, lineno, geometry):
        if self.highlight_mode is 'block':
            # print 'highlighting ',lineno,' block'
            return self.highlight2(lineno, geometry)
        glLineWidth(3)
        c = self.colors['selected']
        glColor3f(*c)
        glBegin(GL_LINES)
        coords = []
        for line in self.traverse:
            if line[0] != lineno: continue
            linuxcnc.line9(geometry, line[1], line[2])
            coords.append(line[1][:3])
            coords.append(line[2][:3])
        for line in self.arcfeed:
            if line[0] != lineno: continue
            linuxcnc.line9(geometry, line[1], line[2])
            coords.append(line[1][:3])
            coords.append(line[2][:3])
        for line in self.feed:
            if line[0] != lineno: continue
            linuxcnc.line9(geometry, line[1], line[2])
            coords.append(line[1][:3])
            coords.append(line[2][:3])
        glEnd()
        for line in self.dwells:
            if line[0] != lineno: continue
            self.draw_dwells([(line[0], c) + line[2:]], 2, 0)
            coords.append(line[2:5])
        glLineWidth(1)
        if coords:
            x = reduce(lambda x,y:x+y, [c[0] for c in coords]) / len(coords)
            y = reduce(lambda x,y:x+y, [c[1] for c in coords]) / len(coords)
            z = reduce(lambda x,y:x+y, [c[2] for c in coords]) / len(coords)
        else:
            x = (self.min_extents[0] + self.max_extents[0])/2
            y = (self.min_extents[1] + self.max_extents[1])/2
            z = (self.min_extents[2] + self.max_extents[2])/2
        return x, y, z

    def color_with_alpha(self, name):
        glColor4f(*(self.colors[name] + (self.colors.get(name+'_alpha', 1/3.),)))
    def color(self, name):
        glColor3f(*self.colors[name])

    def draw(self, for_selection=0, no_traverse=True):
        if not no_traverse:
            glEnable(GL_LINE_STIPPLE)
            self.colored_lines('traverse', self.traverse, for_selection)
            glDisable(GL_LINE_STIPPLE)
        else:
            self.colored_lines('straight_feed', self.feed, for_selection, len(self.traverse))

            self.colored_lines('arc_feed', self.arcfeed, for_selection, len(self.traverse) + len(self.feed))

            glLineWidth(2)
            self.draw_dwells(self.dwells, self.colors.get('dwell_alpha', 1/3.), for_selection, len(self.traverse) + len(self.feed) + len(self.arcfeed))
            glLineWidth(1)

def with_context(f):
    def inner(self, *args, **kw):
        self.activate()
        try:
            return f(self, *args, **kw)
        finally:
            self.deactivate()
    return inner

def with_context_swap(f):
    def inner(self, *args, **kw):
        self.activate()
        try:
            return f(self, *args, **kw)
        finally:
            self.swapbuffers()
            self.deactivate()
    return inner

class GlCanonDraw:
    colors = {
        'traverse': (0.30, 0.50, 0.50),
        'traverse_alpha': 1/3.,
        'traverse_xy': (0.30, 0.50, 0.50),
        'traverse_alpha_xy': 1/3.,
        'traverse_uv': (0.30, 0.50, 0.50),
        'traverse_alpha_uv': 1/3.,
        'backplotprobing_alpha': 0.75,
        'backplotprobing': (0.63, 0.13, 0.94),
        'backplottraverse': (0.30, 0.50, 0.50),
        'label_ok': (1.00, 0.51, 0.53),
        'backplotjog_alpha': 0.75,
        'tool_diffuse': (0.60, 0.60, 0.60),
        'backplotfeed': (0.75, 0.25, 0.25),
        'back': (0.00, 0.00, 0.00),
        'lathetool_alpha': 0.10,
        # 'axis_x': (0, 0, 0),
        'axis_x': (0.20, 1.00, 0.20), # original
        'cone': (1.00, 1.00, 0.00), # yellow
        # 'cone': (1.00, 1.00, 1.00), # original
        'cone_xy': (0.00, 1.00, 0.00),
        'cone_uv': (0.00, 0.00, 1.00),
        # 'axis_z': (0, 0, 0),
        'axis_z': (0.20, 0.20, 1.00),
        'label_limit': (1.00, 0.21, 0.23),
        'backplotjog': (1.00, 1.00, 0.00),
        'selected': (0.00, 1.00, 1.00),
        'lathetool': (0.80, 0.80, 0.80),
        'dwell': (1.00, 0.50, 0.50),
        'overlay_foreground': (1.00, 1.00, 1.00),
        'overlay_background': (0.00, 0.00, 0.00),
        'straight_feed': (1.00, 1.00, 1.00),
        'straight_feed_alpha': 1/3.,
        'straight_feed_xy': (0.20, 1.00, 0.20),
        'straight_feed_alpha_xy': 1/3.,
        'straight_feed_uv': (0.20, 0.20, 1.00),
        'straight_feed_alpha_uv': 1/3.,
        'small_origin': (0.00, 1.00, 1.00),
        'backplottoolchange_alpha': 0.25,
        'backplottraverse_alpha': 0.25,
        'overlay_alpha': 0.75,
        'tool_ambient': (0.40, 0.40, 0.40),
        'tool_alpha': 0.20,
        'backplottoolchange': (1.00, 0.65, 0.00),
        'backplotarc': (0.75, 0.25, 0.50),
        'm1xx': (0.50, 0.50, 1.00),
        'backplotfeed_alpha': 0.75,
        'backplotarc_alpha': 0.75,
        'arc_feed': (1.00, 1.00, 1.00),
        'arc_feed_alpha': .5,
        'arc_feed_xy': (0.20, 1.00, 0.20),
        'arc_feed_alpha_xy': 1/3.,
        'arc_feed_uv': (0.20, 0.20, 1.00),
        'arc_feed_alpha_uv': 1/3.,
        'axis_y': (1.00, 0.20, 0.20), # original color
        # 'axis_y': (0, 0, 0),
        'grid': (0.15, 0.15, 0.15),
    }
    def __init__(self, s, lp, g=None):
        self.stat = s
        self.lp = lp
        self.canon = g
        self._dlists = {}
        self.select_buffer_size = 100
        self.cached_tool = -1
        self.initialised = 0
        self.fix_tool_size = False  # set to True to disable tool_size scaling

    def realize(self):
        self.hershey = hershey.Hershey()
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
        self.basic_lighting()
        self.initialised = 1

    def set_canon(self, canon):
        self.canon = canon

    @with_context
    def basic_lighting(self):
        glLightfv(GL_LIGHT0, GL_POSITION, (1, -1, 1, 0))
        glLightfv(GL_LIGHT0, GL_AMBIENT, self.colors['tool_ambient'] + (0,))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, self.colors['tool_diffuse'] + (0,))
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, (1,1,1,0))
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def select(self, x, y):
        selected_line = 0
        if self.canon is None: return
        pmatrix = glGetDoublev(GL_PROJECTION_MATRIX)
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        vport = glGetIntegerv(GL_VIEWPORT)
        gluPickMatrix(x, vport[3]-y, 5, 5, vport)
        glMultMatrixd(pmatrix)
        glMatrixMode(GL_MODELVIEW)

        while 1:
            glSelectBuffer(self.select_buffer_size)
            glRenderMode(GL_SELECT)
            glInitNames()
            glPushName(0)

            if self.get_show_rapids():
                glCallList(self.dlist('select_rapids', gen=self.make_selection_list))
            glCallList(self.dlist('select_norapids', gen=self.make_selection_list))
            try:
                buffer = list(glRenderMode(GL_RENDER))
            except OverflowError:
                self.select_buffer_size *= 2
                continue
            break

        if buffer:
            min_depth, max_depth, names = min(buffer)
            # print 'names', names
            # call to draw highlight line
            self.set_highlight_line(names[0]) # input lineno to find block
            if self.canon.highlight_mode is 'block':
                selected_line = self.canon.selected_block
            else:
                selected_line = int(names[0])
        else:
            self.set_highlight_line(None)
            selected_line = None

        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        return selected_line 
    def dlist(self, name, n=1, gen=lambda n: None):
        if name not in self._dlists:
            base = glGenLists(n)
            self._dlists[name] = base, n
            gen(base)
        return self._dlists[name][0]

    def stale_dlist(self, name):
        if name not in self._dlists: return
        base, count = self._dlists.pop(name)
        glDeleteLists(base, count)

    def __del__(self):
        for base, count in self._dlists.values():
            glDeleteLists(base, count)

    def set_current_line(self, line): pass
    def set_highlight_line(self, line):
        if line == self.get_highlight_line(): return
        highlight = self.dlist('highlight')
        glNewList(highlight, GL_COMPILE)
        if line is not None and self.canon is not None:
            if self.is_foam():
                glPushMatrix()
                glTranslatef(0, 0, self.get_foam_z()) 
                x, y, z = self.canon.highlight(line, "XY")
                glTranslatef(0, 0, self.get_foam_w()-self.get_foam_z())
                u, v, w = self.canon.highlight(line, "UV")
                glPopMatrix()
                x = (x+u)/2
                y = (y+v)/2
                z = (self.get_foam_z() + self.get_foam_w())/2
            else:
                x, y, z = self.canon.highlight(line, self.get_geometry())
        elif self.canon is not None:
            x = (self.canon.min_extents[0] + self.canon.max_extents[0])/2
            y = (self.canon.min_extents[1] + self.canon.max_extents[1])/2
            z = (self.canon.min_extents[2] + self.canon.max_extents[2])/2
        else:
            x, y, z = 0.0, 0.0, 0.0
        glEndList()
        self.set_centerpoint(x, y, z)

    @with_context_swap
    def redraw_perspective(self):

        w = self.winfo_width()
        h = self.winfo_height()
        glViewport(0, 0, w, h)

        # Clear the background and depth buffer.
        glClearColor(*(self.colors['back'] + (0,)))
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(self.fovy, float(w)/float(h), self.near, self.far + self.distance)

        gluLookAt(0, 0, self.distance,
            0, 0, 0,
            0., 1., 0.)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        try:
            self.redraw()
        finally:
            glFlush()                               # Tidy up
            glPopMatrix()                   # Restore the matrix

    @with_context_swap
    def redraw_ortho(self):
        if not self.initialised: return

        w = self.winfo_width()
        h = self.winfo_height()
        glViewport(0, 0, w, h)

        # Clear the background and depth buffer.
        glClearColor(*(self.colors['back'] + (0,)))
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        ztran = self.distance
        k = (abs(ztran or 1)) ** .55555
        l = k * h / w
        glOrtho(-k, k, -l, l, -1000, 1000.)

        gluLookAt(0, 0, 1,
            0, 0, 0,
            0., 1., 0.)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        try:
            self.redraw()
        finally:
            glFlush()                               # Tidy up
            glPopMatrix()                   # Restore the matrix

    def color_limit(self, cond):
        if cond:
            glColor3f(*self.colors['label_limit'])
        else:
            glColor3f(*self.colors['label_ok'])
        return cond


    def show_extents(self):
        s = self.stat
        g = self.canon

        if g is None: return

        # Dimensions
        x,y,z,p = 0,1,2,3
        view = self.get_view()
        is_metric = self.get_show_metric()
        dimscale = is_metric and 25.4 or 1.0
        fmt = is_metric and "%.1f" or "%.2f"

        machine_limit_min, machine_limit_max = self.soft_limits()

        pullback = max(g.max_extents[x] - g.min_extents[x],
                       g.max_extents[y] - g.min_extents[y],
                       g.max_extents[z] - g.min_extents[z],
                       2 ) * .1

        dashwidth = pullback/4
        charsize = dashwidth * 1.5
        halfchar = charsize * .5

        if view == z or view == p:
            z_pos = g.min_extents[z]
            zdashwidth = 0
        else:
            z_pos = g.min_extents[z] - pullback
            zdashwidth = dashwidth
        # x dimension

        self.color_limit(0)
        glBegin(GL_LINES)
        if view != x and g.max_extents[x] > g.min_extents[x]:
            y_pos = g.min_extents[y] - pullback
            glVertex3f(g.min_extents[x], y_pos, z_pos)
            glVertex3f(g.max_extents[x], y_pos, z_pos)

            glVertex3f(g.min_extents[x], y_pos - dashwidth, z_pos - zdashwidth)
            glVertex3f(g.min_extents[x], y_pos + dashwidth, z_pos + zdashwidth)

            glVertex3f(g.max_extents[x], y_pos - dashwidth, z_pos - zdashwidth)
            glVertex3f(g.max_extents[x], y_pos + dashwidth, z_pos + zdashwidth)

        # y dimension
        if view != y and g.max_extents[y] > g.min_extents[y]:
            x_pos = g.min_extents[x] - pullback
            glVertex3f(x_pos, g.min_extents[y], z_pos)
            glVertex3f(x_pos, g.max_extents[y], z_pos)

            glVertex3f(x_pos - dashwidth, g.min_extents[y], z_pos - zdashwidth)
            glVertex3f(x_pos + dashwidth, g.min_extents[y], z_pos + zdashwidth)

            glVertex3f(x_pos - dashwidth, g.max_extents[y], z_pos - zdashwidth)
            glVertex3f(x_pos + dashwidth, g.max_extents[y], z_pos + zdashwidth)

        # z dimension
        if view != z and g.max_extents[z] > g.min_extents[z]:
            x_pos = g.min_extents[x] - pullback
            y_pos = g.min_extents[y] - pullback
            glVertex3f(x_pos, y_pos, g.min_extents[z])
            glVertex3f(x_pos, y_pos, g.max_extents[z])

            glVertex3f(x_pos - dashwidth, y_pos - zdashwidth, g.min_extents[z])
            glVertex3f(x_pos + dashwidth, y_pos + zdashwidth, g.min_extents[z])

            glVertex3f(x_pos - dashwidth, y_pos - zdashwidth, g.max_extents[z])
            glVertex3f(x_pos + dashwidth, y_pos + zdashwidth, g.max_extents[z])

        glEnd()

        # Labels
        if self.get_show_relative():
            offset = self.to_internal_units(s.g5x_offset + s.g92_offset)
        else:
            offset = 0, 0, 0
        if view != z and g.max_extents[z] > g.min_extents[z]:
            if view == x:
                x_pos = g.min_extents[x] - pullback
                y_pos = g.min_extents[y] - 6.0*dashwidth
            else:
                x_pos = g.min_extents[x] - 6.0*dashwidth
                y_pos = g.min_extents[y] - pullback

            bbox = self.color_limit(g.min_extents[z] < machine_limit_min[z])
            glPushMatrix()
            f = fmt % ((g.min_extents[z]-offset[z]) * dimscale)
            glTranslatef(x_pos, y_pos, g.min_extents[z] - halfchar)
            glScalef(charsize, charsize, charsize)
            glRotatef(-90, 0, 1, 0)
            glRotatef(-90, 0, 0, 1)
            if view != x:
                glRotatef(-90, 0, 1, 0)
            self.hershey.plot_string(f, 0, bbox)
            glPopMatrix()

            bbox = self.color_limit(g.max_extents[z] > machine_limit_max[z])
            glPushMatrix()
            f = fmt % ((g.max_extents[z]-offset[z]) * dimscale)
            glTranslatef(x_pos, y_pos, g.max_extents[z] - halfchar)
            glScalef(charsize, charsize, charsize)
            glRotatef(-90, 0, 1, 0)
            glRotatef(-90, 0, 0, 1)
            if view != x:
                glRotatef(-90, 0, 1, 0)
            self.hershey.plot_string(f, 0, bbox)
            glPopMatrix()

            self.color_limit(0)
            glPushMatrix()
            f = fmt % ((g.max_extents[z] - g.min_extents[z]) * dimscale)
            glTranslatef(x_pos, y_pos, (g.max_extents[z] + g.min_extents[z])/2)
            glScalef(charsize, charsize, charsize)
            if view != x:
                glRotatef(-90, 0, 0, 1)
            glRotatef(-90, 0, 1, 0)
            self.hershey.plot_string(f, .5, bbox)
            glPopMatrix()

        if view != y and g.max_extents[y] > g.min_extents[y]:
            x_pos = g.min_extents[x] - 6.0*dashwidth

            bbox = self.color_limit(g.min_extents[y] < machine_limit_min[y])
            glPushMatrix()
            f = fmt % ((g.min_extents[y] - offset[y]) * dimscale)
            glTranslatef(x_pos, g.min_extents[y] + halfchar, z_pos)
            glRotatef(-90, 0, 0, 1)
            glRotatef(-90, 0, 0, 1)
            if view == x:
                glRotatef(90, 0, 1, 0)
                glTranslatef(dashwidth*1.5, 0, 0)
            glScalef(charsize, charsize, charsize)
            self.hershey.plot_string(f, 0, bbox)
            glPopMatrix()

            bbox = self.color_limit(g.max_extents[y] > machine_limit_max[y])
            glPushMatrix()
            f = fmt % ((g.max_extents[y] - offset[y]) * dimscale)
            glTranslatef(x_pos, g.max_extents[y] + halfchar, z_pos)
            glRotatef(-90, 0, 0, 1)
            glRotatef(-90, 0, 0, 1)
            if view == x:
                glRotatef(90, 0, 1, 0)
                glTranslatef(dashwidth*1.5, 0, 0)
            glScalef(charsize, charsize, charsize)
            self.hershey.plot_string(f, 0, bbox)
            glPopMatrix()

            self.color_limit(0)
            glPushMatrix()
            f = fmt % ((g.max_extents[y] - g.min_extents[y]) * dimscale)

            glTranslatef(x_pos, (g.max_extents[y] + g.min_extents[y])/2,
                        z_pos)
            glRotatef(-90, 0, 0, 1)
            if view == x:
                glRotatef(-90, 1, 0, 0)
                glTranslatef(0, halfchar, 0)
            glScalef(charsize, charsize, charsize)
            self.hershey.plot_string(f, .5)
            glPopMatrix()

        if view != x and g.max_extents[x] > g.min_extents[x]:
            y_pos = g.min_extents[y] - 6.0*dashwidth

            bbox = self.color_limit(g.min_extents[x] < machine_limit_min[x])
            glPushMatrix()
            f = fmt % ((g.min_extents[x] - offset[x]) * dimscale)
            glTranslatef(g.min_extents[x] - halfchar, y_pos, z_pos)
            glRotatef(-90, 0, 0, 1)
            if view == y:
                glRotatef(90, 0, 1, 0)
                glTranslatef(dashwidth*1.5, 0, 0)
            glScalef(charsize, charsize, charsize)
            self.hershey.plot_string(f, 0, bbox)
            glPopMatrix()

            bbox = self.color_limit(g.max_extents[x] > machine_limit_max[x])
            glPushMatrix()
            f = fmt % ((g.max_extents[x] - offset[x]) * dimscale)
            glTranslatef(g.max_extents[x] - halfchar, y_pos, z_pos)
            glRotatef(-90, 0, 0, 1)
            if view == y:
                glRotatef(90, 0, 1, 0)
                glTranslatef(dashwidth*1.5, 0, 0)
            glScalef(charsize, charsize, charsize)
            self.hershey.plot_string(f, 0, bbox)
            glPopMatrix()

            self.color_limit(0)
            glPushMatrix()
            f = fmt % ((g.max_extents[x] - g.min_extents[x]) * dimscale)

            glTranslatef((g.max_extents[x] + g.min_extents[x])/2, y_pos,
                        z_pos)
            if view == y:
                glRotatef(-90, 1, 0, 0)
                glTranslatef(0, halfchar, 0)
            glScalef(charsize, charsize, charsize)
            self.hershey.plot_string(f, .5)
            glPopMatrix()

    def to_internal_linear_unit(self, v, unit=None):
        if unit is None:
            unit = self.stat.linear_units
        lu = (unit or 1) * 25.4
        return v/lu


    def to_internal_units(self, pos, unit=None):
        if unit is None:
            unit = self.stat.linear_units
        lu = (unit or 1) * 25.4

        lus = [lu, lu, lu, 1, 1, 1, lu, lu, lu]
        return [a/b for a, b in zip(pos, lus)]

    def soft_limits(self):
        def fudge(x):
            if abs(x) > 1e30: return 0
            return x

        ax = self.stat.joint
        return (
            self.to_internal_units([fudge(ax[i]['min_position_limit'])
                for i in range(3)]),
            self.to_internal_units([fudge(ax[i]['max_position_limit'])
                for i in range(3)]))

    def get_foam_z(self):
        if self.canon: return self.canon.foam_z
        return 0

    def get_foam_w(self):
        if self.canon: return self.canon.foam_w
        return 1.5

    def get_grid(self):
        if self.canon and self.canon.grid: return self.canon.grid
        return 5./25.4

    def comp(self, (sx, sy), (cx, cy)):
        return -(sx*cx + sy*cy) / (sx*sx + sy*sy)

    def param(self, (x1, y1), (dx1, dy1), (x3, y3), (dx3, dy3)):
        den = (dy3)*(dx1) - (dx3)*(dy1)
        if den == 0: return 0
        num = (dx3)*(y1-y3) - (dy3)*(x1-x3)
        return num * 1. / den

    def draw_grid_lines(self, space, (ox, oy), (dx, dy), lim_min, lim_max,
            inverse_permutation):
        # draw a series of line segments of the form
        #   dx(x-ox) + dy(y-oy) + k*space = 0
        # for integers k that intersect the AABB [lim_min, lim_max]
        lim_pts = [
                (lim_min[0], lim_min[1]),
                (lim_max[0], lim_min[1]),
                (lim_min[0], lim_max[1]),
                (lim_max[0], lim_max[1])]
        od = self.comp((dy, -dx), (ox, oy))
        d0, d1 = minmax(*(self.comp((dy, -dx), i)-od for i in lim_pts))
        k0 = int(math.ceil(d0/space))
        k1 = int(math.floor(d1/space))
        delta = (dx, dy)
        for k in range(k0, k1+1):
            d = k*space
            # Now we're drawing the line dx(x-ox) + dx(y-oy) + d = 0
            p0 = (ox - dy * d, oy + dx * d)
            # which is the same as the line p0 + u * delta

            # but we only want the part that's inside the box lim_pts...
            if dx and dy:
                times = [
                        self.param(p0, delta, lim_min[:2], (0, 1)),
                        self.param(p0, delta, lim_min[:2], (1, 0)),
                        self.param(p0, delta, lim_max[:2], (0, 1)),
                        self.param(p0, delta, lim_max[:2], (1, 0))]
                times.sort()
                t0, t1 = times[1], times[2] # Take the middle two times
            elif dx:
                times = [
                        self.param(p0, delta, lim_min[:2], (0, 1)),
                        self.param(p0, delta, lim_max[:2], (0, 1))]
                times.sort()
                t0, t1 = times[0], times[1] # Take the only two times
            else:
                times = [
                        self.param(p0, delta, lim_min[:2], (1, 0)),
                        self.param(p0, delta, lim_max[:2], (1, 0))]
                times.sort()
                t0, t1 = times[0], times[1] # Take the only two times
            x0, y0 = p0[0] + delta[0]*t0, p0[1] + delta[1]*t0
            x1, y1 = p0[0] + delta[0]*t1, p0[1] + delta[1]*t1
            xm, ym = (x0+x1)/2, (y0+y1)/2
            # The computation of k0 and k1 above should mean that
            # the lines are always in the limits, but I observed
            # that this wasn't always the case...
            #if xm < lim_min[0] or xm > lim_max[0]: continue
            #if ym < lim_min[1] or ym > lim_max[1]: continue
            glVertex3f(*inverse_permutation((x0, y0, lim_min[2])))
            glVertex3f(*inverse_permutation((x1, y1, lim_min[2])))

    def draw_grid_permuted(self, rotation, permutation, inverse_permutation):
        grid_size=self.get_grid_size()
        if not grid_size: return

        glLineWidth(1)
        glColor3f(*self.colors['grid'])
        lim_min, lim_max = self.soft_limits()
        lim_min = permutation(lim_min)
        lim_max = permutation(lim_max)

        lim_pts = (
                (lim_min[0], lim_min[1]),
                (lim_max[0], lim_min[1]),
                (lim_min[0], lim_max[1]),
                (lim_max[0], lim_max[1]))
        s = self.stat
        g5x_offset = permutation(self.to_internal_units(s.g5x_offset)[:3])[:2]
        g92_offset = permutation(self.to_internal_units(s.g92_offset)[:3])[:2]
        if self.get_show_relative():
            cos_rot = math.cos(rotation)
            sin_rot = math.sin(rotation)
            offset = (
                    g5x_offset[0] + g92_offset[0] * cos_rot
                                  - g92_offset[1] * sin_rot,
                    g5x_offset[1] + g92_offset[0] * sin_rot
                                  + g92_offset[1] * cos_rot)
        else:
            offset = 0., 0.
            cos_rot = 1.
            sin_rot = 0.
        glDepthMask(False)
        glBegin(GL_LINES)
        self.draw_grid_lines(grid_size, offset, (cos_rot, sin_rot),
                lim_min, lim_max, inverse_permutation)
        self.draw_grid_lines(grid_size, offset, (sin_rot, -cos_rot),
                lim_min, lim_max, inverse_permutation)
        glEnd()
        glDepthMask(True)

    def draw_grid(self):
        x,y,z,p = 0,1,2,3
        view = self.get_view()
        if view == p: return
        rotation = math.radians(self.stat.rotation_xy % 90)
        if rotation != 0 and view != z and self.get_show_relative(): return
        permutations = [
                lambda (x, y, z): (z, y, x),  # YZ X
                lambda (x, y, z): (z, x, y),  # ZX Y
                lambda (x, y, z): (x, y, z),  # XY Z
        ]
        inverse_permutations = [
                lambda (z, y, x): (x, y, z),  # YZ X
                lambda (z, x, y): (x, y, z),  # ZX Y
                lambda (x, y, z): (x, y, z),  # XY Z
        ]
        self.draw_grid_permuted(rotation, permutations[view],
                inverse_permutations[view])

    def redraw(self):
        s = self.stat
        s.poll()
        # to keep tracking tool with program position
        # s.position should be always equal to program_pos
        # use difference between s.position and program_pos to update 
        # pattern position
        try:
            if self.get_path_tracking() == True:
                diff = []
                diff2 = []
                pos = s.position[:s.axes]
                for i in range(0, len(pos)):
                    d = pos[i] - self.program_pos[i]
                    diff.append(d)
                    diff2.append(d - self.canon.prev_diff[i])
                self.canon.prev_diff = diff
                diff2 = self.to_internal_units(diff2)
                diff = self.to_internal_units(diff)
            else:
                diff2 = [0.0, 0.0, 0.0, 0.0]
                diff = diff2 
        except:
            diff2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            diff = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        machine_limit_min, machine_limit_max = self.soft_limits()

        glDisable(GL_LIGHTING)
        glMatrixMode(GL_MODELVIEW)
        self.draw_grid()
        if self.get_show_program():
            if self.get_program_alpha():
                glDisable(GL_DEPTH_TEST)
                glEnable(GL_BLEND)
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
   
            glTranslatef(diff[0], diff[1], 0)
            if self.get_show_rapids():
                glCallList(self.dlist('program_rapids', gen=self.make_main_list))
            glCallList(self.dlist('program_norapids', gen=self.make_main_list))
            glCallList(self.dlist('highlight'))

            if self.get_program_alpha():
                glDisable(GL_BLEND)
                glEnable(GL_DEPTH_TEST)

            if self.get_show_extents():
                self.show_extents()

        glTranslatef(-diff[0], -diff[1], 0)

        if self.get_show_live_plot() or self.get_show_program():
    
            alist = self.dlist(('axes', self.get_view()), gen=self.draw_axes)
            glPushMatrix()
            if self.get_show_relative() and (s.g5x_offset[0] or s.g5x_offset[1] or s.g5x_offset[2] or
                                             s.g92_offset[0] or s.g92_offset[1] or s.g92_offset[2] or
                                             s.rotation_xy):
                olist = self.dlist('draw_small_origin',
                                        gen=self.draw_small_origin)
                glCallList(olist)
                g5x_offset = self.to_internal_units(s.g5x_offset)[:3]
                g92_offset = self.to_internal_units(s.g92_offset)[:3]


                if self.get_show_offsets() and (g5x_offset[0] or g5x_offset[1] or g5x_offset[2]):
                    glBegin(GL_LINES)
                    glVertex3f(0,0,0)
                    glVertex3f(*g5x_offset)
                    glEnd()

                    i = s.g5x_index
                    if i<7:
                        label = "G5%d" % (i+3)
                    else:
                        label = "G59.%d" % (i-6)
                    glPushMatrix()
                    glScalef(0.2,0.2,0.2)
                    if self.is_lathe:
                        g5xrot=math.atan2(g5x_offset[0], -g5x_offset[2])
                        glRotatef(90, 1, 0, 0)
                        glRotatef(-90, 0, 0, 1)
                    else:
                        g5xrot=math.atan2(g5x_offset[1], g5x_offset[0])
                    glRotatef(math.degrees(g5xrot), 0, 0, 1)
                    glTranslatef(0.5, 0.5, 0)
                    self.hershey.plot_string(label, 0.1)
                    glPopMatrix()

                glTranslatef(*g5x_offset)
                glRotatef(s.rotation_xy, 0, 0, 1)

                
                if  self.get_show_offsets() and (g92_offset[0] or g92_offset[1] or g92_offset[2]):
                    glBegin(GL_LINES)
                    glVertex3f(0,0,0)
                    glVertex3f(*g92_offset)
                    glEnd()

                    glPushMatrix()
                    glScalef(0.2,0.2,0.2)
                    if self.is_lathe:
                        g92rot=math.atan2(g92_offset[0], -g92_offset[2])
                        glRotatef(90, 1, 0, 0)
                        glRotatef(-90, 0, 0, 1)
                    else:
                        g92rot=math.atan2(g92_offset[1], g92_offset[0])
                    glRotatef(math.degrees(g92rot), 0, 0, 1)
                    glTranslatef(0.5, 0.5, 0)
                    self.hershey.plot_string("G92", 0.1)
                    glPopMatrix()

                glTranslatef(*g92_offset)

            if self.is_foam():
                glTranslatef(0, 0, self.get_foam_z())
                glCallList(alist)
                uwalist = self.dlist(('axes_uw', self.get_view()), gen=lambda n: self.draw_axes(n, 'UVW'))
                glTranslatef(0, 0, self.get_foam_w()-self.get_foam_z())
                glCallList(uwalist)
            else:
                glCallList(alist)
            glPopMatrix()
        
        try:
            if self.draw_material():
                pos_2, pos_3, pos_0, pos_1 = self.get_material_dimension()
                # if pos_2 is not None and pos_3 is not None and pos_0 is\
                #  not None and pos_1 is not None:
                if pos_2 is None:
#                     print "PLATEVIEW: trying to draw but position doesn't assigned"
#                     print 'pos2 is None'
                    pass
                else:
#                     print 'PLATEVIEW: drawing gl for plateview'
#                     print 'x0(%f) y0(%f)' % (pos_0[0], pos_0[1])
#                     print 'x1(%f) y1(%f)' % (pos_1[0], pos_1[1])
#                     print 'x2(%f) y2(%f)' % (pos_2[0], pos_2[1])
#                     print 'x3(%f) y3(%f)' % (pos_3[0], pos_3[1])
                    glDisable(GL_DEPTH_TEST)
                    glEnable(GL_BLEND)
                    glLineWidth(10)
                    glColor3f(0.4,0.4,0.2)
                    # glColor3f(0.5,1.0,0.2)
                    glBegin(GL_QUADS)
                    glVertex3f(pos_2[0], pos_2[1],0)
                    glVertex3f(pos_3[0], pos_3[1],0)
                    glVertex3f(pos_0[0], pos_0[1],0)
                    glVertex3f(pos_1[0], pos_1[1],0)

                    glEnd()
                    glEnable(GL_DEPTH_TEST)
                    glDisable(GL_BLEND)
            else:
                # print 'PLATEVIEW: glcanon does not want to draw'
                pass
        except:
            # print 'PLATEVIEW: glcanon exception error'
            pass
        if self.get_show_limits():
            glLineWidth(1)
            glColor3f(1.0,0.0,0.0)
            glLineStipple(1, 0x1111)
            glEnable(GL_LINE_STIPPLE)
            glBegin(GL_LINES)

            glVertex3f(machine_limit_min[0], machine_limit_min[1], machine_limit_max[2])
            glVertex3f(machine_limit_min[0], machine_limit_min[1], machine_limit_min[2])

            glVertex3f(machine_limit_min[0], machine_limit_min[1], machine_limit_min[2])
            glVertex3f(machine_limit_min[0], machine_limit_max[1], machine_limit_min[2])

            glVertex3f(machine_limit_min[0], machine_limit_max[1], machine_limit_min[2])
            glVertex3f(machine_limit_min[0], machine_limit_max[1], machine_limit_max[2])

            glVertex3f(machine_limit_min[0], machine_limit_max[1], machine_limit_max[2])
            glVertex3f(machine_limit_min[0], machine_limit_min[1], machine_limit_max[2])


            glVertex3f(machine_limit_max[0], machine_limit_min[1], machine_limit_max[2])
            glVertex3f(machine_limit_max[0], machine_limit_min[1], machine_limit_min[2])

            glVertex3f(machine_limit_max[0], machine_limit_min[1], machine_limit_min[2])
            glVertex3f(machine_limit_max[0], machine_limit_max[1], machine_limit_min[2])

            glVertex3f(machine_limit_max[0], machine_limit_max[1], machine_limit_min[2])
            glVertex3f(machine_limit_max[0], machine_limit_max[1], machine_limit_max[2])

            glVertex3f(machine_limit_max[0], machine_limit_max[1], machine_limit_max[2])
            glVertex3f(machine_limit_max[0], machine_limit_min[1], machine_limit_max[2])


            glVertex3f(machine_limit_min[0], machine_limit_min[1], machine_limit_min[2])
            glVertex3f(machine_limit_max[0], machine_limit_min[1], machine_limit_min[2])

            glVertex3f(machine_limit_min[0], machine_limit_max[1], machine_limit_min[2])
            glVertex3f(machine_limit_max[0], machine_limit_max[1], machine_limit_min[2])

            glVertex3f(machine_limit_min[0], machine_limit_max[1], machine_limit_max[2])
            glVertex3f(machine_limit_max[0], machine_limit_max[1], machine_limit_max[2])

            glVertex3f(machine_limit_min[0], machine_limit_min[1], machine_limit_max[2])
            glVertex3f(machine_limit_max[0], machine_limit_min[1], machine_limit_max[2])

            glEnd()
            glDisable(GL_LINE_STIPPLE)
            glLineStipple(2, 0x5555)

        if self.get_show_live_plot():
            glDepthFunc(GL_LEQUAL)
            glLineWidth(3)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            glEnable(GL_BLEND)
            glPushMatrix()
            lu = 1/((s.linear_units or 1)*25.4)
            glScalef(lu, lu, lu)
            glMatrixMode(GL_PROJECTION)
            glPushMatrix()
            glTranslatef(0,0,.003)

            self.lp.call()

            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()
            glDisable(GL_BLEND)
            glLineWidth(1)
            glDepthFunc(GL_LESS)

        if self.get_show_tool():
            pos = self.lp.last(self.get_show_live_plot())
            if pos is None: pos = [0] * 6
            rx, ry, rz = pos[3:6]
            pos = self.to_internal_units(pos[:3])
            if self.is_foam():
                glEnable(GL_COLOR_MATERIAL)
                glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
                glPushMatrix()
                glTranslatef(pos[0], pos[1], self.get_foam_z())
                glRotatef(180, 1, 0, 0)
                cone = self.dlist("cone", gen=self.make_cone)
                glColor3f(*self.colors['cone_xy'])
                glCallList(cone)
                glPopMatrix()
                u = self.to_internal_linear_unit(rx)
                v = self.to_internal_linear_unit(ry)
                glPushMatrix()
                glTranslatef(u, v, self.get_foam_w())
                glColor3f(*self.colors['cone_uv'])
                glCallList(cone)
                glPopMatrix()
            else:
                glPushMatrix()
                glTranslatef(*pos)
                sign = 1
                for ch in self.get_geometry():
                    if ch == '-':
                        sign = -1
                    elif ch == 'A':
                        glRotatef(rx*sign, 1, 0, 0)
                        sign = 1
                    elif ch == 'B':
                        glRotatef(ry*sign, 0, 1, 0)
                        sign = 1
                    elif ch == 'C':
                        glRotatef(rz*sign, 0, 0, 1)
                        sign = 1
                glEnable(GL_BLEND)
                glEnable(GL_CULL_FACE)
                glBlendFunc(GL_ONE, GL_CONSTANT_ALPHA)

                current_tool = self.get_current_tool()
                if self.fix_tool_size == True:
                    if self.canon != None:
                        self.canon.fix_tool_size = True
                    self.cache_tool(current_tool)
                    glCallList(self.dlist('tool'))
                else:
                    if current_tool is None or current_tool.diameter == 0:
                        if self.canon:
                            g = self.canon
                            x,y,z = 0,1,2
                            cone_scale = max(g.max_extents[x] - g.min_extents[x],
                                           g.max_extents[y] - g.min_extents[y],
                                           g.max_extents[z] - g.min_extents[z],
                                           2 ) * .5
                        else:
                            cone_scale = 1
                        if self.is_lathe():
                            glRotatef(90, 0, 1, 0)
                        cone = self.dlist("cone", gen=self.make_cone)
                        glScalef(cone_scale, cone_scale, cone_scale)
                        glColor3f(*self.colors['cone'])
                        glCallList(cone)
                    else:
                        if current_tool != self.cached_tool:
                            self.cache_tool(current_tool)
                        glColor3f(*self.colors['cone'])
                        glCallList(self.dlist('tool'))
                glPopMatrix()

        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        ypos = self.winfo_height()
        glOrtho(0.0, self.winfo_width(), 0.0, ypos, -1.0, 1.0)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()

        limit, homed, posstrs, droposstrs = self.posstrs()

        charwidth, linespace, base = self.get_font_info()

        maxlen = max([len(p) for p in posstrs])
        pixel_width = charwidth * max(len(p) for p in posstrs)

        glDepthFunc(GL_ALWAYS)
        glDepthMask(GL_FALSE)
        glEnable(GL_BLEND)
        glBlendFunc(GL_ONE, GL_CONSTANT_ALPHA)
        glColor3f(*self.colors['overlay_background'])
        glBlendColor(0,0,0,1-self.colors['overlay_alpha'])
        glBegin(GL_QUADS)
        glVertex3f(0, ypos, 1)
        glVertex3f(0, ypos - 8 - linespace*len(posstrs), 1)
        glVertex3f(pixel_width+42, ypos - 8 - linespace*len(posstrs), 1)
        glVertex3f(pixel_width+42, ypos, 1)
        glEnd()
        glDisable(GL_BLEND)

        maxlen = 0
        ypos -= linespace+5
        i=0
        glColor3f(*self.colors['overlay_foreground'])
        if not self.get_show_offsets():
            for string in posstrs:
                maxlen = max(maxlen, len(string))
                glRasterPos2i(5, ypos)
                for char in string:
                    glCallList(base + ord(char))
                if i < len(homed) and homed[i]:
                    glRasterPos2i(pixel_width + 8, ypos)
                    glBitmap(13, 16, 0, 3, 17, 0, homeicon)
                if i < len(homed) and limit[i]:
                    glBitmap(13, 16, 0, 1, 17, 0, limiticon)
                ypos -= linespace
                i = i + 1
        if self.get_show_offsets():
            if self.is_lathe():
                homed.insert(0,homed[0])
            i=0
            for string in droposstrs:
                maxlen = max(maxlen, len(string))
                glRasterPos2i(5, ypos)
                for char in string:
                    glCallList(base + ord(char))
                if i < len(homed) and homed[i]:
                    glRasterPos2i(charwidth *3, ypos)
                    glBitmap(13, 16, 0, 3, 17, 0, homeicon)
                ypos -= linespace
                i = i + 1

        glDepthFunc(GL_LESS)
        glDepthMask(GL_TRUE)

        glPopMatrix()
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)

    def cache_tool(self, current_tool):
        self.cached_tool = current_tool
        glNewList(self.dlist('tool'), GL_COMPILE)
        if self.is_lathe() and current_tool and current_tool.orientation != 0:
            glBlendColor(0,0,0,self.colors['lathetool_alpha'])
            self.lathetool(current_tool)
        else:
            glBlendColor(0,0,0,self.colors['tool_alpha'])
            if self.is_lathe():
                glRotatef(90, 0, 1, 0)
            else:
                if self.fix_tool_size == True:
                    dia = 4 * math.sqrt((self.distance / 10))
                else:
                    dia = current_tool.diameter 
                r = self.to_internal_linear_unit(dia) / 2.
                q = gluNewQuadric()
                # glEnable(GL_LIGHTING)
                glColor3f(*self.colors['cone'])
                gluCylinder(q, r, r, 8*r, 32, 1)
                glPushMatrix()
                glRotatef(180, 1, 0, 0)
                gluDisk(q, 0, r, 32, 1)
                glPopMatrix()
                glTranslatef(0,0,8*r)
                gluDisk(q, 0, r, 32, 1)
                # glDisable(GL_LIGHTING)
                gluDeleteQuadric(q)
        glEndList()

    def posstrs(self):
        s = self.stat
        limit = s.limit[:]
        homed = s.homed[:]

        if self.is_lathe() and not s.axis_mask & 2:
            homed.insert(1, 0)
            limit.insert(1, 0)

        if not self.get_joints_mode():
            if self.get_show_commanded():
                positions = s.position
            else:
                positions = s.actual_position

            if self.get_show_relative():
                positions = [(i-j) for i, j in zip(positions, s.tool_offset)]
                positions = [(i-j) for i, j in zip(positions, s.g5x_offset)]

                t = -s.rotation_xy
                t = math.radians(t)
                x = positions[0]
                y = positions[1]
                positions[0] = x * math.cos(t) - y * math.sin(t)
                positions[1] = x * math.sin(t) + y * math.cos(t)
                positions = [(i-j) for i, j in zip(positions, s.g92_offset)]

            if self.get_a_axis_wrapped():
                positions[3] = math.fmod(positions[3], 360.0)
                if positions[3] < 0: positions[3] += 360.0

            if self.get_b_axis_wrapped():
                positions[4] = math.fmod(positions[4], 360.0)
                if positions[4] < 0: positions[4] += 360.0

            if self.get_c_axis_wrapped():
                positions[5] = math.fmod(positions[5], 360.0)
                if positions[5] < 0: positions[5] += 360.0

            positions = self.to_internal_units(positions)
            axisdtg = self.to_internal_units(s.dtg)
            g5x_offset = self.to_internal_units(s.g5x_offset)
            g92_offset = self.to_internal_units(s.g92_offset)
            tlo_offset = self.to_internal_units(s.tool_offset)

            if self.get_show_metric():
                positions = self.from_internal_units(positions, 1)
                axisdtg = self.from_internal_units(axisdtg, 1)
                g5x_offset = self.from_internal_units(g5x_offset, 1)
                g92_offset = self.from_internal_units(g92_offset, 1)
                tlo_offset = self.from_internal_units(tlo_offset, 1)
                format = "% 6s:% 9.3f"
                if self.get_show_distance_to_go():
                    droformat = " " + format + "  DTG %1s:% 9.3f"
                else:
                    droformat = " " + format
                offsetformat = "% 5s %1s:% 9.3f  G92 %1s:% 9.3f"
                rotformat = "% 5s %1s:% 9.3f"
            else:
                format = "% 6s:% 9.4f"
                if self.get_show_distance_to_go():
                    droformat = " " + format + "  DTG %1s:% 9.4f"
                else:
                    droformat = " " + format
                offsetformat = "% 5s %1s:% 9.4f  G92 %1s:% 9.4f"
                rotformat = "% 5s %1s:% 9.4f"
            diaformat = " " + format

            posstrs = []
            droposstrs = []
            for i in range(9):
                a = "XYZABCUVW"[i]
                if s.axis_mask & (1<<i):
                    posstrs.append(format % (a, positions[i]))
                    if self.get_show_distance_to_go():
                        droposstrs.append(droformat % (a, positions[i], a, axisdtg[i]))
                    else:
                        droposstrs.append(droformat % (a, positions[i]))
            droposstrs.append("")

            for i in range(9):
                index = s.g5x_index
                if index<7:
                    label = "G5%d" % (index+3)
                else:
                    label = "G59.%d" % (index-6)

                a = "XYZABCUVW"[i]
                if s.axis_mask & (1<<i):
                    droposstrs.append(offsetformat % (label, a, g5x_offset[i], a, g92_offset[i]))
            droposstrs.append(rotformat % (label, 'R', s.rotation_xy))

            droposstrs.append("")
            for i in range(9):
                a = "XYZABCUVW"[i]
                if s.axis_mask & (1<<i):
                    droposstrs.append(rotformat % ("TLO", a, tlo_offset[i]))


            if self.is_lathe():
                posstrs[0] = format % ("Rad", positions[0])
                posstrs.insert(1, format % ("Dia", positions[0]*2.0))
                if self.get_show_distance_to_go():
                    droposstrs[0] = droformat % ("Rad", positions[0], "R", axisdtg[0])
                    droposstrs.insert(1, droformat % ("Dia", positions[0]*2.0, "D", axisdtg[0]*2.0))
                else:
                    droposstrs[0] = droformat % ("Rad", positions[0])
                    droposstrs.insert(1, diaformat % ("Dia", positions[0]*2.0))

            if self.get_show_machine_speed():
                spd = self.to_internal_linear_unit(s.current_vel)
                if self.get_show_metric():
                    spd = spd * 25.4 * 60
                else:
                    spd = spd * 60
                posstrs.append(format % ("Vel", spd))
                pos=0
                for i in range(9):
                    if s.axis_mask & (1<<i): pos +=1
                if self.is_lathe():
                    pos +=1
                droposstrs.insert(pos, " " + format % ("Vel", spd))

            if self.get_show_distance_to_go():
                dtg = self.to_internal_linear_unit(s.distance_to_go)
                if self.get_show_metric():
                    dtg *= 25.4
                posstrs.append(format % ("DTG", dtg))
        else:
            # N.B. no conversion here because joint positions are unitless
            posstrs = ["  %s:% 9.4f" % i for i in
                zip(range(self.get_num_joints()), s.joint_actual_position)]
            droposstrs = posstrs
        return limit, homed, posstrs, droposstrs


    def draw_small_origin(self, n):
        glNewList(n, GL_COMPILE)
        r = 2.0/25.4
        glColor3f(*self.colors['small_origin'])

        glBegin(GL_LINE_STRIP)
        for i in range(37):
            theta = (i*10)*math.pi/180.0
            glVertex3f(r*math.cos(theta),r*math.sin(theta),0.0)
        glEnd()
        glBegin(GL_LINE_STRIP)
        for i in range(37):
            theta = (i*10)*math.pi/180.0
            glVertex3f(0.0, r*math.cos(theta), r*math.sin(theta))
        glEnd()
        glBegin(GL_LINE_STRIP)
        for i in range(37):
            theta = (i*10)*math.pi/180.0
            glVertex3f(r*math.cos(theta),0.0, r*math.sin(theta))
        glEnd()

        glBegin(GL_LINES)
        glVertex3f(-r, -r, 0.0)
        glVertex3f( r,  r, 0.0)
        glVertex3f(-r,  r, 0.0)
        glVertex3f( r, -r, 0.0)

        glVertex3f(-r, 0.0, -r)
        glVertex3f( r, 0.0,  r)
        glVertex3f(-r, 0.0,  r)
        glVertex3f( r, 0.0, -r)

        glVertex3f(0.0, -r, -r)
        glVertex3f(0.0,  r,  r)
        glVertex3f(0.0, -r,  r)
        glVertex3f(0.0,  r, -r)
        glEnd()
        glEndList()

    def draw_axes(self, n, letters="XYZ"):
        return  # bypass draw_axes as it might overlay on the program path
        glNewList(n, GL_COMPILE)
        x,y,z,p = 0,1,2,3
        s = self.stat
        view = self.get_view()


        glColor3f(*self.colors['axis_x'])
        glBegin(GL_LINES)
        glVertex3f(1.0,0.0,0.0)
        glVertex3f(0.0,0.0,0.0)
        glEnd()

        if view != x:
            glPushMatrix()
            if self.is_lathe():
                glTranslatef(1.3, -0.1, 0)
                glTranslatef(0, 0, -0.1)
                glRotatef(-90, 0, 1, 0)
                glRotatef(90, 1, 0, 0)
                glTranslatef(0.1, 0, 0)
            else:
                glTranslatef(1.2, -0.1, 0)
                if view == y:
                    glTranslatef(0, 0, -0.1)
                    glRotatef(90, 1, 0, 0)
            glScalef(0.2, 0.2, 0.2)
            self.hershey.plot_string(letters[0], 0.5)
            glPopMatrix()

        glColor3f(*self.colors['axis_y'])
        glBegin(GL_LINES)
        glVertex3f(0.0,0.0,0.0)
        glVertex3f(0.0,1.0,0.0)
        glEnd()

        if view != y:
            glPushMatrix()
            glTranslatef(0, 1.2, 0)
            if view == x:
                glTranslatef(0, 0, -0.1)
                glRotatef(90, 0, 1, 0)
                glRotatef(90, 0, 0, 1)
            glScalef(0.2, 0.2, 0.2)
            self.hershey.plot_string(letters[1], 0.5)
            glPopMatrix()

        glColor3f(*self.colors['axis_z'])
        glBegin(GL_LINES)
        glVertex3f(0.0,0.0,0.0)
        glVertex3f(0.0,0.0,1.0)
        glEnd()

        if view != z:
            glPushMatrix()
            glTranslatef(0, 0, 1.2)
            if self.is_lathe():
                glRotatef(-90, 0, 1, 0)
            if view == x:
                glRotatef(90, 0, 1, 0)
                glRotatef(90, 0, 0, 1)
            elif view == y or view == p:
                glRotatef(90, 1, 0, 0)
            if self.is_lathe():
                glTranslatef(0, -.1, 0)
            glScalef(0.2, 0.2, 0.2)
            self.hershey.plot_string(letters[2], 0.5)
            glPopMatrix()

        glEndList()

    def make_cone(self, n):
        q = gluNewQuadric()
        glNewList(n, GL_COMPILE)
        glEnable(GL_LIGHTING)
        gluCylinder(q, 0, .1, .25, 32, 1)
        glPushMatrix()
        glTranslatef(0,0,.25)
        gluDisk(q, 0, .1, 32, 1)
        glPopMatrix()
        glDisable(GL_LIGHTING)
        glEndList()
        gluDeleteQuadric(q)


    lathe_shapes = [
        None,                           # 0
        (1,-1), (1,1), (-1,1), (-1,-1), # 1..4
        (0,-1), (1,0), (0,1), (-1,0),   # 5..8
        (0,0)                           # 9
    ]
    def lathetool(self, current_tool):
        glDepthFunc(GL_ALWAYS)
        diameter, frontangle, backangle, orientation = current_tool[-4:]
        w = 3/8.

        radius = self.to_internal_linear_unit(diameter) / 2.
        glColor3f(*self.colors['lathetool'])
        glBegin(GL_LINES)
        glVertex3f(-radius/2.0,0.0,0.0)
        glVertex3f(radius/2.0,0.0,0.0)
        glVertex3f(0.0,0.0,-radius/2.0)
        glVertex3f(0.0,0.0,radius/2.0)
        glEnd()

        glNormal3f(0,1,0)

        if orientation == 9:
            glBegin(GL_TRIANGLE_FAN)
            for i in range(37):
                t = i * math.pi / 18
                glVertex3f(radius * math.cos(t), 0.0, radius * math.sin(t))
            glEnd()
        else:
            dx, dy = self.lathe_shapes[orientation]

            min_angle = min(backangle, frontangle) * math.pi / 180
            max_angle = max(backangle, frontangle) * math.pi / 180

            sinmax = math.sin(max_angle)
            cosmax = math.cos(max_angle)
            tanmax = math.cos(max_angle)
            sinmin = math.sin(min_angle)
            cosmin = math.cos(min_angle)
            tanmin = math.cos(min_angle)

            circleminangle = - math.pi/2 + min_angle
            circlemaxangle = - 3*math.pi/2 + max_angle
            d0 = 0

            x1 = (w - d0)

            sz = max(w, 3*radius)

            glBegin(GL_TRIANGLE_FAN)
            glVertex3f(
                radius * dx + radius * math.sin(circleminangle) + sz * sinmin,
                0,
                radius * dy + radius * math.cos(circleminangle) + sz * cosmin)
            for i in range(37):
                #t = circleminangle + i * (circlemaxangle - circleminangle)/36.
                t = circleminangle + i * (circlemaxangle - circleminangle)/36.
                glVertex3f(radius*dx + radius * math.sin(t), 0.0, radius*dy + radius * math.cos(t))

            glVertex3f(
                radius * dx + radius * math.sin(circlemaxangle) + sz * sinmax,
                0,
                radius * dy + radius * math.cos(circlemaxangle) + sz * cosmax)

            glEnd()
        glDepthFunc(GL_LESS)

    def segment_info(self):
        '''
            return mid point of segments in a a line number and 
            size of segments in a line number.
        '''
        pass
    def extents_info(self):
        if self.canon:
            mid = [(a+b)/2 for a, b in zip(self.canon.max_extents, self.canon.min_extents)]
            size = [(a-b) for a, b in zip(self.canon.max_extents, self.canon.min_extents)]
        else:
            mid = [0, 0, 0]
            size = [3, 3, 3]
        return mid, size

    def make_selection_list(self, unused=None):
        select_rapids = self.dlist('select_rapids')
        select_program = self.dlist('select_norapids')
        glNewList(select_rapids, GL_COMPILE)
        if self.canon: self.canon.draw(1, False)
        glEndList()
        glNewList(select_program, GL_COMPILE)
        if self.canon: self.canon.draw(1, True)
        glEndList()

    def make_main_list(self, unused=None):
        program = self.dlist('program_norapids')
        rapids = self.dlist('program_rapids')
        glNewList(program, GL_COMPILE)
        if self.canon: self.canon.draw(0, True)
        glEndList()

        glNewList(rapids, GL_COMPILE)
        if self.canon: self.canon.draw(0, False)
        glEndList()

    def load_preview(self, f, canon, unitcode, initcode, interpname=""):
        self.set_canon(canon)
        result, seq = gcode.parse(f, canon, unitcode, initcode, interpname)
        canon.ofeed = canon.feed

        if result <= gcode.MIN_ERROR:
            self.canon.progress.nextphase(1)
            canon.calc_extents() # so that the milling path will fit to the screen
            self.stale_dlist('program_rapids')
            self.stale_dlist('program_norapids')
            self.stale_dlist('select_rapids')
            self.stale_dlist('select_norapids')
        else:
            error_str = _(gcode.strerror(result))
            # print self
            # print 'Near about line %d: %s' % (seq, error_str)
            self.canon_error = [seq, error_str] 
        return result, seq

    def from_internal_units(self, pos, unit=None):
        if unit is None:
            unit = self.stat.linear_units
        lu = (unit or 1) * 25.4

        lus = [lu, lu, lu, 1, 1, 1, lu, lu, lu]
        return [a*b for a, b in zip(pos, lus)]


# vim:ts=8:sts=4:sw=4:et:
