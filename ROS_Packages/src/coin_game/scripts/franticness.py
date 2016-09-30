#!/usr/bin/python

from __future__ import print_function
import sys
import math
import rospy
# Needs Xlib (python-xlib and for C++ libxtst-dev)
from Xlib import X, XK, display
from Xlib.ext import record
from Xlib.protocol import rq


class Franticness(object):
    """
    Franticness is measured using decision interval, error correction, franticness. Follow paper for the definitions.
    """
    def __init__(self, robot_name):
        self._robot_name = robot_name
        self._local_dpy = display.Display()
        self._record_dpy = display.Display()
        self._robot_clicks = 0.
        self._time_arrival_waypoints = rospy.Time.now().to_sec()
        self._time_newgoal_after_arrival = rospy.Time.now().to_sec()
        self._is_first_goal_after_arrival = True
        self._SCREEN_HEIGHT = 1920.
        self._SCREEN_WIDTH = 1080.
        self._PREV_MOUSE_POSX = 0.
        self._PREV_MOUSE_POSY = 0.
        self._CUR_MOUSE_POSX = 0.
        self._CUR_MOUSE_POSY = 0.
        self._SCRN_DIST_NORMALIZER = math.sqrt(self._SCREEN_HEIGHT**2. + self._SCREEN_WIDTH**2.)
        self._distance_covered_by_mouse = 0.
        self._save_ignorance_time = -1.0

    def __str__(self):
        return "franticness: " + self._robot_name + "\n"

    @property
    def decision_intervals(self):
        if self._time_newgoal_after_arrival > self._time_arrival_waypoints:
            return self._time_newgoal_after_arrival - self._time_arrival_waypoints
        else:
            return 0.
    @property
    def error_correction(self):
        return self._robot_clicks

    @error_correction.setter
    def error_correction(self, value):
        self._robot_clicks = float(value)

    @property
    def franticness(self):
        return self._distance_covered_by_mouse

    @franticness.setter
    def franticness(self, value):
        self._distance_covered_by_mouse = float(value)

    def record_callback(self, reply):
        if reply.category != record.FromServer:
            return
        if reply.client_swapped:
            return
        if not len(reply.data) or reply.data[0] < 2:
            # not an event
            return

        data = reply.data
        while len(data):
            event, data = rq.EventField(None).parse_binary_value(data, self._record_dpy.display, None, None)

            if event.type == X.ButtonPress and event.detail == 1:
                # print("name: {} eve: {} er: {} clks: {}".format(self._robot_name, event.detail, self.error_correction, self._robot_clicks))
                # print("dst={} ".format(self._distance_covered_by_mouse))
                self._robot_clicks += 1.
            elif event.type == X.MotionNotify:
                self._PREV_MOUSE_POSX = self._CUR_MOUSE_POSX
                self._PREV_MOUSE_POSY = self._CUR_MOUSE_POSY
                self._CUR_MOUSE_POSX = float(event.root_x)
                self._CUR_MOUSE_POSY = float(event.root_y)
                self._distance_covered_by_mouse += (math.sqrt((self._CUR_MOUSE_POSX - self._PREV_MOUSE_POSX)**2. +
                                                             (self._CUR_MOUSE_POSY - self._PREV_MOUSE_POSY)**2.)) \
                                                  / self._SCRN_DIST_NORMALIZER

    def mouse_events(self):
        # Check if the extension is present

        if not self._record_dpy.has_extension("RECORD"):
            print("RECORD extension not found")
            sys.exit(1)
        r = self._record_dpy.record_get_version(0, 0)
        print("RECORD extension version %d.%d" % (r.major_version, r.minor_version))

        # Create a recording context; we only want key and mouse events
        ctx = self._record_dpy.record_create_context(
                0,
                [record.AllClients],
                [{
                        'core_requests': (0, 0),
                        'core_replies': (0, 0),
                        'ext_requests': (0, 0, 0, 0),
                        'ext_replies': (0, 0, 0, 0),
                        'delivered_events': (0, 0),
                        'device_events': (X.KeyPress, X.MotionNotify),
                        'errors': (0, 0),
                        'client_started': False,
                        'client_died': False,
                }])

        # Enable the context; this only returns after a call to record_disable_context,
        # while calling the callback function in the meantime
        self._record_dpy.record_enable_context(ctx, self.record_callback)

        # Finally free the context
        self._record_dpy.record_free_context(ctx)

if __name__ == "__main__":
    try:
        fran = Franticness("defaul_robot")
        fran.mouse_events()
    except Exception as e:
        print("Error in Franticness!")
        print(e.message)
