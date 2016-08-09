//
// Created by Mahi on 6/27/16.
//

#ifndef FRANTICNESS_H
#define FRANTICNESS_H

#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <X11/Xlibint.h>
#include <X11/extensions/record.h>
#include <X11/Xproto.h>
#include <X11/Xlib.h>

typedef union {
    unsigned char type;
    xEvent event;
} DataPack;

int SCREEN_HEIGHT = 0;
int SCREEN_WIDTH = 0;

std::map<std::string, int> map_robot_clicks = {{"miranda", 0}, {"trinculo", 0}, {"ferdinand", 0}, {"caliban", 0}};
std::map<std::string, double> map_robot_moves = {{"miranda", 0}, {"trinculo", 0}, {"ferdinand", 0}, {"caliban", 0}};
std::map<std::string, int> clicks_when_new_goal = {{"miranda", 0}, {"trinculo", 0}, {"ferdinand", 0}, {"caliban", 0}};
std::map<std::string, double> moves_when_new_goal = {{"miranda", 0}, {"trinculo", 0}, {"ferdinand", 0}, {"caliban", 0}};

std::map<std::string, std::vector<double>> time_arrival_waypoints = {
        {"miranda", {}},
        {"trinculo", {}},
        {"ferdinand", {}},
        {"caliban", {}}
};

std::map<std::string, std::vector<double>> time_newgoal_after_arrival = {
    {"miranda", {}},
    {"trinculo", {}},
    {"ferdinand", {}},
    {"caliban", {}},
};

int PREV_MOUSE_POSX = 0;
int PREV_MOUSE_POSY = 0;
double SCRN_DIST_NORMALIZER = 0;
double distance_covered_by_mouse = 0;
double save_ignorance_time = -1.0;

void callback(XPointer xPointer, XRecordInterceptData *hook) {
    if (hook->category != XRecordFromServer) {
        XRecordFreeData(hook);
        return;
    }
    
    DataPack *data = (DataPack *) hook->data;
    int event_type = data->type;
    int time = hook->server_time;

    switch (event_type) {
        case ButtonPress:
//            printf("ButtonPress: X=%d, Y=%d time=%d\n", data->event.u.keyButtonPointer.rootX,
//                   data->event.u.keyButtonPointer.rootY, time);
            if((int) data->event.u.u.detail == 1) {
                map_robot_clicks["miranda"] += 1.0;
                map_robot_clicks["trinculo"] += 1.0;
                map_robot_clicks["ferdinand"] += 1.0;
                map_robot_clicks["caliban"] += 1.0;
            }
            break;
        case MotionNotify:
//            printf("CursorPose: X=%d, Y=%d dist=%f miranda's dist=%f\n", data->event.u.keyButtonPointer.rootX,
//                   data->event.u.keyButtonPointer.rootY, distance_covered_by_mouse, map_robot_moves["miranda"]);
            distance_covered_by_mouse = sqrt(
                    (data->event.u.keyButtonPointer.rootX-PREV_MOUSE_POSX)*
                            (data->event.u.keyButtonPointer.rootX-PREV_MOUSE_POSX) +
                            (data->event.u.keyButtonPointer.rootY-PREV_MOUSE_POSY)*
                                    (data->event.u.keyButtonPointer.rootY-PREV_MOUSE_POSY)) / SCRN_DIST_NORMALIZER;
            map_robot_moves["miranda"] += distance_covered_by_mouse;
            map_robot_moves["trinculo"] += distance_covered_by_mouse;
            map_robot_moves["ferdinand"] += distance_covered_by_mouse;
            map_robot_moves["caliban"] += distance_covered_by_mouse;
            PREV_MOUSE_POSX = data->event.u.keyButtonPointer.rootX;
            PREV_MOUSE_POSY = data->event.u.keyButtonPointer.rootY;
            break;
        default:
            break;
    }
    XRecordFreeData(hook);
}

int mouse_events() {
    Display *record_disp = XOpenDisplay(NULL);
    Display *local_disp = XOpenDisplay(NULL);

    SCREEN_HEIGHT = DefaultScreenOfDisplay(local_disp)->height;
    SCREEN_WIDTH = DefaultScreenOfDisplay(local_disp)->width;
    SCRN_DIST_NORMALIZER = sqrt(SCREEN_HEIGHT*SCREEN_HEIGHT + SCREEN_WIDTH*SCREEN_WIDTH);

    XSynchronize(record_disp, true);
    int major, minor;
    if (!XRecordQueryVersion(record_disp, &major, &minor)) {
        printf("RECORD extension not supported on this X server!\n");
        exit(1);
    }

    printf("RECORD version is %d.%d\n", major, minor);

    XRecordRange *rec_ranges;
    rec_ranges = XRecordAllocRange();

    if (!rec_ranges) {
        printf("Could not alloc record range object!\n");
        exit(1);
    }

    rec_ranges->device_events.first = KeyPress;
    rec_ranges->device_events.last = MotionNotify;

    XRecordClientSpec rec_client_spec = XRecordAllClients;

//    XRecordContext rcxt = XRecordCreateContext(record_disp, 0, (XRecordClientSpec*)XRecordAllClients, 1, &rec_ranges, 1);
    auto rcxt = XRecordCreateContext(record_disp, 0, &rec_client_spec, 1, &rec_ranges, 1);

    if (!rcxt) {
        printf("Error creating a record context!\n");
        exit(1);
    }

    if (!XRecordEnableContext(local_disp, rcxt, callback, NULL)) {
        printf("Error in enable record context\n");
        exit(1);
    }

    while (1)
        XRecordProcessReplies(local_disp);

    XRecordDisableContext(record_disp, rcxt);
    XRecordFreeContext(record_disp, rcxt);
    XFree(rec_ranges);

    XCloseDisplay(local_disp);
    XCloseDisplay(record_disp);
    return 0;
}


#endif //PROJECT_FRANTICNESS_H
