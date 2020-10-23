/* Name: Nibras Alam & Al Hisham Anik
# ID: 1617818 & 1585385
# CMPUT 275, Winter 2020
#
# Assignment 2: Driving Route Finder Part 2
#
# Consulted C++ Documentation; Used Lecture Slides
*/
#include "draw_route.h"
#include "map_drawing.h"

extern shared_vars shared;

enum {WAIT_FOR_START, WAIT_FOR_STOP} curr_mode = WAIT_FOR_START;

void draw_route() {
    /*
        Draws the path route with a blue line in the map after converting
        lon/lat to x/y

        Arguments:
            None

        Returns:
            None
    */
    if(shared.num_waypoints > 0 && shared.num_waypoints <= 500){
        // going through all but last waypoint and coverting to map coords
        for(int i=0;i<shared.num_waypoints-1;i++) {
            int32_t x1,y1,x2,y2;
            x1 = longitude_to_x(shared.map_number,shared.waypoints[i].lon);
            x2 = longitude_to_x(shared.map_number,shared.waypoints[i+1].lon);
            y1 = latitude_to_y(shared.map_number,shared.waypoints[i].lat);
            y2 = latitude_to_y(shared.map_number,shared.waypoints[i+1].lat);

            x1 = x1-shared.map_coords.x;
            y1 = y1-shared.map_coords.y;
            x2 = x2-shared.map_coords.x;
            y2 = y2-shared.map_coords.y;

            bool outside1 = (x1 < 0 || x1 > 480) && (y1 < 0 || y1 > 320);
            bool outside2 = (x2 < 0 || x2 > 480) && (y2 < 0 || y2 > 320);
            if (!outside1 && !outside2){
                shared.tft->drawLine(x1,y1,x2,y2, TFT_BLUE);
            }
          
        }
    }
}
