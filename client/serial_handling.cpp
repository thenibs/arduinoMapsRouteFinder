/* Name: Nibras Alam & Al Hisham Anik
# ID: 1617818 & 1585385
# CMPUT 275, Winter 2020
#
# Assignment 2: Driving Route Finder Part 2
#
# Consulted C++ Documentation; Used Lecture Slides
*/
#include "serial_handling.h"
#include <Arduino.h>
#include "consts_and_types.h"

extern shared_vars shared;

bool get_waypoints(const lon_lat_32& start, const lon_lat_32& end) {
    /*
        Uses Finite State Machine to request and receive the path
        
        Invalid/No Path: Ends Function, return false
        Timeout/Error: Automatically resends request
        Successful Path and Protocol: Stores waypoints, return true

        Arguments:
            start (const lon_lat_32&): start lon/lat of request
            end (const lon_lat_32&): end lon/lat of request

        Returns:
            true: if valid path
            false: if invalid (> 500 waypoints) or no path
    */
    enum {REQUEST, NUMBER, WAYPOINT, TIMEOUT, DONE, NONE} current = REQUEST;

    // arduino-desktop communication buffers
    String identifier, points, lat, lon;

    // waypoint parsing
    int waypoint_index = 0;

    while (current != DONE) {
        if (current == REQUEST) {
            // send routing request "R start_lat start_lon end_lat end_lon"
            Serial.print(routing_flag);
            Serial.print(space);
            Serial.print(start.lat);
            Serial.print(space);
            Serial.print(start.lon);
            Serial.print(space);
            Serial.print(end.lat);
            Serial.print(space);
            Serial.print(end.lon);
            Serial.print(newline);

            current = NUMBER;

        } else if (current == NUMBER) {
            // waits up to 10 seconds for valid "N x" where x is # waypoints
            unsigned long start_time = millis();
            while (current == NUMBER) {
                if (millis() >= (start_time + routing_timeout)) {
                    current = TIMEOUT;
                    break;

                } else if (Serial.available()) {
                    // reads and parses serial to obtain server message
                    identifier = Serial.readStringUntil(' ');
                    if (identifier[0] == 'N') {
                        points = Serial.readStringUntil('\n');
                        int temp = points.toInt();

                        // special case for no path
                        if (temp == 0 || temp > 500) {
                            current = NONE;
                        
                        } else {
                            shared.num_waypoints = temp;

                            Serial.print("A");
                            Serial.print(newline);

                            current = WAYPOINT;
                        }
                    
                    } else {
                        current = TIMEOUT;
                    }
                }
            }
            // reset buffers
            identifier[0] = 0;
            points[0] = 0;

        } else if (current == WAYPOINT) {
            while (current == WAYPOINT) {
                // waits 1 second waiting for each valid "lat lon" waypoint
                unsigned long start_time = millis();
                if (millis() >= (start_time + timeout)) {
                    current = TIMEOUT;
                    break;

                } else if (Serial.available()) {
                    // reads and parses serial to obtain server message
                    identifier = Serial.readStringUntil(' ');
                    if (identifier[0] == 'W') {
                        lat = Serial.readStringUntil(' ');
                        lon = Serial.readStringUntil('\n');

                        shared.waypoints[waypoint_index] 
                            = lon_lat_32(lon.toInt(), lat.toInt());
                        waypoint_index++;

                        Serial.print("A");
                        Serial.print(newline);

                        current = WAYPOINT;
                    
                    } else if (identifier[0] == 'E') {
                        current = DONE;
                    
                    } else {
                        current = TIMEOUT;
                    }
                }
            }
   
            // reset buffers
            identifier[0] = 0;
            lat[0] = 0;
            lon[0] = 0;

        } else if (current == TIMEOUT) {
            current = REQUEST;
        
        } else if (current == NONE) {
            return false;
        }
    }

    return true;
}
