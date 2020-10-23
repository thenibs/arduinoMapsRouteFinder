/* Name: Nibras Alam & Al Hisham Anik
# ID: 1617818 & 1585385
# CMPUT 275, Winter 2020
#
# Assignment 2: Driving Route Finder Part 2
#
# Consulted C++ Documentation; Used Lecture Slides
*/
#ifndef __SERIAL_HANDLING_H
#define __SERIAL_HANDLING_H

#include "map_drawing.h"
#include "consts_and_types.h"
extern shared_vars shared;

// communication characters
const char space = ' ';
const char newline = '\n';
const char end_flag = 'E';
const char routing_flag = 'R';

// buffer
const uint16_t buffer_size = 256;

// timeout in seconds
const int routing_timeout = 10000;
const int timeout = 1000;

// function declarations
bool get_waypoints(const lon_lat_32& start, const lon_lat_32& end);

#endif
