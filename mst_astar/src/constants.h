/*******************************************************************************
* @file constants.h
* @author Jason Gassel <jmgkn6@mst.edu>
* @version 1.0
* @date 11/8/12
* @brief Constants related to A* navigation and mapping.
******************************************************************************/

#define PI 3.14159265359

const float MAP_RESOLUTION = 1.0;
const uint32_t MAP_WIDTH = 10;
const uint32_t MAP_HEIGHT = 10;
const double MAP_ORIGIN_POSITION_X = 0.0;
const double MAP_ORIGIN_POSITION_Y = 0.0;
const double MAP_ORIGIN_POSITION_Z = 0.0;
const double MAP_ORIGIN_ORIENTATION_X = 0.0;
const double MAP_ORIGIN_ORIENTATION_Y = 0.0;
const double MAP_ORIGIN_ORIENTATION_Z = 0.0;
const double MAP_ORIGIN_ORIENTATION_W = 0.0;

const int LASER_SCAN_NUM_RAYS = 20;
const double LASER_SCAN_FOV = 60.0;
const int LASER_SCAN_HEALTH = 5;
