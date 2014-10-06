/*
 * navigation.h
 *
 *  Created on: Sep 18, 2014
 *      Author: Ted
 */

#ifndef NAVIGATION_H_
#define NAVIGATION_H_

//#pragma once

/**
 * GPS navigation global declaration
 */
#define MAX_WAYPOINTS 32  // needed for EEPROM adr offset declarations
#define PRE_WAYPOINT -1
#define UNINITIALIZED -2

#define AUTO_NAVIGATION 1
#define POSITION_HOLD 2
#define RETURN_TO_HOME 3
#define SET_HOME_POSITION 4

//#include <GpsAdapter.h>

//#define DEFAULT_HOME_ALTITUDE 5  // default home base altitude is equal to 5 meter
//GeodeticPosition homePosition = GPS_INVALID_POSITION;
//GeodeticPosition missionPositionToReach = GPS_INVALID_POSITION;  // in case of no GPS navigator, indicate the home position into the OSD

int autoNavState;
int waypointCount;
//int gpsRollAxisCorrection = 0;
//int gpsPitchAxisCorrection = 0;
//int gpsYawAxisCorrection = 0;
//uint8_t isPositionHoldInitialized = false;
//uint8_t isGpsNavigationInitialized = false;
//
//int waypointIndex = UNINITIALIZED;
//float distanceToDestination = 99999999.0;
//GeodeticPosition waypoint[MAX_WAYPOINTS] = {
//  GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION,
//  GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION,
//  GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION,
//  GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION};
//GeodeticPosition positionHoldPointToReach = GPS_INVALID_POSITION;

typedef struct waypointType
{
	int latitude;
	int longitude;
	int altitude;
	int speed;
	int type;
} waypointType;

//waypointType waypoint[MAX_WAYPOINTS];

struct homePositionType
{
	int latitude;
	int longitude;
	int altitude;
} homePosition;
//
///* New updated GPS Navigation variables */
//double fromVector[3], toVector[3], presentPosition[3];
//double presentPositionEast[3], presentPositionNorth[3];
//double normalRangeVector[3], rangeVector[3];
//double zVector[3] = {0.0, 0.0, 1.0};
//double normalVector[3], normalPerpendicularVector[3], alongPathVector[3], negNormalVector[3];
//GeodeticPosition fromWaypoint, toWaypoint, currentLocation, followingWaypoint;
//double desiredHeading, currentHeading, groundTrackHeading;
//double trackAngleError, crossTrackError, crossTrack, alongPathDistance;
//double distanceToNextWaypoint = 99999999.0;
//double distanceToFollowingWaypoint = 99999999.0;
//double testDistanceWaypoint = 99999999.0;
//const double earthRadius = 6378100.0; // meters
//double waypointCaptureDistance = 2.0; // meters
//float forwardSpeed = 15.0;
//byte navigatorSerialCommand = OFF; // TODO: remove when autopilot working
//bool isRouteInitialized = false;
//double distanceToGoAlongPath, distanceToGoPosition; // TODO: remove?
//float posRollCommand, posPitchCommand; // TODO: remove?
//long latDelta, lonDelta; // TODO: remove?
//
//double velocityVector[3];
//float velX = 0.0, velY = 0.0, velZ = 0.0;
//int velRollCommand = 0;
//int velPitchCommand = 0;
//double estSpeed, estCourse;
//double previousLat = 0.0;
//double previousLon = 0.0;
//double gpsVelocity[3], accVelocity[3];
//uint8_t routeComplete;
//uint8_t setHomePosition;
//
//// make local when working
//float distanceFromStartToPosition;
//float bearingFromStartToPosition;
//float bearingFromStartToNextWP;
//
//
//void evaluateMissionPositionToReach();
//void processGpsNavigation();




#endif /* NAVIGATION_H_ */
