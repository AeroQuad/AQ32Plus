/*
 * navigation.h
 *
 *  Created on: Sep 18, 2014
 *      Author: Ted
 */

#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#define MAX_WAYPOINTS 32  // needed for EEPROM adr offset declarations
#define PRE_WAYPOINT -1
#define UNINITIALIZED -2

#define AUTONAV_DISABLED 0
#define AUTONAV_ENABLED 1
#define POSITION_HOLD 2
#define RETURN_TO_HOME 3
#define SET_HOME_POSITION 4

//#define MIN_DISTANCE_TO_REACHED 2000
#define MAX_ALTITUDE 2000.0
#define MAXBANKANGLE 30 // degrees, should I change all angle errors to max bank angle error?

#define MAXPWM 500 // max PWM to output
#define MAXTRACKANGLE 60 // degrees
extern const double Deg2PWMFactor;// = MAXPWM/MAXTRACKANGLE; // convert degrees into PWM

#define MAXCROSSTRACKANGLE 60 // degrees
#define MAXCROSSTRACKDISTANCE 15 // meters
extern const double Meters2DegFactor;// = MAXCROSSTRACKANGLE/MAXCROSSTRACKDISTANCE; // convert meters into degrees

typedef enum autoNavStates
{
	INITIALIZE,
	NEXT_WAYPOINT,
	PROCESS_NAVIGATION,
	CAPTURE_WAYPOINT,
	FINISH_ROUTE,
	DISABLED
} autoNavStates;
autoNavStates nextNavState;

typedef struct waypoint_t // this is for external comm/storage
{
	int latitude;
	int longitude;
	int altitude;
	int speed;
	int type;
} waypoint_t;

typedef struct waypointType // this is for internal use in AutoNav
{
	double latitude;
	double longitude;
	double altitude;
	double speed;
	int type;
} waypointType;

struct homePositionType
{
	int latitude;
	int longitude;
	int altitude;
} homePosition;

int autoNavState;
int waypointCount;
extern int waypointIndex;

/* New updated GPS Navigation variables */
double fromVector[3], toVector[3], presentPosition[3];
double presentPositionEast[3], presentPositionNorth[3];
double normalRangeVector[3], rangeVector[3];
extern const double zVector[3];
double normalVector[3], normalPerpendicularVector[3], alongPathVector[3], negNormalVector[3];
waypointType fromWaypoint, toWaypoint, currentPosition, followingWaypoint;
double desiredHeading, courseHeading;
double trackAngleError, crossTrackError, crossTrack, alongPathDistance;
double distanceToNextWaypoint;
extern const double earthRadius; // meters
double waypointCaptureDistance; // meters
//float distanceToDestination = 99999999.0;
double currentHeading;

int autoNavRollAxisCorrection;
int autoNavPitchAxisCorrection;
int autoNavYawAxisCorrection;
float forwardSpeed;

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

// Matrix operations
double vectorDotProduct(double vector1[], double vector2[]);
void vectorCrossProduct(double vectorC[3], double vectorA[3], double vectorB[3]);
void vectorNormalize(double result[]);
void positionVector(double *vector, waypointType position);

double adjustHeading(double currentHeading, double desiredHeading);
void setAutoNavState(int state);
int getAutoNavState();
int getWaypointCount();

void processAutoNavigation();
void processPositionHold();
void processReturnToHome();


#endif /* NAVIGATION_H_ */
