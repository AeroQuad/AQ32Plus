/*
 * navigation.c
 *
 *  Created on: Sep 18, 2014
 *      Author: Ted
 */

//#include "navigation.h"
#include "board.h"

int waypointIndex = UNINITIALIZED;
const double Deg2PWMFactor = MAXPWM/MAXTRACKANGLE; // convert degrees into PWM
const double Meters2DegFactor = MAXCROSSTRACKANGLE/MAXCROSSTRACKDISTANCE; // convert meters into degrees
const double zVector[3] = {0.0, 0.0, 1.0};
double const earthRadius = 6378100.0; // meters

////////////////////////////////////////////////////////////////////////////////
//  Vector Dot Product for Double
//  Return the Dot product of vectors a and b with length m
//
//  Call as: vectorDotProduct(a, b)
////////////////////////////////////////////////////////////////////////////////

double vectorDotProduct(double vector1[], double vector2[])
{
  double dotProduct = 0;
  int i;
  for (i = 0; i < 3; i++)
      dotProduct += vector1[i] * vector2[i];
  return dotProduct;
}

////////////////////////////////////////////////////////////////////////////////
//  Vector Cross Product for Double
//  Compute the cross product of vectors a and b with length 3
//  Place result in vector C
//
//  Call as: vectorDotProduct(c, a, b)
////////////////////////////////////////////////////////////////////////////////

void vectorCrossProduct(double vectorC[3], double vectorA[3], double vectorB[3])
{
  vectorC[0] = (vectorA[1] * vectorB[2]) - (vectorA[2] * vectorB[1]);
  vectorC[1] = (vectorA[2] * vectorB[0]) - (vectorA[0] * vectorB[2]);
  vectorC[2] = (vectorA[0] * vectorB[1]) - (vectorA[1] * vectorB[0]);
}

////////////////////////////////////////////////////////////////////////////////
//  Normalize the vector
//  This assumes that result is an array of 3 floats
////////////////////////////////////////////////////////////////////////////////

void vectorNormalize(double result[])
{
  double magnitude = sqrt((result[0]*result[0])+(result[1]*result[1])+(result[2]*result[2]));
  result[0] = result[0]/magnitude;
  result[1] = result[1]/magnitude;
  result[2] = result[2]/magnitude;
}

////////////////////////////////////////////////////////////////////////////////
//  Create position vector
//  Convert lat/lon to ECEF
////////////////////////////////////////////////////////////////////////////////

void positionVector(double *vector, waypointType position) {
  double lat = (double)position.latitude / 10000000.0;
  double lon = (double)position.longitude / 10000000.0;
  vector[0] = cos(lat) * cos(lon);
  vector[1] = cos(lat) * sin(lon);
  vector[2] = sin(lat);
}

double adjustHeading(double currentHeading, double desiredHeading) {
  if ((desiredHeading < -90.0) && (currentHeading > (desiredHeading + 180.0)))
    return currentHeading -= 360.0;
  if ((desiredHeading > 90.0) && ((desiredHeading - 180.0) > currentHeading))
    return currentHeading += 360.0;
  if ((desiredHeading > -90.0) && ((desiredHeading + 180.0) < currentHeading))
    return currentHeading -= 360.0;
  if ((desiredHeading < 90.0) && (currentHeading < (desiredHeading - 180.0)))
    return currentHeading += 360.0;
  return currentHeading;
}

void setAutoNavState(int state)
{
	autoNavState = state;
	if (autoNavState == AUTONAV_ENABLED)
		nextNavState = INITIALIZE;
	if (autoNavState == AUTONAV_DISABLED)
		nextNavState = DISABLED;
}

int getAutoNavState()
{
	return autoNavState;
}

int getWaypointCount()
{
	return waypointCount;
}

void processNavigation()
{
	currentPosition.latitude = gps.latitude;
	currentPosition.longitude = gps.longitude;
	currentPosition.altitude = gps.height;
	double heading = sensors.attitude500Hz[YAW] * R2D;
	heading = (heading > 180.0) ? (heading - 360.0) : heading;

	switch(nextNavState)
	{
	case INITIALIZE:
		waypointCaptureDistance = 2.0; // meters
		forwardSpeed = 15.0;

		distanceToNextWaypoint = 99999999.0;
		autoNavRollAxisCorrection	= 0;
		autoNavPitchAxisCorrection	= 0;
		autoNavYawAxisCorrection	= 0;

		// Creates one time path from current position to first waypoint
		fromWaypoint = currentPosition;
		toWaypoint = eepromConfig.route[0];
		followingWaypoint = eepromConfig.route[1];
		positionVector(fromVector, fromWaypoint);
		positionVector(toVector, toWaypoint);
		vectorCrossProduct(normalVector, fromVector, toVector);
		vectorNormalize(normalVector);
		negNormalVector[0] = -normalVector[0];
		negNormalVector[1] = -normalVector[1];
		negNormalVector[2] = -normalVector[2];
		waypointIndex = PRE_WAYPOINT;
		nextNavState = PROCESS_NAVIGATION;
		break;
	case PROCESS_NAVIGATION:
		// Convert lat/lon to ECI unit vector
		positionVector(presentPosition, currentPosition);

		// Calculate track angle error
		vectorCrossProduct(presentPositionEast, zVector, presentPosition);
		vectorNormalize(presentPositionEast);
		vectorCrossProduct(presentPositionNorth, presentPosition, presentPositionEast);
		vectorNormalize(presentPositionNorth);
		desiredHeading = atan2(vectorDotProduct(normalVector, presentPositionNorth), vectorDotProduct(negNormalVector, presentPositionEast)) * R2D;
		currentHeading = adjustHeading(heading, desiredHeading);
		// units of track angle error is degrees
		trackAngleError = constrain((desiredHeading-currentHeading) * eepromConfig.taeScaling, -MAXTRACKANGLE, MAXTRACKANGLE);

		// Calculate cross track error
		vectorCrossProduct(normalPerpendicularVector, presentPosition, normalVector);
		vectorNormalize(normalPerpendicularVector);
		vectorCrossProduct(alongPathVector, normalVector, normalPerpendicularVector);
		vectorNormalize(alongPathVector);
		crossTrack = earthRadius * atan2(vectorDotProduct(negNormalVector, presentPosition), vectorDotProduct(alongPathVector, presentPosition));
		crossTrackError = -crossTrack * eepromConfig.xteScaling;

		// Calculate distance to next waypoint
		vectorCrossProduct(normalRangeVector, presentPosition, toVector);
		vectorNormalize(normalRangeVector);
		vectorCrossProduct(rangeVector, toVector, normalRangeVector);
		vectorNormalize(rangeVector);
		distanceToNextWaypoint = earthRadius * atan2(vectorDotProduct(rangeVector, presentPosition), vectorDotProduct(presentPosition, toVector));

		// These corrections need to be PWM centered around 0
		autoNavPitchAxisCorrection = forwardSpeed * Deg2PWMFactor * eepromConfig.PID[AUTONAV_PITCH_PID].P; // pitch forward in degrees
		autoNavRollAxisCorrection = constrain(trackAngleError + crossTrackError, -MAXBANKANGLE, MAXBANKANGLE) * Deg2PWMFactor * eepromConfig.PID[AUTONAV_ROLL_PID].P;
		autoNavYawAxisCorrection = constrain(trackAngleError + crossTrackError, -MAXBANKANGLE, MAXBANKANGLE) * Deg2PWMFactor * eepromConfig.PID[AUTONAV_YAW_PID].P;

		double check = ((toWaypoint.longitude - currentPosition.longitude) * (toWaypoint.longitude - fromWaypoint.longitude)) +
		               ((toWaypoint.latitude  - currentPosition.latitude)  * (toWaypoint.latitude  - fromWaypoint.latitude));
		if (check < 0.0)
			nextNavState = NEXT_WAYPOINT;
		else
			nextNavState = PROCESS_NAVIGATION;
		break;
	case NEXT_WAYPOINT:
		waypointIndex++;
		if (waypointIndex < (waypointCount-1))
		{
			fromWaypoint = eepromConfig.route[waypointIndex];
			toWaypoint = eepromConfig.route[waypointIndex+1];
			followingWaypoint = eepromConfig.route[waypointIndex+2];
			positionVector(fromVector, fromWaypoint);
			positionVector(toVector, toWaypoint);
			vectorCrossProduct(normalVector, fromVector, toVector);
			vectorNormalize(normalVector);
			negNormalVector[0] = -normalVector[0];
			negNormalVector[1] = -normalVector[1];
			negNormalVector[2] = -normalVector[2];
			nextNavState = PROCESS_NAVIGATION;
		}
		else
			nextNavState = FINISH_ROUTE;
		break;
	case CAPTURE_WAYPOINT:
		nextNavState = NEXT_WAYPOINT;
		break;
	case FINISH_ROUTE:
		autoNavPitchAxisCorrection = 0.0;
		nextNavState = DISABLED;
		break;
	case DISABLED:
		autoNavPitchAxisCorrection = 0.0;
		break;
	default:
		break;
	}
}
