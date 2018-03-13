/*
 * AutonScript.cpp
 *
 *  Created on: Mar 6, 2018
 *      Author: PatriotRobotics
 */

#include <AutonScripts/AutonScript.h>

AutonScript::AutonScript(int numTrajectories, RobotLogic * bot) {
	this->trajectories = (Segment**)malloc(numTrajectories * sizeof(Segment*));
	this->rightTrajectories = (Segment**)malloc(numTrajectories * sizeof(Segment*));
	this->leftTrajectories = (Segment**)malloc(numTrajectories * sizeof(Segment*));
	this->trajectoriesLength = (int*)malloc(numTrajectories * sizeof(int));

	this->bot = bot;
}

void AutonScript::BuildTrajectory(Waypoint * points, int numPoints, int pathNumber){
	TrajectoryCandidate candidate;
	pathfinder_prepare(points, numPoints, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_FAST, this->bot->timeStep, this->bot->maxVel, this->bot->maxAcc, this->bot->maxJer, &candidate);
	trajectoriesLength[pathNumber] = candidate.length;
	trajectories[pathNumber] = (Segment*)malloc(trajectoriesLength[pathNumber] * sizeof(Segment));
	leftTrajectories[pathNumber] = (Segment*)malloc(trajectoriesLength[pathNumber] * sizeof(Segment));
	rightTrajectories[pathNumber] = (Segment*)malloc(trajectoriesLength[pathNumber] * sizeof(Segment));
	pathfinder_generate(&candidate, trajectories[pathNumber]);
	pathfinder_modify_tank(trajectories[pathNumber], trajectoriesLength[pathNumber], leftTrajectories[pathNumber], rightTrajectories[pathNumber], this->bot->wheelbaseWidth);
}

AutonScript::~AutonScript() {
	// TODO Auto-generated destructor stub
}

