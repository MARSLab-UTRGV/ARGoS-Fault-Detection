#include "BaseController.h"
#include <random>

using namespace std;
using namespace argos;

/**
 * Constructor for the BaseController. Several important variables are defined here.
 * <p>
 * TODO: update xml configuration file to allow these to be adjusted from configuration without recompiling.
 */
BaseController::BaseController() :
	LF(argos::CSimulator::GetInstance().GetLoopFunctions()),
	WaitTime(0),
	NestDistanceTolerance(0.01),
	NestAngleTolerance(0.05),
	TargetDistanceTolerance(0.01),
	TargetAngleTolerance(0.04),
	SearchStepSize(0.16),
	RobotForwardSpeed(16.0),
	RobotRotationSpeed(4.0),
	TicksToWaitWhileMoving(0.0),
	CurrentMovementState(STOP),
	heading_to_nest(false),
	DestinationNoiseStdev(0),
	PositionNoiseStdev(0),
	collision_counter(0),
	collisionDelay(0),
	RNG(argos::CRandom::CreateRNG("argos")),
	hasFault(0),
	CurrentFaultType(NONE),
	controllerID("none"),
	cbiasSet(false)
{
	// calculate the forage range and compensate for the robot's radius of 0.085m
	argos::CVector3 ArenaSize = LF.GetSpace().GetArenaSize();
	argos::Real rangeX = (ArenaSize.GetX() / 2.0) - 0.085 - 0.1; //ryan luna 12/08/22 ** add -0.1 to avoid robots getting too close to wall
	argos::Real rangeY = (ArenaSize.GetY() / 2.0) - 0.085 - 0.1;
	ForageRangeX.Set(-rangeX, rangeX);
	ForageRangeY.Set(-rangeY, rangeY);
	GoStraightAngleRangeInDegrees.Set(-80, 80);

	//GoStraightAngleRangeInDegrees.Set(-37.5, 37.5);	//does not work when collides with walls
}

argos::CRadians BaseController::GetHeading() {
	/* in ARGoS, the robot's orientation is represented by a quaternion */
	const argos::CCI_PositioningSensor::SReading& readings = compassSensor->GetReading();
	argos::CQuaternion orientation = readings.Orientation;

	/* convert the quaternion to euler angles */
	argos::CRadians z_angle, y_angle, x_angle;
	orientation.ToEulerAngles(z_angle, y_angle, x_angle);

	/* the angle to the z-axis represents the compass heading */

	return z_angle;
}

void BaseController::SetControllerID(string id) {
	controllerID = id;
}

argos::CVector2 BaseController::GetPosition() {
	/* the robot's compass sensor gives us a 3D position */
	argos::CVector3 position3D = compassSensor->GetReading().Position;
	/* return the 2D position components of the compass sensor reading */

	float x = position3D.GetX();
	float y = position3D.GetY();

	CVector2 Offset = GenerateOffset();

	// if (hasFault){
	// 	LOG << "Offset Generated: (" << Offset.GetX() << ", " << Offset.GetY() << ')' << endl;
	// }

	x += Offset.GetX();
	y += Offset.GetY();

	// Add noise to the current position unless travelling to the nest
	// Make the noise proportional to the distance to the target
	/*
	if (!heading_to_nest)
	{
		argos::Real noise_x = RNG->Gaussian(PositionNoiseStdev);
		argos::Real noise_y = RNG->Gaussian(PositionNoiseStdev);

		x += noise_x;
		y += noise_y;
	}
	*/

	return argos::CVector2(x, y);
}

argos::CVector2 BaseController::GetRealPosition() {
	argos::CVector3 position3D = compassSensor->GetReading().Position;
	float x = position3D.GetX();
	float y = position3D.GetY();
	return argos::CVector2(x, y);
}

argos::CVector2 BaseController::GetTarget() {
	return TargetPosition;
}

void BaseController::SetTarget(argos::CVector2 t) {

	argos::Real x(t.GetX()), y(t.GetY());

	if (!heading_to_nest) {
		argos::Real distanceToTarget = (TargetPosition - GetPosition()).Length();
		argos::Real noise_x = RNG->Gaussian(DestinationNoiseStdev*distanceToTarget);
		argos::Real noise_y = RNG->Gaussian(DestinationNoiseStdev*distanceToTarget);
		x += noise_x;
		y += noise_y;
	} 

	if(y > ForageRangeY.GetMax() || y < ForageRangeY.GetMin() ||
			x > ForageRangeX.GetMax() || x < ForageRangeX.GetMin()) {
		x = GetPosition().GetX();
		y = GetPosition().GetY();
		SetRightTurn(37.5);
	}

	TargetPosition = argos::CVector2(x, y);
	argos::Real distanceToTarget = (TargetPosition - GetPosition()).Length();
	
}

void BaseController::SetStartPosition(argos::CVector3 sp) {
	StartPosition = sp;
}

argos::CVector3 BaseController::GetStartPosition() {
	return StartPosition;
}

size_t BaseController::GetMovementState() {
	return CurrentMovementState;
}


void BaseController::SetIsHeadingToNest(bool n) {
	heading_to_nest = n;
}

void BaseController::SetNextMovement() {
	argos::CRadians AngleTol;
	float DistTol;
	
	// Allow the searcher to treat movement to the nest
	// differently than other movement
	if (heading_to_nest) {
		DistTol = NestDistanceTolerance;
		AngleTol = NestAngleTolerance;
	}
	else {
		DistTol = TargetDistanceTolerance;
		AngleTol = TargetAngleTolerance;
	}
 
	if(MovementStack.size() == 0 && CurrentMovementState == STOP) {
		argos::Real distanceToTarget = (TargetPosition - GetPosition()).Length();
		argos::CRadians headingToTarget = (TargetPosition - GetPosition()).Angle();
		argos::CRadians headingToTargetError = (GetHeading() - headingToTarget).SignedNormalize();

		if(!IsAtTarget()) {
			if(headingToTargetError > AngleTol) {
				PushMovement(LEFT, -ToDegrees(headingToTargetError).GetValue());
			} else if(headingToTargetError < -AngleTol) {
				PushMovement(RIGHT, ToDegrees(headingToTargetError).GetValue());
			} else {
				PushMovement(FORWARD, distanceToTarget);
			}
		} else {
			PushMovement(STOP, 0.0);
		}
	} else {
		PopMovement();
	}
}

void BaseController::SetTargetAngleDistance(argos::Real newAngleToTurnInDegrees) {
	// NOTE: the footbot robot has a radius of 0.085 m... or 8.5 cm...
	// adjusting with + 0.02 m, or + 2 cm, increases accuracy...
	
	argos::Real s = 0.105 * newAngleToTurnInDegrees;
        TicksToWaitWhileMoving = std::ceil((SimulationTicksPerSecond() * s) / RobotRotationSpeed);
}

void BaseController::SetTargetTravelDistance(argos::Real newTargetDistance) {
	// convert meters into cm
	argos::Real d = newTargetDistance * 100.0;
	TicksToWaitWhileMoving = std::ceil((SimulationTicksPerSecond() * d) / RobotForwardSpeed);
}

void BaseController::SetLeftTurn(argos::Real newAngleToTurnInDegrees) {
	if(newAngleToTurnInDegrees > 0.0) {
		SetTargetAngleDistance(newAngleToTurnInDegrees);
		CurrentMovementState = LEFT;
	} else if(newAngleToTurnInDegrees < 0.0) {
		SetTargetAngleDistance(-newAngleToTurnInDegrees);
		CurrentMovementState = RIGHT;  
	} else {
		Stop();
	}
}

void BaseController::SetRightTurn(argos::Real newAngleToTurnInDegrees) {
	if(newAngleToTurnInDegrees > 0.0) {
		SetTargetAngleDistance(newAngleToTurnInDegrees);
		CurrentMovementState = RIGHT;
	} else if(newAngleToTurnInDegrees < 0.0) {
		SetTargetAngleDistance(-newAngleToTurnInDegrees);
		CurrentMovementState = LEFT;
	} else {
		Stop();
	}
}

void BaseController::SetMoveForward(argos::Real newTargetDistance) {
	if(newTargetDistance > 0.0) {
		SetTargetTravelDistance(newTargetDistance);
		CurrentMovementState = FORWARD;
	} else if(newTargetDistance < 0.0) {
		SetTargetTravelDistance(newTargetDistance);
		CurrentMovementState = BACK;
	} else {
		Stop();
	}
}

void BaseController::SetMoveBack(argos::Real newTargetDistance) {
	if(newTargetDistance > 0.0) {
		SetTargetTravelDistance(newTargetDistance);
		CurrentMovementState = BACK;
	} else if(newTargetDistance < 0.0) {
		SetTargetTravelDistance(newTargetDistance);
		CurrentMovementState = FORWARD;
	} else {
		Stop();
	}
}

void BaseController::PushMovement(size_t moveType, argos::Real moveSize) {
	Movement newMove = { moveType, moveSize };
	MovementStack.push(newMove);
}

void BaseController::PopMovement() {
	Movement nextMove = MovementStack.top();

	previous_movement = nextMove;

	MovementStack.pop();

	switch(nextMove.type) {

		case STOP: {
			Stop();
			break;
		}

		case LEFT: {
			SetLeftTurn(nextMove.magnitude);
			break;
		}

		case RIGHT: {
			SetRightTurn(nextMove.magnitude);
			break;
		}

		case FORWARD: {
			SetMoveForward(nextMove.magnitude);
			break;
		}

		case BACK: {
			SetMoveBack(nextMove.magnitude);
			break;
		}

	}

}

unsigned int BaseController::GetCollisionTime(){
 return collision_counter;
 }
 
bool BaseController::CollisionDetection() {

	argos::CVector2 collisionVector = GetCollisionVector();
	argos::Real collisionAngle = ToDegrees(collisionVector.Angle()).GetValue();
	bool isCollisionDetected = false;
        
	if(GoStraightAngleRangeInDegrees.WithinMinBoundIncludedMaxBoundIncluded(collisionAngle)
		 && collisionVector.Length() > 0.0) {
	 
		Stop();
		 isCollisionDetected = true;
		 collision_counter++;
   
		while(MovementStack.size() > 0) MovementStack.pop();

		PushMovement(FORWARD, SearchStepSize);

		if(collisionAngle <= 0.0)  {
			//argos::LOG << collisionAngle << std::endl << collisionVector << std::endl << std::endl;
			SetLeftTurn(collisionAngle); //qilu 09/24/2016
		} else {
			//argos::LOG << collisionAngle << std::endl << collisionVector << std::endl << std::endl;
			SetRightTurn(collisionAngle); //qilu 09/24/2016
		}
		Real randomNumber = RNG->Uniform(CRange<Real>(0.5, 1.0)); // Ryan Luna 11/29/22 ** previously (0.5, 1.0)
        collisionDelay = SimulationTick() + (size_t)(randomNumber*SimulationTicksPerSecond());//qilu 10/26/2016
	}

	return isCollisionDetected;
}

argos::CVector2 BaseController::GetCollisionVector() {
	/* Get readings from proximity sensor */
	const argos::CCI_FootBotProximitySensor::TReadings& proximityReadings = proximitySensor->GetReadings();

	/* Sum them together */
	argos::CVector2 collisionVector;

	for(size_t i = 0; i < proximityReadings.size(); ++i) {
		collisionVector += argos::CVector2(proximityReadings[i].Value, proximityReadings[i].Angle);
	}

	collisionVector /= proximityReadings.size();

	return collisionVector;
}

void BaseController::Stop() {
	SetTargetTravelDistance(0.0);
	SetTargetAngleDistance(0.0);
	TicksToWaitWhileMoving = 0.0;
	CurrentMovementState = STOP;
}

void BaseController::Move() {

	if(Wait() == true) return;

	collisionFlag = CollisionDetection();
 //double randomNumber = RNG->Uniform(argos::CRange<double>(0.0, 1.0));//qilu 09/24/2016
 
	/* move based on the movement state flag */
	switch(CurrentMovementState) {

		/* stop movement */
		case STOP: {
			//argos::LOG << "STOP\n";
			wheelActuator->SetLinearVelocity(0.0, 0.0);
			SetNextMovement();
			break;
		}

		/* turn left until the robot is facing an angle inside of the TargetAngleTolerance */
		case LEFT: {
			if((TicksToWaitWhileMoving--) <= 0.0) {
				Stop();
			/*} else {
				//argos::LOG << "LEFT\n";
				wheelActuator->SetLinearVelocity(-RobotRotationSpeed, RobotRotationSpeed);
			}*/
			 }else if(collisionDelay< SimulationTick() || collisionFlag){
				//argos::LOG << "LEFT\n";
				wheelActuator->SetLinearVelocity(-RobotRotationSpeed, RobotRotationSpeed);
			}
		    else wheelActuator->SetLinearVelocity(RobotForwardSpeed, RobotForwardSpeed);  //qilu 10/26/2016 
			break;
		}

		/* turn right until the robot is facing an angle inside of the TargetAngleTolerance */
		case RIGHT: {
			if((TicksToWaitWhileMoving--) <= 0.0) {
				Stop();
			/*} else {
				//argos::LOG << "RIGHT\n";
				wheelActuator->SetLinearVelocity(RobotRotationSpeed, -RobotRotationSpeed);
			}*/
			} else if(collisionDelay< SimulationTick()|| collisionFlag){
				//argos::LOG << "RIGHT\n";
				wheelActuator->SetLinearVelocity(RobotRotationSpeed, -RobotRotationSpeed);
			}
            else wheelActuator->SetLinearVelocity(RobotForwardSpeed, RobotForwardSpeed);  //qilu 10/26/2016  
			break;
		}

		/* move forward until the robot has traveled the specified distance */
		case FORWARD: {
			if((TicksToWaitWhileMoving--) <= 0.0) {
				Stop();
			} else {
			//argos::LOG << "FORWARD\n";
				wheelActuator->SetLinearVelocity(RobotForwardSpeed, RobotForwardSpeed);             
			}
			break;
		}

		/* move backward until the robot has traveled the specified distance */
		case BACK: {
			if((TicksToWaitWhileMoving--) <= 0.0) {
				Stop();
			} else {
			//argos::LOG << "BACK\n";
				wheelActuator->SetLinearVelocity(-RobotForwardSpeed, -RobotForwardSpeed);
			}
			break;
		}
	}
}

bool BaseController::Wait() {

	bool wait = false;

	if(WaitTime > 0) {
		WaitTime--;
		wait = true;
	}

	return wait;
}

void BaseController::Wait(size_t wait_time_in_seconds) {

	WaitTime += (wait_time_in_seconds * SimulationTicksPerSecond());

}

size_t BaseController::SimulationTick() {
	return LF.GetSpace().GetSimulationClock();
}

size_t BaseController::SimulationTicksPerSecond() {
	//return LF.GetSimulator().GetPhysicsEngine("default").GetInverseSimulationClockTick();
    return LF.GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();//qilu 02/06/2021 
}

argos::Real BaseController::SimulationSecondsPerTick() {
	//return LF.GetSimulator().GetPhysicsEngine("default").GetSimulationClockTick();
    return LF.GetSimulator().GetPhysicsEngine("dyn2d").GetSimulationClockTick();//qilu 02/06/2021
}

argos::Real BaseController::SimulationTimeInSeconds() {
	return (argos::Real)(SimulationTick()) * SimulationSecondsPerTick();
}

bool BaseController::IsAtTarget() 
{
	argos::CRadians AngleTol;
	float DistTol;
	
	// Allow the searcher to treat movement to the nest
	// differently than other movement
	if (heading_to_nest)
	{
		DistTol = NestDistanceTolerance;
		AngleTol = NestAngleTolerance;
	}
	else
	{
		DistTol = TargetDistanceTolerance;
		AngleTol = TargetAngleTolerance;
	}

	argos::Real distanceToTarget = (TargetPosition - GetPosition()).Length();

	//argos::LOG << "IsAtTarget: Distance to Target: " << distanceToTarget << endl;
	//argos::LOG << "IsAtTarget: TargetDistanceTolerance: " << DistTol << endl;

	return (distanceToTarget < DistTol) ? (true) : (false);
}

void BaseController::SetFault(	FaultType faultCode, argos::Real desiredOffsetDistance, 
								argos::Real desiredBiasFrequency, argos::CVector2 desiredFrozenCoordinate,
								argos::Real desiredSignalLossDuration, argos::Real desiredDriftRatePerSecond)
{
	if (faultCode != NONE) {
		hasFault = true;
	} else {
		hasFault = false;
	}
	CurrentFaultType = faultCode;
	offsetDistance = desiredOffsetDistance;
	biasFrequency = desiredBiasFrequency;
	frozenCoordinate = desiredFrozenCoordinate;
	signalLossDuration = desiredSignalLossDuration;
	driftRatePerSecond = desiredDriftRatePerSecond;
}

void BaseController::ClearFault(){
	SetFault(NONE);
}

CVector2 BaseController::GenerateOffset() {
    switch (CurrentFaultType) {
		
		case NONE:{
			return CVector2(0,0);
		}
        case C_BIAS: {
			if (cbiasSet){
				return cbiasOffset;
			}
            else{
				return ConsistentBias();
			}
        }
        // case P_BIAS: {
        //     // Implement your logic
        //     break;
        // }
        // case FREEZE: {
        //     // Implement your logic
        //     break;
        // }
        // case T_LOSS: {
        //     // Implement your logic
        //     break;
        // }
        // case DRIFT: {
        //     // Implement your logic
        //     break;
        // }
        default: {
            throw std::runtime_error("Invalid fault type...\nCurrently Available Fault Types: Consistent Bias/Offset (C_BIAS)");
			break;
        }
    }
    // Return default value or throw an exception here if needed
}

/**
 * Generate a bias/offset in a random direction given a desired distance.
*/
CVector2 BaseController::ConsistentBias(){
	CRadians randomAngle = RNG->Uniform(CRange<CRadians>(CRadians::ZERO, CRadians::TWO_PI));
	Real offset_X = offsetDistance * Cos(randomAngle);
	Real offset_Y = offsetDistance * Sin(randomAngle);
	cbiasOffset = CVector2(offset_X, offset_Y);
	cbiasSet = true;
	// LOG << "Offset Generated: " << cbiasOffset << endl;
	return cbiasOffset;
}

/**
 * Broadcast the input string to all controllers using the Range and Bearing Actuator
 * 
 * @param msg The message to be broadcast
*/
void BaseController::Broadcast(string msg){
	argos::CByteArray msgBuf;
	msgBuf << msg << ',';
	while (msgBuf.Size() < 192){
		if (msgBuf.Size() < 192){
			msgBuf << uint8_t(0);
		} else {
			break;
		}
	}
	if (msgBuf.Size() > 192){
		LOG << "msgBuf size: " << msgBuf.Size() << endl;
		LOG << "msgBuf: " << msgBuf << endl;
		LOG << "msg: " << msg << endl;
	}
	// LOG << "send: " << msgBuf << endl;
	RABActuator->SetData(msgBuf);
}

/**
 * Receive the broadcasted data from all controllers using the Range and Bearing Sensor
 * 
 * @return The received data as vector of tuples containing the message, range, and bearing.
*/
vector<tuple<string, Real, CRadians>> BaseController::Receive(){
	const CCI_RangeAndBearingSensor::TReadings& packetQ = RABSensor->GetReadings();
	vector<tuple<string, Real, CRadians>> dataQ;

	for (const CCI_RangeAndBearingSensor::SPacket& p : packetQ) {
		string dataStr(reinterpret_cast<const char*>(p.Data.ToCArray()), p.Data.Size());
		// LOG << "receive: " << p.Data << endl;

		dataQ.push_back(make_tuple(dataStr, p.Range, p.HorizontalBearing));
	}

	return dataQ;
}

void BaseController::ClearRAB(){
	RABActuator->ClearData();
}
//REGISTER_CONTROLLER(BaseController, "BaseController")
