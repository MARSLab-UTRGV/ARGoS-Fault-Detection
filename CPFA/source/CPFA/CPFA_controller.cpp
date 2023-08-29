#include "CPFA_controller.h"
#include <unistd.h>

CPFA_controller::CPFA_controller() :
	RNG(argos::CRandom::CreateRNG("argos")),
	isInformed(false),
	isHoldingFood(false),
	isHoldingFakeFood(false),	// Ryan Luna 11/12/22
	isUsingSiteFidelity(false),
	isGivingUpSearch(false),
	ResourceDensity(0),
	MaxTrailSize(50),
	SearchTime(0),
	CPFA_state(DEPARTING),
	LoopFunctions(NULL),
	survey_count(0),
	isUsingPheromone(0),
    SiteFidelityPosition(1000, 1000), 
	searchingTime(0),
	travelingTime(0),
	startTime(0),
    m_pcLEDs(NULL),
	updateFidelity(false),
	UseQZones(false),
	MergeMode(0),
	FFdetectionAcc(0.0),
	RFdetectionAcc(0.0),
	faultInjected(false),
	faultDetected(false),
	faultLogged(false),
	proximityQueueFull(false)
{
}

void CPFA_controller::Init(argos::TConfigurationNode &node) {
	LOG << "In controller init" << endl;
	compassSensor   = GetSensor<argos::CCI_PositioningSensor>("positioning");
	wheelActuator   = GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering");
	proximitySensor = GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity");
	RABSensor       = GetSensor<argos::CCI_RangeAndBearingSensor>("range_and_bearing");
	RABActuator     = GetActuator<argos::CCI_RangeAndBearingActuator>("range_and_bearing");

	argos::TConfigurationNode settings = argos::GetNode(node, "settings");

	argos::GetNodeAttribute(settings, "FoodDistanceTolerance",   	FoodDistanceTolerance);
	argos::GetNodeAttribute(settings, "TargetDistanceTolerance", 	TargetDistanceTolerance);
	argos::GetNodeAttribute(settings, "NestDistanceTolerance", 		NestDistanceTolerance);
	argos::GetNodeAttribute(settings, "NestAngleTolerance",    		NestAngleTolerance);
	argos::GetNodeAttribute(settings, "TargetAngleTolerance",    	TargetAngleTolerance);
	argos::GetNodeAttribute(settings, "SearchStepSize",          	SearchStepSize);
	argos::GetNodeAttribute(settings, "RobotForwardSpeed",       	RobotForwardSpeed);
	argos::GetNodeAttribute(settings, "RobotRotationSpeed",      	RobotRotationSpeed);
	argos::GetNodeAttribute(settings, "ResultsDirectoryPath",      	results_path);
	argos::GetNodeAttribute(settings, "DestinationNoiseStdev",      DestinationNoiseStdev);
	argos::GetNodeAttribute(settings, "PositionNoiseStdev",      	PositionNoiseStdev);
	argos::GetNodeAttribute(settings, "UseQZones",					UseQZones);
	argos::GetNodeAttribute(settings, "MergeMode",					MergeMode);
	argos::GetNodeAttribute(settings, "FFdetectionAcc",				FFdetectionAcc);
	argos::GetNodeAttribute(settings, "RFdetectionAcc",				RFdetectionAcc);
	

	argos::CVector2 p(GetPosition());
	SetStartPosition(argos::CVector3(p.GetX(), p.GetY(), 0.0));
	
	FoodDistanceTolerance *= FoodDistanceTolerance;
	SetIsHeadingToNest(true);
	/**
	 * Let robots start to search immediately	- qilu 10/21/2016 
	 * 
	 * No need to check for QZones during initialization, as none should exist		- Ryan Luna 01/25/23
	*/
	SetTarget(p);
    controllerID= GetId();
	SetControllerID(controllerID);
    m_pcLEDs   = GetActuator<CCI_LEDsActuator>("leds");
    controllerID= GetId();//qilu 07/26/2016
	m_pcLEDs->SetAllColors(CColor::GREEN);

	/* Fault Detection Initialization */
	for (uint8_t i = 0; i < pow(2,l); i++){
		boost::dynamic_bitset<uint8_t> b_i(l, i);
		stringstream bStr_i;
		bStr_i << b_i;

		E[bStr_i.str()] = E0;
		R[bStr_i.str()] = R0;
		T[bStr_i.str()] = E0 + R0;
		APC[bStr_i.str()] = 0;
		for (uint8_t j = 0; j < pow(2,l); i++){
			boost::dynamic_bitset<uint8_t> b_j(l, j);
			stringstream bStr_j;
			bStr_j << b_j;
			Ec_ij[bStr_i.str()][bStr_j.str()] = 0;
			Rc_ij[bStr_i.str()][bStr_j.str()] = 0;
		}
	}
}

// Ryan Luna 12/28/22
void CPFA_controller::ClearLocalFoodList(){
	LocalFoodList.clear();
}

// Ryan Luna 12/28/22
void CPFA_controller::ClearZoneList(){
	QZoneList.clear();
}

// Ryan Luna 12/28/22
void CPFA_controller::AddZone(QZone newZone){
	QZoneList.push_back(newZone);
}

// Ryan Luna 12/28/22
void CPFA_controller::AddLocalFood(Food newFood){
	LocalFoodList.push_back(newFood);
}

// Ryan Luna 12/28/22
void CPFA_controller::RemoveZone(QZone Z){
	int i = 0;
	for(QZone z : QZoneList){
		if(Z.GetLocation() == z.GetLocation()){
			QZoneList.erase(QZoneList.begin()+i);
		}
		i++;
	}
}

// Ryan Luna 12/28/22 
void CPFA_controller::RemoveLocalFood(Food F){
	int i = 0;
	for(Food f : LocalFoodList){
		if(F.GetLocation() == f.GetLocation()){
			LocalFoodList.erase(LocalFoodList.begin()+i);
		}
		i++;
	}
}

void CPFA_controller::ControlStep() {

	#pragma region Draw Trails

	// Add line so we can draw the trail

	/**
	 * MODIFIED: Use the true position of the robot to draw trail, not the robot's estimated position (potenetially faulty)
	*/

	CVector3 position3d(GetRealPosition().GetX(), GetRealPosition().GetY(), 0.00);
	CVector3 target3d(previous_position.GetX(), previous_position.GetY(), 0.00);
	CRay3 targetRay(target3d, position3d);
	myTrail.push_back(targetRay);
	LoopFunctions->TargetRayList.push_back(targetRay);
	LoopFunctions->TargetRayColorList.push_back(TrailColor);

	previous_position = GetRealPosition();

	#pragma endregion

	// // check if the vote cap has been reached
	// if (voterIDs.size() >= LoopFunctions->VoteCap){
	// 	// process votes
	// 	// if (controllerID == "fb00") LOG << "fboo processing votes. Time:" << LoopFunctions->getSimTimeInSeconds() << endl;
	// 	ProcessVotes();
	// 	voterIDs.clear();
	// }
	// if (faultDetected && !faultLogged){
	// 	stringstream ssLog;
	// 	ssLog << "Fault Detected in foot-bot: " << controllerID;
	// 	ssLog << " Perceived Coord: (" << GetPosition().GetX() << ", " << GetPosition().GetY() << ") True Coord: (" << GetRealPosition().GetX() << ", " << GetRealPosition().GetY() << ")" << endl;
	// 	faultLogged = true;
	// 	throw std::runtime_error(ssLog.str());
	// }

	//UpdateTargetRayList();
	CPFA();
	Move();

	/**
	 * Do fault detection if it is enabled and the bot doesn't have a fault that has been detected
	*/

	// // reset response logged boolean to repeat fault detection process
	// // if (fmod(LoopFunctions->BroadcastFrequency,LoopFunctions->getSimTimeInSeconds() == 0)){
	// if (LoopFunctions->getSimTimeInSeconds() - lastBroadcastTime >= LoopFunctions->BroadcastFrequency){
	// 	responseLogged = false;
	// 	if (controllerID == "fb00") LOG << "fb00 responseLogged set False," << setw(setwidth) << right << "Time:" << LoopFunctions->getSimTimeInSeconds() << endl;
	// }

	// // only begin fault detection process if the response has not been logged
	// // the response logged signifies that the last step of the process has been completed
	// // and we must wait till the next broadcast period to begin the process again
	// // response logged is set to true in ProcessMessages() once the replies are being processed
	// if (!responseLogged){
	// 	if (LoopFunctions->UseFaultDetection && !faultDetected){
	// 		/* Monitor for messages every control step */
	// 		if (controllerID == "fb00") LOG << "fb00 processing messages," << setw(setwidth) << right << "Time:" << LoopFunctions->getSimTimeInSeconds() << endl;
	// 		ProcessMessages();
	// 		/* Broadcast location every 5 seconds */
	// 		// if (fmod(LoopFunctions->BroadcastFrequency,LoopFunctions->getSimTimeInSeconds()) == 0){
	// 		if (LoopFunctions->getSimTimeInSeconds() - lastBroadcastTime >= LoopFunctions->BroadcastFrequency){
	// 			if (controllerID == "fb00") LOG << "fb00 broadcasting location" << setw(setwidth) << right << "Time:" << LoopFunctions->getSimTimeInSeconds() << endl;
	// 			BroadcastLocation();
	// 			lastBroadcastTime = LoopFunctions->getSimTimeInSeconds();
	// 		} else if (!responseQueue.empty()){	
	// 			if (controllerID == "fb00") LOG << "fb00 broadcasting response," << setw(setwidth) << right << "Time:" << LoopFunctions->getSimTimeInSeconds() << endl;
	// 			BroadcastTargetedResponse();
	// 			// if (controllerID == "fb00") LOG << "fb00 broadcast response. " << ", Voter list size: " << voterIDs.size() << endl;
	// 		}
	// 	}
	// }
}

void CPFA_controller::InjectFault(size_t faultCode){
	faultInjected = true;
	FaultType FT = NONE;
	string ft = "NONE";
	switch (faultCode){
		case 0: FT = NONE; ft = "NONE"; break;
		case 1: FT = C_BIAS; ft = "C_BIAS"; break;
		case 2: FT = P_BIAS; ft = "P_BIAS"; break;
		case 3: FT = FREEZE; ft = "FREEZE"; break;
		case 4: FT = T_LOSS; ft = "T_LOSS"; break;
		case 5: FT = DRIFT; ft = "DRIFT"; break;
		default: {
            throw std::runtime_error("Invalid fault type...\nCurrently Available Fault Types: Consistent Bias/Offset (C_BIAS)");
			break;
        }
	}
	LOG << "Fault Type: " << ft << endl;
	SetFault(FT, LoopFunctions->OffsetDistance);
}

bool CPFA_controller::HasFault(){
	return faultInjected;
}

void CPFA_controller::Reset() {
 num_targets_collected =0;
 isHoldingFood   = false;
    isInformed      = false;
    SearchTime      = 0;
    ResourceDensity = 0;
    collisionDelay = 0;
    
  	LoopFunctions->CollisionTime=0; //qilu 09/26/2016
    
    
    /* Set LED color */
    /* m_pcLEDs->SetAllColors(CColor::BLACK); //qilu 09/04 */
    SetTarget(LoopFunctions->NestPosition); //qilu 09/08
    updateFidelity = false;
    TrailToShare.clear();
    TrailToFollow.clear();
    	MyTrail.clear();

	myTrail.clear();

	isInformed = false;
	isHoldingFood = false;
	isUsingSiteFidelity = false;
	isGivingUpSearch = false;
}

bool CPFA_controller::IsHoldingFood() {
		return isHoldingFood;
}

// Ryan Luna 11/12/22
bool CPFA_controller::IsHoldingFakeFood(){
	return isHoldingFakeFood;
}

bool CPFA_controller::IsUsingSiteFidelity() {
		return isUsingSiteFidelity;
}

void CPFA_controller::CPFA() {
	
	switch(CPFA_state) {
		// depart from nest after food drop off or simulation start
		case DEPARTING:
			// if (GetId().compare("fb21") == 0){
			// 	argos::LOG << GetId() <<": DEPARTING" << std::endl;
			// }
			//SetIsHeadingToNest(false);
			Departing();
			break;
		// after departing(), once conditions are met, begin searching()
		case SEARCHING:
			// if (GetId().compare("fb21") == 0){
			// 	argos::LOG << GetId() << ": SEARCHING" << std::endl;
			// }
			//SetIsHeadingToNest(false);
			if((SimulationTick() % (SimulationTicksPerSecond() / 2)) == 0) {
				Searching();
			}
			break;
		// return to nest after food pick up or giving up searching()
		case RETURNING:
			// if (GetId().compare("fb21") == 0){
			// 	argos::LOG << GetId() << ": RETURNING" << std::endl;
			// }
			//SetIsHeadingToNest(true);
			Returning();
			break;
		case SURVEYING:
			// if (GetId().compare("fb21") == 0){
			// 	argos::LOG << GetId() << ": SURVEYING" << std::endl;
			// }
			//SetIsHeadingToNest(false);
			Surveying();
			break;
	}
}

bool CPFA_controller::ThinksIsInTheNest() {
    
	return ((GetPosition() - LoopFunctions->NestPosition).SquareLength()
		< LoopFunctions->NestRadiusSquared);
	}

bool CPFA_controller::ActuallyIsInTheNest() {
	return ((GetRealPosition() - LoopFunctions->NestPosition).SquareLength()
		< LoopFunctions->NestRadiusSquared);
}
void CPFA_controller::SetLoopFunctions(CPFA_loop_functions* lf) {
	LoopFunctions = lf;

	// Initialize the SiteFidelityPosition

	// Create the output file here because it needs LoopFunctions
		
	// Name the results file with the current time and date

	// Quite sure this isn't needed here, defined in CPFA_loop_functions::PostExperiment() ** Ryan Luna 11/11/22

	// time_t t = time(0);   // get time now
	// struct tm * now = localtime(&t);
	// stringstream ss;

	// char hostname[1024];                                                   
	// hostname[1023] = '\0';                    
	// gethostname(hostname, 1023);  

	// ss << "CPFA-"<<GIT_BRANCH<<"-"<<GIT_COMMIT_HASH<<"-"
	// 	<< hostname << '-'
	// 	<< getpid() << '-'
	// 	<< (now->tm_year) << '-'
	// 	<< (now->tm_mon + 1) << '-'
	// 	<<  now->tm_mday << '-'
	// 	<<  now->tm_hour << '-'
	// 	<<  now->tm_min << '-'
	// 	<<  now->tm_sec << ".csv";

	// 	string results_file_name = ss.str();
	// 	results_full_path = results_path+"/"+results_file_name;

	// Only the first robot should do this:	 
	if (GetId().compare("CPFA_0") == 0) {
		/*
		ofstream results_output_stream;
		results_output_stream.open(results_full_path, ios::app);
		results_output_stream << "NumberOfRobots, "
			<< "TargetDistanceTolerance, "
			<< "TargetAngleTolerance, "
			<< "FoodDistanceTolerance, "
			<< "RobotForwardSpeed, "
			<< "RobotRotationSpeed, "
			<< "RandomSeed, "
			<< "ProbabilityOfSwitchingToSearching, "
			<< "ProbabilityOfReturningToNest, "
			<< "UninformedSearchVariation, "   
			<< "RateOfInformedSearchDecay, "   
			<< "RateOfSiteFidelity, "          
			<< "RateOfLayingPheromone, "       
			<< "RateOfPheromoneDecay" << endl
			<< LoopFunctions->getNumberOfRobots() << ", "
			<< CSimulator::GetInstance().GetRandomSeed() << ", "  
			<< TargetDistanceTolerance << ", "
			<< TargetAngleTolerance << ", "
			<< FoodDistanceTolerance << ", "
			<< RobotForwardSpeed << ", "
			<< RobotRotationSpeed << ", "
			<< LoopFunctions->getProbabilityOfSwitchingToSearching() << ", "
			<< LoopFunctions->getProbabilityOfReturningToNest() << ", "
			<< LoopFunctions->getUninformedSearchVariation() << ", "
			<< LoopFunctions->getRateOfInformedSearchDecay() << ", "
			<< LoopFunctions->getRateOfSiteFidelity() << ", "
			<< LoopFunctions->getRateOfLayingPheromone() << ", "
			<< LoopFunctions->getRateOfPheromoneDecay()
			<< endl;
				
			results_output_stream.close();
		*/
	}

}

void CPFA_controller::Departing()
{
	argos::Real distanceToTarget = (GetPosition() - GetTarget()).Length();
	argos::Real randomNumber = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));

	/* When not informed, continue to travel until randomly switching to the searching state. */
	if((SimulationTick() % (SimulationTicksPerSecond() / 2)) == 0) {
		if(isInformed == false){
			if(SimulationTick()%(5*SimulationTicksPerSecond())==0 && randomNumber < LoopFunctions->ProbabilityOfSwitchingToSearching){
				Stop();
				SearchTime = 0;
				CPFA_state = SEARCHING;
				travelingTime+=SimulationTick()-startTime;//qilu 10/22
				startTime = SimulationTick();//qilu 10/22

				/**
				 * Need to check if the initial random search target is in a quarantine zone.
				 * 
				 * Ryan Luna 1/25/23
				 * 
				 * NO LONGER REQUIRED
				 * 
				 * Ryan Luna 1/30/23
				*/


				// bool badTarget = true;
				// int count = 0;
				// bool Terminate = false;
			
				// while(badTarget){

				// 	if (count > 2000){
				// 		Terminate = true;
				// 		break;
				// 	} else if (count % 50 == 0){	// every 50 iterations, increase the search variation
				// 		USV += 0.05;
				// 	}
					
				// 	argos::Real rand = RNG->Gaussian(USV);
				// 	argos::CRadians rotation(rand);
				// 	argos::CRadians angle1(rotation.UnsignedNormalize());
				// 	argos::CRadians angle2(GetHeading().UnsignedNormalize());
				// 	argos::CRadians turn_angle(angle1 + angle2);
				// 	argos::CVector2 turn_vector(SearchStepSize, turn_angle);
				// 	target = turn_vector + GetPosition();
				// 	// cout << "From Departing(), target = " << target << endl;
				// 	if (!TargetInQZone(target)){
				// 		badTarget = false;
				// 	}
				// 	count++;
				// }
				// if (Terminate){
				// 	cout << "Call to Terminate() from Departing()" << endl;
				// 	LoopFunctions->Terminate();
				// }
				
				CVector2 target;
				argos::Real USV = LoopFunctions->UninformedSearchVariation.GetValue();
				argos::Real rand = RNG->Gaussian(USV);
				argos::CRadians rotation(rand);
				argos::CRadians angle1(rotation.UnsignedNormalize());
				argos::CRadians angle2(GetHeading().UnsignedNormalize());
				argos::CRadians turn_angle(angle1 + angle2);
				argos::CVector2 turn_vector(SearchStepSize, turn_angle);
				target = turn_vector + GetPosition();

				SetIsHeadingToNest(false);
				SetTarget(target);
			}
			else if(distanceToTarget < TargetDistanceTolerance){
				SetRandomSearchLocation();
			}
		}
    }
	
	/* Are we informed? I.E. using site fidelity or pheromones. */	
	if(isInformed && distanceToTarget < TargetDistanceTolerance) {
		SearchTime = 0;
		CPFA_state = SEARCHING;
		travelingTime+=SimulationTick()-startTime;//qilu 10/22
		startTime = SimulationTick();//qilu 10/22

		if(isUsingSiteFidelity) {
			isUsingSiteFidelity = false;
			SetFidelityList();
		}
	}
}

void CPFA_controller::Searching() {
 //LOG<<"Searching..."<<endl;
	// "scan" for food only every half of a second
	if((SimulationTick() % (SimulationTicksPerSecond() / 2)) == 0) {
		SetHoldingFood();
	}
	// When not carrying food, calculate movement.
	if(IsHoldingFood() == false) {
		argos::CVector2 distance = GetPosition() - GetTarget();
		argos::Real     random   = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
     
		// If we reached our target search location, set a new one. The 
		// new search location calculation is different based on whether
		// we are currently using informed or uninformed search.
		if(distance.SquareLength() < TargetDistanceTolerance) {
			// randomly give up searching
			if(SimulationTick()% (5*SimulationTicksPerSecond())==0 && random < LoopFunctions->ProbabilityOfReturningToNest) {

				SetFidelityList();
				TrailToShare.clear();
				SetIsHeadingToNest(true);
				SetTarget(LoopFunctions->NestPosition);
				isGivingUpSearch = true;
				LoopFunctions->FidelityList.erase(controllerID);
				isUsingSiteFidelity = false; 
				updateFidelity = false; 
				CPFA_state = RETURNING;
				searchingTime+=SimulationTick()-startTime;
				startTime = SimulationTick();
		
				return; 
				
			}
			argos::Real USCV = LoopFunctions->UninformedSearchVariation.GetValue();
			argos::Real rand = RNG->Gaussian(USCV);

			// uninformed search
			if(isInformed == false) {

				// bool badTarget = true;
				CVector2 target;
				// int count = 0;
				// bool Terminate = false;

				// if (badTarget){
				// 	// cout << "Entering while loop in Searching State..." << endl;
				// 	while(badTarget){
				// 		if (count > 1000){
				// 			Terminate = true;
				// 			break;
				// 		} else if (count % 50 == 0){	// every 50 iterations, increase the search variation
				// 			USCV += 0.05;
				// 		}
				// 		rand = RNG->Gaussian(USCV);
				// 		argos::CRadians rotation(rand);
				// 		argos::CRadians angle1(rotation);
				// 		argos::CRadians angle2(GetHeading());
				// 		argos::CRadians turn_angle(angle1 + angle2);
				// 		argos::CVector2 turn_vector(SearchStepSize, turn_angle);
				// 		target = turn_vector + GetPosition();
				// 		// cout << "Target Location: " << target << endl;
				// 		if (!TargetInQZone(target)) {		// if the target is NOT in a bad location
				// 			badTarget = false;
				// 		}
				// 		count++;
				// 	}
				// 	if (Terminate){
				// 		cout << "Call to Terminate() from Searching()..." << endl;
				// 		LoopFunctions->Terminate();
				// 	}
				// }
				// cout << "Exiting while loop in Searching State..." << endl;

				argos::CRadians rotation(rand);
				argos::CRadians angle1(rotation);
				argos::CRadians angle2(GetHeading());
				argos::CRadians turn_angle(angle1 + angle2);
				argos::CVector2 turn_vector(SearchStepSize, turn_angle);
				target = turn_vector + GetPosition();

				SetIsHeadingToNest(false);
				SetTarget(target);
			}
			// informed search
			else{
				
				/**
				 * No need to check for target in QZone here.
				 * Bots are not laying trails to fake food.
				 * Bots are not choosing site fidelity when fake food is collected.
				 * 
				 * Ryan Luna 01/25/23
				*/

				SetIsHeadingToNest(false);
				
				if(IsAtTarget()) {
					size_t          t           = SearchTime++;
					argos::Real     twoPi       = (argos::CRadians::TWO_PI).GetValue();
					argos::Real     pi          = (argos::CRadians::PI).GetValue();
					argos::Real     isd         = LoopFunctions->RateOfInformedSearchDecay;
					/*argos::Real     correlation = GetExponentialDecay((2.0 * twoPi) - LoopFunctions->UninformedSearchVariation.GetValue(), t, isd);
					argos::Real     rand = RNG->Gaussian(correlation + LoopFunctions->UninformedSearchVariation.GetValue());
					*/ //qilu 09/24/2016
					Real correlation = GetExponentialDecay(rand, t, isd);
					//argos::CRadians rotation(GetBound(rand, -pi, pi));
					argos::CRadians rotation(GetBound(correlation, -pi, pi));//qilu 09/24/2016
					argos::CRadians angle1(rotation);
					argos::CRadians angle2(GetHeading());
					argos::CRadians turn_angle(angle2 + angle1);
					argos::CVector2 turn_vector(SearchStepSize, turn_angle);
		
					SetTarget(turn_vector + GetPosition());
				}
			}
		} else {
			//argos::LOG << "SEARCH: Haven't reached destination. " << GetPosition() << "," << GetTarget() << std::endl;
		}
	}
	else {
		   //argos::LOG << "SEARCH: Carrying food." << std::endl;
	}
}

// Cause the robot to rotate in place as if surveying the surrounding targets
// Turns 36 times by 10 degrees
void CPFA_controller::Surveying() {
 //LOG<<"Surveying..."<<endl;
	if (survey_count <= 4) { 
		CRadians rotation(survey_count*3.14/2); // divide by 10 so the vecot is small and the linear motion is minimized
		argos::CVector2 turn_vector(SearchStepSize, rotation.SignedNormalize());
			
		SetIsHeadingToNest(true); // Turn off error for this
		SetTarget(turn_vector + GetPosition());
		
		if(fabs((GetHeading() - rotation).SignedNormalize().GetValue()) < TargetAngleTolerance.GetValue()) survey_count++;
			//else Keep trying to reach the turning angle
	}
	// Set the survey countdown
	else {
		SetIsHeadingToNest(false); // Turn on error for this
		SetTarget(LoopFunctions->NestPosition); 
		CPFA_state = RETURNING;
		survey_count = 0; // Reset
                searchingTime+=SimulationTick()-startTime;//qilu 10/22
                startTime = SimulationTick();//qilu 10/22
	}
}


/*****
 * RETURNING: Stay in this state until the robot has returned to the nest.
 * This state is triggered when a robot has found food or when it has given
 * up on searching and is returning to the nest.
 *****/
void CPFA_controller::Returning() {
 //LOG<<"Returning..."<<endl;
	//SetHoldingFood();

	/**
	 * MODIFIED:
	*/
	if (IsAtTarget() && ActuallyIsInTheNest()) {

		#pragma region _Deposit Resource

			// Based on a Poisson CDF, the robot may or may not create a pheromone
			// located at the last place it picked up food.
			argos::Real poissonCDF_pLayRate    = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfLayingPheromone);
			argos::Real poissonCDF_sFollowRate = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfSiteFidelity);
			argos::Real r1 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
			argos::Real r2 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
			
			/**
			 * Regardless of holding food or not, reset badfood params
			 * 
			 * Ryan Luna 01/31/23
			*/
			BadFoodCount = 0;
			CurrentZone = NULL;

			if (isHoldingFood) { 
			//drop off the food and display in the nest 
				argos::CVector2 placementPosition;
				placementPosition.Set(LoopFunctions->NestPosition.GetX()+RNG->Gaussian(LoopFunctions->NestRadius/1.2, 0.5), LoopFunctions->NestPosition.GetY()+RNG->Gaussian(LoopFunctions->NestRadius/1.2, 0.5));
			
				while((placementPosition-LoopFunctions->NestPosition).SquareLength()>pow(LoopFunctions->NestRadius/2.0-LoopFunctions->FoodRadius, 2))
					placementPosition.Set(LoopFunctions->NestPosition.GetX()+RNG->Gaussian(LoopFunctions->NestRadius/1.2, 0.5), LoopFunctions->NestPosition.GetY()+RNG->Gaussian(LoopFunctions->NestRadius/1.2, 0.5));

				// only count it if the food is real ** Ryan Luna 11/12/22
				if (!isHoldingFakeFood){	// IF HOLDING REAL FOOD

					num_targets_collected++;
					LoopFunctions->currNumCollectedFood++;
					LoopFunctions->RealFoodCollected++;
					LoopFunctions->setScore(num_targets_collected);
					// delete local food list		Ryan Luna 01/24/23
					ClearLocalFoodList();

					/**
					 * Always lay the pheromone trail if the food is real
					 * 
					 * Ryan Luna 01/25/23
					*/
					if(poissonCDF_pLayRate > r1 && updateFidelity) {
						LoopFunctions->numRealTrails++;
						TrailToShare.push_back(SiteFidelityPosition);	// moved from SetLocalResourseDensity() Ryan Luna 02/05/23
						TrailToShare.push_back(LoopFunctions->NestPosition); //qilu 07/26/2016
						argos::Real timeInSeconds = (argos::Real)(SimulationTick() / SimulationTicksPerSecond());
						Pheromone sharedPheromone(SiteFidelityPosition, TrailToShare, timeInSeconds, LoopFunctions->RateOfPheromoneDecay, ResourceDensity, isHoldingFakeFood);
						LoopFunctions->PheromoneList.push_back(sharedPheromone);
						sharedPheromone.Deactivate(); // make sure this won't get re-added later...
					}
					TrailToShare.clear(); 
					// the nest will detect real food with <RFdetectionAcc> accuracy.
					Real random = RNG->Uniform(CRange<Real>(0.0, 1.0));
					
					if (random <= RFdetectionAcc) {	// passed real food detection probability
						//argos::LOG << "Real Food Aquired" << endl;


					} else {	// TREAT IT AS FAKE FOOD

						//argos::LOG << "Fake Food Aquired" << endl;
						// LoopFunctions->FakeFoodCollected++;

						// if (!LocalFoodList.empty() && UseQZones){	// IF THE LOCAL FOOD LIST IS NOT EMPTY

						// 	// give local food info to nest to create a quarantine zone		Ryan Luna 01/24/23
						// 	LoopFunctions->MainNest.CreateZone(MergeMode, LoopFunctions->FoodList, LocalFoodList, FoodBeingHeld, LoopFunctions->SearchRadius);
						// 	ClearLocalFoodList();
						// 	// possible unsafe usage of FoodBeingHeld (unsure how to clean object memory without destroying it)		// Ryan Luna 01/25/23
						// }
						// if (!UseQZones){
						// 	/**
						// 	 * If we are NOT using QZones, lay a pheromone trail for fake food too
						// 	 * 
						// 	 * Ryan Luna 02/5/23
						// 	*/
						// 	if(poissonCDF_pLayRate > r1 && updateFidelity) {
						// 		LoopFunctions->numRealTrails++;
						// 		TrailToShare.push_back(SiteFidelityPosition);
						// 		TrailToShare.push_back(LoopFunctions->NestPosition); //qilu 07/26/2016
						// 		argos::Real timeInSeconds = (argos::Real)(SimulationTick() / SimulationTicksPerSecond());
						// 		Pheromone sharedPheromone(SiteFidelityPosition, TrailToShare, timeInSeconds, LoopFunctions->RateOfPheromoneDecay, ResourceDensity, isHoldingFakeFood);
						// 		LoopFunctions->PheromoneList.push_back(sharedPheromone);
						// 		sharedPheromone.Deactivate(); // make sure this won't get re-added later...
						// 	}
						// 	TrailToShare.clear(); 
						// }

					}
				} else {	// IF HOLDING FAKE FOOD

					LoopFunctions->FakeFoodCollected++;
					// the nest will detect fake food with <FFdetectionAcc> accuracy.
					Real random = RNG->Uniform(CRange<Real>(0.0, 1.0));
					if (random <= FFdetectionAcc){	// Passed fake food detection probability
						//argos::LOG << "Fake Food Aquired" << endl;

						if (!LocalFoodList.empty() && UseQZones){	// IF THE LOCAL FOOD LIST IS NOT EMPTY

							// give local food info to nest to create a quarantine zone		Ryan Luna 01/24/23
							LoopFunctions->MainNest.CreateZone(MergeMode, LoopFunctions->FoodList, LocalFoodList, FoodBeingHeld, LoopFunctions->SearchRadius);
							ClearLocalFoodList();
							// possible unsafe usage of FoodBeingHeld (unsure how to clean object memory without destroying it)		// Ryan Luna 01/25/23
						}
						if (!UseQZones){
							/**
							 * If we are NOT using QZones, lay a pheromone trail for fake food too
							 * 
							 * Ryan Luna 02/5/23
							*/
							if(poissonCDF_pLayRate > r1 && updateFidelity) {
								LoopFunctions->numFakeTrails++;
								TrailToShare.push_back(SiteFidelityPosition);
								TrailToShare.push_back(LoopFunctions->NestPosition); //qilu 07/26/2016
								argos::Real timeInSeconds = (argos::Real)(SimulationTick() / SimulationTicksPerSecond());
								Pheromone sharedPheromone(SiteFidelityPosition, TrailToShare, timeInSeconds, LoopFunctions->RateOfPheromoneDecay, ResourceDensity, isHoldingFakeFood);
								LoopFunctions->PheromoneList.push_back(sharedPheromone);
								sharedPheromone.Deactivate(); // make sure this won't get re-added later...
							}
							TrailToShare.clear(); 
						}
					} else { // TREAT IT AS REAL FOOD
						LOG << "False Positive Collected..." << endl;
						LoopFunctions->numFalsePositives++;		// increment number of false positives on real food detected
						//argos::LOG << "Real Food Aquired" << endl;
						// num_targets_collected++;
						// LoopFunctions->currNumCollectedFood++;
						// LoopFunctions->RealFoodCollected++;
						// LoopFunctions->setScore(num_targets_collected);

						// delete local food list		Ryan Luna 01/24/23
						ClearLocalFoodList();

						/**
						 * Always lay the pheromone trail if the food is real
						 * 
						 * Ryan Luna 01/25/23
						*/
						if(poissonCDF_pLayRate > r1 && updateFidelity) {
							LoopFunctions->numFakeTrails++;
							TrailToShare.push_back(SiteFidelityPosition);	// moved from SetLocalResourseDensity() Ryan Luna 02/05/23
							TrailToShare.push_back(LoopFunctions->NestPosition); //qilu 07/26/2016
							argos::Real timeInSeconds = (argos::Real)(SimulationTick() / SimulationTicksPerSecond());
							Pheromone sharedPheromone(SiteFidelityPosition, TrailToShare, timeInSeconds, LoopFunctions->RateOfPheromoneDecay, ResourceDensity, isHoldingFakeFood);
							LoopFunctions->PheromoneList.push_back(sharedPheromone);
							sharedPheromone.Deactivate(); // make sure this won't get re-added later...
						}
						TrailToShare.clear(); 
					}
				}
			}

			// Get Quarantine Zone info from nest		// Ryan Luna 01/24/23
			if (!LoopFunctions->MainNest.GetZoneList().empty() && UseQZones){
				ClearZoneList();
				for(int i=0;i<LoopFunctions->MainNest.GetZoneList().size();i++){
					AddZone(LoopFunctions->MainNest.GetZoneList()[i]);
				}
			}

			/**
			 * Determine probabilistically whether to use site fidelity, pheromone
			 * trails, or random search.
			 * 
			 * If pheromone trails are NOT created when fake food is collected, then we don't have to
			 * worry about bots using trails to fake food.
			 * 
			 * Our concern is a bot using site fidelity after fake food is collected. So we must take
			 * into account the 'isHoldingFakeFood' variable when deciding to use site fidelity. Only
			 * when this boolean is false, we may use site fidelity.
			 * 
			 * Ryan Luna 01/25/23
			*/

			// use site fidelity
			if(updateFidelity && poissonCDF_sFollowRate > r2 && !isHoldingFakeFood) {
				SetIsHeadingToNest(false);
				SetTarget(SiteFidelityPosition);
				isInformed = true;
			}
			// use pheromone waypoints
			else if(SetTargetPheromone()) {
				isInformed = true;
				isUsingSiteFidelity = false;
			}
			// use random search
			else {
				SetRandomSearchLocation();
				isInformed = false;
				isUsingSiteFidelity = false;
			}

			isGivingUpSearch = false;
			CPFA_state = DEPARTING;   
			isHoldingFood = false;
			isHoldingFakeFood = false;	// Ryan Luna 11/12/22 
			travelingTime+=SimulationTick()-startTime;//qilu 10/22
			startTime = SimulationTick();//qilu 10/22

			#pragma endregion
	
	} else if (IsAtTarget()) {

		#pragma region _Randomly Search For Nest

			argos::Real USCV = LoopFunctions->UninformedSearchVariation.GetValue();
			argos::Real rand = RNG->Gaussian(USCV);

			argos::CRadians rotation(rand);
			argos::CRadians angle1(rotation);
			argos::CRadians angle2(GetHeading());
			argos::CRadians turn_angle(angle1 + angle2);
			argos::CVector2 turn_vector(SearchStepSize, turn_angle);
			SetIsHeadingToNest(false);
			SetTarget(turn_vector + GetPosition());

		#pragma endregion
	}
}

/**
 * When setting a random search location, avoid using locations within Quarantine Zones
 * 
 * Ryan Luna 01/25/23
*/
void CPFA_controller::SetRandomSearchLocation() {
	argos::Real random_wall = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
	argos::Real x = 0.0, y = 0.0;

	/* north wall */
	if(random_wall < 0.25) {
		x = RNG->Uniform(ForageRangeX);
		y = ForageRangeY.GetMax();
	}
	/* south wall */
	else if(random_wall < 0.5) {
		x = RNG->Uniform(ForageRangeX);
		y = ForageRangeY.GetMin();
	}
	/* east wall */
	else if(random_wall < 0.75) {
		x = ForageRangeX.GetMax();
		y = RNG->Uniform(ForageRangeY);
	}
	/* west wall */
	else {
		x = ForageRangeX.GetMin();
		y = RNG->Uniform(ForageRangeY);
	}
	if (UseQZones){
		cout << "Calling TargetInQZone() from SetRandomSearchLocation()" << endl;
		if (!TargetInQZone(CVector2(x,y))){	// set target if not in bad location
			SetIsHeadingToNest(true); 
			SetTarget(argos::CVector2(x, y));
		} else {
			SetRandomSearchLocation(); // recurse and try again
		}
	} else {
		SetIsHeadingToNest(true); 
		SetTarget(argos::CVector2(x, y));
	}
}

/**
 * Helper function to check whether the referenced target is in a Quarantine Zone
 * in the bot's QZoneList.
 * 
 * Ryan Luna 01/25/23
*/
bool CPFA_controller::TargetInQZone(CVector2 target){
	bool badLocation = false;

	int count = 0;
	
	// cout << "Entering TargetInQZone() loop, checking if target is in a QZone" << endl;
	
	// iterate through the bot's QZoneList
	for (QZone qz : QZoneList){		
		// pythagorean theorem to get distance between two points
		Real d = sqrt( pow( abs(target.GetX()) - abs(qz.GetLocation().GetX()), 2) + pow( abs(target.GetY()) - abs(qz.GetLocation().GetY()), 2) );
		
		if (d <= qz.GetRadius()){	// point is inside qzone
			badLocation = true;
			break;
		}
	}
	// cout << "Exited TargetInQZone()..." << endl;
	return badLocation;
}

/*****
 * Check if the iAnt is finding food. This is defined as the iAnt being within
 * the distance tolerance of the position of a food item. If the iAnt has found
 * food then the appropriate boolean flags are triggered.
 *****/
void CPFA_controller::SetHoldingFood() {
	// Is the iAnt already holding food?
	if(IsHoldingFood() == false) {
		// No, the iAnt isn't holding food. Check if we have found food at our
		// current position and update the food list if we have.
		size_t i = 0, j = 0;

		for(i = 0; i < LoopFunctions->FoodList.size(); i++) {
			if((GetRealPosition() - LoopFunctions->FoodList[i].GetLocation()).SquareLength() < FoodDistanceTolerance ) {
				// We found food!
				// Now check if this food is in Quarantine Zone (if QZoneStrategy is ON)	// Ryan Luna 01/25/23
				bool badFood = false;
				if (UseQZones){
					for (QZone qz : QZoneList){
						for (Food f : qz.GetFoodList()){
							if (f.GetLocation()==LoopFunctions->FoodList[i].GetLocation()){	// bad food found
								badFood = true;

								/**
								 * If we don't have a CurrentZone set, set it
								 * 
								 * Else if the zone we are in matches the CurrentZone we have set,
								 * Increment the BadFoodCount and check if the limit is reached,
								 * If so, return to the nest
								 * 
								 * Else, we are in a new zone that doesn't match our previous CurrentZone,
								 * reset the BadFoodCount and set CurrentZone to this new zone we are in
								*/

								if (CurrentZone == NULL){
									CurrentZone = &qz;
									BadFoodCount++;
								} else if (CurrentZone == &qz){
									BadFoodCount++;
									if (BadFoodCount >= BadFoodLimit){
										SetFidelityList();
										TrailToShare.clear();
										SetIsHeadingToNest(true);
										SetTarget(LoopFunctions->NestPosition);
										isGivingUpSearch = true;
										LoopFunctions->FidelityList.erase(controllerID);
										isUsingSiteFidelity = false; 
										updateFidelity = false; 
										CPFA_state = RETURNING;
										searchingTime+=SimulationTick()-startTime;
										startTime = SimulationTick();
									}
								} else {
									BadFoodCount = 0;
									CurrentZone = &qz;
								}
								
								break;
							}
						}
						if (badFood){break;}
					}
				}
				if (!badFood){	// IF THE FOOD IS NOT IN QZONE THEN PROCEED 
					isHoldingFood = true;
					// Update food variable		// Ryan Luna 1/24/23
					FoodBeingHeld = LoopFunctions->FoodList[i];
					// Check if the food is fake
					if (LoopFunctions->FoodList[i].GetType() == Food::FAKE){	// Ryan Luna 11/12/22
						isHoldingFakeFood = true;
					}
					CPFA_state = SURVEYING;
					j = i + 1;
					searchingTime+=SimulationTick()-startTime;
					startTime = SimulationTick();
					break;
				}
			}
		}
		// We picked up food. Erase the food we picked up from the food list. ** Ryan Luna 11/11/22
		if(IsHoldingFood()){
			LoopFunctions->FoodList.erase(LoopFunctions->FoodList.begin() + i);
			SetLocalResourceDensity();
		}
	}
		
}

/*****
 * If the robot has just picked up a food item, this function will be called
 * so that the food density in the local region is analyzed and saved. This
 * helps facilitate calculations for pheromone laying.
 *
 * Ideally, given that: [*] is food, and [=] is a robot
 *
 * [*] [*] [*] | The maximum resource density that should be calculated is
 * [*] [=] [*] | equal to 9, counting the food that the robot just picked up
 * [*] [*] [*] | and up to 8 of its neighbors.
 *
 * That being said, the random and non-grid nature of movement will not
 * produce the ideal result most of the time. This is especially true since
 * item detection is based on distance calculations with circles.
 *****/
void CPFA_controller::SetLocalResourceDensity() {
	argos::CVector2 distance;

	// remember: the food we picked up is removed from the foodList before this function call
	// therefore compensate here by counting that food (which we want to count)
	ResourceDensity = 1;

	/* Calculate resource density based on the global food list positions. */

	/**
	 * MODIFIED: Now using the robot's true position (no offset from simulated faults)
	 * 
	 * EXPLANATION: Here we are simulating the use of sensors to calculate resource density in the local region. 
	 * 				We must use the true position and not the potentially faulty position the robot thinks it is in.
	*/
	for(size_t i = 0; i < LoopFunctions->FoodList.size(); i++) {
		distance = GetRealPosition() - LoopFunctions->FoodList[i].GetLocation();	// modified ** Ryan Luna 11/11/22

		// Local food found
		if(distance.SquareLength() < LoopFunctions->SearchRadiusSquared*2) {
			ResourceDensity++;
			LoopFunctions->FoodList[i].SetColor(argos::CColor::ORANGE);	// modified ** Ryan Luna 11/11/22
			LoopFunctions->ResourceDensityDelay = SimulationTick() + SimulationTicksPerSecond() * 10;

			// Add to lcoal food list to give to nest 		// Ryan Luna 01/24/23
			if (UseQZones){
				AddLocalFood(LoopFunctions->FoodList[i]);
			}
		}
	}
 
	/**
	 * Set the fidelity position to the robot's current position.
	 * 
	 * UNMODIFIED: We are using the robot's estimated position (potentially faulty)
	*/
    SiteFidelityPosition = GetPosition();
    isUsingSiteFidelity = true;
    updateFidelity = true; 
    // TrailToShare.push_back(SiteFidelityPosition);  // *pheromone waypoint bug fix* -- moved to Returning() -- Ryan Luna 02/25/23
    LoopFunctions->FidelityList[controllerID] = SiteFidelityPosition;
}

/*****
 * Update the global site fidelity list for graphics display and add a new fidelity position.
 *****/
void CPFA_controller::SetFidelityList(argos::CVector2 newFidelity) {
	std::vector<argos::CVector2> newFidelityList;

	/* Remove this robot's old fidelity position from the fidelity list. */
	/*for(size_t i = 0; i < LoopFunctions->FidelityList.size(); i++) {
  if((LoopFunctions->FidelityList[i] - SiteFidelityPosition).SquareLength() != 0.0) {
			newFidelityList.push_back(LoopFunctions->FidelityList[i]);
		}
	} */


	/* Update the global fidelity list. */
	//LoopFunctions->FidelityList = newFidelityList;

        LoopFunctions->FidelityList[controllerID] = newFidelity;
	/* Add the robot's new fidelity position to the global fidelity list. */
	//LoopFunctions->FidelityList.push_back(newFidelity);
 

	/* Update the local fidelity position for this robot. */
	SiteFidelityPosition = newFidelity;
 
  updateFidelity = true;
}

/*****
 * Update the global site fidelity list for graphics display and remove the old fidelity position.
 *****/
void CPFA_controller::SetFidelityList() {
	std::vector<argos::CVector2> newFidelityList;

	/* Remove this robot's old fidelity position from the fidelity list. */
	/* Update the global fidelity list. */
        LoopFunctions->FidelityList.erase(controllerID);
 SiteFidelityPosition = CVector2(10000, 10000);
 updateFidelity = true; 
}

/*****
 * Update the pheromone list and set the target to a pheromone position.
 * return TRUE:  pheromone was successfully targeted
 *        FALSE: pheromones don't exist or are all inactive
 *****/
bool CPFA_controller::SetTargetPheromone() {
	argos::Real maxStrength = 0.0, randomWeight = 0.0;
	bool isPheromoneSet = false;

 	if(LoopFunctions->PheromoneList.size()==0) return isPheromoneSet; //the case of no pheromone.
	/* update the pheromone list and remove inactive pheromones */

	/* default target = nest; in case we have 0 active pheromones */
	//SetIsHeadingToNest(true);
	//SetTarget(LoopFunctions->NestPosition);
	/* Calculate a maximum strength based on active pheromone weights. */
	for(size_t i = 0; i < LoopFunctions->PheromoneList.size(); i++) {
		if(LoopFunctions->PheromoneList[i].IsActive()) {
			maxStrength += LoopFunctions->PheromoneList[i].GetWeight();
		}
	}

	/* Calculate a random weight. */
	randomWeight = RNG->Uniform(argos::CRange<argos::Real>(0.0, maxStrength));

	/* Randomly select an active pheromone to follow. */
	for(size_t i = 0; i < LoopFunctions->PheromoneList.size(); i++) {
		   if(randomWeight < LoopFunctions->PheromoneList[i].GetWeight()) {
			       /* We've chosen a pheromone! */
			       SetIsHeadingToNest(false);
          SetTarget(LoopFunctions->PheromoneList[i].GetLocation());
          TrailToFollow = LoopFunctions->PheromoneList[i].GetTrail();
          isPheromoneSet = true;
          /* If we pick a pheromone, break out of this loop. */
          break;
     }

     /* We didn't pick a pheromone! Remove its weight from randomWeight. */
     randomWeight -= LoopFunctions->PheromoneList[i].GetWeight();
	}

	//ofstream log_output_stream;
	//log_output_stream.open("cpfa_log.txt", ios::app);
	//log_output_stream << "Found: " << LoopFunctions->PheromoneList.size()  << " waypoints." << endl;
	//log_output_stream << "Follow waypoint?: " << isPheromoneSet << endl;
	//log_output_stream.close();

	return isPheromoneSet;
}

/*****
 * Calculate and return the exponential decay of "value."
 *****/
argos::Real CPFA_controller::GetExponentialDecay(argos::Real w, argos::Real time, argos::Real lambda) {
	/* convert time into units of haLoopFunctions-seconds from simulation frames */
	//time = time / (LoopFunctions->TicksPerSecond / 2.0);

	//LOG << "time: " << time << endl;
	//LOG << "correlation: " << (value * exp(-lambda * time)) << endl << endl;

	//return (value * std::exp(-lambda * time));
    Real     twoPi       = (CRadians::TWO_PI).GetValue();
    return w + (twoPi-w)* exp(-lambda * time);
}

/*****
 * Provides a bound on the value by rolling over a la modulo.
 *****/
argos::Real CPFA_controller::GetBound(argos::Real value, argos::Real min, argos::Real max) {
	/* Calculate an offset. */
	argos::Real offset = std::abs(min) + std::abs(max);

	/* Increment value by the offset while it's less than min. */
	while (value < min) {
			value += offset;
	}

	/* Decrement value by the offset while it's greater than max. */
	while (value > max) {
			value -= offset;
	}

	/* Return the bounded value. */
	return value;
}

size_t CPFA_controller::GetSearchingTime(){//qilu 10/22
    return searchingTime;
}
size_t CPFA_controller::GetTravelingTime(){//qilu 10/22
    return travelingTime;
}

string CPFA_controller::GetStatus(){//qilu 10/22
    //DEPARTING, SEARCHING, RETURNING
    if (CPFA_state == DEPARTING) return "DEPARTING";
    else if (CPFA_state ==SEARCHING)return "SEARCHING";
    else if (CPFA_state == RETURNING)return "RETURNING";
    else if (CPFA_state == SURVEYING) return "SURVEYING";
    //else if (MPFA_state == INACTIVE) return "INACTIVE";
    else return "SHUTDOWN";
    
}

/*****
 * Return the Poisson cumulative probability at a given k and lambda.
 *****/
argos::Real CPFA_controller::GetPoissonCDF(argos::Real k, argos::Real lambda) {
	argos::Real sumAccumulator       = 1.0;
	argos::Real factorialAccumulator = 1.0;

	for (size_t i = 1; i <= floor(k); i++) {
		factorialAccumulator *= i;
		sumAccumulator += pow(lambda, i) / factorialAccumulator;
	}

	return (exp(-lambda) * sumAccumulator);
}

void CPFA_controller::UpdateTargetRayList() {
	if(SimulationTick() % LoopFunctions->DrawDensityRate == 0 && LoopFunctions->DrawTargetRays == 1) {

		/**
		 * MODIFIED: Must use the robot's true position. This function is used for visualization only.
		*/

		/* Get position values required to construct a new ray */
		argos::CVector2 t(GetTarget());
		argos::CVector2 p(GetRealPosition());
		argos::CVector3 position3d(p.GetX(), p.GetY(), 0.02);
		argos::CVector3 target3d(t.GetX(), t.GetY(), 0.02);

		/* scale the target ray to be <= searchStepSize */
		argos::Real length = std::abs(t.Length() - p.Length());

		if(length > SearchStepSize) {
			MyTrail.clear();
		} else {
			/* add the ray to the robot's target trail */
			argos::CRay3 targetRay(target3d, position3d);
			MyTrail.push_back(targetRay);

			/* delete the oldest ray from the trail */
			if(MyTrail.size() > MaxTrailSize) {
				MyTrail.erase(MyTrail.begin());
			}

			LoopFunctions->TargetRayList.insert(LoopFunctions->TargetRayList.end(), MyTrail.begin(), MyTrail.end());
			// loopFunctions.TargetRayList.push_back(myTrail);
		}
	}
}


/********************************************************************************/
/*								Fault Detection									*/
/********************************************************************************/

/// @brief This is a wrapper for use in loop function code
void CPFA_controller::ClearRABData(){
	ClearRAB();
}

/// @brief Broadcast a simple message for RAB data collection. Mode = 'r'.
void CPFA_controller::Ping(){
	string selfID = controllerID;
	Broadcast("p," + selfID);
}

/// @brief Get the RAB Data (the range and bearing of the signal).
/// @brief The 'mode' determines how to handle the incoming RAB data/msgs.
/// @param mode 
void CPFA_controller::GetRABData(){

	vector<tuple<string, Real, CRadians>> msgQueue = Receive();
	// LOG << LoopFunctions->getSimTimeInSeconds() << endl;

	// to store the range values gathered with the RAB sensor
	vector<Real> proximityData;

	for(auto it = msgQueue.begin(); it != msgQueue.end(); ++it) {

		// process message (the message should always begin with the message type)
		stringstream ss(get<0>(*it));
		// LOG << "converted: " << ss.str() << endl;
		string msgType;
		getline(ss, msgType, ',');

		// if (controllerID == "fb00") LOG << "fb00 received: " << ss.str() << setw(setwidth) << right << "Time: " << LoopFunctions->getSimTimeInSeconds() << endl;
		if (msgType == "r"){ 
			LOG << "WARNING: received response type message during ping mode in ProcessMessages()" << endl;
		} else if (msgType == "p"){

			string senderID;
			getline(ss, senderID, ',');

			Real signalRange = get<1>(*it);						// range of signal provided by RAB Sensor
			CRadians signalBearing = get<2>(*it);				// bearing of signal provided by RAB Sensor
			// if (controllerID == "fb00") LOG << "fb00 received broadast: " << ss.str() << setw(setwidth) << right << "Time: " << LoopFunctions->getSimTimeInSeconds() << endl;
			
			/**
			 * - The signal range from the RAB sensor is given in cm, so we convert to meters.
			 * - The bearing from the RAB sensor is given relative to the robot, so we must consider the 
			 * 		robot's heading to convert it into a global representation.
			*/
			
			/* Push data into a list and this list into a queue after all data is read */
			proximityData.push_back(signalRange);

		}else if (ss.eof()){
			if (controllerID == "fb00") LOG << "fb00 received EOF" << setw(setwidth) << right << "Time: " << LoopFunctions->getSimTimeInSeconds() << endl;
		
		} else {
			LOG << "runtime_error: " << msgType << endl;
			// throw runtime_error("Runtime Error: " + msgType + "is not a valid message type...\n");
		}
	}

	/* Always update proximity data even when there is no msgs (no other bots in range) 
	* 	- In the even that there is no bots within range then a value of 0 will be pushed into the queue
	* 	- This is important because the queue is used to determine the number of bots within range
	*/
	UpdateProximityQueue(proximityData);
	proximityData.clear();
}

void CPFA_controller::GetFVData(){

	/**
	 * Clear FV Map
	 * 
	 * - We need a fresh map to store the new FVs that we will receive
	 * - I think this can be done here. 
	*/
	for (auto it = bfvMap.begin(); it != bfvMap.end(); ++it){
		it->second.clear();
	}

	vector<tuple<string, Real, CRadians>> msgQueue = Receive();

	// loop through message queue
	for(auto it = msgQueue.begin(); it != msgQueue.end(); ++it) {
		// process message (the message should always begin with the message type)
		stringstream ss(get<0>(*it));
		// LOG << "converted: " << ss.str() << endl;
		string msgType;
		getline(ss, msgType, ',');

		if (msgType == "p") LOG << "WARNING: Received ping type message in GetFVData()..." << endl;
		else if (msgType == "b"){
			// if (controllerID == "fb00") LOG << "fb00 received response" << setw(setwidth) << right << "Time: " << LoopFunctions->getSimTimeInSeconds() << endl;
			// else LOG << controllerID << " received response" << setw(setwidth) << right << "Time: " << LoopFunctions->getSimTimeInSeconds() << endl;
			
			/*TODO: Evaluate ss while loop */
			/**
			 * I have concerns on whether this is necessary. Might need a better method.
			 * I think that if we do not reach eof after a single iteration then there is a problem. (this is what needs to be checked)
			*/
			while (!ss.eof()){
				// see format details of ShareBFV()
				string senderID, f1, f2;
				getline(ss, senderID, ',');
				getline(ss, f1, ',');
				getline(ss, f2, ',');
				// more features F3, F4, ... will be added here upon implementation

				/*TODO: Check if everything below can be done outside this while loop */

				// vector<char> xbfv;
				// xbfv.push_back(<char>f1);
				// xbfv.push_back(<char>f2);

				/* Rebuild the feature fector of the sender */
				stringstream xbfv;
				xbfv << f1 << f2;


				/**
				 *									**** Update FV Map *****
				 * 
				 * /*TODO: Evaluate method to update FV map
				 * 
				 * - I am wondering if we should just keep track of the senderID's that we have already received a FV from
				 * instead of looping through the map to check if the senderID already exists in the map
				 * Might be more efficient...
				 * 
				 * - We now have a method above to clear the FV map. This might not be a concern anymore.
				*/
				if (bfvMap.find(xbfv.str()) != bfvMap.end()){	// the FV exists
					for (auto it = bfvMap[xbfv.str()].begin(); it != bfvMap[xbfv.str()].end(); ++it){	// make sure we don't put the robot in the list twice
						if (*it == senderID){
							LOG << "WARNING: " << senderID << " already exists for FV " << xbfv.str() << endl;
							break;
						} else {	// add the robot to the list
							bfvMap[xbfv.str()].push_back(senderID);
						}
					}
				
				} else {	// the FV does not exist
					bfvMap[xbfv.str()].push_back(senderID);	// add it to the map along with the robot ID
					
					/* We are now initializing the T-Cells and APCs in the init() function. */

					// /* the t-cells in the CRM don't exist yet either for this FV, so we should create them here */
					// E[xbfv.str()] = E0;	// add the FV to the Ei list with E0 as the initial value
					// R[xbfv.str()] = R0;	// add the FV to the Ri list with R0 as the initial value
					// T[xbfv.str()] = E0 + R0;	// add the FV to the Ti list with E0 + R0 as the initial value
				}

			}
		}
		else if (ss.eof()){
			if (controllerID == "fb00") LOG << "fb00 received EOF" << setw(setwidth) << right << "Time: " << LoopFunctions->getSimTimeInSeconds() << endl;
		} else {
			LOG << "runtime_error: " << msgType << endl;
			// throw runtime_error("Runtime Error: " + msgType + "is not a valid message type...\n");
		}
	}

	/* After all FV data is gathered */
	SetupCRM();
}

/// @brief Setup/Update the densities of the APCs, Effector cells, and Regulatory cells for the CRM. 
void CPFA_controller::SetupCRM(){

	/* Assign FVs to APCs with scaling factor k */
	for (auto it = bfvMap.begin(); it != bfvMap.end(); ++it){
		APC[it->first] = k * it->second.size();	// Aj is <the number of robots that have FV_j> * k
	}

	/* Compute influx of new T-Cells */
	for (auto it = APC.begin(); it != APC.end(); ++it){
		if (it->second > 0){	// if A[j] > 0, then increment E[j] and R[j] by Ie and Ir, respectively
			E[it->first] += Ie;
			R[it->first] += Ir;
			T[it->first] += Ie + Ir;
		}
	}
}

/// @brief Get Hamming Distance between two binary strings
/// @param a The first binary string
/// @param b The second binary string
/// @return The Hamming Distance
uint8_t CPFA_controller::HammingDistance(const string& a, const string& b){
	uint8_t distance = 0;
	for (uint8_t i = 0; i < a.size(); i++){
		if (a[i] != b[i]){
			distance++;
		}
	}
	return distance;
}

pair<map<string,Real>,map<string,Real>> CPFA_controller::CalculateConjugates(){
	
	map<string, Real> C, Ec, Rc;	// Cj, Ecj, and Rcj

	/* Initialize Cj, Ecj, Rcj (i.e., create keys corresponding to the FVs similar to init function)*/
	for (auto it = APC.begin(); it != APC.end(); ++it){
		C[it->first] = 0;
		Ec[it->first] = 0;
		Rc[it->first] = 0;
	}

	/* Compute Cj, Ecj, and Rcj */
	for (uint8_t j = 0; j < M; j++){
		
		/* Encode iterator j as a binary string representing a fv combination (e.g., 00,01,10,11) */
		boost::dynamic_bitset<uint8_t> b_j(l,j);
		stringstream fv_j;
		fv_j << b_j;

		/* only consider APCs that have a sub-population density > 0 */
		if (!APC[fv_j.str()] > 0){	// if APC_j <= 0, skip this iteration
			continue;
		}

		/* Initialize summation variables */
		Real sum_theta_Ti = 0;
		Real sum_theta_Ei = 0;
		Real sum_theta_Ri = 0;
		
		for (uint8_t i = 0; i < N; i++){
			
			/* Encode iterator i as a binary string representing a fv combination (e.g., 00,01,10,11) */
			boost::dynamic_bitset<uint8_t> b_i(l,i);
			stringstream fv_i;
			fv_i << b_i;

			/* only consider T-cell clones that have a density > 0 */
			if (!T[fv_i.str()] > 0){	// if T_i <= 0, skip this iteration
				continue;
			}
			
			/* Compute the affinity (interaction) between T-cell clone i and APC j */
			Real theta_ij = exp(-HammingDistance(fv_i.str(), fv_j.str())/(c*l));	// calculate theta_ij

			/* Compute the summations */
			sum_theta_Ti += theta_ij * T[fv_i.str()];	// calculate sum(theta_ij * T_i)
			sum_theta_Ei += theta_ij * E[fv_i.str()];	// calculate sum(theta_ij * E_i)
			sum_theta_Ri += theta_ij * R[fv_i.str()];	// calculate sum(theta_ij * R_i)
		}

		/* Compute Cj, Ecj, and Rcj */
		C[fv_j.str()] = (gamma_c * APC[fv_j.str()] * s * sum_theta_Ti) / (gamma_d + gamma_c * sum_theta_Ti);	// calculate C_j
		Ec[fv_j.str()] = C[fv_j.str()] * sum_theta_Ei / sum_theta_Ti;	// calculate Ec_j
		Rc[fv_j.str()] = C[fv_j.str()] * sum_theta_Ri / sum_theta_Ti;	// calculate Rc_j
	}

	/* Compute Ec_ij and Rc_ij */
	for (uint8_t j = 0; j < M; j++){

		/* Encode iterator j as a binary string representing a fv combination (e.g., 00,01,10,11) */
		boost::dynamic_bitset<uint8_t> b_j(l,j);
		stringstream fv_j;
		fv_j << b_j;

		/* only consider APCs that have a sub-population density > 0 */
		if (!APC[fv_j.str()] > 0){	// if APC_j <= 0, skip this iteration
			continue;
		}

		for (uint8_t i = 0; i < N; i++){

			/* Encode iterator i as a binary string representing a fv combination (e.g., 00,01,10,11) */
			boost::dynamic_bitset<uint8_t> b_i(l,i);
			stringstream fv_i;
			fv_i << b_i;

			/* only consider T-cell clones that have a density > 0 */
			if (!T[fv_i.str()] > 0){	// if T_i <= 0, skip this iteration
				continue;
			}

			/* Compute the affinity (interaction) between T-cell clone i and APC j */
			Real theta_ij = exp(-HammingDistance(fv_i.str(), fv_j.str())/(c*l));	// calculate theta_ij

			/* Initialize summation variables */
			Real sum_theta_Eu = 0;
			Real sum_theta_Ru = 0;

			for (uint8_t u = 0; u < N; u++){

				/* Encode iterator u as a binary string representing a fv combination (e.g., 00,01,10,11) */
				boost::dynamic_bitset<uint8_t> b_u(l,u);
				stringstream fv_u;
				fv_u << b_u;

				/* only consider T-cell clones that have a density > 0 */
				if (!T[fv_u.str()] > 0){	// if T_i <= 0, skip this iteration
					continue;
				}

				/* Compute the affinity (interaction) between T-cell clone u and APC j */
				Real theta_uj = exp(-HammingDistance(fv_u.str(), fv_j.str())/(c*l));	// calculate theta_uj

				/* Compute the summations */
				sum_theta_Eu += theta_uj * E[fv_u.str()];	// calculate sum(theta_uj * E_u)
				sum_theta_Ru += theta_uj * R[fv_u.str()];	// calculate sum(theta_uj * R_u)		Not sure we need this but computed it anyway...
			}

			/* Final Calculations: Compute the number of conjugated effector and regulatory cells in quasi-steady state (update Ec_ij and Rc_ij) */
			Ec_ij[fv_i.str()][fv_j.str()] = Ec[fv_j.str()] * theta_ij * E[fv_i.str()] / sum_theta_Eu;	// calculate Ec_ij
			Rc_ij[fv_i.str()][fv_j.str()] = Rc[fv_j.str()] * theta_ij * R[fv_i.str()] / sum_theta_Eu;	// calculate Rc_ij
		}
	}

	return make_pair(Ec, Rc);
}

Real CPFA_controller::P_e(Real Aj, Real Ecj, Real Rcj){
	return ((Rcj - 3 * Aj) * (Rcj - 3 * Aj)) / (9 * Aj * Aj);
}

Real CPFA_controller::P_r(Real Aj, Real Ecj, Real Rcj){
	return ((6 * Aj - Ecj) * Ecj) / (9 * Aj * Aj);
}

void CPFA_controller::ForwardEuler(){

	Real t = 0;	// time

	while (t <= S){

		/* Compute the conjugates */
		pair<map<string,Real>,map<string,Real>> conjugates = CalculateConjugates();
		map<string,Real> Ec = conjugates.first;
		map<string,Real> Rc = conjugates.second;

		/* Initialize E_star and R_star */
		map<string, Real> E_star;
		map<string, Real> R_star;
		for (uint8_t i = 0; i < N; i++){
			boost::dynamic_bitset<uint8_t> b_i(l, i);
			stringstream bStr_i;
			bStr_i << b_i;
			E_star[bStr_i.str()] = 0;
			R_star[bStr_i.str()] = 0;
		}

		/* Compute E_star and R_star */
		for (uint8_t i = 0; i < N; i++){

			/* Encode iterator i as a binary string representing a fv combination (e.g., 00,01,10,11) */
			boost::dynamic_bitset<uint8_t> b_i(l,i);
			stringstream fv_i;
			fv_i << b_i;

			for (uint8_t j = 0; j < M; j++){

				/* Encode iterator j as a binary string representing a fv combination (e.g., 00,01,10,11) */
				boost::dynamic_bitset<uint8_t> b_j(l,j);
				stringstream fv_j;
				fv_j << b_j;

				/* Compute summation for E_star and R_star */
				E_star[fv_i.str()] += P_e(APC[fv_j.str()], Ec[fv_j.str()], Rc[fv_j.str()]) * Ec_ij[fv_i.str()][fv_j.str()];
				R_star[fv_i.str()] += P_r(APC[fv_j.str()], Ec[fv_j.str()], Rc[fv_j.str()]) * Rc_ij[fv_i.str()][fv_j.str()];
			}

			/* Euler Step for E[i] */
			E[fv_i.str()] = E[fv_i.str()] + h * (rho_E * E_star[fv_i.str()] - Delta * E[fv_i.str()]);
			
			/* Euler Step for R[i] */
			R[fv_i.str()] = R[fv_i.str()] + h * (rho_R * R_star[fv_i.str()] - Delta * R[fv_i.str()]);

			/* Update T[i] */
			T[fv_i.str()] = E[fv_i.str()] + R[fv_i.str()];
		}

		/* update t using step size h */
		t += h;
	}
}

void CPFA_controller::RunCRMInstance(){
	ForwardEuler();
	CRMDone = true;
}

void CPFA_controller::UpdateProximityQueue(vector<Real> proxData){
	
	size_t closeProxCount = 0;
	size_t farProxCount = 0;
	
	/* If proxData is empty then we just push 0's into the proxmityQueue */
	/* Loop through the proximity data (proxData) and count # of bots in close/far proximity */
	for (auto it = proxData.begin(); it != proxData.end(); ++it){
		if (*it < LoopFunctions->closeProxRange){
			closeProxCount++;
		} else if (*it > LoopFunctions->closeProxRange && *it < LoopFunctions->farProxRange){
			farProxCount++;
		}else{
			LOG << "ERROR: invalid range value in UpdateProximityLists()" << endl;
		}
	}

	/* Update the proximity queue */
	proximityQueue.push(make_pair(closeProxCount, farProxCount));
	if (proximityQueue.size() > LoopFunctions->obsvWindowLength){
		proximityQueueFull = true;
		proximityQueue.pop();
	}
}

vector<char> CPFA_controller::BuildBFV(){
	
	/* Build the BFV */
	vector<char> bfv;
	Real c_avg = 0;	// close proximity presence average
	Real f_avg = 0;	// far proximity presence average
	queue<pair<Real,Real>> proxQcpy = proximityQueue;	// a copy of the proximity queue

	/* Loop through the copy proximity queue */
	while (!proxQcpy.empty()){
		if (proxQcpy.front().first > 0){	// H(x) = 1 if x > 0
			c_avg += 1/LoopFunctions->obsvWindowLength;  	// [sum of H(N_c(t))/T_s], where N_c(t) is the current number of bots in close proximity at time t (or control step technically) and T_s is the observation window length
		} else {	// H(x) = 0 if x <= 0
			c_avg += 0;										// 0/T_s = 0
		}
		if (proxQcpy.front().second > 0){	// H(x) = 1 if x > 0
			f_avg += 1/LoopFunctions->obsvWindowLength;		// [sum of H(N_f(t))/T_s], where N_f(t) is the current number of bots in far proximity at time t (or control step technically) and T_s is the observation window length
		} else {	// H(x) = 0 if x <= 0
			f_avg += 0;										// 0/T_s = 0
		}
		proxQcpy.pop();
	}

	/* The proximity features are set if this robot has had at least 1 neighbor in close or far proximity, respectively, for the majority of the past time window of length T_s (obsvWindowLength) */
	if (c_avg >= 0.5) bfv.push_back('1');	// if c_avg >= 0.5, then set the first bit to 1 (this is F_1 or Feature 1)
	else bfv.push_back('0');				// otherwise set it to 0

	if (f_avg >= 0.5) bfv.push_back('1');	// if f_avg >= 0.5, then set the second bit to 1 (this is F_2 or Feature 2)
	else bfv.push_back('0');				// otherwise set it to 0

	return bfv;
}

/// @brief Broadcast the BFV to all other robots in a single long message (as a byte array). Mode = 'b'.
/// @details The data-packet is formatted as follows: <message_type>,<sender_id>,<F_1>,<F_2>,...
void CPFA_controller::ShareBFV(){
	
	/* Build the BFV */
	BuildBFV();

	/* Create the "data-packet" to be broadcast */
	stringstream ss;
	ss << "b," << controllerID << ",";
	for (auto it = bfv.begin(); it != bfv.end(); ++it){
		ss << *it;
		if (it != bfv.end()-1) ss << ",";  // if it's not the last element, add a comma
	}

	/* Broadcast the data-packet */
	Broadcast(ss.str());
	shareBFVDone = true;
}

void CPFA_controller::ShareCellCount(){
	stringstream ss;
	ss << "c," << controllerID << "," << T.size();
	Broadcast(ss.str());
}

map<string, Real> CPFA_controller::ReceiveCellCount(){
	vector<tuple<string, Real, CRadians>> msgQueue = Receive();
	map<string, Real> cellCountMap;

	/* loop through the message queue */
	for(auto it = msgQueue.begin(); it != msgQueue.end(); ++it) {
		
		// process message (the message should always begin with the message type)
		stringstream ss(get<0>(*it));
		
		string msgType;
		getline(ss, msgType, ',');

		if (msgType == "p") LOG << "WARNING: Received ping type message in GetCellCount()..." << endl;
		else if (msgType == "b") LOG << "WARNING: Received fv broadcast type message in GetCellCount()..." << endl;
		else if (msgType == "c"){
					
			while (!ss.eof()){

				// see format details of ShareCellCount()
				string senderID, cellCount;
				getline(ss, senderID, ',');
				getline(ss, cellCount, ',');

				cellCountMap[senderID] = stoi(cellCount);
			}
		}
		else if (ss.eof()){
			if (controllerID == "fb00") LOG << "fb00 received EOF" << setw(setwidth) << right << "Time: " << LoopFunctions->getSimTimeInSeconds() << endl;
		} else {
			LOG << "runtime_error: " << msgType << endl;
			// throw runtime_error("Runtime Error: " + msgType + "is not a valid message type...\n");
		}
	}

	return cellCountMap;
}

void CPFA_controller::DiffusePhase1(){
	
	/* select a random target (robot) to diffuse cells with and send a portion of those cells*/
	SendCells(SelectRandomAgent(ReceiveCellCount()));
}

string CPFA_controller::SelectRandomAgent(map<string, Real> ccmap){
	random_device rd;
	mt19937 gen(rd());
	uniform_real_distribution<> dis(0,1);
	Real random_number = dis(gen);

	/* Calculate the total number of cells across all neighbors */
	uint16_t total_cc = 0;		// the total cell count
	for (auto i : ccmap){
		total_cc += i.second;
	}

	/* Select the agent based on the random number and cumulative distribution */
	Real cumulative_weight = 0;
	for (auto i : ccmap){
		cumulative_weight += static_cast<Real>(i.second/total_cc);
		if (random_number <= cumulative_weight){
			return i.first;
		}
	}

	return "-1"; // Return an error code if no agent is selected (this shouldn't be reached)
}

void CPFA_controller::SendCells(string targetID){

	map<string,Real> E_d;	// the number of effector cells to diffuse per sub-population
	map<string,Real> R_d;	// the number of regulatory cells to diffuse per sub-population
	/* Get number of cells to diffuse while subtracting it from the current cells */
	for (uint8_t i = 0; i < N; i++){
		/* Encode iterator i as a binary string representing a fv combination (e.g., 00,01,10,11) */
			boost::dynamic_bitset<uint8_t> b_i(l,i);
			stringstream fv_i;
			fv_i << b_i;
			
			/* Get 'd' portion of cells in the current sub-population to diffuse */
			E_d[fv_i.str()] = E[fv_i.str()] * d;
			R_d[fv_i.str()] = R[fv_i.str()] * d;

			/* Remove these cells from the current sub-population */
			E[fv_i.str()] -= E_d[fv_i.str()];
			R[fv_i.str()] -= R_d[fv_i.str()];

			/* Update T[i] */
			T[fv_i.str()] = E[fv_i.str()] + R[fv_i.str()];
	}

	stringstream ss_E, ss_R;

	/* Build the effector and regulatory sub-strings for the data-packet to be sent */
	for (uint8_t i = 0; i < N; i++){
		/* Encode iterator i as a binary string representing a fv combination (e.g., 00,01,10,11) */
			boost::dynamic_bitset<uint8_t> b_i(l,i);
			stringstream fv_i;
			fv_i << b_i;

			/* Add the number of effector cells to the data-packet */
			ss_E << E_d[fv_i.str()];
			if (i != N-1) ss_E << "|";  // if it's not the last element, add a bar (|) to delimit the data

			/* Add the number of regulatory cells to the data-packet */
			ss_R << R_d[fv_i.str()];
			if (i != N-1) ss_R << "|";  // if it's not the last element, add a bar (|) to delimit the data
	}

	/* Build the data-packet and send it */
	stringstream ss;
	ss << "s," << targetID << "," << controllerID << "," << ss_E.str() << "," << ss_R.str() << endl;
	Broadcast(ss.str());
}

queue<pair<string, map<string, pair<Real,Real>>>> CPFA_controller::ReceiveCells(){

	vector<tuple<string, Real, CRadians>> msgQueue = Receive();
	queue<pair<string, map<string, pair<Real,Real>>>> crQMap;		// Queue of neighbors that sent cells paired with the crMap (map of the cells they sent to the respective FVs)

	/* loop through the message queue */
	for(auto it = msgQueue.begin(); it != msgQueue.end(); ++it) {
		
		// process message (the message should always begin with the message type)
		stringstream ss(get<0>(*it));

		
		string msgType;
		getline(ss, msgType, ',');

		if (msgType == "p") LOG << "WARNING: Received p-type message in ReceiveCellsFT()..." << endl;
		else if (msgType == "b") LOG << "WARNING: Received b-type message in ReceiveCellsFT()..." << endl;
		else if (msgType == "c") LOG << "WARNING: Received c-type message in ReceiveCellsFT()..." << endl;
		else if (msgType == "s"){
			
					
			// see format details of SendCells()
			string targetID, senderID, E_str, R_str;

			while (!ss.eof()){
				getline(ss, targetID, ',');
				getline(ss, senderID, ',');
				getline(ss, E_str, ',');		// E_str = <E_1>|<E_2>|...|<E_N>
				getline(ss, R_str, ',');		// R_str = <R_1>|<R_2>|...|<R_N>
				if (!ss.eof()) LOG << "WARNING: incorrect format / extra data in ReceiveCellsFT()..." << endl;
			}

			if (targetID == controllerID){	// if this robot is the target
				
				// /* check that the sender ID isn't already in the map */ /* I'm not sure if this is a concern or how this would happen... */
				// if (crMap.find(senderID) != crMap.end()){
				// 	LOG << "WARNING: sender ID " << senderID << " already exists in crMap of " << controllerID << endl;
				// }
				
				map<string, pair<Real,Real>> crMap;	// map of the cells received from the sender paired with their respective FVs 

				/* parse cells received */
				stringstream ss_E(E_str);
				stringstream ss_R(R_str);
				string E_d_str, R_d_str;
				
				uint8_t subpop_counter = 0; // to count number of clonal types received
				uint8_t fv_it = 0;	// to iterate through the FVs
				/* update queue */
				while (!ss_E.eof()){
					getline(ss_E, E_d_str, '|');
					getline(ss_R, R_d_str, '|');

					/* encode fv as binary string */
					boost::dynamic_bitset<uint8_t> b_i(l,fv_it);
					stringstream fv_i;
					fv_i << b_i;

					/* update queue */
					crMap[fv_i.str()] = make_pair(stod(E_d_str), stod(R_d_str));


					/* increment */
					subpop_counter++;
					fv_it++;
				}

				/* make sure we received the correct number of subpopulations */
				if (subpop_counter != N){
					LOG << "WARNING: subpop_counter = " << subpop_counter << " in ReceiveCellsFT(), expected " << N << "." << endl;
				}

				/* add the sender ID and crMap to the queue */
				/* Wondering if I should be checking here if the sender ID is already in the queue. Maybe I can check later. */
				crQMap.push(make_pair(senderID, crMap));
			}

		}
		else if (ss.eof()){
			if (controllerID == "fb00") LOG << "fb00 received EOF" << setw(setwidth) << right << "Time: " << LoopFunctions->getSimTimeInSeconds() << endl;
		} else {
			LOG << "runtime_error: " << msgType << endl;
			// throw runtime_error("Runtime Error: " + msgType + "is not a valid message type...\n");
		}
	}

	return crQMap;
}

queue<string> CPFA_controller::GenerateResponsePackets(queue<pair<string, map<string, pair<Real,Real>>>> crQMap){
	
	queue<string> responseQueue;

	/* loop through the queue of neighbors that sent cells */
	while (!crQMap.empty()){
		
		/* get the sender ID and crMap */
		string senderID = crQMap.front().first;
		map<string, pair<Real,Real>> crMap = crQMap.front().second;

		/* update E, R, and T per the cells received */
		for (auto it = crMap.begin(); it != crMap.end(); ++it){
			E[it->first] += it->second.first;
			R[it->first] += it->second.second;
			T[it->first] += it->second.first + it->second.second;
		}

		/* add the response packet to the response queue */
		// responseQueue.push(ss.str());

		/* pop the front of the queue */
		crQMap.pop();
	}

	return responseQueue;
}


/*TODO: Consensus methods*/

map<string,bool> CPFA_controller::ComputeDecision(){

	/* Initialize summation variables */
	map<string, Real> sum_E;
	map<string, Real> sum_R;

	/* Initialize decision variable */
	map<string, bool> decision;

	for (uint8_t i = 0; i < N; i++){
		/* Encode iterator i as a binary string representing a fv combination (e.g., 00,01,10,11) */
		boost::dynamic_bitset<uint8_t> b_i(l,i);
		stringstream fv_i;
		fv_i << b_i;

		sum_E[fv_i.str()] = 0;
		sum_R[fv_i.str()] = 0;
	}

	/* Compute the summations */
	for (uint8_t j = 0; j < M; j++){
		/* Encode iterator j as a binary string representing a fv combination (e.g., 00,01,10,11) */
		boost::dynamic_bitset<uint8_t> b_j(l,j);
		stringstream fv_j;
		fv_j << b_j;

		for (uint8_t i = 0; i < N; i++){
			/* Encode iterator i as a binary string representing a fv combination (e.g., 00,01,10,11) */
			boost::dynamic_bitset<uint8_t> b_i(l,i);
			stringstream fv_i;
			fv_i << b_i;

			/* Compute the affinity (interaction) between T-cell clone i and APC j */
			Real theta_ij = exp(-HammingDistance(fv_i.str(), fv_j.str())/(c*l));	// calculate theta_ij

			sum_E[fv_j.str()] += theta_ij * E[fv_i.str()];
			sum_R[fv_j.str()] += theta_ij * R[fv_i.str()];
		}

		/* The FV is deemed faulty if E > R, otherwise, if R >= E then the FV is tolerated */
		if (sum_E[fv_j.str()] > sum_R[fv_j.str()]) decision[fv_j.str()] = true;
		else decision[fv_j.str()] = false;
	}

	return decision;
}

void CPFA_controller::ShareDecision(){

	/* Compute the decision */
	map<string,bool> decision = ComputeDecision();

	/* Create the "data-packet" to be broadcast */
	stringstream ss;
	ss << "d," << controllerID << ",";
	for (auto it = decision.begin(); it != decision.end(); ++it){
		auto nextIt = it;
		++nextIt;
		ss << it->first << ":" << it->second;	// format is <FV>:<decision> (e.g., 00:1,01:0,10:1,11:0)
		if (nextIt != decision.end()) ss << ",";  // if it's not the last element, add a comma
	}

	/* Broadcast the data-packet */
	Broadcast(ss.str());

	shareDecisionDone = true;
}

map<string, vector<bool>> CPFA_controller::ReceiveDecisions(){

	vector<tuple<string, Real, CRadians>> msgQueue = Receive();
	map<string, vector<bool>> decisionMap;		// map of the decisions received from neighbors paired with their respective FVs
	vector<string> idLog;						// log of the IDs of the neighbors that you have received decisions from

	/* loop through the message queue */
	for(auto it = msgQueue.begin(); it != msgQueue.end(); ++it) {
		
		// process message (the message should always begin with the message type)
		stringstream ss(get<0>(*it));
		
		string msgType;
		getline(ss, msgType, ',');

		if (msgType == "p") LOG << "WARNING: Received ping type message in GetCellCount()..." << endl;
		else if (msgType == "b") LOG << "WARNING: Received fv broadcast type message in GetCellCount()..." << endl;
		else if (msgType == "c"){ LOG << "WARNING: Received cell count type message in GetCellCount()..." << endl;}
		else if (msgType == "d"){

			// see format details of ShareCellCount()
			string senderID;
			getline(ss, senderID, ',');

			if (find(idLog.begin(), idLog.end(), senderID) != idLog.end()){	// if the sender ID is already in the log, skip this iteration
				continue;
			} else {
				idLog.push_back(senderID);
			}
			
			while (!ss.eof()){
				string decision;
				getline(ss, decision, ',');

				/* Parse the decision */
				stringstream ss_d(decision);
				string fv_str, d_str;
				while (!ss_d.eof()){
					getline(ss_d, fv_str, ':');
					getline(ss_d, d_str, ':');
					if (!ss_d.eof()) LOG << "WARNING: incorrect format / extra data in ReceiveDecision()..." << endl;

					/* Add the decision to the decision map */
					decisionMap[fv_str].push_back(stoi(d_str));
				}
			}
		}
		else if (ss.eof()){
			if (controllerID == "fb00") LOG << "fb00 received EOF" << setw(setwidth) << right << "Time: " << LoopFunctions->getSimTimeInSeconds() << endl;
		} else {
			LOG << "runtime_error: " << msgType << endl;
			// throw runtime_error("Runtime Error: " + msgType + "is not a valid message type...\n");
		}
	}

	return decisionMap;
}

void CPFA_controller::ComputeConsensus(){

	map<string, vector<bool>> decisionMap = ReceiveDecisions();
	map<string, bool> consensus;

	/* Go through the decision map and check the majority vote for each FV */
	for (auto it = decisionMap.begin(); it != decisionMap.end(); ++it){
		uint8_t trueCount = 0;
		uint8_t falseCount = 0;
		for (auto it2 = it->second.begin(); it2 != it->second.end(); ++it2){
			if (*it2) trueCount++;
			else falseCount++;
		}
		if (trueCount > falseCount) consensus[it->first] = true;
		else consensus[it->first] = false;
	}

	/* convert bfv vector to string */
	stringstream ss_bfv;
	for (auto it = bfv.begin(); it != bfv.end(); ++it){
		ss_bfv << *it;
	}

	/* loop through the consensus map */
	for (auto it = consensus.begin(); it != consensus.end(); ++it){
		/* check if this robots bfv is considered faulty */
		if (it->second){
			if (it->first == ss_bfv.str()){
				faultDetected = true;
				LOG << "Fault detected in" << controllerID << endl;
			}
		}
	}

	consensusDone = true;
}



// /**
//  * Broadcast the responses to all other robots in a single long message.
// */
// void CPFA_controller::BroadcastTargetedResponse(){
// 	stringstream  ss;
// 	ss << "r,";

// 	/**
// 	 * Format of message: <message_type>,<target_id>,<sender_id>,<estimate_X>,<estimate_Y>,<target_id>,<sender_id>,<estimate_X>,<estimate_Y>,...
// 	*/
// 	while(!responseQueue.empty()){
// 		if (responseQueue.size() == 1) { // don't add comma at the end
// 			ss << get<0>(responseQueue.front()) << "," << controllerID << "," << get<1>(responseQueue.front()) << "," << get<2>(responseQueue.front());
// 			responseQueue.pop();
// 		} else {
// 			ss << get<0>(responseQueue.front()) << "," << controllerID << "," << get<1>(responseQueue.front()) << "," << get<2>(responseQueue.front()) << ",";
// 			responseQueue.pop();
// 		}
// 	}
// 	/* Broadcast the message. */
// 	Broadcast(ss.str());
// 	// if (controllerID == "fb00") LOG << "fb00 responded: " << ss.str() << setw(setwidth) << right << "Time: " << LoopFunctions->getSimTimeInSeconds() << endl;
// }

// void CPFA_controller::ProcessVotes(){
// 	size_t trueCount = 0;	// coordinate was correct
// 	size_t falseCount = 0;	// coordinate was incorrect
// 	for(const auto& vote : voteQueue){
// 		if (vote) trueCount++;
// 		else falseCount++;
// 	}
// 	if (trueCount < falseCount) faultDetected = true;
// 	if (hasFault && !faultDetected){
// 		LOG << controllerID << ": false negative, voteQueue: ";
// 		for (const auto& vote : voteQueue){
// 			LOG << vote << ",";
// 		}
// 		LOG << endl;
// 	} else if (!hasFault && faultDetected){
// 		LOG << controllerID << ": false positive, voteQueue: ";
// 		for (const auto& vote : voteQueue){
// 			LOG << vote << ",";
// 		}
// 		LOG << endl;
// 	}
// 	voteQueue.clear();
// }

REGISTER_CONTROLLER(CPFA_controller, "CPFA_controller")











/***** ARCHIVED *****/
#pragma region LocalizationCheck() //section commented out
/**
 * Calculate a coordinate from a given bearing and range and compare it against the given coordinate.
 * @return true if the given coordinate matches the calculated coordinate.
*/
// CVector2 CPFA_controller::LocalizationCheck(Real range, CRadians bearing, string senderID){

// 	// must consider our orientation and adjust the bearing to compensate
// 	CRadians adjustedBearing = bearing + GetHeading();

// 	// construct a vector from the receiver to the sender (CVector2 will automatically convert this to cartesian coordinates)
// 	// range is given in cm from the RAB sensor and must be converted to meters for use with the coordinate system (source: https://opentechschool-brussels.github.io/AI-for-robots-and-swarms/ref_argos.html)
// 	CVector2 offset(range/100, adjustedBearing);

// 	// Calculate the origin of the signal (the estimated position of the sender)
// 	CVector2 origin = GetPosition() + offset;

// 	CVector2 realOffset = GetPosition() - LoopFunctions->getTargetLocation(senderID);
// 	if (realOffset != offset){
// 		// throw runtime_error("Real offset = " + to_string(realOffset.GetX()) + ", " + to_string(realOffset.GetY()) + ", calculated offset = " + to_string(offset.GetX()) + ", " + to_string(offset.GetY()));
// 	}

// 	// Check if the given coordinate is within the acceptable range of the calculated coordinate (the origin of the signal)
// 	if ((givenCoord - origin).Length() < 0.5){
// 		// LOG << "true coord detected" << endl;
// 		return true;
// 	} else {
// 		LOG << "Robot " << controllerID << " detected localization error in footbot "<< senderID << endl;
// 		LOG << "Given coord: " << givenCoord << ", Calculated coord: " << origin << ", Real coord: "<< LoopFunctions->getTargetLocation(senderID) << endl;
// 		// throw runtime_error("Robot " + controllerID + " detected localization error in footbot " + senderID);
// 		return false;
// 	}
// }
#pragma endregion
/***** ARCHIVED *****/

/***** ARCHIVED *****/
#pragma region ProcessMessages() old_version // section commented out
// void CPFA_controller::ProcessMessages(char mode){
// 	vector<tuple<string, Real, CRadians>> msgQueue = Receive();
// 	// LOG << LoopFunctions->getSimTimeInSeconds() << endl;

// 	for(auto it = msgQueue.begin(); it != msgQueue.end(); ++it) {

// 		// process message (the message should always begin with the message type)
// 		stringstream ss(get<0>(*it));
// 		// LOG << "converted: " << ss.str() << endl;
// 		string msgType;
// 		getline(ss, msgType, ',');

// 		// if (controllerID == "fb00") LOG << "fb00 received: " << ss.str() << setw(setwidth) << right << "Time: " << LoopFunctions->getSimTimeInSeconds() << endl;
// 		if (mode == 'b'){
// 			if (msgType == "r"){ 
// 				// LOG << "WARNING: received response type message during broadcast mode in ProcessMessages()" << endl;
// 			} else if (msgType == "b"){
// 				string senderID, x_str, y_str;
// 				getline(ss, senderID, ',');
// 				getline(ss, x_str, ',');
// 				getline(ss, y_str, ',');
// 				CVector2 senderEstPos(stod(x_str), stod(y_str)); 	// position given by the sender (possibly faulted)
// 				Real signalRange = get<1>(*it);						// range of signal provided by RAB Sensor
// 				CRadians signalBearing = get<2>(*it);				// bearing of signal provided by RAB Sensor
// 				// if (controllerID == "fb00") LOG << "fb00 received broadast: " << ss.str() << setw(setwidth) << right << "Time: " << LoopFunctions->getSimTimeInSeconds() << endl;
// 				responseQueue.push(make_pair(senderID, LocalizationCheck(senderEstPos, signalRange, signalBearing, senderID)));
// 				broadcastProcessed = true;
// 			}
// 			else if (ss.eof()){
// 				if (controllerID == "fb00") LOG << "fb00 received EOF" << setw(setwidth) << right << "Time: " << LoopFunctions->getSimTimeInSeconds() << endl;
// 			} else {
// 				LOG << "runtime_error: " << msgType << endl;
// 				// throw runtime_error("Runtime Error: " + msgType + "is not a valid message type...\n");
// 			}
// 		} else if (mode == 'r'){
// 			if (msgType == "b") LOG << "WARNING: received broadcast type message during response mode in ProcessMessages()" << endl;
// 			else if (msgType == "r"){
// 				// if (controllerID == "fb00") LOG << "fb00 received response" << setw(setwidth) << right << "Time: " << LoopFunctions->getSimTimeInSeconds() << endl;
// 				// else LOG << controllerID << " received response" << setw(setwidth) << right << "Time: " << LoopFunctions->getSimTimeInSeconds() << endl;
// 				while (!ss.eof()){
// 					string targetID, senderID, f_str;
// 					getline(ss, targetID, ',');
// 					getline(ss, senderID, ',');
// 					getline(ss, f_str, ',');
					
// 					// check if the message is for this bot and make sure the sender hasn't already voted
// 					if (controllerID == targetID && voterIDs.find(senderID) == voterIDs.end()){
						
// 						bool faulty = stoi(f_str);		// fault boolean
// 						voteQueue.push_back(faulty);	// store vote
// 						voterIDs.insert(senderID);		// store voter ID

// 						// if (controllerID == "fb00") LOG << "fb00 responseLogged set to True" << setw(setwidth) << right << "Time: " << LoopFunctions->getSimTimeInSeconds() << endl;
// 					}
// 				}
// 				responseProcessed = true;
// 			}
// 			else if (ss.eof()){
// 				if (controllerID == "fb00") LOG << "fb00 received EOF" << setw(setwidth) << right << "Time: " << LoopFunctions->getSimTimeInSeconds() << endl;
// 			} else {
// 				LOG << "runtime_error: " << msgType << endl;
// 				// throw runtime_error("Runtime Error: " + msgType + "is not a valid message type...\n");
// 			}
// 		} else {
// 			throw runtime_error("Unknown mode for ProcessMessages() encountered...");
// 		}
// 	}
// }
#pragma endregion
/***** ARCHIVED *****/