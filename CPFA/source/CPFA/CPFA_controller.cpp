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
	RFdetectionAcc(0.0)
{
}

void CPFA_controller::Init(argos::TConfigurationNode &node) {
	compassSensor   = GetSensor<argos::CCI_PositioningSensor>("positioning");
	wheelActuator   = GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering");
	proximitySensor = GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity");
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
    m_pcLEDs   = GetActuator<CCI_LEDsActuator>("leds");
    controllerID= GetId();//qilu 07/26/2016
	m_pcLEDs->SetAllColors(CColor::GREEN);
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

	// Add line so we can draw the trail

	CVector3 position3d(GetPosition().GetX(), GetPosition().GetY(), 0.00);
	CVector3 target3d(previous_position.GetX(), previous_position.GetY(), 0.00);
	CRay3 targetRay(target3d, position3d);
	myTrail.push_back(targetRay);
	LoopFunctions->TargetRayList.push_back(targetRay);
	LoopFunctions->TargetRayColorList.push_back(TrailColor);

	previous_position = GetPosition();

	//UpdateTargetRayList();
	CPFA();
	Move();
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

bool CPFA_controller::IsInTheNest() {
    
	return ((GetPosition() - LoopFunctions->NestPosition).SquareLength()
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

	// Are we there yet? (To the nest, that is.)
	if(IsInTheNest()) {
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
                
    }
	// Take a small step towards the nest so we don't overshoot by too much is we miss it
    else 
    {
        if(IsAtTarget())
        {
        //argos::LOG<<"heading to true in returning"<<endl;
        //SetIsHeadingToNest(false); // Turn off error for this
        //SetTarget(LoopFunctions->NestPosition);
        //randomly search for the nest
        argos::Real USCV = LoopFunctions->UninformedSearchVariation.GetValue();
        argos::Real rand = RNG->Gaussian(USCV);

        argos::CRadians rotation(rand);
        argos::CRadians angle1(rotation);
        argos::CRadians angle2(GetHeading());
        argos::CRadians turn_angle(angle1 + angle2);
        argos::CVector2 turn_vector(SearchStepSize, turn_angle);
        SetIsHeadingToNest(false);
        SetTarget(turn_vector + GetPosition());
        }
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
			if((GetPosition() - LoopFunctions->FoodList[i].GetLocation()).SquareLength() < FoodDistanceTolerance ) {
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
	 * QUESTION: Should a resource within a QZone count when surveying for resource density
	 * 
	 * HYPOTHESIS: This doesn't matter if the main resource located is fake, as a trail won't be
	 * created either way. However if the resource is real, it depends on whether we want to recruit
	 * bots to an area close to a QZone. It would affect how long the trail stays there recruiting bots
	 * departing from the nest.
	*/

	for(size_t i = 0; i < LoopFunctions->FoodList.size(); i++) {
		distance = GetPosition() - LoopFunctions->FoodList[i].GetLocation();	// modified ** Ryan Luna 11/11/22

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
 
	/* Set the fidelity position to the robot's current position. */
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
		/* Get position values required to construct a new ray */
		argos::CVector2 t(GetTarget());
		argos::CVector2 p(GetPosition());
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

REGISTER_CONTROLLER(CPFA_controller, "CPFA_controller")
