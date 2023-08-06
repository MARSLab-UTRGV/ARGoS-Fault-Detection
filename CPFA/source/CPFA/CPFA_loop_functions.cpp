#include "CPFA_loop_functions.h"

CPFA_loop_functions::CPFA_loop_functions() :
	RNG(argos::CRandom::CreateRNG("argos")),
	SimTime(0),
    MaxSimTime(0),	//qilu 02/05/2021
	CollisionTime(0), 
	lastNumCollectedFood(0),
	currNumCollectedFood(0),
	TotalFoodCollected(0),		// Ryan Luna 11/17/22
	RealFoodCollected(0),		// Ryan Luna 11/17/22
	FakeFoodCollected(0),		// Ryan Luna 11/17/22
	ResourceDensityDelay(0),
	RandomSeed(GetSimulator().GetRandomSeed()),
	SimCounter(0),
	MaxSimCounter(1),
	VariableFoodPlacement(0),
	OutputData(0),
	DrawDensityRate(4),
	DrawIDs(1),
	DrawTrails(1),
	DrawTargetRays(1),
	FoodDistribution(2),
	FakeFoodDistribution(2),
	NumRealFood(256),			// name modified ** Ryan Luna 11/12/22
	NumFakeFood(0),			// Ryan Luna 11/12/22
	PowerlawFoodUnitCount(256),
	NumberOfClusters(4),
	ClusterWidthX(8),
	ClusterWidthY(8),
	PowerRank(4),
	ProbabilityOfSwitchingToSearching(0.0),
	ProbabilityOfReturningToNest(0.0),
	UninformedSearchVariation(0.0),
	RateOfInformedSearchDecay(0.0),
	RateOfSiteFidelity(0.0),
	RateOfLayingPheromone(0.0),
	RateOfPheromoneDecay(0.0),
	FoodRadius(0.05),
	FoodRadiusSquared(0.0025),
	NestRadius(0.12),
	NestRadiusSquared(0.0625),
	NestElevation(0.01),
	// We are looking at a 4 by 4 square (3 targets + 2*1/2 target gaps)
	SearchRadiusSquared((4.0 * FoodRadius) * (4.0 * FoodRadius)),
	SearchRadius(4.0*FoodRadius),
	NumDistributedRealFood(0),	// name modified ** Ryan Luna 11/12/22
	NumDistributedFakeFood(0),	// Ryan Luna 11/12/22
	TotalDistributedFood(0),	// name modified ** Ryan Luna 11/12/22
	score(0),
	PrintFinalScore(0),
	UseFakeFoodDoS(false),		// Ryan Luna 11/13/22
	FilenameHeader("\0"),			// Ryan Luna 12/09/22
	terminate(false),
	densify(true),
	numRealTrails(0),
	numFakeTrails(0),
	UseAltDistribution(false),
	AltClusterWidth(0),
	AltClusterLength(0),
	UseFakeFoodOnly(false),
	numFalsePositives(0),
	numQZones(0),
	faultInjected(false),
	FaultNumber(0),
	NumBotsToInject(0),
	InjectionTime(0),
	OffsetDistance(0),
	FaultHighlightRadius(0),
	UseFaultDetection(false),
	CommunicationDistance(2.0), 
	VoteCap(3),
	BroadcastFrequency(5),
	closeProxRange(15.0),	// in cm
	farProxRange(30.0),		// in cm
	obsvWindowLength(10)	// # of control steps to observe for proximity features
{}

void CPFA_loop_functions::Init(argos::TConfigurationNode &node) {	
 
	argos::CDegrees USV_InDegrees;
	argos::TConfigurationNode CPFA_node = argos::GetNode(node, "CPFA");

	argos::GetNodeAttribute(CPFA_node, "ProbabilityOfSwitchingToSearching", ProbabilityOfSwitchingToSearching);
	argos::GetNodeAttribute(CPFA_node, "ProbabilityOfReturningToNest",      ProbabilityOfReturningToNest);
	argos::GetNodeAttribute(CPFA_node, "UninformedSearchVariation",         USV_InDegrees);
	argos::GetNodeAttribute(CPFA_node, "RateOfInformedSearchDecay",         RateOfInformedSearchDecay);
	argos::GetNodeAttribute(CPFA_node, "RateOfSiteFidelity",                RateOfSiteFidelity);
	argos::GetNodeAttribute(CPFA_node, "RateOfLayingPheromone",             RateOfLayingPheromone);
	argos::GetNodeAttribute(CPFA_node, "RateOfPheromoneDecay",              RateOfPheromoneDecay);
	argos::GetNodeAttribute(CPFA_node, "PrintFinalScore",                   PrintFinalScore);

	UninformedSearchVariation = ToRadians(USV_InDegrees);
	argos::TConfigurationNode settings_node = argos::GetNode(node, "settings");
	
	argos::GetNodeAttribute(settings_node, "MaxSimTimeInSeconds", MaxSimTime);

	MaxSimTime *= GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();//qilu 02/05/2021 dyn2d error

	argos::GetNodeAttribute(settings_node, "MaxSimCounter", 				MaxSimCounter);
	argos::GetNodeAttribute(settings_node, "VariableFoodPlacement", 		VariableFoodPlacement);
	argos::GetNodeAttribute(settings_node, "OutputData", 					OutputData);
	argos::GetNodeAttribute(settings_node, "DrawIDs", 						DrawIDs);
	argos::GetNodeAttribute(settings_node, "DrawTrails", 					DrawTrails);
	argos::GetNodeAttribute(settings_node, "DrawTargetRays", 				DrawTargetRays);
	argos::GetNodeAttribute(settings_node, "FoodDistribution", 				FoodDistribution);
	argos::GetNodeAttribute(settings_node, "UseAltDistribution", 			UseAltDistribution);
	argos::GetNodeAttribute(settings_node, "AltClusterWidth", 				AltClusterWidth);
	argos::GetNodeAttribute(settings_node, "AltClusterLength", 				AltClusterLength);
	argos::GetNodeAttribute(settings_node, "UseFakeFoodOnly", 				UseFakeFoodOnly);
	argos::GetNodeAttribute(settings_node, "FakeFoodDistribution", 			FakeFoodDistribution);
	argos::GetNodeAttribute(settings_node, "NumRealFood", 					NumRealFood);					// name modified ** Ryan Luna 11/13/22
	argos::GetNodeAttribute(settings_node, "NumFakeFood", 					NumFakeFood);					// Ryan Luna 11/12/22
	argos::GetNodeAttribute(settings_node, "PowerlawFoodUnitCount", 		PowerlawFoodUnitCount);
	argos::GetNodeAttribute(settings_node, "PowerlawFakeFoodUnitCount", 	PowerlawFakeFoodUnitCount);		// Ryan Luna 11/12/22
	argos::GetNodeAttribute(settings_node, "NumberOfClusters", 				NumberOfClusters);
	argos::GetNodeAttribute(settings_node, "ClusterWidthX", 				ClusterWidthX);
	argos::GetNodeAttribute(settings_node, "ClusterWidthY", 				ClusterWidthY);
	argos::GetNodeAttribute(settings_node, "NumFakeClusters", 				NumFakeClusters);				// Ryan Luna 11/12/22
	argos::GetNodeAttribute(settings_node, "FakeClusterWidthX", 			FakeClusterWidthX);				// Ryan Luna 11/12/22
	argos::GetNodeAttribute(settings_node, "FakeClusterWidthY", 			FakeClusterWidthY);				// Ryan Luna 11/12/22
	argos::GetNodeAttribute(settings_node, "FoodRadius", 					FoodRadius);
    argos::GetNodeAttribute(settings_node, "NestRadius", 					NestRadius);
	argos::GetNodeAttribute(settings_node, "NestElevation", 				NestElevation);
    argos::GetNodeAttribute(settings_node, "NestPosition", 					NestPosition);
	argos::GetNodeAttribute(settings_node, "UseFakeFoodDoS",				UseFakeFoodDoS);				// Ryan Luna 11/13/22
	argos::GetNodeAttribute(settings_node, "FilenameHeader",				FilenameHeader);				// Ryan Luna 12/06/22
    argos::GetNodeAttribute(settings_node, "Densify", 						densify);						// Ryan Luna 02/08/22
	argos::GetNodeAttribute(settings_node, "FaultNumber",					FaultNumber);
	argos::GetNodeAttribute(settings_node, "OffsetDistance",				OffsetDistance);
	argos::GetNodeAttribute(settings_node, "NumBotsToInject",				NumBotsToInject);
	argos::GetNodeAttribute(settings_node, "InjectionTime",					InjectionTime);
	argos::GetNodeAttribute(settings_node, "FaultHighlightRadius",			FaultHighlightRadius);
	argos::GetNodeAttribute(settings_node, "UseFaultDetection",				UseFaultDetection);
	argos::GetNodeAttribute(settings_node, "CommunicationDistance",			CommunicationDistance);
	argos::GetNodeAttribute(settings_node, "VoteCap",						VoteCap);

	FoodRadiusSquared = FoodRadius*FoodRadius;

    //Number of distributed foods ** modified ** Ryan Luna 11/13/22
    if (FoodDistribution == 1){
		if (UseFakeFoodDoS){
			NumDistributedRealFood = ClusterWidthX*ClusterWidthY*NumberOfClusters;
			NumDistributedFakeFood = FakeClusterWidthX*FakeClusterWidthY*NumFakeClusters;
        	TotalDistributedFood = NumDistributedFakeFood+NumDistributedRealFood;
		} else {
			NumDistributedFakeFood = 0;
			NumDistributedRealFood = ClusterWidthX*ClusterWidthY*NumberOfClusters;
			TotalDistributedFood = NumDistributedRealFood;
		}
    } else {
		if (UseFakeFoodDoS){
        	NumDistributedRealFood = NumRealFood;
			NumDistributedFakeFood = NumFakeFood;
			TotalDistributedFood = NumRealFood+NumFakeFood;
		} else {
			NumDistributedRealFood = NumRealFood;
			NumDistributedFakeFood = 0;
			TotalDistributedFood = NumDistributedRealFood;
		}
    }
    

	// calculate the forage range and compensate for the robot's radius of 0.085m
	argos::CVector3 ArenaSize = GetSpace().GetArenaSize();
	argos::Real rangeX = (ArenaSize.GetX() / 2.0) - 0.085 - 0.1; // ryan luna 12/08/22 ** take away 0.1 so robots avoid getting to close to the wall
	argos::Real rangeY = (ArenaSize.GetY() / 2.0) - 0.085 - 0.1;
	ForageRangeX.Set(-rangeX, rangeX);
	ForageRangeY.Set(-rangeY, rangeY);

	ArenaWidth = ArenaSize[0];
	
	if(abs(NestPosition.GetX()) < -1){ //quad arena
		NestRadius *= sqrt(1 + log(ArenaWidth)/log(2));
	}else{
		NestRadius *= sqrt(log(ArenaWidth)/log(2));
	}
	argos::LOG<<"NestRadius="<<NestRadius<<endl;

	MainNest.SetLocation(NestPosition);	// Ryan Luna 1/24/23

	// Send a pointer to this loop functions object to each controller.
	argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
	argos::CSpace::TMapPerType::iterator it;
    
    Num_robots = footbots.size();
    argos::LOG<<"Number of robots="<<Num_robots<<endl;

	for(it = footbots.begin(); it != footbots.end(); it++) {
		argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
		BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
		CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
		c2.SetLoopFunctions(this);
	}
     
   	NestRadiusSquared = NestRadius*NestRadius;
	
    SetFoodDistribution();
  
	ForageList.clear(); 
	last_time_in_minutes=0;
}


void CPFA_loop_functions::Reset() {
	if(VariableFoodPlacement == 0) {
			RNG->Reset();
	}

    GetSpace().Reset();
    GetSpace().GetFloorEntity().Reset();
    MaxSimCounter = SimCounter;
    SimCounter = 0;
    score = 0;
   
    FoodList.clear();
    CollectedFoodList.clear();	
	PheromoneList.clear();
	FidelityList.clear();
    TargetRayList.clear();

	RealFoodCollected = 0;
	FakeFoodCollected = 0;
	TotalFoodCollected = 0;
    
    SetFoodDistribution();
    
    argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    argos::CSpace::TMapPerType::iterator it;
   
    for(it = footbots.begin(); it != footbots.end(); it++) {
        argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
        BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
        CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
        MoveEntity(footBot.GetEmbodiedEntity(), c2.GetStartPosition(), argos::CQuaternion(), false);
    	c2.Reset();
    }
}

void CPFA_loop_functions::PreStep() {
    SimTime++;
    curr_time_in_minutes = getSimTimeInSeconds()/60.0;
    if(curr_time_in_minutes - last_time_in_minutes==1){
        ForageList.push_back(currNumCollectedFood - lastNumCollectedFood);
        lastNumCollectedFood = currNumCollectedFood;
        last_time_in_minutes++;
    }

	UpdatePheromoneList();

	if (!faultInjected){
		FaultInjection();
	}

	// Ryan Luna 11/10/22
	if(GetSpace().GetSimulationClock() > ResourceDensityDelay) {
    	for(size_t i = 0; i < FoodList.size(); i++) {
			if (FoodList[i].GetType() == Food::REAL){
				FoodList[i].SetColor(CColor::BLACK);
			} else {
				FoodList[i].SetColor(CColor::PURPLE);
			}
            
        }
	}
 
    if(FoodList.size() == 0) {
		FidelityList.clear();
		PheromoneList.clear();
        TargetRayList.clear();
    }
}

void CPFA_loop_functions::PostStep() {
	// do fault detection post step
	FaultDetection();
}

void CPFA_loop_functions::Terminate(){
	terminate = true;
	cout << "Terminating program" << endl;
}

bool CPFA_loop_functions::IsExperimentFinished() {
	bool isFinished = false;

	if(FoodList.size() == 0 || GetSpace().GetSimulationClock() >= MaxSimTime) {
		isFinished = true;
		// PostExperiment();
	}

	if (terminate) {isFinished = true;}

    //set to collected 88% food and then stop
    // if(score >= NumDistributedRealFood){
	// 	isFinished = true;
	// }

	// if(isFinished == true && MaxSimCounter > 1) {
	// 	size_t newSimCounter = SimCounter + 1;
	// 	size_t newMaxSimCounter = MaxSimCounter - 1;
    //     argos::LOG<< "time out..."<<endl; 
	// 	PostExperiment();
	// 	// Reset();

	// 	SimCounter    = newSimCounter;
	// 	MaxSimCounter = newMaxSimCounter;
	// 	isFinished    = false;
	// }

	return isFinished;
}

void CPFA_loop_functions::PostExperiment() {
	  
	printf("%f, %f, %lu\n", score, getSimTimeInSeconds(), RandomSeed);
       
                  
    if (PrintFinalScore == 1) {
        string type="";
        if (FoodDistribution == 0) type = "random";
        else if (FoodDistribution == 1) type = "cluster";
        else type = "powerlaw";
            
        ostringstream num_tag;
        num_tag << NumRealFood; 
              
        ostringstream num_robots;
        num_robots <<  Num_robots;
   
        ostringstream arena_width;
        arena_width << ArenaWidth;
        
        ostringstream quardArena;
        if(abs(NestPosition.GetX())>=1){ //the central nest is not in the center, this is a quard arena
             quardArena << 1;
        }
         else{
             quardArena << 0;
        }
        
		// Now using FilenameHeader defined through the XML ** Ryan Luna 12/09/22
        // header = "./results/"+ type+"_CPFA_r"+num_robots.str()+"_tag"+num_tag.str()+"_"+arena_width.str()+"by"+arena_width.str()+"_quard_arena_" + quardArena.str() +"_";

        unsigned int ticks_per_second = GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();//qilu 02/06/2021
        
        argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
         
        for(argos::CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++) {
            argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
            BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
            CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
            CollisionTime += c2.GetCollisionTime();
        }
             
        // ofstream dataOutput( (FilenameHeader+ "iAntTagData.txt").c_str(), ios::app);
        // // output to file
        // if(dataOutput.tellp() == 0) {
        //     dataOutput << "tags_collected, collisions_in_seconds, time_in_minutes, random_seed\n";//qilu 08/18
        // }
    
        // dataOutput << Score() << ", "<<CollisionTime/(2*ticks_per_second)<< ", "<< curr_time_in_minutes << ", " << RandomSeed << endl;
        // dataOutput.close();
    
        // ofstream forageDataOutput((FilenameHeader+"ForageData.txt").c_str(), ios::app);
        // if(ForageList.size()!=0) forageDataOutput<<"Forage: "<< ForageList[0];
        // for(size_t i=1; i< ForageList.size(); i++) forageDataOutput<<", "<<ForageList[i];
        // forageDataOutput<<"\n";
        // forageDataOutput.close();

		// Write to file ** Ryan Luna 11/17/22
		ofstream DoSDataOutput((FilenameHeader+"DoSData.txt").c_str(), ios::app);
		if (DoSDataOutput.tellp() == 0){

			DoSDataOutput 	<< "Simulation Time (seconds), Total Food Collected, Total Food Collection Rate (per second), " 
							<< "Real Food Collected, Real Food Collection Rate (per second), "
							<< "Fake Food Collected, Fake Food Collection Rate (per second), " 
							<< "Real Food Trails Created, Fake Food Trails Created, False Positives, QZones" << endl;
		}

		TotalFoodCollected = RealFoodCollected + FakeFoodCollected;

		DoSDataOutput 	<< getSimTimeInSeconds() << ',' << TotalFoodCollected << ',' << TotalFoodCollected/getSimTimeInSeconds() << ','
						<< RealFoodCollected << ',' << RealFoodCollected/getSimTimeInSeconds() << ','
						<< FakeFoodCollected << ',' << FakeFoodCollected/getSimTimeInSeconds() << ','
						<< numRealTrails << ',' << numFakeTrails << ',' << numFalsePositives << ',' << MainNest.GetZoneList().size() << endl;
      }

	ofstream TerminateCount ((FilenameHeader+"TerminatedCount.txt").c_str(), ios::app);
	if(terminate){
		TerminateCount	<< 0 << ", ";
	} else {
		TerminateCount	<< 1 << ", ";
	}
}


/**
 * Using the defined number of bots to inject, this function randomly selects a subset of bots to inject 
 * the fault into if the current simulation time is greater than or equal to the set injection time.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void CPFA_loop_functions::FaultInjection() {
	if (getSimTimeInSeconds() >=  InjectionTime) {

		LOG << "Fault Injection: Begin..." << endl;

		// Get all footbot entities
		CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");

		// Check robots exist
		if(footbots.size() < 1) return;

		// Random number generator
		std::random_device rd;
		std::mt19937 gen(rd());

		// Distribute numbers between 0 and <size of foot-bot map> - 1
		std::uniform_int_distribution<> dis(0, footbots.size() - 1);

		// Randomly select 'NumBotsToInject' many robots
		int x = NumBotsToInject;

		for(int i = 0; i < x; ++i) {
			// Randomly select a foot-bot
			auto it = std::next(std::begin(footbots), dis(gen));
			CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);

			// Inject fault
			BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
			CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
			LOG << "Injecting fault on foot-bot: " << footBot.GetId().c_str() << ", ";
			c2.InjectFault(FaultNumber);
		}

		faultInjected = true;
	}
}

/**
 * This function handles the fault detection mechanism. This is handled in the loop function code
 * to fascilitate synchronization of the fault detection process.
 * 
 * @param None
 *
 * @return None
 *
 * @throws None
*/
void CPFA_loop_functions::FaultDetection() {

	argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
	argos::CSpace::TMapPerType::iterator it;

	switch (CommunicationMode){
		case 0:{
			// Share BFVs(next mode)
			if (getSimTimeInSeconds() - lastBroadcastTime >= BroadcastFrequency){
				lastBroadcastTime = getSimTimeInSeconds();
				// broadcastDone = true;
				CommunicationMode = 1;
			}
			// ping every control loop unless it is time to share BFVs
			for(it = footbots.begin(); it != footbots.end(); it++) {
				argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
				BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
				CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
				c2.Ping();
				// c2.broadcastProcessed = false;
				// c2.responseProcessed = false;
			}
			break;
		}
		case 1:{
			CommunicationMode = 2;
			// process broadcasted messages
			LOG << "Time:" << fixed << setprecision(4) << getSimTimeInSeconds() << "\tAll Robots: Processing Messages: Begin..." << endl;
			for(it = footbots.begin(); it != footbots.end(); it++) {
				argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
				BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
				CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
				c2.GetRABData('b');
				if (!c2.broadcastProcessed){
					CommunicationMode = 1;
				}
			}
			if (CommunicationMode == 2) LOG << "Time:" << fixed << setprecision(4) << getSimTimeInSeconds() << "\tAll Robots: Processing Messages: Complete" << endl;
			else if (CommunicationMode == 1) LOG << "Time:" << fixed << setprecision(4) << getSimTimeInSeconds() << "\tAll Robots: Processing Messages: Ongoing..." << endl;
			break;
		}
			// response broadcast
			LOG << "Time:" << fixed << setprecision(4) << getSimTimeInSeconds() << "\tAll Robots: Response Broadcast: Begin..." << endl;
			for(it = footbots.begin(); it != footbots.end(); it++) {
				argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
				BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
				CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
				c2.BroadcastTargetedResponse();
			}
			CommunicationMode = 3;
			LOG << "Time:" << fixed << setprecision(4) << getSimTimeInSeconds() << "\tAll Robots: Response Broadcast: Done." << endl;
			break;
		}
		case 3:{
			CommunicationMode = 0;
			// process responses
			LOG << "Time:" << fixed << setprecision(4) << getSimTimeInSeconds() << "\tAll Robots: Response Processing: Begin..." << endl;
			for(it = footbots.begin(); it != footbots.end(); it++) {
				argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
				BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
				CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
				c2.GetRABData('r');
				if (!c2.responseProcessed){
					CommunicationMode = 3;
				}
			}
			if (CommunicationMode == 0) LOG << "Time:" << fixed << setprecision(4) << getSimTimeInSeconds() << "\tAll Robots: Response Processing: Complete" << endl;
			else if (CommunicationMode == 3) LOG << "Time:" << fixed << setprecision(4) << getSimTimeInSeconds() << "\tAll Robots: Response Processing: Ongoing..." << endl;
			break;
		}
		default:{
			LOG << "Fault Detection: Unknown Communication Mode: " << CommunicationMode << endl;
		}
	}

	if (getSimTimeInSeconds() >=15) throw std::runtime_error("Fault Detection: Timeout");
}

argos::CVector2 CPFA_loop_functions::getTargetLocation(string targetID){
	argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
	argos::CSpace::TMapPerType::iterator it;

	for(it = footbots.begin(); it != footbots.end(); it++) {
		argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
		BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
		CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
		if (footBot.GetId() == targetID){
			return c2.GetRealPosition();
		}
	}
	return argos::CVector2::ZERO;
}

argos::CColor CPFA_loop_functions::GetFloorColor(const argos::CVector2 &c_pos_on_floor) {
	return argos::CColor::WHITE;
}

void CPFA_loop_functions::UpdatePheromoneList() {
	// Return if this is not a tick that lands on a 0.5 second interval
	if ((int)(GetSpace().GetSimulationClock()) % ((int)(GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick()) / 2) != 0) return;
	
	std::vector<Pheromone> new_p_list; 

	argos::Real t = GetSpace().GetSimulationClock() / GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();

	for(size_t i = 0; i < PheromoneList.size(); i++) {

		PheromoneList[i].Update(t);
		if(PheromoneList[i].IsActive()) {
			new_p_list.push_back(PheromoneList[i]);
		}
	}

	PheromoneList = new_p_list;
	new_p_list.clear();
}

// modified to include FakeFoodDistribution ** Ryan Luna 11/13/22
void CPFA_loop_functions::SetFoodDistribution() {

	if (UseAltDistribution){
		AlternateFakeFoodDistribution();
	} else if (UseFakeFoodDoS){
		switch(FoodDistribution) {
			case 0:
				RandomFoodDistribution();
				break;
			case 1:
				ClusterFoodDistribution();
				break;
			case 2:
				PowerLawFoodDistribution();
				break;
			default:
				argos::LOGERR << "ERROR: Invalid food distribution in XML file.\n";
		}
		switch(FakeFoodDistribution) {
			case 0:
				RandomFakeFoodDistribution();
				break;
			case 1:
				ClusterFakeFoodDistribution();
				break;
			case 2:
				PowerLawFakeFoodDistribution();
				break;
			default:
				argos::LOGERR << "ERROR: Invalid food distribution in XML file.\n";
		}
	} else if (UseFakeFoodOnly) {
		LOG << "[WARNING] This simulation is ONLY using Fake Food [WARNING] Real Food is NOT being distributed [WARNING]" << endl;
		switch(FakeFoodDistribution) {
			case 0:
				RandomFakeFoodDistribution();
				break;
			case 1:
				ClusterFakeFoodDistribution();
				break;
			case 2:
				PowerLawFakeFoodDistribution();
				break;
			default:
				argos::LOGERR << "ERROR: Invalid food distribution in XML file.\n";
		}
	} else {
		switch(FoodDistribution) {
			case 0:
				RandomFoodDistribution();
				break;
			case 1:
				ClusterFoodDistribution();
				break;
			case 2:
				PowerLawFoodDistribution();
				break;
			default:
				argos::LOGERR << "ERROR: Invalid food distribution in XML file.\n";
		}
	}
}

void CPFA_loop_functions::AlternateFakeFoodDistribution(){
	
	Real foodOffset;
	size_t foodPlaced = 0;
	if (densify){
		foodOffset = 2.0 * FoodRadius;
	} else {
		foodOffset = 3.0 * FoodRadius;
	}
	argos::CVector3 ArenaSize = GetSpace().GetArenaSize();
	float wallbuffer = 0.5;
	size_t ClusterLength = AltClusterLength;
	size_t ClusterWidth = AltClusterWidth;

	// West Wall
	CVector2 WestClusterPosition;
	size_t WestClusterX = ClusterLength;
	size_t WestClusterY = ClusterWidth;
	WestClusterPosition.Set(-ArenaSize.GetX()/2 + wallbuffer, foodOffset/2 + (((WestClusterY/2)-1) * foodOffset));
	
	for(size_t i = 0; i < WestClusterY; i++) {
		for(size_t j = 0; j < WestClusterX; j++) {
			foodPlaced++;

			Food tmp(WestClusterPosition, Food::FoodType::FAKE);
			FoodList.push_back(tmp);

			WestClusterPosition.SetX(WestClusterPosition.GetX() + foodOffset);
		}

		WestClusterPosition.SetX(WestClusterPosition.GetX() - (WestClusterX * foodOffset));
		WestClusterPosition.SetY(WestClusterPosition.GetY() - foodOffset);
	}

	// East Wall
	CVector2 EastClusterPosition;
	size_t EastClusterX = ClusterLength;
	size_t EastClusterY = ClusterWidth;
	EastClusterPosition.Set(ArenaSize.GetX()/2 - wallbuffer, foodOffset/2 + (((EastClusterY/2)-1) * foodOffset));
	
	for(size_t i = 0; i < EastClusterY; i++) {
		for(size_t j = 0; j < EastClusterX; j++) {
			foodPlaced++;

			Food tmp(EastClusterPosition, Food::FoodType::FAKE);
			FoodList.push_back(tmp);

			EastClusterPosition.SetX(EastClusterPosition.GetX() - foodOffset);
		}

		EastClusterPosition.SetX(EastClusterPosition.GetX() + (EastClusterX * foodOffset));
		EastClusterPosition.SetY(EastClusterPosition.GetY() - foodOffset);
	}

	// North Wall
	CVector2 NorthClusterPosition;
	size_t NorthClusterX = ClusterWidth;
	size_t NorthClusterY = ClusterLength;
	NorthClusterPosition.Set(foodOffset/2 + (((NorthClusterX/2)-1) * foodOffset), ArenaSize.GetY()/2 - wallbuffer);
	
	for(size_t i = 0; i < NorthClusterY; i++) {
		for(size_t j = 0; j < NorthClusterX; j++) {
			foodPlaced++;

			Food tmp(NorthClusterPosition, Food::FoodType::FAKE);
			FoodList.push_back(tmp);

			NorthClusterPosition.SetX(NorthClusterPosition.GetX() - foodOffset);
		}

		NorthClusterPosition.SetX(NorthClusterPosition.GetX() + (NorthClusterX * foodOffset));
		NorthClusterPosition.SetY(NorthClusterPosition.GetY() - foodOffset);
	}

	// South Wall
	CVector2 SouthClusterPosition;
	size_t SouthClusterX = ClusterWidth;
	size_t SouthClusterY = ClusterLength;
	SouthClusterPosition.Set(foodOffset/2 + (((SouthClusterX/2)-1) * foodOffset), -ArenaSize.GetY()/2 + wallbuffer);
	
	for(size_t i = 0; i < SouthClusterY; i++) {
		for(size_t j = 0; j < SouthClusterX; j++) {
			foodPlaced++;

			Food tmp(SouthClusterPosition, Food::FoodType::FAKE);
			FoodList.push_back(tmp);

			SouthClusterPosition.SetX(SouthClusterPosition.GetX() - foodOffset);
		}

		SouthClusterPosition.SetX(SouthClusterPosition.GetX() + (SouthClusterX * foodOffset));
		SouthClusterPosition.SetY(SouthClusterPosition.GetY() + foodOffset);
	}

}

void CPFA_loop_functions::RandomFoodDistribution() {
	// FoodList.clear();
// 	FoodColoringList.clear();
	argos::CVector2 placementPosition;

	for(size_t i = 0; i < NumRealFood; i++) {
		placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

		while(IsOutOfBounds(placementPosition, 1, 1)) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
		}

		Food tmp(placementPosition, Food::FoodType::REAL);
		FoodList.push_back(tmp);							// Ryan Luna 11/10/22
	}
}

// Ryan Luna 11/13/22
void CPFA_loop_functions::RandomFakeFoodDistribution(){

	CVector2 placementPosition;

	// distribute fake food 
	for(size_t i = 0; i < NumFakeFood; i++){
		placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

		while(IsOutOfBounds(placementPosition, 1, 1) || IsCollidingWithFood(placementPosition)) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
		}

		Food tmp(placementPosition, Food::FoodType::FAKE);
		FoodList.push_back(tmp);
	}
}

 
void CPFA_loop_functions::ClusterFoodDistribution() {
    // FoodList.clear();
	argos::Real     foodOffset  = 3.0 * FoodRadius;
	size_t          foodToPlace = NumberOfClusters * ClusterWidthX * ClusterWidthY;
	size_t          foodPlaced = 0;
	argos::CVector2 placementPosition;

	NumRealFood = foodToPlace;

	for(size_t i = 0; i < NumberOfClusters; i++) {
		placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

		while(IsOutOfBounds(placementPosition, ClusterWidthY, ClusterWidthX)) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
		}

		for(size_t j = 0; j < ClusterWidthY; j++) {
			for(size_t k = 0; k < ClusterWidthX; k++) {
				foodPlaced++;

				Food tmp(placementPosition, Food::FoodType::REAL);	// Ryan Luna 11/10/22
				FoodList.push_back(tmp);							// Ryan Luna 11/10/22

				placementPosition.SetX(placementPosition.GetX() + foodOffset);
			}

			placementPosition.SetX(placementPosition.GetX() - (ClusterWidthX * foodOffset));
			placementPosition.SetY(placementPosition.GetY() + foodOffset);
		}
	}
}

// Ryan Luna 11/13/22
void CPFA_loop_functions::ClusterFakeFoodDistribution(){
	
	size_t			fakefoodToPlace = NumFakeClusters * FakeClusterWidthX * FakeClusterWidthY;
	size_t			fakefoodPlaced = 0;
	CVector2		placementPosition;
	argos::Real		foodOffset;

	if (densify){
		foodOffset  = 2.0 * FoodRadius;
	} else {
		foodOffset  = 3.0 * FoodRadius;
	}

	NumFakeFood = fakefoodToPlace;

	for(size_t i = 0; i < NumFakeClusters; i++) {
		placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

		while(IsOutOfBounds(placementPosition, FakeClusterWidthY, FakeClusterWidthX)) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
		}

		for(size_t j = 0; j < FakeClusterWidthY; j++) {
			for(size_t k = 0; k < FakeClusterWidthX; k++) {
				fakefoodPlaced++;

				Food tmp(placementPosition, Food::FoodType::FAKE);
				FoodList.push_back(tmp);

				placementPosition.SetX(placementPosition.GetX() + foodOffset);
			}

			placementPosition.SetX(placementPosition.GetX() - (FakeClusterWidthX * foodOffset));
			placementPosition.SetY(placementPosition.GetY() + foodOffset);
		}
	}
}

void CPFA_loop_functions::PowerLawFoodDistribution() {
 	// FoodList.clear();
	argos::Real foodOffset     = 3.0 * FoodRadius;
	size_t      foodPlaced     = 0;
	size_t      powerLawLength = 1;
	size_t      maxTrials      = 200;
	size_t      trialCount     = 0;

	std::vector<size_t> powerLawClusters;
	std::vector<size_t> clusterSides;
	argos::CVector2     placementPosition;

    //-----Wayne: Dertermine PowerRank and food per PowerRank group
    size_t priorPowerRank = 0;
    size_t power4 = 0;
    size_t FoodCount = 0;
    size_t diffFoodCount = 0;
    size_t singleClusterCount = 0;
    size_t otherClusterCount = 0;
    size_t modDiff = 0;

	// use local variable for power rank value not global ** Ryan Luna 11/13/22
	size_t localPowerRank = PowerRank;
    
    //Wayne: priorPowerRank is determined by what power of 4
    //plus a multiple of power4 increases the food count passed required count
    //this is how powerlaw works to divide up food into groups
    //the number of groups is the powerrank
    while (FoodCount < NumRealFood){
        priorPowerRank++;
        power4 = pow (4.0, priorPowerRank);
        FoodCount = power4 + priorPowerRank * power4;
    }
    
    //Wayne: Actual powerRank is prior + 1
    localPowerRank = priorPowerRank + 1;
    
    //Wayne: Equalizes out the amount of food in each group, with the 1 cluster group taking the
    //largest loss if not equal, when the powerrank is not a perfect fit with the amount of food.
    diffFoodCount = FoodCount - NumRealFood;
    modDiff = diffFoodCount % localPowerRank;
    
    if (NumRealFood % localPowerRank == 0){
        singleClusterCount = NumRealFood / localPowerRank;
        otherClusterCount = singleClusterCount;
    }
    else {
        otherClusterCount = NumRealFood / localPowerRank + 1;
        singleClusterCount = otherClusterCount - modDiff;
    }
    //-----Wayne: End of PowerRank and food per PowerRank group
    
	for(size_t i = 0; i < localPowerRank; i++) {
		powerLawClusters.push_back(powerLawLength * powerLawLength);
		powerLawLength *= 2;
	}

	for(size_t i = 0; i < localPowerRank; i++) {
		powerLawLength /= 2;
		clusterSides.push_back(powerLawLength);
	}
    /*Wayne: Modified to break from loops if food count reached.
     Provides support for unequal clusters and odd food numbers.
     Necessary for DustUp and Jumble Distribution changes. */
    
	for(size_t h = 0; h < powerLawClusters.size(); h++) {
		for(size_t i = 0; i < powerLawClusters[h]; i++) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

			while(IsOutOfBounds(placementPosition, clusterSides[h], clusterSides[h])) {
				trialCount++;
				placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

				if(trialCount > maxTrials) {
					argos::LOGERR << "PowerLawDistribution(): Max trials exceeded!\n";
					break;
				}
			}

            trialCount = 0;
			for(size_t j = 0; j < clusterSides[h]; j++) {
				for(size_t k = 0; k < clusterSides[h]; k++) {
					foodPlaced++;
					// FoodList.push_back(placementPosition);
					// FoodColoringList.push_back(argos::CColor::BLACK);

					Food tmp(placementPosition, Food::FoodType::REAL);
					FoodList.push_back(tmp);							// Ryan Luna 11/10/22
					placementPosition.SetX(placementPosition.GetX() + foodOffset);
                    if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
				}

				placementPosition.SetX(placementPosition.GetX() - (clusterSides[h] * foodOffset));
				placementPosition.SetY(placementPosition.GetY() + foodOffset);
                if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
			}
            if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
			}
		}
	NumRealFood = foodPlaced;
}

void CPFA_loop_functions::PowerLawFakeFoodDistribution() {

	// variable name modification 'L_' to denote a local function variable ** Ryan Luna 11/13/22

	argos::Real L_foodOffset     = 3.0 * FoodRadius;
	size_t      L_fakefoodPlaced     = 0;
	size_t      L_powerLawLength = 1;
	size_t      L_maxTrials      = 200;
	size_t      L_trialCount     = 0;

	std::vector<size_t> L_powerLawFakeClusters;
	std::vector<size_t> L_fakeClusterSides;
	argos::CVector2     L_placementPosition;

    //-----Wayne: Dertermine PowerRank and food per PowerRank group
    size_t L_priorPowerRank = 0;
    size_t L_power4 = 0;
    size_t L_FakeFoodCount = 0;
    size_t L_diffFakeFoodCount = 0;
    size_t L_singleFakeClusterCount = 0;
    size_t L_otherFakeClusterCount = 0;
    size_t L_modDiff = 0;

	size_t localPowerRank = PowerRank;
    
    //Wayne: L_priorPowerRank is determined by what power of 4
    //plus a multiple of L_power4 increases the food count passed required count
    //this is how powerlaw works to divide up food into groups
    //the number of groups is the powerrank
    while (L_FakeFoodCount < NumFakeFood){
        L_priorPowerRank++;
        L_power4 = pow (4.0, L_priorPowerRank);
        L_FakeFoodCount = L_power4 + L_priorPowerRank * L_power4;
    }
    
    //Wayne: Actual powerRank is prior + 1
    localPowerRank = L_priorPowerRank + 1;
    
    //Wayne: Equalizes out the amount of food in each group, with the 1 cluster group taking the
    //largest loss if not equal, when the powerrank is not a perfect fit with the amount of food.
    L_diffFakeFoodCount = L_FakeFoodCount - NumFakeFood;
    L_modDiff = L_diffFakeFoodCount % localPowerRank;
    
    if (NumFakeFood % localPowerRank == 0){
        L_singleFakeClusterCount = NumFakeFood / localPowerRank;
        L_otherFakeClusterCount = L_singleFakeClusterCount;
    }
    else {
        L_otherFakeClusterCount = NumFakeFood / localPowerRank + 1;
        L_singleFakeClusterCount = L_otherFakeClusterCount - L_modDiff;
    }
    //-----Wayne: End of PowerRank and food per PowerRank group
    
	for(size_t i = 0; i < localPowerRank; i++) {
		L_powerLawFakeClusters.push_back(L_powerLawLength * L_powerLawLength);
		L_powerLawLength *= 2;
	}

	for(size_t i = 0; i < localPowerRank; i++) {
		L_powerLawLength /= 2;
		L_fakeClusterSides.push_back(L_powerLawLength);
	}
    /*Wayne: Modified to break from loops if food count reached.
     Provides support for unequal clusters and odd food numbers.
     Necessary for DustUp and Jumble Distribution changes. */
    
	for(size_t h = 0; h < L_powerLawFakeClusters.size(); h++) {
		for(size_t i = 0; i < L_powerLawFakeClusters[h]; i++) {
			L_placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

			while(IsOutOfBounds(L_placementPosition, L_fakeClusterSides[h], L_fakeClusterSides[h])) {
				L_trialCount++;
				L_placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

				if(L_trialCount > L_maxTrials) {
					argos::LOGERR << "PowerLawDistribution(): Max trials exceeded!\n";
					break;
				}
			}

            L_trialCount = 0;
			for(size_t j = 0; j < L_fakeClusterSides[h]; j++) {
				for(size_t k = 0; k < L_fakeClusterSides[h]; k++) {
					L_fakefoodPlaced++;
					// FoodList.push_back(L_placementPosition);
					// FoodColoringList.push_back(argos::CColor::BLACK);

					Food tmp(L_placementPosition, Food::FoodType::FAKE);
					FoodList.push_back(tmp);							
					L_placementPosition.SetX(L_placementPosition.GetX() + L_foodOffset);
                    if (L_fakefoodPlaced == L_singleFakeClusterCount + h * L_otherFakeClusterCount) break;
				}

				L_placementPosition.SetX(L_placementPosition.GetX() - (L_fakeClusterSides[h] * L_foodOffset));
				L_placementPosition.SetY(L_placementPosition.GetY() + L_foodOffset);
                if (L_fakefoodPlaced == L_singleFakeClusterCount + h * L_otherFakeClusterCount) break;
			}
            if (L_fakefoodPlaced == L_singleFakeClusterCount + h * L_otherFakeClusterCount) break;
			}
		}
	NumFakeFood = L_fakefoodPlaced;
}

bool CPFA_loop_functions::IsOutOfBounds(argos::CVector2 p, size_t length, size_t width) {
	argos::CVector2 placementPosition = p;

	argos::Real foodOffset   = 3.0 * FoodRadius;
	argos::Real widthOffset  = 3.0 * FoodRadius * (argos::Real)width;
	argos::Real lengthOffset = 3.0 * FoodRadius * (argos::Real)length;

	argos::Real x_min = p.GetX() - FoodRadius;
	argos::Real x_max = p.GetX() + FoodRadius + widthOffset;

	argos::Real y_min = p.GetY() - FoodRadius;
	argos::Real y_max = p.GetY() + FoodRadius + lengthOffset;

	if((x_min < (ForageRangeX.GetMin() + FoodRadius))
			|| (x_max > (ForageRangeX.GetMax() - FoodRadius)) ||
			(y_min < (ForageRangeY.GetMin() + FoodRadius)) ||
			(y_max > (ForageRangeY.GetMax() - FoodRadius)))
	{
		return true;
	}

	for(size_t j = 0; j < length; j++) {
		for(size_t k = 0; k < width; k++) {
			if(IsCollidingWithFood(placementPosition)) return true;
			if(IsCollidingWithNest(placementPosition)) return true;
			placementPosition.SetX(placementPosition.GetX() + foodOffset);
		}

		placementPosition.SetX(placementPosition.GetX() - (width * foodOffset));
		placementPosition.SetY(placementPosition.GetY() + foodOffset);
	}

	return false;
}

  
bool CPFA_loop_functions::IsCollidingWithNest(argos::CVector2 p) {
	argos::Real nestRadiusPlusBuffer = NestRadius + FoodRadius;
	argos::Real NRPB_squared = nestRadiusPlusBuffer * nestRadiusPlusBuffer;

      return ( (p - NestPosition).SquareLength() < NRPB_squared) ;
}

bool CPFA_loop_functions::IsCollidingWithFood(argos::CVector2 p) {
	argos::Real foodRadiusPlusBuffer = 2.0 * FoodRadius;
	argos::Real FRPB_squared = foodRadiusPlusBuffer * foodRadiusPlusBuffer;

	for(size_t i = 0; i < FoodList.size(); i++) {
		if((p - FoodList[i].GetLocation()).SquareLength() < FRPB_squared) return true;
	}

	return false;
}

unsigned int CPFA_loop_functions::getNumberOfRobots() {
	return GetSpace().GetEntitiesByType("foot-bot").size();
}

double CPFA_loop_functions::getProbabilityOfSwitchingToSearching() {
	return ProbabilityOfSwitchingToSearching;
}

double CPFA_loop_functions::getProbabilityOfReturningToNest() {
	return ProbabilityOfReturningToNest;
}

// Value in Radians
double CPFA_loop_functions::getUninformedSearchVariation() {
	return UninformedSearchVariation.GetValue();
}

double CPFA_loop_functions::getRateOfInformedSearchDecay() {
	return RateOfInformedSearchDecay;
}

double CPFA_loop_functions::getRateOfSiteFidelity() {
	return RateOfSiteFidelity;
}

double CPFA_loop_functions::getRateOfLayingPheromone() {
	return RateOfLayingPheromone;
}

double CPFA_loop_functions::getRateOfPheromoneDecay() {
	return RateOfPheromoneDecay;
}

argos::Real CPFA_loop_functions::getSimTimeInSeconds() {
	int ticks_per_second = GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick(); //qilu 02/06/2021
	float sim_time = GetSpace().GetSimulationClock();
	return sim_time/ticks_per_second;
}

void CPFA_loop_functions::SetTrial(unsigned int v) {
}

void CPFA_loop_functions::setScore(double s) {
	score = s;
    
	// if (score >= NumDistributedRealFood) {
	// 	PostExperiment();
	// }
}

double CPFA_loop_functions::Score() {	
	return score;
}

// modified ** Ryan Luna 11/12/22
void CPFA_loop_functions::increaseNumDistributedFoodByOne(){
    NumDistributedRealFood++;
	TotalDistributedFood++;
}

void CPFA_loop_functions::ConfigureFromGenome(Real* g)
{
	// Assign genome generated by the GA to the appropriate internal variables.
	ProbabilityOfSwitchingToSearching = g[0];
	ProbabilityOfReturningToNest      = g[1];
	UninformedSearchVariation.SetValue(g[2]);
	RateOfInformedSearchDecay         = g[3];
	RateOfSiteFidelity                = g[4];
	RateOfLayingPheromone             = g[5];
	RateOfPheromoneDecay              = g[6];
}

argos::Real CPFA_loop_functions::getOffsetDistance(){
	return OffsetDistance;
}

REGISTER_LOOP_FUNCTIONS(CPFA_loop_functions, "CPFA_loop_functions")
