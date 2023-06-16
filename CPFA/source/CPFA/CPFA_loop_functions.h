#ifndef CPFA_LOOP_FUNCTIONS_H
#define CPFA_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <source/CPFA/CPFA_controller.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <source/Base/Food.h>	// Ryan Luna 11/10/22
#include <source/Base/Nest.h>	// Ryan Luna 1/24/23
#include <cmath>				// Ryan Luna 1/25/23

using namespace argos;
using namespace std;

static const size_t GENOME_SIZE = 7; // There are 7 parameters to evolve

class CPFA_loop_functions : public argos::CLoopFunctions
{

	friend class CPFA_controller;
	friend class CPFA_qt_user_functions;

	public:

		CPFA_loop_functions();
	   
		void Init(argos::TConfigurationNode &t_tree);
		void Reset();
		void PreStep();
		void PostStep();
		bool IsExperimentFinished();
		void PostExperiment();
		argos::CColor GetFloorColor(const argos::CVector2 &c_pos_on_floor);

		// GA Functions
		
		/* Configures the robot controller from the genome */
		void ConfigureFromGenome(Real* pf_genome);
		/* Calculates the performance of the robot in a trial */
		Real Score();
	
		/**
		 * Returns the current trial.
		 */
		UInt32 GetTrial() const;
	
		/**
		 * Sets the current trial.
		 * @param un_trial The trial number.
		 */
		void SetTrial(UInt32 un_trial);
	
		/* public helper functions */
		void UpdatePheromoneList();
		void SetFoodDistribution();

		argos::Real getSimTimeInSeconds();

		std::vector<argos::CColor>   TargetRayColorList;

		unsigned int getNumberOfRobots();
        void increaseNumDistributedFoodByOne();
		double getProbabilityOfSwitchingToSearching();
		double getProbabilityOfReturningToNest();
		double getUninformedSearchVariation();
		double getRateOfInformedSearchDecay();
		double getRateOfSiteFidelity();
		double getRateOfLayingPheromone();
		double getRateOfPheromoneDecay();


		void Terminate();

	protected:

		void setScore(double s);

		argos::CRandom::CRNG* RNG;
        size_t NumDistributedRealFood;		// modified name ** Ryan Luna 11/12/22
		size_t NumDistributedFakeFood;		// Ryan Luna 11/12/22
		size_t TotalDistributedFood;		// Ryan Luna 11/12/22
		size_t MaxSimTime;
		size_t ResourceDensityDelay;
		size_t RandomSeed;
		size_t SimCounter;
		size_t MaxSimCounter;
		size_t VariableFoodPlacement;
		size_t OutputData;
		size_t DrawDensityRate;
		size_t DrawIDs;
		size_t DrawTrails;
		size_t DrawTargetRays;
		size_t FoodDistribution;
		size_t FakeFoodDistribution;	// Ryan Luna 11/13/22
		size_t NumRealFood;			// modified name ** Ryan Luna 11/12/22
		size_t NumFakeFood;			// Ryan Luna 11/12/22
		size_t PowerlawFoodUnitCount;
		size_t PowerlawFakeFoodUnitCount;	// Ryan Luna 11/12/22
		size_t NumberOfClusters;
		size_t ClusterWidthX;
		size_t ClusterWidthY;
		size_t NumFakeClusters;			// Ryan Luna 11/12/22
		size_t FakeClusterWidthX;		// Ryan Luna 11/12/22
		size_t FakeClusterWidthY;		// Ryan Luna 11/12/22
		size_t PowerRank;
		size_t ArenaWidth;
		size_t SimTime; 
		Real curr_time_in_minutes; 
		Real last_time_in_minutes; 

		bool UseFakeFoodDoS;	// Ryan Luna 11/13/22
		bool UseAltDistribution;
		bool UseFakeFoodOnly;
		size_t AltClusterWidth;
		size_t AltClusterLength;

		size_t numRealTrails;
		size_t numFakeTrails;
		size_t numFalsePositives;
		size_t numQZones;

		/* Result Collection */
		string FilenameHeader;	// Ryan Luna 12/09/22
  
		/* CPFA variables */
		argos::Real ProbabilityOfSwitchingToSearching;
		argos::Real ProbabilityOfReturningToNest;
		argos::CRadians UninformedSearchVariation;
		argos::Real RateOfInformedSearchDecay;
		argos::Real RateOfSiteFidelity;
		argos::Real RateOfLayingPheromone;
		argos::Real RateOfPheromoneDecay;

		/* physical robot & world variables */
		argos::Real FoodRadius;
		argos::Real FoodRadiusSquared;
		argos::Real NestRadius;
		argos::Real NestRadiusSquared;
		argos::Real NestElevation;
		argos::Real SearchRadiusSquared;
		argos::Real SearchRadius;

		/* list variables for food & pheromones */
		std::vector<Food>				FoodList;				// Ryan Luna 11/10/22
		vector<Food> 					CollectedFoodList;		// Ryan Luna 11/10/22
        map<string, argos::CVector2> 	FidelityList; 
		std::vector<Pheromone>  	 	PheromoneList; 
		std::vector<argos::CRay3>    	TargetRayList;
		argos::CRange<argos::Real>   	ForageRangeX;
		argos::CRange<argos::Real>   	ForageRangeY;
  
                Real   CollisionTime;
                size_t currCollisionTime; 
                size_t lastCollisionTime; 
                size_t lastNumCollectedFood;
                size_t currNumCollectedFood;
                size_t Num_robots;

		size_t TotalFoodCollected;		// Ryan Luna 11/17/22
		size_t RealFoodCollected;		// Ryan Luna 11/17/22
		size_t FakeFoodCollected;		// Ryan Luna 11/17/22

		Nest MainNest;					// Ryan Luna 1/24/23
      
        vector<size_t>		ForageList;
		argos::CVector2 NestPosition;

		bool terminate;
		bool densify;


	private:

		/* private helper functions */
		void AlternateFakeFoodDistribution();
		void RandomFoodDistribution();
		void ClusterFoodDistribution();
		void PowerLawFoodDistribution();
		void RandomFakeFoodDistribution();		// Ryan Luna 11/13/22
		void ClusterFakeFoodDistribution();		// Ryan Luna 11/13/22	
		void PowerLawFakeFoodDistribution();	// Ryan Luna 11/13/22
        bool IsOutOfBounds(argos::CVector2 p, size_t length, size_t width);
		bool IsCollidingWithNest(argos::CVector2 p);
		bool IsCollidingWithFood(argos::CVector2 p);
		double score;
		int PrintFinalScore;
};

#endif /* CPFA_LOOP_FUNCTIONS_H */
