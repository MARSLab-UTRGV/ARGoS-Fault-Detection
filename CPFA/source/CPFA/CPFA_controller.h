#ifndef CPFA_CONTROLLER_H
#define CPFA_CONTROLLER_H

#include <source/Base/BaseController.h>
#include <source/Base/Pheromone.h>
#include <source/CPFA/CPFA_loop_functions.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>

// Ryan Luna 12/28/22
#include <source/Base/QuarantineZone.h>
#include <source/Base/Food.h>

#include <unordered_set>
#include <queue>

using namespace std;
using namespace argos;

static unsigned int num_targets_collected = 0;

class CPFA_loop_functions;

class CPFA_controller : public BaseController {

	public:

		CPFA_controller();

		// CCI_Controller inheritence functions
		void Init(argos::TConfigurationNode &node);
		void ControlStep();
		void Reset();

		bool IsHoldingFood();
		bool IsHoldingFakeFood();	// Ryan Luna 11/12/22
		bool IsUsingSiteFidelity();
		bool ThinksIsInTheNest();

		Real FoodDistanceTolerance;

		void SetLoopFunctions(CPFA_loop_functions* lf);
  
		size_t     GetSearchingTime();//qilu 09/26/2016
		size_t      GetTravelingTime();//qilu 09/26/2016
		string      GetStatus();//qilu 09/26/2016
		size_t      startTime;//qilu 09/26/2016

		/* quarantine zone functions */		// Ryan Luna 12/28/22

		void ClearZoneList();
		void ClearLocalFoodList();
		void AddZone(QZone newZone);
		void AddLocalFood(Food newFood);
		void RemoveZone(QZone Z);
		void RemoveLocalFood(Food F);
		bool TargetInQZone(CVector2 target);

		/* fault injection */

		void InjectFault(size_t faultCode);
		bool HasFault();
		bool ActuallyIsInTheNest();

		/* fault detection */

		void Ping();
		void GetRABData(char mode);
		void GetFVData(char mode);
		void ShareBFV();
		void BroadcastTargetedResponse();
		void ClearRABData();

		bool broadcastProcessed = false;
		bool responseProcessed = false;

	private:

		/* quarantine zone variables */		// Ryan Luna 12/28/22
		vector<QZone>	QZoneList;
		vector<Food>	LocalFoodList;

		Food FoodBeingHeld;		// Ryan Luna 1/24/23

  		string 			controllerID;//qilu 07/26/2016

		CPFA_loop_functions* LoopFunctions;
		argos::CRandom::CRNG* RNG;

		/* pheromone trail variables */
		std::vector<argos::CVector2> TrailToShare;
		std::vector<argos::CVector2> TrailToFollow;
		std::vector<argos::CRay3>    MyTrail;

		/* robot position variables */
		argos::CVector2 SiteFidelityPosition;
  		bool			 updateFidelity; //qilu 09/07/2016
  
		vector<CRay3> myTrail;
		CColor        TrailColor;

		bool isInformed;
		bool isHoldingFood;
		bool isHoldingFakeFood;		// Ryan Luna 11/12/22
		bool isUsingSiteFidelity;
		bool isGivingUpSearch;
		bool QZoneStrategy;		// to turn ON/OFF Quarantine Zones
  
		size_t ResourceDensity;
		size_t MaxTrailSize;
		size_t SearchTime;//for informed search
		size_t BadFoodCount;	// Ryan Luna 01/30/23
		size_t BadFoodLimit;	// Ryan Luna 01/30/23
		QZone* CurrentZone;		// Ryan Luna 01/30/23
		bool UseQZones;			// Ryan Luna 02/05/23
		size_t MergeMode;
  
		size_t           searchingTime; //qilu 09/26
		size_t           travelingTime;//qilu 09/26

		Real	FFdetectionAcc;
		Real    RFdetectionAcc;
        
  
		/* iAnt CPFA state variable */
		enum CPFA_state {
			DEPARTING = 0,
			SEARCHING = 1,
			RETURNING = 2,
			SURVEYING = 3
		} CPFA_state;

		/* iAnt CPFA state functions */
		void CPFA();
		void Departing();
		void Searching();
		void Returning();
		void Surveying();

		/* CPFA helper functions */
		void SetRandomSearchLocation();
		void SetHoldingFood();
		void SetLocalResourceDensity();
		void SetFidelityList(argos::CVector2 newFidelity);
		void SetFidelityList();
		bool SetTargetPheromone();

		argos::Real GetExponentialDecay(argos::Real value, argos::Real time, argos::Real lambda);
		argos::Real GetBound(argos::Real value, argos::Real min, argos::Real max);
		argos::Real GetPoissonCDF(argos::Real k, argos::Real lambda);

		void UpdateTargetRayList();
  
		CVector2 previous_position;

		string results_path;
		string results_full_path;
		bool isUsingPheromone;

		bool faultInjected;
		vector<CVector2> estimationQueue;				// for storing estimated poisions of self from other robots
		unordered_set<string> voterIDs;		// for storing IDs of those who voted (can't vote twice)
		void ProcessVotes();
		void UpdateProximityQueue(vector<Real> sRangeList);
		queue<pair<Real,Real>> proximityQueue;
		vector<uint8_t> BuildBFV();
		vector<uint8_t> bfv;	// Binary Feature Vector (one bit per feature)
		bool faultDetected;
		bool faultLogged;
		queue<pair<string, bool>> responseQueue;
		// bool broadcastLogged = false;
		float lastBroadcastTime;

		int setwidth = 3;


		unsigned int survey_count;
		/* Pointer to the LEDs actuator */
        CCI_LEDsActuator* m_pcLEDs;
};

#endif /* CPFA_CONTROLLER_H */
