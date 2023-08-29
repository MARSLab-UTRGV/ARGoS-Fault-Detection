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
#include <bitset>
#include <boost/dynamic_bitset.hpp>

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

		/********************************************************************************/
		/*								Fault Detection									*/
		/********************************************************************************/
		/* fault injection */
		void InjectFault(size_t faultCode);
		bool HasFault();
		bool ActuallyIsInTheNest();

		/* fault detection */
		void Ping();
		void GetRABData();
		void GetFVData();
		void ShareBFV();
		void BroadcastTargetedResponse();
		void ClearRABData();
		/**
		 * ### Run an instance of the CRM 
		 * @brief This is just a wrapper to call the desired numerical integration method.
		*/
		void RunCRMInstance();
		/**
		 * ### Share the decision on whether or not a FV is abnormal.
		 * @brief This is a step in the consensus formation on whether or not a FV is abnormal.
		*/
		void ShareDecision();
		/**
		 * ### Compute the consensus on whether or not a FV is abnormal.
		 * @brief This is the final step in the consensus formation on whether or not a FV is abnormal.
		 * @brief A robot will run the consensus to decide whether it has exibited a fault depending on its FV.
		*/
		void ComputeConsensus();

		bool shareBFVDone = false;
		bool CRMDone = false;
		bool shareDecisionDone = false;
		bool consensusDone = false;
		bool proximityQueueFull = false;
		/********************************************************************************/

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

		int setwidth = 3;


		unsigned int survey_count;
		/* Pointer to the LEDs actuator */
        CCI_LEDsActuator* m_pcLEDs;

		/********************************************************************************/
		/*								Fault Detection									*/
		/********************************************************************************/

		bool faultInjected;
		vector<CVector2> estimationQueue;				// for storing estimated poisions of self from other robots
		unordered_set<string> voterIDs;					// for storing IDs of those who voted (can't vote twice)
		queue<pair<Real,Real>> proximityQueue;			// sliding window of proximity data		 pair<close_prox, far_prox>
		vector<char> bfv;								// Binary Feature Vector (one bit per feature)
		bool faultDetected;
		bool faultLogged;
		queue<pair<string, bool>> responseQueue;
		// bool broadcastLogged = false;
		float lastBroadcastTime;
		map<string,vector<string>> bfvMap;				// map of FVs to robot IDs <FV, vector of IDs>
		
		/*TODO: Move constants to .argos file*/

		/* Parameters for the stochastic simulator */
		const Real k = 0.002;				// scaling factor for assigning FVs to APCs 
		const uint8_t Ie = 10;				// Effector cell influx density
		const uint8_t Ir = 10;				// Regulatory cell influx density
		const uint32_t S = 5*10e7;			// Simulated time steps for CRM instance (50 million)
		const Real c = 0.15;				// Cross-reactivity between T-cells and APCs
		const uint8_t l = 2;				// length of binary feature vector 
		const uint8_t N = 4;				// Number of sub-populations (2^l where l is the length of the FV right now it is two bits long)
		const uint8_t M = 4;				// Number of APCs (2^l where l is the length of the FV right now it is two bits long)
		const Real d = 0.5; 				// portion of cells to diffuse to a neighbor (and receive)

		/*TODO: Set up h to be tuned through multiple iterations */
		const uint16_t h = 1000;				// Numerical Integration step size (needs to be tuned)

		/* Parameters for the CRM */
		map<string, Real> APC;				// APC density per sub-population  						<key: FV, value: Density>
		const uint8_t s = 3;				// Max number of cells that can bind to an APC
		const uint8_t E0 = 10;				// Effector cell seed density
		const uint8_t R0 = 10;				// Regulatory cell seed density
		map<string, Real> E;				// Effector cell density per sub-population 			<key: FV, value: Density>
		map<string, Real> R;				// Regulatory cell density per sub-population			<key: FV, value: Density>
		map<string, Real> T;				// T-cell density per sub-population (|Ei|+|Ri|)		<key: FV, value: Density>

		// Number of conjugated effector cells of T-cell clone i at APC sub-population j |		map<key: FV_i, value: map<key: FV_j, value: Density>>
		map<string, map<string, Real>> Ec_ij;
		// Number of conjugated regulatory cells of T-cell clone i at APC sub-population j |	map<key: FV_i, value: map<key: FV_j, value: Density>>
		map<string, map<string, Real>> Rc_ij;

		const Real gamma_c = 0.1;			// Conjugation rate
		const Real gamma_d = 0.1;			// Deconjugation rate
		const Real rho_E = 10e-3;			// Effector cell proliferation rate
		const Real rho_R = 0.7*10e-3;		// Regulatory cell proliferation rate
		const Real Delta = 10e-6;			// T-cell death rate

		vector<char> BuildBFV();
		void UpdateProximityQueue(vector<Real> sRangeList);
		void SetupCRM(); 				// Setup the CRM (i.e., compute FV distribution, assign FVs to APCs, init/update T-cells)
		pair<map<string,Real>,map<string,Real>> CalculateConjugates();		// Calculate the number of conjugated cells per sub-population
		uint8_t HammingDistance(const string& a, const string& b);

		/**
		 * The probability that an effector cell is conjugated with no neighbouring regulatory 
		 * cell at the same APC
		 * @param Aj APC density of sub-population j
		 * @param Ecj Number of conjugated effector cells at APC sub-population j
		 * @param Rcj Number of conjugated regulatory cells at APC sub-population j
		*/
		Real P_e(Real Aj, Real Ecj, Real Rcj);

		/**
		 * The probability that a regulatory cell is conjugated with an APC that has at least one 
		 * effector cell conjugated simultaneously
		 * @param Aj APC density of sub-population j
		 * @param Ecj Number of conjugated effector cells at APC sub-population j
		 * @param Rcj Number of conjugated regulatory cells at APC sub-population j
		*/
		Real P_r(Real Aj, Real Ecj, Real Rcj);

		/**
		 * Uses the Forward Euler method to numerically integrate the CRM
		*/
		void ForwardEuler();

		/*TODO: Research the Euler Heun adaptive step method to implement the CRM instance. */

		/**
		 * ### Share the number of T-cells with neighbors
		 * @brief This is part of the cell diffusion process which is executed after the CRM instance is completed.
		 * 
		 * Message Type: "c"
		*/
		void ShareCellCount();

		/**
		 * ### Receive the number of T-cells from neighbors
		 * @brief This is part of the cell diffusion process which should be executed after ShareCellCount().
		 * @return A map of the total number of T-cells per neighbor <key: fb_ID, value: cell_count>
		*/
		map<string, Real> ReceiveCellCount();

		/**
		 * ### Select a random agent to diffuse cells with.
		 * @brief Select a neighbor randomly from a uniform distribution weighted by their T-cell count (normalized).
		 * @param ccmap the map of cell counts for each neighbor
		 * @return ID of the selected agent as a string
		*/
		string SelectRandomAgent(map<string, Real> ccmap);
		
		
		/**
		 * ### Send a portion of cells to a selected neighbor.
		 * 
		 * Message Type: "s"
		*/
		void SendCells(string targetID);

		/**
		 * #### Get the number of T-cells per neighbor then begin diffusion
		 * 1. Receive the shared cell counts from neighbors. 
		 * 2. Randomly select a neighbor to diffuse cells with weighted by their T-cell count (normalized).
		 * 3. Send a portion of cells 'd' to the selected neighbor.
		*/
		void DiffusePhase1();

		/*TODO: Finish diffusion process*/
		/**
		 * The remainder of the process consists of receiving the cells from the selected neighbor.
		 * And also receiving any cells sent by other neighbors, then sending back a portion of cells.
		 * 
		 * This is an exchange of cells. 
		 * - Send to target, receive from target. 
		 * - Receive from any other agent (if you are their target), then send back to that agent.
		 * 
		 *
		 * - The code on the github for Dr. Tarapores experiment is commented out. Saying that they believe it is not
		 * necessary for the experiments in the most recent paper.
		 * 
		 * link: https://github.com/daneshtarapore/faultdetection-epuckswarm/blob/master/controllers/fault_detection_model/crm/crminrobotagent_optimised.cpp
		 * comment on line 216
		*/

		/**
		 * ### Receive cells from neighbors (whom you are the target of).
		 * @return A queue of neighbors that have sent cells for diffusion queue<pair<fb_ID, map<key: FV, value: pair<E_d, R_d>>>>
		*/
		queue<pair<string, map<string, pair<Real,Real>>>> ReceiveCells();

		/**
		 * ### Generate a queue of packets to be sent to neighbors (who targeted you for diffusion).
		 * @brief Each packet is similar to that of SendCells(). They will be sent one at a time to the targets.
		 * @return A queue of packets to be sent to neighbors that contains cell count data. queue<string>
		*/
		queue<string> GenerateResponsePackets(queue<pair<string, map<string, pair<Real,Real>>>> crQMap);

		/**
		 * ### Compute the sum of Effector and Regulatory cells weighted by their affinity.
		 * @brief This is used to begin the vote on whether or not a FV is abnormal.
		 * @returns A map of the the decision on whether a FV is tolerated or not based on the sums of E and R cells weighted by their affinity. <key: FV, value: decision>
		*/
		map<string, bool> ComputeDecision();

		/**
		 * ### Receive the decisions from other robots on whether or not a FV is abnormal.
		 * @brief This is a step in the consensus formation on whether or not a FV is abnormal.
		*/
		map<string, vector<bool>> ReceiveDecisions();
		
		/********************************************************************************/

		// void ProcessVotes();
};

#endif /* CPFA_CONTROLLER_H */