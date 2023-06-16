#ifndef NEST_H_
#define NEST_H_

#include <map> //qilu 09/11/2016
#include <argos3/core/utility/math/vector2.h>
//#include <argos3/core/utility/math/ray3.h>
#include <argos3/core/utility/logging/argos_log.h>
#include "Pheromone.h"
#include "Food.h"       // Ryan Luna 11/10/22
#include "QuarantineZone.h" // Ryan Luna 1/24/23
using namespace argos;
using namespace std;

/*****
 * Implementation of the iAnt nest object used by the iAnt MPFA. iAnts
 * build and maintain a list of these nest objects.
 *****/
class Nest {

	public:
                Nest();
                Nest(CVector2 location);
                
                vector<Pheromone> PheromoneList;
                map<string, argos::CVector2> FidelityList; //qilu 09/10/2016
                map<string, size_t> DensityOnFidelity; //qilu 09/11/2016
                vector<Food> FoodList;
                size_t num_collected_tags;
                size_t visited_time_point_in_minute;
                CVector2 GetLocation();
                void SetLocation();
                void SetLocation(CVector2 newLocation); //qilu 09/11/2016

                void SetNestIdx(size_t idx);
                size_t GetNestIdx();

                void CreateZone(size_t merge_mode, vector<Food>AllFood, vector<Food> LocalList, Food CentralResource, Real ScanDistance);

                vector<QZone> GetZoneList();
        
	private:

                void DistanceBasedMerging(vector<Food>AllFood, QZone Catalyst); 
                CVector2 nestLocation;
                size_t nest_idx;

                vector<QZone> ZoneList;         // Ryan Luna 1/24/23

};

#endif /* IANT_NEST_H_ */
