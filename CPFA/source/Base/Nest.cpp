#include "Nest.h"

/*****
 * The iAnt nest needs to keep track of four things:
 *
 * [1] location
 * [2] nest id 
 * [3] site fidelity
 * [4] pheromone trails
 *
 *****/
	Nest::Nest(){}
	Nest::Nest(CVector2   location)
{
    /* required initializations */
	   nestLocation    = location;
    PheromoneList.clear();
    FidelityList.clear();
    DensityOnFidelity.clear(); //qilu 09/11/2016
    FoodList.clear(); //qilu 09/07/2016
    //num_collected_tags=0;
    visited_time_point_in_minute=0;
    nest_idx=-1;
}

/*****
 *****/

/*****
 * Return the nest's location.
 *****/
CVector2 Nest::GetLocation() {
    return nestLocation;
}

void Nest::SetLocation() {
    nestLocation=CVector2(0.0, 0.0);
}

void Nest::SetLocation(CVector2 newLocation) {
    nestLocation = newLocation;
}

void Nest:: SetNestIdx(size_t idx){
     nest_idx = idx;
 }
 
size_t Nest:: GetNestIdx(){
     return nest_idx;
 } 

void Nest::CreateZone(size_t merge_mode, vector<Food>AllFood, vector<Food> LocalList, Food CentralResource, Real ScanDistance){

    Real radius = ScanDistance;

    QZone newZone(CentralResource.GetLocation(), ScanDistance);

    for(Food f : LocalList){
        if ((newZone.GetLocation() - f.GetLocation()).Length() <= newZone.GetRadius()){
                newZone.AddFood(f);
            }
    }
    
    ZoneList.push_back(newZone);

    // cout << "Zone Created: Location = " << newZone.GetLocation() << ", Radius = " << newZone.GetRadius() << 
    //     ", QFood size = " << newZone.GetFoodList().size() << endl;

    /**
     * Merge Mode 0 -> No Merging
     * Merge Mode 1 -> Distance-Based Merging
    */
    switch(merge_mode){
        case 0:
            // No merging
            break;
        case 1:
            DistanceBasedMerging(AllFood, newZone);
            break;
        default:
            argos::LOGERR << "ERROR: Invalid Merge Mode in XML file.\n";
    }
}

vector<QZone> Nest::GetZoneList(){
    return ZoneList;
}

void Nest::DistanceBasedMerging(vector<Food> AllFood, QZone Catalyst){

    bool CanMerge = false;
    QZone* qz_i;
    QZone* qz_j;
    int i,j = 0;

    for(i = 0; i < ZoneList.size()-1; i++){
        qz_i = &ZoneList[i];
        for (j = i+1; j < ZoneList.size(); j++){
            qz_j = &ZoneList[j];

            if ((qz_i->GetLocation() - qz_j->GetLocation()).SquareLength() <= qz_i->GetRadius() + qz_j->GetRadius()){
                CanMerge = true;
                break;
            }

        }
        if (CanMerge){break;}
    }
    
    /**
     * Approximation involving center of mass of the points:

        Let 𝑐(𝑥𝑖,𝑦𝑖,𝑟𝑖)
        define a circle with position (𝑥𝑖,𝑦𝑖) and radius 𝑟𝑖

        Let (𝑥𝑐,𝑦𝑐)=(∑𝑛𝑖=1𝑥𝑖/𝑛,∑𝑛𝑖=1𝑦𝑖/𝑛)
        be the center of of mass of the 𝑛 circle positions.

        Next our concern is finding the smallest radius 𝑟𝑐 that encloses the 
        circles from the center of mass.

        This is done by finding the maximum value of the distance from the center (𝑥𝑐,𝑦𝑐)
        to the circle center positions (𝑥𝑖,𝑦𝑖) plus their radius 𝑟𝑖:

        𝑟𝑐=𝑚𝑎𝑥(sqrt((𝑥𝑐−𝑥𝑗,𝑦𝑐−𝑦𝑗)⋅(𝑥𝑐−𝑥𝑗,𝑦𝑐−𝑦𝑗))+𝑟𝑗)

        We now have a circle 𝐶(𝑥𝑐,𝑦𝑐,𝑟𝑐) enclosing the other circles. 
        
        This is not the optimal circle but the algorithm to find it is linearly bounded, 𝑜(𝑛).
    */

    if (CanMerge){

        bool ConfirmMerge = true;

        // get params from merging zones

        Real x1 = qz_i->GetLocation().GetX();
        Real x2 = qz_j->GetLocation().GetX();
        Real y1 = qz_i->GetLocation().GetY();
        Real y2 = qz_j->GetLocation().GetY();
        Real r1 = qz_i->GetRadius();
        Real r2 = qz_j->GetRadius();

        // get new radius

        Real newRadius = ((qz_i->GetLocation() - qz_j->GetLocation()).Length() + r1 + r2) / 2;
        
        /**
         * If the new radius is smaller than either of the two originals, don't merge
         * 
         * This means one of the zones is already inside the other.
        */
        if (newRadius < r1 || newRadius < r2){
            ConfirmMerge = false;
        }

        // get new center

        CVector2 newCenter;

        // find new point on the line connecting the two centers of the smaller circles
        // this point needs to be on the circumference
        float m = (y2-y1)/(x2-x1);  // slope
        Real x,y,X,Y;

        if (x1 > x2){
            // use c1
            x = r1*cos(atan(m)) + x1;  // get x
            y = r1*sin(atan(m)) + y1;  // get y
        }else{
            // use c2
            x = r2*cos(atan(m)) + x2;  // get x
            y = r2*sin(atan(m)) + y2;  // get y
        }

        // the center of the new circle should be on the same line
        // the point should be the distance of the radius away from the
        // point we previously discovered
        X = -newRadius*cos(atan(m)) + x;
        Y = -newRadius*sin(atan(m)) + y;

        newCenter.SetX(X);
        newCenter.SetY(Y);

        // create new zone object

        QZone MergedZone(newCenter,newRadius);
        MergedZone.SetColor(CColor::RED);

        // go through food list and add food within new zone

        for (Food fp: AllFood){
            if ((MergedZone.GetLocation() - fp.GetLocation()).Length() <= MergedZone.GetRadius()){
                MergedZone.AddFood(fp);
            }
        }

        if(ConfirmMerge){
            // erase old zones and push new merged zone
            // cout << endl << "Merging Zones: " << endl;
            // cout << "(x1,y1) = (" << x1 << ',' << y1 << "), " << "r1 = " << r1 << endl;
            // cout << "(x2,y2) = (" << x2 << ',' << y2 << "), " << "r2 = " << r2 << endl;
            // cout << "New Radius = " << newRadius << endl;
            // cout << "Slope: " << m << endl;
            // cout << "New Center: (X,Y) = (" << newCenter.GetX() << ',' << newCenter.GetY() << ")" << endl; 
            // cout << "# Food In New QZone = " << MergedZone.GetFoodList().size() << endl << endl;

            ZoneList.erase(ZoneList.begin()+i);
            ZoneList.erase(ZoneList.begin()+j-1);
            ZoneList.push_back(MergedZone);
        } else {
            // delete the zone inside the larger zone
            if (r1 < newRadius){
                // cout << "Zone Removed: " << '(' << x1 << ',' << y1 << ')' << endl;
                ZoneList.erase(ZoneList.begin()+i);
                if (x1 != Catalyst.GetLocation().GetX() && y1 != Catalyst.GetLocation().GetY()){
                    cout << endl << "NEWLY CREATED ZONE WAS NOT THE ONE THAT WAS REMOVED..." << endl;
                }
            }else{
                // cout << "Zone Removed: " << '(' << x2 << ',' << y2 << ')' << endl;
                ZoneList.erase(ZoneList.begin()+j);
                if (x2 != Catalyst.GetLocation().GetX() && y2 != Catalyst.GetLocation().GetY()){
                    cout << endl << "NEWLY CREATED ZONE WAS NOT THE ONE THAT WAS REMOVED..." << endl;
                }
            }
        }
        DistanceBasedMerging(AllFood, Catalyst);
    }
}
