// Ryan Luna 12/28/22

#ifndef QZONE_H_
#define QZONE_H_

#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/simulator/entity/floor_entity.h>

#include <source/Base/Food.h>
#include <source/Base/QuarantineZone.h>

using namespace argos;
using namespace std;

class QZone {

    public:
        QZone(CVector2 location, Real radius);
        CVector2    GetLocation();
        CColor      GetColor();
        Real      GetRadius();
        vector<Food>    GetFoodList();

        void RemoveFood(Food foodObj);
        void AddFood(Food newFood);

    // protected:
        void        SetLocation(CVector2 newLocation);
        void        SetColor(CColor newColor);
        void        SetRadius(Real newRadius);

    private:
        CVector2        Location;
        CColor          Color;
        Real            Radius;
        vector<Food>    QFood;

};

#endif