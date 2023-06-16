// Ryan Luna 11/10/22

#ifndef FOOD_H_
#define FOOD_H_

#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/simulator/entity/floor_entity.h>

using namespace argos;
using namespace std;

/**
 * The food object enables additional customization of food resources
 * For the DoS using fake food, we need identifiers for what is real food and what is fake
 **/

class Food {

    public:
        enum FoodType{
            FAKE = 0,
            REAL = 1
        };
        Food(){}
        Food(CVector2 location, FoodType type);
        void SetColor(CColor newColor);
        CColor GetColor();
        void SetLocation(CVector2 newLocation);
        CVector2 GetLocation();
        FoodType GetType();

    private:

        CVector2    Location;
        CColor      Color;
        FoodType    Type;

};

#endif