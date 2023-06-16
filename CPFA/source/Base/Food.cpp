// Ryan Luna 11/10/22

#include "Food.h"

Food::Food(CVector2 location, FoodType type):
    Location(location),
    Type(type)
{
    if(type == FAKE){
        Color = CColor::MAGENTA;
    } else {
        Color = CColor::BLACK;
    }
}

void            Food::SetColor(CColor newColor)         {Color = newColor;}
CColor          Food::GetColor()                        {return Color;}

void            Food::SetLocation(CVector2 newLocation) {Location = newLocation;}
CVector2        Food::GetLocation()                     {return Location;}

Food::FoodType  Food::GetType()                         {return Type;}