// Ryan Luna 12/28/22

#include "QuarantineZone.h"

QZone::QZone(CVector2 location, Real radius):
    Location(location), 
    Radius(radius),
    Color(CColor::RED)
{
    // nothing yet
}

CVector2        QZone::GetLocation()    {return Location;}
CColor          QZone::GetColor()       {return Color;}
Real            QZone::GetRadius()      {return Radius;}
vector<Food>    QZone::GetFoodList()    {return QFood;}

void        QZone::SetLocation(CVector2 newLocation)        {Location = newLocation;}
void        QZone::SetColor(CColor newColor)                {Color = newColor;}
void        QZone::SetRadius(Real newRadius)                {Radius = newRadius;}

void        QZone::AddFood(Food newFood)                    {QFood.push_back(newFood);}
void        QZone::RemoveFood(Food foodObj){
    
    int i = 0;
    for(Food f : QFood){
        if(f.GetLocation()==foodObj.GetLocation()){
            QFood.erase(QFood.begin()+i);
        }
        i++;
    }
}