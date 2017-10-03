/*
 *    Copyright (C) 2017 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	timer.start(Period);	

	return true;
}


void SpecificWorker::compute()
{
    //qDebug() << "asdasd";
    TLaserData data = laser_proxy->getLaserData();
    std::sort(data.begin()+20, data.end()-20, [](auto a,auto b){return a.dist < b.dist;});
    //for (auto d:data)
     // qDebug() << d.angle << d.dist;
    //qDebug() << "---------------------------";
    
    if(Coordinate.clicked){
      
  qDebug()<< "------------------------";
  qDebug() << "EJE X - COMPUTE" << Coordinate.x;
  qDebug() << "EJE y" << Coordinate.y;  
  qDebug() << "EJE Z" << Coordinate.z;
  qDebug()<< "------------------------";
      
      
	Coordinate.clicked = false;
    }

    
    //NO BORRRRRRRRRRAAAAAAAAAAAAAARRRRRRRRRRR
//     qDebug() << "PRIMERO" << data[20].dist;
//     differentialrobot_proxy->setSpeedBase(400, 0);
//     if (data[20].dist < 210){
//      qDebug() << "PELIGRO"; 
//      differentialrobot_proxy->setSpeedBase(0, 1);
//      
//      sleep(1);
//     }
     

}

void SpecificWorker::setPick(const Pick &myPick)
{

  
  
  if(!Coordinate.clicked){
    Coordinate.activate();
    
    Coordinate.x = myPick.x;
    Coordinate.y = myPick.y;
    Coordinate.z = myPick.z;
    
  }
  
//el meotod que va a llamar el ice middleware
  qDebug()<< "------------------------";
  qDebug() << "EJE X" << myPick.x;
  qDebug() << "EJE y" << myPick.y;  
  qDebug() << "EJE Z" << myPick.z;
  qDebug()<< "------------------------";
  
  
  
  
  
}










