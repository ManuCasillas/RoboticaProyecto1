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
	innermodel = new InnerModel("/home/salabeta/robocomp/files/innermodel/simpleworld.xml");
	timer.start(Period);
	return true;
}


//COMPUTE TAG	
void SpecificWorker::compute()
{
  RoboCompDifferentialRobot::TBaseState bState;
  differentialrobot_proxy->getBaseState(bState);
  
  
  innermodel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
  
   //TLaserData laserData = laser_proxy->getLaserData();
   
   qDebug() << "-------------Valor de ID: " << tag.getID();
   qDebug() << "-------------Valor de TX: " << tag.getTX();
   qDebug() << "-------------Valor de TZ: " << tag.getTZ();
   
   
   
   //newAprilTags();

  // innermodel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);

  switch( stateTag )
  {
    case StateTag::INITIAL:
      qDebug() << "INITIAL";
      initial();
    break;

   case StateTag::SEARCH:
     qDebug() << "SEARCH";
     search();

    break;
    
   case StateTag::GOTO:
     qDebug() << "GOTO";
     gotoT();
   break;

   case StateTag::WAIT:
     qDebug() << "WAIT";
      wait();
    break;
  
   
  }
  

}
	
void SpecificWorker::newAprilTag(const tagsList& tags)
{ 
  tag.setID(tags.front().id);
  tag.setTX(tags.front().tx);
  tag.setTZ(tags.front().tz);
  innermodel->transform("world", QVec::vec3(tag.getTX(), 0, tag.getTZ()), "rgbd");  
}

void SpecificWorker::initial()
{
  current = 0;
  stateTag = StateTag::SEARCH;
}

void SpecificWorker::search()
{
  qDebug() << "ENTRA EN SEARCH";
    gotopoint_proxy->turn(1);
    //differentialrobot_proxy->setSpeedBase(0 , speed);
     qDebug() << "DESPUES DE TURN";
     if(tag.getID() == current)
     {
      gotopoint_proxy->stop();
      stateTag = StateTag::GOTO;
     }
     qDebug() << "SALE DE SEARCH";
}

void SpecificWorker::gotoT()
{
      gotopoint_proxy->go("", tag.getTX(), tag.getTZ(),0);
      stateTag = StateTag::WAIT;  
  
}

void SpecificWorker::wait()
{
        if(gotopoint_proxy->atTarget() == true){
	  gotopoint_proxy->stop();
	current++;
	if(current == 4)
	 stateTag = StateTag::INITIAL;
	else
	 stateTag = StateTag::SEARCH;
      }
     
}