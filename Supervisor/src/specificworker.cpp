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
//   RoboCompDifferentialRobot::TBaseState bState;
//   differentialrobot_proxy->getBaseState(bState);
//   
//   
//   innermodel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
  
  
   

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
 

//   if(!entra){
    
  tag.setID(tags.front().id);
  tag.setTX(tags.front().tx);
  tag.setTZ(tags.front().tz);
  
//   qDebug() << "ID: " << tag.getID() << " TX: " << tag.getTX() << " TZ: " << tag.getTZ();
  
 innermodel->transform("world", QVec::vec3(tag.getTX(), 0, tag.getTZ()), "rgbd");  
 
//  if(tag.getID() == current)
//    entra = true;
//  
//  
//   }
 
}

void SpecificWorker::initial()
{
  current = 0;
  entra = false;
  stateTag = StateTag::SEARCH;
}

void SpecificWorker::search()
{
  
   try
    { 
    gotopoint_proxy->turn(1);
    } 
    catch(const Ice::Exception &e)  { std::cout << e << std::endl;}
    
    qDebug()<< " --------tagid "<< tag.getID()<< " current id -------" << current;
    
    if(tag.getID() == current)
    {
      entra = true;
      try 
      {
      gotopoint_proxy->stop(); 
	
      } catch(const Ice::Exception &e) { std::cout << e << std::endl;}
      
      stateTag = StateTag::GOTO;
     }
     
}

void SpecificWorker::gotoT()
{
  try 
      {
	gotopoint_proxy->stop(); 
	
//       qDebug() << " -------X Despues: " << tag.getTX();
//       qDebug() << " -------Z Despues: " << tag.getTZ();
      
      gotopoint_proxy->go("", tag.getTX(), tag.getTZ(),0);
	
      } catch(const Ice::Exception &e) { std::cout << e << std::endl;}
      
      stateTag = StateTag::WAIT;   
  
}

void SpecificWorker::wait()
{
  bool aux = false;
  
  try 
      {
      aux = gotopoint_proxy->atTarget();
      } catch(const Ice::Exception &e) { std::cout << e << std::endl;}

  
  if(aux == true){
    
    try 
      {
	gotopoint_proxy->stop();
      } catch(const Ice::Exception &e) { std::cout << e << std::endl;}
    
    current++;
  if(current == 4){
    stateTag = StateTag::INITIAL;
  }else{
    stateTag = StateTag::SEARCH;
    entra = false;
  }
}
     
}