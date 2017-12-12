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
	innermodel = new InnerModel("/home/salabeta/robocomp/files/innermodel/betaWorldArm.xml");
	timer.start(Period);
	return true;
}


//COMPUTE TAG	
void SpecificWorker::compute()
{
  RoboCompDifferentialRobot::TBaseState bState;
  differentialrobot_proxy->getBaseState(bState);
//   
//   
  innermodel->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
  
  
   

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
    
    case StateTag::SEARCH_CORNER:
     qDebug() << "SEARCH_CORNER";
     searchCorner();

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
  
  QMutexLocker ml(&mutexCam);
  int auxID = 0;
  float auxTX  = 0 ,dist = 0, auxTZ = 0;
//   bool aux = false;


 for (auto d:tags){
   
  if(d.id == basurero && goBasurero == true){ //
//     qDebug() << " HOLAHOLA---HOLAHOLA";
    rt = innermodel->transform("world", QVec::vec3(d.tx, 0, d.tz), "robot");
    auxID = d.id;
    auxTX = d.tx;
    auxTZ = d.tz;
//     aux = true;
    
    
    tag.setID(auxID);
	tag.setTX(auxTX);
	tag.setTZ(auxTZ);
      
      
      rt = innermodel->transform("world", QVec::vec3(tag.getTX(), 0, tag.getTZ()), "robot");
      
      distanciaMax = 99999;

  }else  if (d.id >= primeraCaja && goBasurero == false && visitado(d.id) == false ){
   qDebug()<<"Tags mostrados  " << d.id;
   QVec rd  = innermodel->transform("world", QVec::vec3(d.tx, 0, d.tz), "robot");
    dist = rd.norm2();
    
    if(dist < distanciaMax && dist > 250){
      distanciaMax = dist;
      auxID = d.id;
      auxTX = d.tx;
      auxTZ = d.tz;
      
       tag.setID(auxID);
	tag.setTX(auxTX);
	tag.setTZ(auxTZ);
      
      
      rt = innermodel->transform("world", QVec::vec3(tag.getTX(), 0, tag.getTZ()), "robot");  
 
      
    }
  }
}
//   tag.setID(auxID);
//   tag.setTX(auxTX);
//   tag.setTZ(auxTZ);
  
//   rt = innermodel->transform("world", QVec::vec3(tag.getTX(), 0, tag.getTZ()), "robot");  
 
 
}

void SpecificWorker::initial()
{
//   current = 10;
  /*entra= false;*/ 
  
  for(int i  = 0; i < 10 ; i++){
    cajasRecogidas[i] = 0;
  }
  
  stateTag = StateTag::SEARCH;
}

void SpecificWorker::search()
{
  
   try
    { 
    gotopoint_proxy->turn(1);
    } 
    catch(const Ice::Exception &e)  { std::cout << e << std::endl;}
    
    
    
    if(tag.getID() >= primeraCaja && visitado(tag.getID()) == false)
    {
      
       bool entra = false;
   for(int i  = 0;i < 10 && !entra; i++){
     
      if(cajasRecogidas[i] == 0 && !entra){
	 qDebug()<< " --------tagid "<< tag.getID();
	cajasRecogidas[i] = tag.getID();
	entra = true;
	stateTag = StateTag::GOTO;
      }
      qDebug()<< " --------cajas recogidas "<< cajasRecogidas[i];
  }
      
     
  	
      try 
      {
      gotopoint_proxy->stop(); 
	
      } catch(const Ice::Exception &e) { std::cout << e << std::endl;}
      
     
     }
     
}

bool SpecificWorker::visitado(int id){
  
  
  for(int i  = 0; i < 10 ; i++){
    
    if(cajasRecogidas[i] == id)
      return true;
  }
  return false;
  
  
}


 
void SpecificWorker::searchCorner()
{
 
  
   try
    { 
    gotopoint_proxy->turn(1);
    } 
    catch(const Ice::Exception &e)  { std::cout << e << std::endl;}
    
   
    
    if(tag.getID() == basurero)
    {
	qDebug()<< " --------tagid Corner "<< tag.getID();
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
	//gotopoint_proxy->stop(); 
//       qDebug() << "Entra tol rato";
	//LE PASAMOS BOX O BASUSERO mirando la variable basusero
      gotopoint_proxy->go("", rt.x(), rt.z(),0);
	
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
    
   goBasurero = !goBasurero;
 
   if(goBasurero == false)
    stateTag = StateTag::SEARCH;
   else
    stateTag = StateTag::SEARCH_CORNER;
   
//     entra = false;
  }else{
    // llamar mejor y menos
    stateTag = StateTag::GOTO;
  }
}
     
