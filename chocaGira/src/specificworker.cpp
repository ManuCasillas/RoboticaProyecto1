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


void SpecificWorker::compute()
{
  RoboCompDifferentialRobot::TBaseState bState;
  differentialrobot_proxy->getBaseState(bState);

   TLaserData laserData = laser_proxy->getLaserData();

  innermodel->updateTranslationPointers("base", bState.x, 0, bState.z ,0, bState.alpha, 0);
//   innermodel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);

  switch( state )
 {

   case State::IDLE:

    if ( coor.getClicked())
     state = State::GOTO;
   
    break;

   case State::GOTO:

    gotoTarget();

    break;
   
   case State::ROTATE:     
   break;
   
   case State::BORDER:
   break;
   
   case State::FINISH:
   break;
   
  }


  
  
  
   /* ----------------------ENTREGA CAMINO RECTO------------------------------
    float angle, distTarget, vadv, vrot;
    const float MaxAdv = 400;
    const float MaxRot = 0.5;
    const float e = 2.71828;
    TLaserData data = laser_proxy->getLaserData();
    std::sort(data.begin()+20, data.end()-20, [](auto a,auto b){return a.dist < b.dist;});

    
    RoboCompDifferentialRobot::TBaseState state;
    differentialrobot_proxy->getBaseState(state);
    innermodel->updateTransformValues("base", state.x, 0, state.z, 0, state.alpha, 0);
      
    if(coor.getClicked())
    {
      QVec dist = innermodel->transform("base",QVec::vec3(coor.getX(), 0, coor.getZ()), "world" );
      angle = atan2(dist.x(), dist.z());
      distTarget = dist.norm2();
  
      if (distTarget < 50 )
      {
	coor.disable();
	differentialrobot_proxy->setSpeedBase(0, 0);  
      } 
      else 
      {
	
      vadv = distTarget;
      vrot = angle;
      if(vrot > angle)
      {
	vrot = MaxRot;
      }

	vadv = (1/(1+pow(e, -distTarget))-0.5); //Cambiar el dist.x() por X
	vadv = vadv * gaus(vrot, 0.3, 0.5) * MaxAdv;

	differentialrobot_proxy->setSpeedBase(vadv, vrot);

       } 
    }
    
      */
      
 
/*    -------------------------ENTREGA CAMINO CURVO---------------------------
 * if(coor.getClicked()){  
	
	 QVec dist = innermodel->transform("base",QVec::vec3(coor.getX(), 0, coor.getZ()), "world" );
	 angle = atan2(dist.x(), dist.z());
	 distTarget = dist.norm2();
	 if (distTarget < 50 ){
	   coor.disable();
	   differentialrobot_proxy->setSpeedBase(0, 0);
	 } else {
	   vadv = distTarget;
	   vrot = angle;
	   if(vrot > angle){
	     vrot = MaxRot;
	   }
	   if (vadv > distTarget){
	     vadv = MaxAdv;
	   }
	   if (angle != 0){
	     differentialrobot_proxy->setSpeedBase(vadv, vrot);
	   } else {
	     differentialrobot_proxy->setSpeedBase(vadv, 0);
	   }
	 }   
      

    }

    
    NO BORRRRRRRRRRAAAAAAAAAAAAAARRRRRRRRRRR
        for (auto d:data)
     qDebug() << d.angle << d.dist;
    qDebug() << "---------------------------";
    
    
    qDebug() << "PRIMERO" << data[20].dist;
    differentialrobot_proxy->setSpeedBase(400, 0);
    if (data[20].dist < 210){
     qDebug() << "PELIGRO"; 
     differentialrobot_proxy->setSpeedBase(0, 1);
     
     sleep(1);
    }
*/
     

}

float SpecificWorker::gaus(float Vrot, float Vx, float h)
{ //vx = 0.3 h = 0.5
  const float e = 2.71828;
  float L = -(Vx * Vx)/log(h);
  float result = pow(e, (-(Vrot * Vrot)/L));

  return result;

}

void SpecificWorker::setPick(const Pick &myPick)
{
  
  if(!coor.getClicked()){
     
    coor.setX(myPick.x);
    coor.setZ(myPick.z);
    coor.activate();
  }  
  
}

void SpecificWorker::gotoTarget()
 {
float vrot;
    
    if( obstacle == true)   // If ther is an obstacle ahead, then transit to BUG
   {

      state = State::ROTATE;

      return;

   }

    QVec rt = innermodel->transform("base", QVec::vec3(coor.getX(), 0, coor.getZ()), "world");

    float dist = rt.norm2();

    float ang  = atan2(rt.x(), rt.z());

   if(dist < 100)          // If close to obstacle stop and transit to IDLE
  {

    state = State::IDLE;
    coor.activate();

   return;
  }

  float adv = dist;

  if ( fabs( vrot) > 0.05 )

   adv = 0;

 }
 
void SpecificWorker::bug()

{

}

bool SpecificWorker::obstacle()

{

}

bool SpecificWorker::targetAtSight()

{

}








