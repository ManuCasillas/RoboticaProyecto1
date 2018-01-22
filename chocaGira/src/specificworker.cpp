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

	goHome();
	sleep(1);
	
	try 
	{ mList = jointmotor_proxy->getAllMotorParams();}
	catch(const Ice::Exception &e){ std::cout << e << std::endl;}
	
	joints << "shoulder_right_1"<<"shoulder_right_2"<<"shoulder_right_3"<<"elbow_right" << "wrist_right_1" << "wrist_right_2";
	// Check that these names are in mList
	motores = QVec::zeros(joints.size());
	
	
	timer.start(Period);
	return true;
}

//COMPUTE CLICK

 void SpecificWorker::compute()
{
  RoboCompDifferentialRobot::TBaseState bState;
  differentialrobot_proxy->getBaseState(bState);

   TLaserData laserData = laser_proxy->getLaserData();

   innermodel->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);

  switch( state )
 {

   case State::IDLE:

    if ( coor.getClicked())
     state = State::GOTO;
   
    break;

   case State::GOTO:
     qDebug() << "GOTO";
    gotoTarget();

    break;
   
   case State::ROTATE:
      qDebug() << "ROTATE";
    bug();     
   break;
   
   case State::BORDER:
      qDebug() << "BORDER";
     border();
   break;
   
   case State::FINISH:
      qDebug() << "FINISH";
     finish();
   break;
   case State::PICKING:
     qDebug() << "PICKING";
     pickingBox();
     
     break;
     case State::RELEASING:
     qDebug() << "RELEASING";
     releasingBox();
     
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
  qDebug() << " NO DEBERIA DE ENTRAR-----------------_";
  if(!coor.getClicked()){
     
    coor.setX(myPick.x);
    coor.setZ(myPick.z);
    coor.activate();
  }  
  
}

void SpecificWorker::gotoTarget()
 {
   coor.disable();
    qDebug() <<"ENTRA EN gotoTarget";   
    float vrot,dist,vadv,ang;
    const float MaxAdv = 400, MaxRot = 0.5, e = 2.71828;
    
    
//    if(shock()){
//       qDebug() << "-------------- SE HA CHOCAO-------------";
//       differentialrobot_proxy->setSpeedBase(-10, 0);
//       
//    }
    
    
    if( obstacle(30))   // If ther is an obstacle ahead, then transit to BUG
   {
      state = State::ROTATE;

      return;
   }

    QVec rt;

     rt = innermodel->transform("robot", QVec::vec3(coor.getX(), 0, coor.getZ()), "world"); // Coordenadas del target

//     qDebug() << "coor.getX(): " << coor.getX() << "coor.getZ(): " << coor.getZ();
//     
    dist = rt.norm2();
    qDebug() << "DIST: " << dist;
    
    ang  = atan2(rt.x(), rt.z()); // con respecto al target
    
    distTarget = dist;
    

   if(distTarget < 400)          // If close to obstacle stop and transit to IDLE
  {
    
    if (box){
      state = State::PICKING;
    } else {
    //SI COOR.GETNODO == BOX , CAMBIAR A PICKING picking_box
    //SINO, FINISH
    state = State::RELEASING;
    }
    

   return;

  }
   else 
      {
	
      vadv = dist;
      vrot = ang;
   
      if(vrot > ang)
      {
	vrot = MaxRot;
      }

	vadv = (1/(1+pow(e, -dist))-0.5); //Cambiar el dist.x() por X
	vadv = vadv * gaus(vrot, 0.3, 0.5) * MaxAdv;
	differentialrobot_proxy->setSpeedBase(vadv, vrot);
	 
       } 

   distTarget = dist;
 }
 
void SpecificWorker::bug()
{
  qDebug() <<"ENTRA EN bug";   
  
// hay obstaculo
  TLaserData data = laser_proxy->getLaserData();
  differentialrobot_proxy->setSpeedBase(0, 0);  
  
  if(obstacle(20) == false)
   {
     
    if (data[20].dist < 400)
    {
//      qDebug() << data[data.size()-20].dist;
      differentialrobot_proxy->setSpeedBase(100 , -0.6);
       
    }
    
     
     state = State::BORDER;
      return;
  }
  
  differentialrobot_proxy->setSpeedBase(0, -1);  
  

}

bool SpecificWorker:: obstacle(int umbral)
{
    TLaserData data = laser_proxy->getLaserData();
    std::sort(data.begin()+umbral, data.end()-umbral, [](auto a,auto b){return a.dist < b.dist;});
 
    if (data[umbral].dist < 230)
      return true;
    else	
      return false;
}

bool SpecificWorker:: shock()
{
  
  int umbral = 10;
    TLaserData data = laser_proxy->getLaserData();
    std::sort(data.begin()+umbral, data.end()-umbral, [](auto a,auto b){return a.dist < b.dist;});
 
//     qDebug() << "----------distancia de choque: "<< data[umbral].dist;
    
    
    if (data[umbral].dist <= 30)
      return true;
    else	
      return false;
}


bool SpecificWorker::targetAtSight()
{

 
  TLaserData data = laser_proxy->getLaserData();
  QPolygonF polygon;
  
  for (auto l:data)
  {
    QVec lr = innermodel->laserTo("world", "laser", l.dist, l.angle);
//     QVec rt = innermodel->transform("base", QVec::vec3(coor.getX(), 0, coor.getZ()), "world");
    polygon << QPointF(lr.x(), lr.z());
  }
  
  QVec t = QVec::vec3(coor.getX(), 0, coor.getZ());
//   inhibit = 80;
  return  polygon.containsPoint( QPointF(t.x(), t.z() ), Qt::WindingFill );
 
}

void  SpecificWorker::finish()
{
  differentialrobot_proxy->setSpeedBase(0, 0);
  coor.disable();
  coor.disableTag();
  state = State::IDLE;
}

void  SpecificWorker::border()
{
    qDebug() <<"ENTRA EN border";   
     TLaserData data = laser_proxy->getLaserData();
//   if(shock()){
//       qDebug() << " SE HA CHOCAO";
//       differentialrobot_proxy->setSpeedBase(-10, 0);
//       
//    }
  
  
  bool entra = true;
  
  QVec rt = innermodel->transform("robot", QVec::vec3(coor.getX(), 0, coor.getZ()), "world");
  float dist = rt.norm2();
  float angle  = atan2(rt.x(), rt.z()); // con respecto al target
 
//   qDebug() <<" ------------distancia " << distTarget; 
  
  if(dist <= distTarget)
  {
    distTarget = dist;
    
  }else {
    
    if(distTarget < 400){
      entra = false;
      state = State::FINISH;
//       return;
    }
  }
  
   if((!obstacle(30) || targetAtSight() == true) && -0.025 < angle < 0.025)
   {
//      qDebug() << " -----------------------TARGET AT SIGHT--------------";
      state = State::GOTO;
    }
  

	
   if (obstacle(30))
   {
//      qDebug() << "Obstaculo";
     differentialrobot_proxy->setSpeedBase(80, -0.6); 
   }	

    //bordear objeto -- derecha
     if (data[data.size()-81].dist > 500 && entra==true && !obstacle(30))
     {
      std::sort(data.begin()+20, data.end()-50, [](auto a,auto b){return a.dist < b.dist;});
      differentialrobot_proxy->setSpeedBase(100 , +0.19);
     
    }
      
}

///////////////////////////////////////////////////////

void SpecificWorker::go(const string &nodo, const float x, const float y, const float alpha)
{
  std::cout << "Nodo" << nodo << std::endl;
   if (nodo == "box"){
    //CAMBIAR ESTADO A PICKING BOX 
    box = true;
    qDebug() << "ENTRA EN go & nodo == basurero";
  }
  if (nodo == "basurero") {
    
   box = false;
   qDebug() << "ENTRA EN go & nodo != basurero";
  }
//     qDebug() << "ENTRA EN go & nodo != basurero";
    coor.setX(x);
    coor.setZ(y);
//     qDebug() << "VALOR X EN GO" << x;
//     qDebug() << "VALOR Y EN GO" << y;
    coor.activateTag();
    coor.activate();
    
}
void SpecificWorker::turn(const float speed)
{
  differentialrobot_proxy->setSpeedBase(0 , speed);
 
}
bool SpecificWorker::atTarget()
{
  QVec rt;
//   innermodel->transform("world", QVec::vec3(coor.getX(), 0, coor.getZ()), "rgbd");
  rt = innermodel->transform("robot", QVec::vec3(coor.getX(), 0, coor.getZ()), "world");
  float dist = rt.norm2();
  

  qDebug() << "Distancia al objetivo: ---------------" << dist;
  
  if (dist < 400){
     return true;
    
    if (box){
      state = State::PICKING;
//       return 
    } else{
    state = State::RELEASING;
//     return false;
    }
    
  }else
    return false; 
  
    
 
}
void SpecificWorker::stop()
{
  differentialrobot_proxy->setSpeedBase(0 , 0);
  
}

void SpecificWorker::pickingBox()
{
  differentialrobot_proxy->setSpeedBase(0 , 0);
  qDebug()<< "Esperando a picking";
  moveArm();
  qDebug()<< "Terminado picking";
  
//     jointmotor_proxy->setPosition(2,0);
//    jointmotor_proxy->
  //LLAMAMOS A FINISH??
   
}

bool SpecificWorker::pickedBox()
{
  if(box == false)
      return true;
  
  return false;
}

void SpecificWorker::releasingBox()
{
  //mover brazo hacia abajo
  // hasta que toque el suelo -> verruga suelta la caja
  // box = true
  //gohome();
  
  //llamar cuando estemos en corner y que haya llegado al objetivo
  
  differentialrobot_proxy->setSpeedBase(0 , 0);
  moveDownArm();
  state = State::FINISH;
  
  
  
}


void SpecificWorker::moveArm()
{

	RoboCompJointMotor::MotorStateMap mMap;
	RoboCompGetAprilTags::listaMarcas tagR;
	float auxTX  = 0 ,auxTY = 0, auxTZ = 0;
	try
	{
	  tagR = getapriltags_proxy->checkMarcas();
	} 
	  catch(const Ice::Exception &e)  { std::cout <<"checkMarcas - " <<e << std::endl;}
	qDebug() << "1";
	

	  qDebug() << "2";
	
 	try
	{
		jointmotor_proxy->getAllMotorState(mMap);
		for(auto m: mMap)
		{
			innermodel->updateJointValue(QString::fromStdString(m.first),m.second.pos);
			//std::cout << m.first << "		" << m.second.pos << std::endl;
		}
		std::cout << "--------------------------" << std::endl;
	}
	catch(const Ice::Exception &e)
	{	std::cout << "JoinMotor -- "<<e.what() << std::endl;}
	
	//Compute Jacobian for the chain of joints and using as tip "cameraHand" 

	QMat jacobian = innermodel->jacobian(joints, motores, "rgbdHand");
	qDebug() << "3";
	RoboCompJointMotor::MotorGoalVelocityList vl;
	error = QVec::vec6(0,0,0,0,0,0);
	  
	  if (!tagR.empty()){
// 	    auxTX = tagR.front().tx;
// 			  auxTY = tagR.front().ty;
// 			  auxTZ = tagR.front().tz;
	    
	    
	    //while()
		try
		{
		  
		  
			  auxTX = tagR.front().tx; //objetivo final 
			  auxTY = tagR.front().ty;
			  auxTZ = tagR.front().tz;
			  qDebug() << "auxTX: "<< auxTX << " auxTY: " << auxTY << "AuxTZ: " << auxTZ;
			  // ESTO ES CORRECTO
			  if (auxTX > INCREMENT){
			    rightSlot();
			  }
			  if (auxTX < -INCREMENT){
			    leftSlot();
			  }
			  if (auxTY > INCREMENT){
			    frontSlot();
			  }
			  if (auxTY < -INCREMENT){
			    backSlot();
			  }
			  if (auxTZ > INCREMENT){
			    downSlot();
			  }
			  if (auxTZ < -INCREMENT){
			    upSlot();
			  }
			  
			
			QVec incs = jacobian.invert() * error;	
			qDebug() << "4.75";
			int i=0;
			qDebug() << "4.8";
			for(auto m: joints)
			{
				//RoboCompJointMotor::MotorGoalPosition mg = {mMap.find(m.toStdString())->second.pos + incs[i], 1.0, m.toStdString()};
				RoboCompJointMotor::MotorGoalVelocity vg{FACTOR*incs[i], 1.0, m.toStdString()};
				//ml.push_back(mg);
				vl.push_back(vg);
				i++;
			}
			
			qDebug() << "5";
		}	
		catch(const QString &e)
		{ qDebug() << e << "Error inverting matrix";}
	  }
	  else { 
	    closeHand();
	    sleep(2);	
	    //EJECUTAR LA VERRUGA
	    //CERRAR DEDOS Y BAJAR  _--> CREAR METODO PARA CERRAR  -> PARA ABRIR= GOHOME
	    
	    //msleep(500);
	    
	    qDebug()<< "Caja Cogida";
		for(auto m: joints)
		{
			RoboCompJointMotor::MotorGoalVelocity vg{0.0, 1.0, m.toStdString()};
			vl.push_back(vg);
		}
	
	qDebug() << "Brazo Parado";
	qDebug() << "Llamar a coger caja";
	box = false;
 	goHome();
	
	  }
		
// // 	//sleep(4);
	  qDebug() << "6";
// 		for(auto m: joints)
// 		{
// 			RoboCompJointMotor::MotorGoalVelocity vg{0.0, 1.0, m.toStdString()};
// 			vl.push_back(vg);
// 		}
	
	//Do the thing
	try
	{ 
	  qDebug() << "7";
		jointmotor_proxy->setSyncVelocity(vl);
	}
	catch(const Ice::Exception &e)
	{	std::cout << "SetSyncVelocity "<<e.what() << std::endl;}
	if (pickedBox()){
	  state = State::FINISH;
	}
	  
} 

void SpecificWorker::moveDownArm()
{
  RoboCompJointMotor::MotorGoalVelocityList vl;
  QMat jacobian = innermodel->jacobian(joints, motores, "rgbdHand");
  error = QVec::vec6(0,0,0,0,0,0);
  downSlot();
  QVec incs = jacobian.invert() * error;
  int i=0;
  qDebug() << "4.8";
  for(auto m: joints)
  {
    //RoboCompJointMotor::MotorGoalPosition mg = {mMap.find(m.toStdString())->second.pos + incs[i], 1.0, m.toStdString()};
    RoboCompJointMotor::MotorGoalVelocity vg{FACTOR*incs[i], 1.0, m.toStdString()};
    //ml.push_back(mg);
    vl.push_back(vg);
    i++;
  }
  sleep(3);
  
  goHome();
  
}

	
	
void SpecificWorker::goHome()
{
	RoboCompJointMotor::MotorStateMap mMap;
	try
	{
		jointmotor_proxy->getAllMotorState(mMap);
		for(auto m: mMap)
		{
			RoboCompJointMotor::MotorGoalPosition mg = { innermodel->getJoint(m.first)->home, 1.0, m.first };
			jointmotor_proxy->setPosition(mg);
		}
		sleep(1);
	}
	catch(const Ice::Exception &e)
	{	std::cout << e.what() << std::endl;}	
}

	
void SpecificWorker::closeHand()
{
	RoboCompJointMotor::MotorGoalPosition finger_right_1, finger_right_2;
	
	finger_right_1.name = "finger_right_1";
	finger_right_1.position = -0.6;
	finger_right_1.maxSpeed = 1;
	
	finger_right_2.name = "finger_right_2";
	finger_right_2.position = 0.6;
	finger_right_2.maxSpeed = 1;
	
	jointmotor_proxy->setPosition(finger_right_1);
	jointmotor_proxy->setPosition(finger_right_2);
		
}



bool SpecificWorker::isPushed()
{
	if (box && atTarget()) {
	  return true;
	} else
	  return false;
}

//////////////////
/// SLOTS
/////////////////

void SpecificWorker::leftSlot()
{
	error += QVec::vec6(INCREMENT,0,0,0,0,0);
}

void SpecificWorker::rightSlot()
{
	error += QVec::vec6(-INCREMENT,0,0,0,0,0);
}

void SpecificWorker::frontSlot()
{
	error += QVec::vec6(0,-INCREMENT,0,0,0,0);
}

void SpecificWorker::backSlot()
{
	error += QVec::vec6(0,INCREMENT,0,0,0,0);
}

void SpecificWorker::upSlot()
{
	error += QVec::vec6(0,0,INCREMENT,0,0,0);
}

void SpecificWorker::downSlot()
{
	error += QVec::vec6(0,0,-INCREMENT,0,0,0);
}

void SpecificWorker::changeSpeed(int s)
{
	FACTOR = s;
}




