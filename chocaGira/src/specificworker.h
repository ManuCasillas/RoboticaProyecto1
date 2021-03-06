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

/**
       \brief
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
// #include <innermodel/innermodelmgr.h>
#include <vector>
#include <Laser.h>
#include <JointMotor.h>

#define INCREMENT 20


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
        void setPick(const Pick &myPick);
	
	void go(const string &nodo, const float x, const float y, const float alpha);
	void turn(const float speed);
	bool atTarget();
	void stop();
	void pickingBox();
	void releasingBox();
	bool pickedBox();
	
	
	float gaus(float Vrot, float Vx, float h);
	void gotoTarget();
	void bug();
	bool obstacle(int umbral);
	bool obstacleBug();
	bool targetAtSight();	
	void finish();
	void border();
	bool shock();
	void moveArm();
	void moveDownArm();
	
	
private:
       enum class State{IDLE, GOTO, ROTATE, BORDER, PICKING , RELEASING, FINISH};
       State state = State::IDLE;
       
       struct Coordinate 
       {

	 Coordinate(){};
	 void activate()
	 {
	   QMutexLocker ml(&mutex);
	    clicked = true; 
	 };
	 void activateTag()
	 {
	   QMutexLocker ml(&mutex);
	    tag = true;
	 };
	void disable()
	 {
	   QMutexLocker ml(&mutex);
	    clicked = false; 
	 };
	void disableTag()
	 {
	   QMutexLocker ml(&mutex);
	    tag = false; 
	 };
	 bool getClicked(){
	   return clicked;
	 };
	 bool getTag(){
	   return tag;
	 };
	 std::pair<float, float> getValues (){ 
       
	    QMutexLocker ml(&mutex);
	    return std::make_pair<>(x, z);	    
	 };
	 void setX(float _x) {
	   QMutexLocker ml(&mutex);
	   x = _x;
	 };
	 void setZ(float _z) {
	   QMutexLocker ml(&mutex);
       
	   z = _z;
	 };
	 float getX(){
	   return x;
	 };
	 float getZ(){
	   return z;
	 };
	 void setFinish(bool f)
	 {
	   QMutexLocker ml(&mutex);
	    finish = f; 
	 };
	 bool getFinish()
	 {
	   QMutexLocker ml(&mutex);
	   return finish;
	 };
	 
	  float x = 0;
	  float y = 0;
	  float z = 0;
	  bool clicked = false;
	  bool tag = false;
	  bool finish = false;
	  QMutex mutex;
	  QVec vec;
	  
	  
       };
       InnerModel *innermodel;

	//Tag tag;
	float distTarget;
	Coordinate coor;
// 	InnerModelMgr innerModel;
	RoboCompJointMotor::MotorParamsList mList;
	QStringList joints;
	QVec motores;
	QVec error;
	bool pushedButton = false;
	int FACTOR = 1;
	bool box = false;
	
bool isPushed();
	 
public slots:
	void compute(); 
	void leftSlot();
	void rightSlot();
	void upSlot();
	void downSlot();
	void frontSlot();
	void backSlot();
	void goHome();
	void closeHand();
	void changeSpeed(int);

	
};

#endif

