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
#include <vector>
#include <Laser.h>


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	
	void newAprilTag(const tagsList& tags);
	
	
private:
     
        enum class StateTag{INITIAL, SEARCH, SEARCH_CORNER, GOTO, WAIT};
       StateTag stateTag = StateTag::INITIAL; 
      
	void initial();
	void search();
	void searchCorner();
	void gotoT();
	void wait();
	bool visitado(int id);
     
         struct Tag{
	 Tag(){};
	 
	  void setTX(float _tx) {
	   QMutexLocker ml(&mutex);
	   tx = _tx;
	 };
	 
	 float getTX(){
	   return tx;
	 };
	 void setTZ(float _tz) {
	   QMutexLocker ml(&mutex);
	   tz = _tz;
	 };
	 
	 float getTZ(){
	   return tz;
	 };
	 
	 void setID(int _id) {
	   QMutexLocker ml(&mutex);
	   id = _id;
	 };
	 
	 int getID(){
	   return id;
	 };
	 
	 bool getFinish()
	 {
	   QMutexLocker ml(&mutex);
	   return finish;
	 };

		int id;
		float tx;
		float tz;
		bool finish;
		QMutex mutex;
	};
	
	QVec rt;
	QMutex mutexCam;
	InnerModel *innermodel;
	float distTarget;
// 	int current = 10;//make the robot turn until the tag.id == current
	Tag tag;
// 	bool entra = false;
	int primeraCaja = 11; //para resetear mas facilmente
	int basurero = 3; //AprilTags que servira como basurero
	bool goBasurero = false;
	int cajasRecogidas[10];
	
public slots:
	void compute(); 	

	
};

#endif

