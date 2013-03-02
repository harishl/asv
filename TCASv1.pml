#define airspace_xlim 10
#define airspace_ylim 10
#define airspace_zlim 10
#define NumAircraft 3 /*Number of Aircraft processes to be created*/
#define maxVelocity 3
#define RA_proportionality_const 1
#define TA_proportionality_const 2

/*Air space*/
typedef Z { bit z[airspace_zlim] };
typedef Y { Z y[airspace_ylim] };
typedef AirSpace { Y x[airspace_xlim] };
AirSpace airspace;

/*A point in the air space*/
typedef Point {byte x; byte y; byte z };

/*channel for interrogation*/
chan interrogation = [100] of {chan};

mtype = {fwd, bkd, left, right, above, below, stay};
mtype = {climb, climb_faster, descend, maintain, traffic}; 
typedef Direction {mtype xdir; mtype ydir; mtype zdir};
typedef AircraftInfo {Point location; Direction dir; byte speed; chan RAChannel; };

inline move() {
	ctr = (ctr+1)%(maxVelocity - velocity + 1);
	if
	::ctr == 0 -> 
		/*Move aircraft in its direction*/
		atomic {
		airspace.x[index.x].y[index.y].z[index.z] = 0; // unsetting occupancy in current location
		// movement in x direction
		if
		::direction.xdir == fwd && index.x < (airspace_xlim-1) -> index.x++ 
		::direction.xdir == fwd && index.x == (airspace_xlim-1) -> index.x = 0
		::direction.xdir == fwd && index.x == (airspace_xlim-1) -> direction.xdir = bkd; index.x--
		::direction.xdir == bkd && index.x > 0 -> index.x--
		::direction.xdir == bkd && index.x == 0 -> index.x = (airspace_xlim-1)
		::direction.xdir == bkd && index.x == 0 -> direction.xdir = fwd; index.x++
		::else -> skip
		fi;
		// movement in y direction
		if
		::direction.ydir == left && index.y < (airspace_ylim-1) -> index.y++ 
		::direction.ydir == left && index.y == (airspace_ylim-1) -> index.y = 0
		::direction.ydir == left && index.y == (airspace_ylim-1) -> direction.ydir = right; index.y--
		::direction.ydir == right && index.y > 0 -> index.y--
		::direction.ydir == right && index.y == 0 -> index.y = (airspace_ylim-1)
		::direction.ydir == right && index.y == 0 -> direction.ydir = left; index.y++
		::else -> skip
		fi;
		// movement in z direction
		if
		::direction.zdir == above && index.z < (airspace_zlim-1) -> index.z++ 
		::direction.zdir == above && index.z == (airspace_zlim-1) -> index.z = 0
		::direction.zdir == above && index.z == (airspace_zlim-1) -> direction.zdir = below; index.z--
		::direction.zdir == below && index.z > 0 -> index.z--
		::direction.zdir == below && index.z == 0 -> index.z = (airspace_zlim-1)
		::direction.zdir == below && index.z == 0 -> direction.zdir = above; index.z++
		::else -> skip
		fi;
	
		airspace.x[index.x].y[index.y].z[index.z] = 1; //setting occupancy in the new location
		}
	::else -> skip
	fi;
}

inline computeDistance(numCells) {
	  //compute distance along x axis
	  xDist = 255; // infinity (ideally)
	  i = 1;
	  do
	  ::i <= numCells && otherAircraftInfo.location.x != (index.x+i)%airspace_xlim -> i++;
	  ::i <= numCells && otherAircraftInfo.location.x == (index.x+i)%airspace_xlim -> xDist = i; break; 
	  ::else -> break
	  od;
	  i = 1;
	  do
	  ::i <= numCells && (airspace_xlim - otherAircraftInfo.location.x - 1) != (airspace_xlim - index.x - 1 + i)%airspace_xlim -> i++;
	  ::i <= numCells && (airspace_xlim - otherAircraftInfo.location.x - 1) == (airspace_xlim - index.x - 1 + i)%airspace_xlim -> xDist = i; break;
	  ::else -> break
	  od;
	
	  //compute distance along y axis
	  yDist = 255; // infinity (ideally)
	  i = 1;
	  do
	  ::i <= numCells && otherAircraftInfo.location.y != (index.y+i)%airspace_ylim -> i++;
	  ::i <= numCells && otherAircraftInfo.location.y == (index.y+i)%airspace_ylim -> yDist = i; break;
	  ::else -> break
	  od;
	  i = 1;
	  do
	  ::i <= numCells && (airspace_ylim - otherAircraftInfo.location.y - 1) != (airspace_ylim - index.y - 1 + i)%airspace_ylim -> i++;
	  ::i <= numCells && (airspace_ylim - otherAircraftInfo.location.y - 1) == (airspace_ylim - index.y - 1 + i)%airspace_ylim -> yDist = i; break;
	  ::else -> break
	  od;

	  //compute distance along z axis
	  zDist = 255; // infinity (ideally)
	  i = 1;
	  do
	  ::i <= numCells && otherAircraftInfo.location.z != (index.z+i)%airspace_zlim -> i++;
	  ::i <= numCells && otherAircraftInfo.location.z == (index.z+i)%airspace_zlim -> zDist = i; break;
	  ::else -> break
	  od;
	  i = 1;
	  do
	  ::i <= numCells && (airspace_zlim - otherAircraftInfo.location.z - 1) != (airspace_zlim - index.z - 1 + i)%airspace_zlim -> i++;
	  ::i <= numCells && (airspace_zlim - otherAircraftInfo.location.z - 1) == (airspace_zlim - index.z - 1 + i)%airspace_zlim -> zDist = i; break;
	  ::else -> break
	  od;
}

inline computeRA() {
	Point myLoc;
	myLoc.x = index.x;
	myLoc.y = index.y;
	myLoc.z = index.z;	
	Point otherLoc;
	otherLoc.x = otherAircraftInfo.location.x;
	otherLoc.y = otherAircraftInfo.location.y;
	otherLoc.z = otherAircraftInfo.location.z;
	byte stepCtr = 0;
	do
	::
	atomic {
		/*Simulate my aircraft in its direction*/
		// movement in x direction
		if
		::(stepCtr%velocity) == 0 && direction.xdir == fwd -> myLoc.x = (myLoc.x + stepCtr)%airspace_xlim;
		::(stepCtr%velocity) == 0 && direction.xdir == bkd -> myLoc.x = myLoc.x - stepCtr;
		::else -> skip
		fi;
		myLoc.x = ((myLoc.x < airspace_xlim) -> myLoc.x : (myLoc.x + airspace_xlim)%256); // fixing issue with datatype
		// movement in y direction
		if
		::(stepCtr%velocity) == 0 && direction.ydir == left -> myLoc.y = (myLoc.y + stepCtr)%airspace_ylim;
		::(stepCtr%velocity) == 0 && direction.ydir == right -> myLoc.y = myLoc.y - stepCtr;
		::else -> skip
		fi;
		myLoc.y = ((myLoc.y < airspace_ylim) -> myLoc.y : (myLoc.y + airspace_ylim)%256);
		// movement in z direction
		if
		::(stepCtr%velocity) == 0 && direction.zdir == above -> myLoc.z = (myLoc.z + stepCtr)%airspace_zlim;
		::(stepCtr%velocity) == 0 && direction.zdir == below -> myLoc.z = myLoc.z - stepCtr;
		::else -> skip
		fi;
		myLoc.z = ((myLoc.z < airspace_zlim) -> myLoc.z : (myLoc.z + airspace_zlim)%256);		

		/*Simulate other aircraft in its direction*/
		if
		::(stepCtr%otherAircraftInfo.speed) == 0 && otherAircraftInfo.dir.xdir == fwd -> otherLoc.x = (otherLoc.x + stepCtr)%airspace_xlim;
		::(stepCtr%otherAircraftInfo.speed) == 0 && otherAircraftInfo.dir.xdir == bkd -> otherLoc.x = otherLoc.x - stepCtr;
		::else -> skip
		fi;
		otherLoc.x = ((otherLoc.x < airspace_xlim) -> otherLoc.x : (otherLoc.x + airspace_xlim)%256); // fixing issue with datatype
		// movement in y direction
		if
		::(stepCtr%otherAircraftInfo.speed) == 0 && otherAircraftInfo.dir.ydir == left -> otherLoc.y = (otherLoc.y + stepCtr)%airspace_ylim;
		::(stepCtr%otherAircraftInfo.speed) == 0 && otherAircraftInfo.dir.ydir == right -> otherLoc.y = otherLoc.y - stepCtr;
		::else -> skip
		fi;
		otherLoc.y = ((otherLoc.y < airspace_ylim) -> otherLoc.y : (otherLoc.y + airspace_ylim)%256);
		// movement in z direction
		if
		::(stepCtr%otherAircraftInfo.speed) == 0 && otherAircraftInfo.dir.zdir == above -> otherLoc.z = (otherLoc.z + stepCtr)%airspace_zlim;
		::(stepCtr%otherAircraftInfo.speed) == 0 && otherAircraftInfo.dir.zdir == below -> otherLoc.z = otherLoc.z - stepCtr;
		::else -> skip
		fi;
		otherLoc.z = ((otherLoc.z < airspace_zlim) -> otherLoc.z : (otherLoc.z + airspace_zlim)%256);	

		if
		::otherLoc.x == myLoc.x && otherLoc.y == myLoc.y && otherLoc.z == myLoc.z -> 
			timeBeforeCollision = stepCtr;
		::else -> stepCtr++; 
		fi;
	}
	od unless { 
		if
		::stepCtr > (RA_proportionality_const * velocity) && stepCtr > (RA_proportionality_const * otherAircraftInfo.speed) -> advisory = maintain
		::timeBeforeCollision < 255 -> 
			if
			::timeBeforeCollision > 6 -> advisory = climb; direction.zdir = above;
			::timeBeforeCollision > 6 -> advisory = descend; direction.zdir = below;
			::timeBeforeCollision <= 6 -> advisory = climb_faster; direction.zdir = above; velocity = maxVelocity; 
			fi;
		fi;
	};
}

active [NumAircraft] proctype Aircraft() {
	Point index; // index keeps track of the location of this aircraft in the airspace
	chan recv_chan = [100] of {byte, AircraftInfo}; // recv_chan is owned by this aircraft. {_pid, otherAircraftinformation} of sender aircraft in airspace
	chan otherAircraftRecvChan = [100] of {byte, AircraftInfo}; // otherAircraftRecvChan is a place holder of the recv_chan of another process
	chan RA_chan = [0] of {mtype};
	chan otherAircraftRAChan = [0] of {mtype};

/*1. Assign a random initial location in the airspace for this aircraft*/
	byte randomNum;
	do
	:: (randomNum < 255) -> randomNum++;	
	:: (randomNum > 0) -> randomNum--;
	:: randomNum = (randomNum * randomNum) % 256; //Max in byte
	:: randomNum = 255 - randomNum;		
	:: index.x = randomNum % airspace_xlim;
	:: index.y = randomNum % airspace_ylim;
	:: index.z = randomNum % airspace_zlim; 
	:: (airspace.x[index.x].y[index.y].z[index.z] == 0) -> break
	od;

	airspace.x[index.x].y[index.y].z[index.z] = 1;

/*2. Choose a direction */
	Direction direction;
	do
	::direction.xdir = fwd;
	::direction.xdir = bkd;
	::direction.xdir = stay;
	::direction.ydir = left;
	::direction.ydir = right;
	::direction.ydir = stay;
	::direction.zdir = above;
	::direction.zdir = below;
	::direction.zdir = stay;
	::!(direction.xdir == stay && direction.ydir == stay && direction.zdir == stay) && direction.xdir != 0 && direction.ydir != 0 && direction.zdir != 0 -> break
	od;

/*3. Choose a speed for the aircraft in the scale of 1 to maxVelocity; 1 being slowest and maxVelocity being fastest. 
An aircraft with velocity 1 will take maxVelocity steps to move 1 cell; an aircraft with velocity 2 will take maxVelocity-1 steps to move 1 cell; and so on; 
an aircraft with velocity = maxVelocity will take 1 step to move 1 cell. */
	byte velocity = maxVelocity;
	do
	::velocity > 0 -> velocity--
	::break
	od;

/*
  4.1 Move Aircraft as per speed
  4.2 Send interrogation message 
  4.3 Receive interrogation msgs and Send responses
  4.4 Recieve responses
*/
	byte ctr; // counter to enforce speed
	byte otherAircraftPid;
	AircraftInfo myAircraftInfo, otherAircraftInfo;
	bit otherAircraftInRA, otherAircraftInTA;
	mtype advisory;
	byte timeBeforeCollision = 255;
	do	
	::nempty(interrogation) && !(interrogation?[eval(recv_chan)]) -> 
		interrogation?otherAircraftRecvChan; 
		myAircraftInfo.location.x = index.x;
		myAircraftInfo.location.y = index.y;
		myAircraftInfo.location.z = index.z;
		myAircraftInfo.dir.xdir = direction.xdir;
		myAircraftInfo.dir.ydir = direction.ydir;
		myAircraftInfo.dir.zdir = direction.zdir;
		myAircraftInfo.speed = velocity;
		myAircraftInfo.RAChannel = RA_chan;
		otherAircraftRecvChan!_pid,myAircraftInfo;
		move();

	::recv_chan?otherAircraftPid,otherAircraftInfo; 
	  byte xDist, yDist, zDist, i;
	  computeDistance(RA_proportionality_const * velocity);
	  if
	  ::xDist < airspace_xlim && yDist < airspace_ylim && zDist < airspace_zlim -> otherAircraftInRA = 1
	  ::else -> 
		otherAircraftInRA = 0;
		computeDistance(TA_proportionality_const * velocity);
		if
	  	::xDist < airspace_xlim && yDist < airspace_ylim && zDist < airspace_zlim -> otherAircraftInTA = 1
		::else -> otherAircraftInTA = 0
	  	fi;
	  fi;
	  computeRA();
	  move();
	::otherAircraftInRA == 1 && advisory == climb -> otherAircraftRAChan!descend; move();
	::otherAircraftInRA == 1 && advisory == descend-> otherAircraftRAChan!climb; move();
	::otherAircraftInRA == 1 && advisory == maintain-> otherAircraftRAChan!maintain; move();
	::otherAircraftInRA == 1 && advisory == climb_faster-> otherAircraftRAChan!descend; move();
	::RA_chan?descend;
	  direction.zdir = below;
	  move();
	::RA_chan?climb;
	  direction.zdir = above;
	  move();
	::RA_chan?maintain;
	  move();
	::else -> 
		interrogation!recv_chan; 
	  	move();
	od;
			
}










