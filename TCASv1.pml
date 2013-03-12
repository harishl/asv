#define airspace_xlim 6
#define airspace_ylim 6
#define airspace_zlim 6
#define NumAircraft 12 /*Number of Aircraft processes to be created*/
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
mtype = {climb, climb_faster, descend, descend_faster, maintain, traffic, collided}; 
typedef Direction {mtype xdir; mtype ydir; mtype zdir};
typedef AircraftInfo {Point location; Direction dir; byte speed; chan AdvChannel; };

inline move() {
	speed_ctr = (speed_ctr+1)%(maxVelocity - velocity + 1);
	if
	::speed_ctr == 0 -> 
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
		::else
		fi;
		// movement in y direction
		if
		::direction.ydir == left && index.y < (airspace_ylim-1) -> index.y++ 
		::direction.ydir == left && index.y == (airspace_ylim-1) -> index.y = 0
		::direction.ydir == left && index.y == (airspace_ylim-1) -> direction.ydir = right; index.y--
		::direction.ydir == right && index.y > 0 -> index.y--
		::direction.ydir == right && index.y == 0 -> index.y = (airspace_ylim-1)
		::direction.ydir == right && index.y == 0 -> direction.ydir = left; index.y++
		::else
		fi;
		// movement in z direction
		if
		::direction.zdir == above && index.z < (airspace_zlim-1) -> index.z++ 
		::direction.zdir == above && index.z == (airspace_zlim-1) -> index.z = 0
		::direction.zdir == above && index.z == (airspace_zlim-1) -> direction.zdir = below; index.z--
		::direction.zdir == below && index.z > 0 -> index.z--
		::direction.zdir == below && index.z == 0 -> index.z = (airspace_zlim-1)
		::direction.zdir == below && index.z == 0 -> direction.zdir = above; index.z++
		::else
		fi;
	
		airspace.x[index.x].y[index.y].z[index.z] = 1; //setting occupancy in the new location
		}
	::else
	fi;
}

inline computeDistance(numCells) {
	  //compute distance along x axis
	  xDist = 255; // infinity (ideally)
	  i = 0;
	  do
	  ::i <= numCells && otherAircraftInfo.location.x != (index.x+i)%airspace_xlim -> i++;
	  ::i <= numCells && otherAircraftInfo.location.x == (index.x+i)%airspace_xlim -> xDist = i; break; 
	  ::else -> break
	  od;
	  i = 0;
	  do // inverted index search for the other side
	  ::i <= numCells && (airspace_xlim - otherAircraftInfo.location.x - 1) != (airspace_xlim - index.x - 1 + i)%airspace_xlim -> i++;
	  ::i <= numCells && (airspace_xlim - otherAircraftInfo.location.x - 1) == (airspace_xlim - index.x - 1 + i)%airspace_xlim -> xDist = i; break;
	  ::else -> break
	  od;
	
	  //compute distance along y axis
	  yDist = 255; // infinity (ideally)
	  i = 0;
	  do
	  ::i <= numCells && otherAircraftInfo.location.y != (index.y+i)%airspace_ylim -> i++;
	  ::i <= numCells && otherAircraftInfo.location.y == (index.y+i)%airspace_ylim -> yDist = i; break;
	  ::else -> break
	  od;
	  i = 0;
	  do
	  ::i <= numCells && (airspace_ylim - otherAircraftInfo.location.y - 1) != (airspace_ylim - index.y - 1 + i)%airspace_ylim -> i++;
	  ::i <= numCells && (airspace_ylim - otherAircraftInfo.location.y - 1) == (airspace_ylim - index.y - 1 + i)%airspace_ylim -> yDist = i; break;
	  ::else -> break
	  od;

	  //compute distance along z axis
	  zDist = 255; // infinity (ideally)
	  i = 0;
	  do
	  ::i <= numCells && otherAircraftInfo.location.z != (index.z+i)%airspace_zlim -> i++;
	  ::i <= numCells && otherAircraftInfo.location.z == (index.z+i)%airspace_zlim -> zDist = i; break;
	  ::else -> break
	  od;
	  i = 0;
	  do
	  ::i <= numCells && (airspace_zlim - otherAircraftInfo.location.z - 1) != (airspace_zlim - index.z - 1 + i)%airspace_zlim -> i++;
	  ::i <= numCells && (airspace_zlim - otherAircraftInfo.location.z - 1) == (airspace_zlim - index.z - 1 + i)%airspace_zlim -> zDist = i; break;
	  ::else -> break
	  od;
}

inline computeRA() {
	if
	::direction.zdir != above -> advisory = climb; direction.zdir = above;
	::direction.zdir != below -> advisory = descend; direction.zdir = below;
	::direction.zdir != above -> advisory = climb_faster; direction.zdir = above; velocity = maxVelocity; 
	::direction.zdir != below -> advisory = descend_faster; direction.zdir = below; velocity = maxVelocity; 
	fi;
	
	if
	::index.x == otherAircraftInfo.location.x && index.y == otherAircraftInfo.location.y && index.z == otherAircraftInfo.location.z -> advisory = collided;/*Detect collision*/
	::else
	fi;
}

active [NumAircraft] proctype Aircraft() {
	Point index; // index keeps track of the location of this aircraft in the airspace
	chan recv_chan = [100] of {byte, AircraftInfo}; // recv_chan is owned by this aircraft. {_pid, otherAircraftinformation} of sender aircraft in airspace
	chan otherAircraftRecvChan = [100] of {byte, AircraftInfo}; // otherAircraftRecvChan is a place holder of the recv_chan of another process
	chan Advisory_chan = [0] of {mtype};

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

/* Main loop
  4.1 Move Aircraft as per speed
  4.2 Send interrogation message 
  4.3 Receive interrogation msgs and Send responses
  4.4 Recieve responses and compute and send RA decision if required
  4.5 Receive RA message from other aircraft and act on it
*/
	byte speed_ctr; // counter to enforce speed
	byte otherAircraftPid;
	AircraftInfo myAircraftInfo, otherAircraftInfo;
	bit otherAircraftInRA, otherAircraftInTA;
	mtype advisory;
	
Loop:	do	
	::nempty(interrogation) && !(interrogation?[eval(recv_chan)]) -> 
		interrogation?otherAircraftRecvChan; 
		myAircraftInfo.location.x = index.x;
		myAircraftInfo.location.y = index.y;
		myAircraftInfo.location.z = index.z;
		myAircraftInfo.dir.xdir = direction.xdir;
		myAircraftInfo.dir.ydir = direction.ydir;
		myAircraftInfo.dir.zdir = direction.zdir;
		myAircraftInfo.speed = velocity;
		myAircraftInfo.AdvChannel = Advisory_chan;
		otherAircraftRecvChan!_pid,myAircraftInfo;
		move();

	::recv_chan?otherAircraftPid,otherAircraftInfo; 
	  byte xDist, yDist, zDist, i;
	  computeDistance(RA_proportionality_const * velocity);
	  if
	  ::xDist < airspace_xlim && yDist < airspace_ylim && zDist < airspace_zlim -> 
		computeRA();
		otherAircraftInRA = 1; 
		
	  ::else -> 
		otherAircraftInRA = 0;
		computeDistance(TA_proportionality_const * velocity);
		if
	  	::xDist < airspace_xlim && yDist < airspace_ylim && zDist < airspace_zlim -> otherAircraftInTA = 1; advisory = traffic;
		::else -> otherAircraftInTA = 0
	  	fi;
	  fi;
	  assert (! (otherAircraftInRA) || (xDist<=(RA_proportionality_const*velocity) && yDist<=(RA_proportionality_const*velocity) && zDist<=(RA_proportionality_const*velocity)));
	  assert (! (otherAircraftInTA) || (xDist<=(TA_proportionality_const*velocity) && yDist<=(TA_proportionality_const*velocity) && zDist<=(TA_proportionality_const*velocity)));
	  move();
	::interrogation!recv_chan; move();
	::otherAircraftInRA == 1 && advisory == climb		-> 	otherAircraftInfo.AdvChannel!descend; move(); goto Loop
	::otherAircraftInRA == 1 && advisory == descend		->	otherAircraftInfo.AdvChannel!climb; move(); goto Loop
	::otherAircraftInRA == 1 && advisory == maintain	-> 	otherAircraftInfo.AdvChannel!maintain; move(); goto Loop
	::otherAircraftInRA == 1 && advisory == climb_faster	-> 	otherAircraftInfo.AdvChannel!descend; move(); goto Loop
	::otherAircraftInRA == 1 && advisory == collided 	-> 	otherAircraftInfo.AdvChannel!collided; /* Mayday! Mayday! ... and Boom! */
	::otherAircraftInTA == 1 && advisory == traffic 	-> 	otherAircraftInfo.AdvChannel!traffic; move(); goto Loop
	::Advisory_chan?descend; direction.zdir = below; move(); goto Loop
	::Advisory_chan?climb; direction.zdir = above; move(); goto Loop
	::Advisory_chan?maintain; move(); goto Loop
	::Advisory_chan?traffic; move(); goto Loop
	::Advisory_chan?collided; /* Mayday! Mayday! ... and Boom! */
	od;
			
}











