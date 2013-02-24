#define airspace_xlim 5
#define airspace_ylim 5
#define airspace_zlim 5
#define NumProcesses 3 /*Number of Aircraft processes to be created*/

/*Air space*/
typedef Z { bit z[airspace_zlim] };
typedef Y { Z y[airspace_ylim] };
typedef AirSpace { Y x[airspace_xlim] };
AirSpace airspace;

/*A point in the air space*/
typedef Point {byte x; byte y; byte z };

/*channel for interrogation*/
chan interrogation = [5] of {chan};

mtype = {fwd, bkd, left, right, above, below, stay};
typedef Direction {mtype xdir; mtype ydir; mtype zdir};

active [NumProcesses] proctype Aircraft() {
	Point index; // index keeps track of the location of this aircraft in the airspace
	chan recv_chan = [5] of {byte, Point}; // recv_chan is owned by this aircraft. {_pid, location} of sender aircraft in airspace
	chan otherAircraftRecvChan = [5] of {byte, Point}; // otherAircraftRecvChan is a place holder of the recv_chan of another process

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

/*
  3.1 Send interrogation message 
  3.2 Receive interrogation msgs and Send responses
  3.3 Recieve responses
*/
	byte otherAircraftPid;
	Point otherAircraftlocation;
	do
	::/*Move aircraft in its direction*/
	  atomic {
		airspace.x[index.x].y[index.y].z[index.z] = 0; // unsetting occuancy in current location
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
		
		airspace.x[index.x].y[index.y].z[index.z] = 0; //setting occupancy in the new location
	  }
	::interrogation!recv_chan
	::nempty(interrogation) && !(interrogation?[eval(recv_chan)]) -> 
		interrogation?otherAircraftRecvChan; 
		otherAircraftRecvChan!_pid,index;
	::recv_chan?otherAircraftPid,otherAircraftlocation
	od;
			
}










