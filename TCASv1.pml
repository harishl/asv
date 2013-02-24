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

active [NumProcesses] proctype Aircraft() {
	Point index; // index keeps track of the location of this aircraft in the airspace
	chan recv_chan = [5] of {byte, Point}; // recv_chan is owned by this aircraft. {_pid, location} of sender aircraft in airspace
	chan otherAircraftRecvChan = [5] of {byte, Point}; // otherAircraftRecvChan is a place holder of the recv_chan of another process

/*1. Assign a random initial location in the airspace for this aircraft*/
	byte randomNum;
	do
	:: randomNum++	
	:: (randomNum > 0) -> randomNum--		
	:: index.x = randomNum % airspace_xlim;
	:: index.y = randomNum % airspace_ylim;
	:: index.z = randomNum % airspace_zlim; 
	:: (airspace.x[index.x].y[index.y].z[index.z] == 0) -> break
	od;

	airspace.x[index.x].y[index.y].z[index.z] = 1;

/*
  2.1 Send interrogation message 
  2.2 Receive interrogation msgs and Send responses
  2.3 Recieve responses
*/
	byte otherAircraftPid;
	Point otherAircraftlocation;
	do
	:: interrogation!recv_chan
	:: nempty(interrogation) && !(interrogation?[eval(recv_chan)]) -> 
		interrogation?otherAircraftRecvChan; 
		otherAircraftRecvChan!_pid,index;
	:: recv_chan?otherAircraftPid,otherAircraftlocation
	od;
			
}










