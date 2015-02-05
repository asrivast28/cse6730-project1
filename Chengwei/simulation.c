//#include <boost/heap/priority_queue.hpp>

#include <time.h>
#include <stdlib.h>
#include <math.h>
#include "simulationEngine.h"
#include "FIFOQueue.h"
#include <stdio.h>

//using namespace boost::heap;

#define Simulation_time 1000
#define Cross_intersection_time 1     //later modified based on the data collected
#define Travle_On_the_Road_time 3     //later modified based on the data collected

#define Vehicle_capacity_10_11 10     //later modified based on the data collected
#define Vehicle_capacity_11_12 10      
#define Vehicle_capacity_12_13 10
#define Vehicle_capacity_13_14 10

//define the capacity of the road
#define Cap_RD_10_11 10                  //later modified based on the data collected
#define Cap_RD_11_12 10                  //later modified based on the data collected
#define Cap_RD_13_14 10                  //later modified based on the data collected

//////////////////////////
//State Variables/////////
//////////////////////////

// EventKind: The kinds of events
// BridgeState: The state of the bridge
// DrivingDirection: the driving direction of the car
typedef enum { ARRIVAL, ENTER_SEC, TRAVLE_ROAD, DEPARTURE } EventKind;
typedef enum { GREEN_THR, RED_STOP, YELLOW, GREEN_LEFT } INTERSECTION_STATE;    //last two for future test
typedef enum { EMPTY, OCCUPIED }STREET_STATE;   ///for  future test 
typedef enum { Tenth, Eleventh, Twelveth, Thirtheenth, Fourtheenth } Street;     //


//////////////////////////////////////////////////////////////////
////initialization of state variable///
////////////////////////////////////////////////////////

//define the four intersection state
INTERSECTION_STATE intersection_10;
INTERSECTION_STATE intersection_11;
INTERSECTION_STATE intersection_12;
INTERSECTION_STATE intersection_14;

//define the road state
STREET_STATE RD10_11;
STREET_STATE RD11_12;
STREET_STATE RD12_14;


//define the Number of vehicle on the road
int GroupSize_RD_10_11 = 0;
int GroupSize_RD_11_12 = 0;
int GroupSize_RD_12_14 = 0;

//define the Number of vehicle in the waiting section
int GroupSize_Sec_10 = 0;
int GroupSize_Sec_11 = 0;
int GroupSize_Sec_12 = 0;
int GroupSize_Sec_14 = 0;


//// FIFO Queues for 10th and 11th intersection queues
FIFOQueue *_10th_q = NULL;
FIFOQueue *_11th_q= NULL;
FIFOQueue *_12th_q = NULL;
FIFOQueue *_14th_q = NULL;



////////////////////////////////////////////////////////////////
///////////Timeer function to change the state of the ///////
///////////////////////////////////////////////////////////


//for synchronized signal timing 
#define Red_light 10
#define Green_light 10
#define Yellow_light 2  //later for the aggressive behavior


// Define a Vehicle
typedef struct Vehicle {
	int vehID;					     // vehicle ID
	Street Origin_direction;	            //  vehicle's origin direction
	Street Depart_direction;              //   Vehicle's Departure Direction
	double startWaiting;		// time when vehicle starts to wait in the queue
	double endWaiting;			// time when vehicle stops waiting
	double totalWaiting;        //time that vehicle spend on waiting
	double Start_enter;         //time when vehicle begin to travle from original point
	double End_depart;          //time when vehicle departs 
	double Current_position;      //indicate the current position of the vehicle
} Vehicle;



// define the event data structure.
typedef struct EventData {
	EventKind eventKind;
	int Continued_Event;                 //parameter to judge whether it is a continued event or a brand new event
	union {
		struct {
			int vehID;
			Street Origin_direct;
			Street Depart_direct;
		} arrival_Event;

		struct {
			Vehicle *vehicle;
		} entered_sec_Event;

		struct {
			Vehicle *vehicle;
		} travel_rd_Event;
		struct {
			Vehicle *vehicle;
			Street direction;
		} departure_event;
	} eventParam;

} EventData;


//to change the state of the intersection to control the behavior of the traffic
//later could be modified for the unsychronized behavior
void Timer_function(){
	double timer=current_time();
	int segement = timer / 10;
	int residual = segement % 2;
	if ( residual==0)
	{
		intersection_10 = GREEN_THR;
		intersection_11 = GREEN_THR;
		intersection_12 = GREEN_THR;
		intersection_14 = GREEN_THR;
	}
	else
	{
		intersection_10 = RED_STOP;
		intersection_11 = RED_STOP;
		intersection_12 = RED_STOP;
		intersection_14 = RED_STOP;
	}
}

/*
 * Generate a uniform random number in the range [0,1)
 */
double urand(void) {
	double x;
	while ((x = (rand() / (double) RAND_MAX)) >= 1.0);		// loop until x < 1
	return x;
}

///to set the time interval for the vehicle to arrive at the 10th
double randexp(double mean) {
	return (-1 * mean) * (log(1.0 - urand()));
}
#define Arrival_at_10th 5



///get the current intersection that the vehicle is heading towards
INTERSECTION_STATE current_intersection_state;
void Get_current_intersection(Vehicle*v){
	if (v->Current_position == 10){
		current_intersection_state = intersection_10;
	}
	else if (v->Current_position == 11){
		current_intersection_state = intersection_11;
	}
	else if (v->Current_position == 12){
		current_intersection_state = intersection_12;
	}
	else if (v->Current_position == 14){
		current_intersection_state = intersection_14;
	}
}


FIFOQueue *Current_Queue;

//get the corresponding intersection queue 
void Get_current_Queque(Vehicle *v){
	if (v->Current_position == 10){
		Current_Queue =_10th_q;
	}
	else if (v->Current_position == 11){
		Current_Queue = _11th_q;
	}
	else if (v->Current_position == 12){
		Current_Queue = _12th_q;
	}
	else if (v->Current_position == 14){
		Current_Queue = _14th_q;
	}
}


//update the current group size related to the vehicle
void Increase_Group_size(Vehicle *V){
	if (V->Current_position == 11)
		 GroupSize_Sec_11+=1;
	else if (V->Current_position == 12)
		GroupSize_Sec_12+=1;
	else if (V->Current_position == 14)
		GroupSize_Sec_14+=1;
}

//update the current group size related to the vehicle
void Decrease_Group_size(Vehicle *V){
	if (V->Current_position == 11)
		 GroupSize_Sec_11-=1;
	else if (V->Current_position == 12)
		 GroupSize_Sec_12-=1;
	else if (V->Current_position == 14)
		 GroupSize_Sec_14-=1;
}




///////////////////////////////////////////////////////////////
/////////////////Statistics collection function//////////////
/////////////////////////////////////////////////////////////////

int Number_of_vehicle_passed_through = 0;
double Average_waiting_time = 0;
double Average_traveling_time = 0;


/////////////////////////////////////////////////////////////////////
/////Direction Generator//////////////////////////////////////////////////////
////use to generate the entering section and departure section of vehicle////
/////////////////////////////////////////////////////////////////////////////




// Event handlers
void arrival(EventData *e);
void entered(EventData *e);
void travle_rd(EventData *e);                  ///for vehicle comming from in between the road
void departure(EventData *e);



//EventHandler for the event for one single lane , starting from 10th to 14th
void arrival(EventData *arrivalData){

	// create a new vehicle with unique vehicle id
	Vehicle *v = malloc(sizeof(Vehicle));
	if (arrivalData->Continued_Event == 0){
		v->vehID = arrivalData->eventParam.arrival_Event.vehID;			// set vehicle id
		v->Origin_direction = arrivalData->eventParam.arrival_Event.Origin_direct;	// set origin direction
		v->Depart_direction = arrivalData->eventParam.arrival_Event.Depart_direct;
		v->Start_enter = current_time();								// set time vehicle start waiting (now)
		v->Current_position = 10.0;                                            //need to be changed later based on the 
		                                                                       //probablistic distribution
	}
	//if the vehicle comes from previous intersection
	else {
		v = arrivalData->eventParam.departure_event.vehicle;
	}


	//if it is the begining point, could schedule a new arrival event
	// Compute the time-stamp of the new arrival, and only schedule a new arrival
	// if it is less than the maximum allowed time-stamp
	//just for simplicity, assume the vehicle only coming from the beginning point
	if (arrivalData->Continued_Event == 0){
		{
			double ts = current_time() + randexp(Arrival_at_10th);

			if (ts < Simulation_time){
				// create new arrival event
				EventData *newArrival = (EventData *)malloc(sizeof(EventData));
				newArrival->eventKind = ARRIVAL;
				newArrival->eventParam.arrival_Event.vehID = v->vehID + 1;
				newArrival->Continued_Event = 0;
				schedule(ts, newArrival, (void *)arrival);
			}
		}

		// if intersect state is Green_THR and there are no cars in the queue, then the vehicle can immediately
		// enter the intersection,to schedule the entered event

		Timer_function(current_time);
		Get_current_intersection(v);
		Get_current_Queque(v);                        

		if (current_intersection_state == GREEN_THR && fifo_size(Current_Queue) == 0) {

			// the vehicle will have "entered" the intersection in Cross time units
			double ts = current_time() + Cross_intersection_time;

			if (ts < Simulation_time) {

				// create new Entered event to simulate the vehicle entering the intersection
				EventData *entered_intersection = (EventData *)malloc(sizeof(EventData));
				v->endWaiting = current_time();     // vehicle stops waiting in the queue
				v->Current_position += 1;

				entered_intersection->eventKind = ENTER_SEC;
				entered_intersection->Continued_Event = 1;
				entered_intersection->eventParam.entered_sec_Event.vehicle = v;
				schedule(ts, entered_intersection, (void *)entered);
				
				//update the group_size of the intersection
			
				Increase_Group_size(v);
			}
		}
		else{
			v->startWaiting = current_time();
			fifo_push(Current_Queue, v);

		}
		free(arrivalData);
	}

}

void entered(EventData *enter_intersection){
	Timer_function(current_time());
	Get_current_Queque(enter_intersection->eventParam.entered_sec_Event.vehicle);
	if (fifo_size(Current_Queue) > 0 && current_intersection_state == GREEN_THR){
		//schedule another enter event
		Vehicle *v = fifo_pop(Current_Queue);
		v->endWaiting = current_time();
		v->totalWaiting += ((v->endWaiting) - (v->startWaiting));
		v->Current_position += 1;
		double ts = current_time() + Cross_intersection_time;	// compute timestamp of new Entered event

		if (ts < Simulation_time) {

			EventData *newEntered = (EventData *)malloc(sizeof(EventData));

			newEntered->eventKind = ENTER_SEC;
			newEntered->eventParam.entered_sec_Event.vehicle = v;
			newEntered->Continued_Event = 1;
			schedule(ts, newEntered, (void *)entered);

			// increment the size of the group of the intersection
			Increase_Group_size(v);
		}

		//schedule the departure_event
		 ts = current_time() + Travle_On_the_Road_time;
		if (ts < Simulation_time) {
			EventData *DepatureEvent = (EventData*)malloc(sizeof(EventData));
			DepatureEvent->eventKind = DEPARTURE;
			DepatureEvent->eventParam.departure_event.vehicle = enter_intersection->eventParam.entered_sec_Event.vehicle;
			schedule(ts, DepatureEvent, (void*)departure);
		}
		free(enter_intersection);

	}
}

void departure(EventData* departure_data){
	
	Decrease_Group_size(departure_data->eventParam.departure_event.vehicle);
	//if the vehicle has not yet arrive at the final destination,then schedule its next arrival event 

	Vehicle *v = malloc(sizeof(Vehicle));
	v = departure_data->eventParam.departure_event.vehicle;
	if (v->Current_position != 11){
		v->Current_position += 1;                                  //later modification
		//schedule arrival event for the next queque         ///get the vehicle passing to the arrival event  !!!
		double ts = current_time();                                                   /////time 
		if (ts < Simulation_time){
			// create new arrival event
			EventData *newArrival = (EventData *)malloc(sizeof(EventData));
			newArrival->eventKind = ARRIVAL;
			newArrival->eventParam.arrival_Event.vehID = v->vehID;
			newArrival->Continued_Event = 0;
			schedule(ts, newArrival, (void *)arrival);
		}
	}
	///else the vehicle disappears
	else {
		v->End_depart = current_time();
		///collect the data from the vehicle
		Average_traveling_time += ((v->End_depart) - (v->Start_enter));
		Number_of_vehicle_passed_through += 1;
		Average_waiting_time += ((v->endWaiting) - (v->startWaiting));
	}
}

int main(){

	srand((unsigned int)time(NULL));        // seed the random number generator
	// create empty  FIFO queues 
	_10th_q = fifo_create();
	_11th_q = fifo_create();
	//create the current_queue
	Current_Queue = fifo_create();
    //schedule the first arrival event at the 10th intersection
	double Start_time = randexp(Arrival_at_10th);
	EventData *newArrival = (EventData *)malloc(sizeof(EventData));
	newArrival->eventKind = ARRIVAL;
	newArrival->eventParam.arrival_Event.vehID = 0;
	newArrival->Continued_Event = 0;
	schedule(Start_time, newArrival, (void *)arrival);
	run_sim();

	//

    printf("the average travle time is %.4f\n", Average_traveling_time / Number_of_vehicle_passed_through);
	printf("the average Waitingg time is %.4f\n", Average_waiting_time / Number_of_vehicle_passed_through);
  return 0;

}
