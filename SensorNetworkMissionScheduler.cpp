//Programmer: DANIEL FUCHS
//Course: Algorithms, CS2500-A
//Date: 10/24/15
//
// - Sensor Network Mission Scheduling Project
//Description: This program simulates a sensor network that is being used
//             to complete a particular set of tasks, or "missions".
//             This program evaluates multiple different algorithmic
//             approaches with regards to how well they make use of the
//             network's resources.
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <vector>
using namespace std;

//////////////////////////////////////////////////////////////////////////
///////////////////////////GLOBAL_CONSTANTS///////////////////////////////
//////////////////////////////////////////////////////////////////////////

const int MSV = 50; //Maximum Variance in Mission Start time
const int AOI_W = 50; //Largest X-Coord within Area of Interest
const int AOI_H = 50; //Largest Y-Coord within Area of Interest
const int M_RAD = 5; //Mission's Range from Epicenter for Sensor Detection
const int M_COUNT = 1000; //Number of missions
const int NUM_TEST = 3; //Amount of times primary loop will run before
//averaging all results together.

//////////////////////////////////////////////////////////////////////////
//////////////////////////SPECIAL_FUNCTIONS///////////////////////////////
//////////////////////////////////////////////////////////////////////////

//PLACEMENT FUNCTION
//Description: Used to generate a random float between 0 and coord_max.
//Value is stored in the passed float. More formally, c = (0, coord_max)
//after this function runs. Thus, this function can be used to "place"
//Mission Epicenters and Sensors by being passed the respective bound of
//the AoI and then the coordinate to be generated. 
void place(const int coord_max, float & c)
{
  int z = 0;
  while(z == 0 || z == RAND_MAX) //Ensures z is on (0, RAND_MAX)
  {
    z = rand();
  }
  c = (static_cast<float>(z)/static_cast<float>(RAND_MAX))+(rand()%coord_max);
  //After we ensure z follows the interval (0, RAND_MAX), we divide it by
  //RAND_MAX. So, z will now be on the interval of (0, 1). We then add that
  //value to an integer that follows the interval [0, coord_max], and then
  //store the result into c. Therefore, c will be some float on the interval 
  //(0, coord_max).
  return;
}

//////////////////////////////////////////////////////////////////////////
/////////////////////////////////CLASSES//////////////////////////////////
//////////////////////////////////////////////////////////////////////////

/////////////
///MISSION///
/////////////

class Mission
{
  private:
    int m_s; //Mission Start
    int m_e; //Mission End
    float m_ex; //Mission Epicenter's X coordinate
    float m_ey; //Mission Epicenter's Y coordinate
    
  public:
    bool attempted; //Stores if mission was attempted to have been completed
        
    //CONSTRUCTOR
    Mission(int start_time, int duration)
    {
      attempted = false;
      m_s = start_time;
      m_e = m_s + duration;
      place(AOI_W, m_ex); //Generates a valid x-coordinate on (0, AOI_W)
      place(AOI_H, m_ey); //Generates a valid y-coordinate on (0, AOI_H)
    }
    
    //INRANGE FUNCTION
    //Description: Using the constant M_RAD as the radius of the mission's
    //range, tests to see if some pair of coordinates is within this
    //range. In other words, it tests to see if the Euclidean Distance
    //of the passed coordinates is < M_RAD.
    bool inRange(const float x, const float y)const
    {
      float xdist = m_ex - x; //Stores the difference in x
      float ydist = m_ey - y; //Stores the difference in y
      float EuD_squared = (xdist*xdist)+(ydist*ydist); //Calculates the
      //Euclidean Distance squared.
      if((M_RAD*M_RAD) >= EuD_squared) //If the EuD is < M_RAD
      {
        return true; //The passed coordinates are "in range"
      }
      else //The EuD is greater than M_RAD
      {
        return false; //The coordinates are not "in range"
      }
    }
    
    //ACCESSOR FUNCTIONS
    int getSTART()const{ return m_s; }
    int getEND()const{ return m_e; }
    float getEX()const{ return m_ex; }
    float getEY()const{ return m_ey; }
    
};
      
//////////////
///SCHEDULE///
//////////////

//This object can store a start time and end time. This object is contained
//in sensors, and is used to determine when they are "busy".

class Schedule
{
  public:
    int s_start; //Stores beginning of some time period
    int s_end; //Stores end of some time period
    
    //CONSTRUCTOR
    Schedule(int s, int e){ s_start = s; s_end = e; }
    
};

////////////
///SENSOR///
////////////

class Sensor
{
  private:
    int m_energy; //Energy level of sensor
    float m_sx; //Sensor's X coordinate
    float m_sy; //Sensor's Y coordinate
    int s_num_assigned; //Stores amount of missions sensor was assigned to
    vector<Schedule> time_plan; //Stores the time that the sensor is busy
    //on missions. Used to evaluate if new missions can be accepted without
    //overlapping with other missions.
    
  public:
  
    //DEFAULT CONSTRUCTOR
    Sensor()
    {
      m_energy = 1000; //All sensors begin with 1000 energy units
      place(AOI_W, m_sx); //Generates a valid x-coordinate on (0, AOI_W)
      place(AOI_H, m_sy); //Generates a valid y-coordinate on (0, AOI_H)
      s_num_assigned = 0;
    }
    
    //SCHEDULABLE FUNCTION
    //Description: Used to see if a mission can fit into the schedule of
    //a sensor. Also checks to see if sensor is within range and has enough
    //energy.
    bool isSchedulable(const Mission & task)
    {
      int res_sta = 0; //Start time restriction
      int res_end = 0; //End time restriction
      int sta = task.getSTART(); //Stores mission start time
      int end = task.getEND();   //Stores mission end time
      int e_cost = end - sta; //Calculates energy cost
      
      if (m_energy >= e_cost && task.inRange(m_sx, m_sy))
      { //If there is enough energy and the sensor is in range
        for (int i = 0; i < s_num_assigned; i++) //Tests for overlap with
        { //previous schedules.
          res_sta = time_plan[i].s_start;
          res_end = time_plan[i].s_end;
          //Res_sta and res_end form the interval over which a previous
          //mission occurs. Therefore, sta and end cannot overlap with
          //this interval if the mission is to be assignable.
          if (sta < res_end && end > res_sta) //If the incoming mission
          { //overlaps with a restricted boundary
            return false;
          }
        }  
        return true;
      }
      else
      {
        return false;
      }
    }
    
    //ENSCHEDULE FUNCTION
    //Description: Used to see if a sensor is capable of being assigned to
    //a mission, and if it is, assigns it to that mission.
    void enschedule(Mission & task)
    {
      if(isSchedulable(task)) //Ensures that mission can be assigned
      {
        Schedule* temp = new Schedule(task.getSTART(), task.getEND());
        time_plan.push_back(*temp); //Adds schedule to sensor's time_plan
        s_num_assigned++; //Increments amount of missions assigned
        int e_cost = task.getEND() - task.getSTART();
        m_energy = m_energy - e_cost;
        delete temp;
        temp = NULL;
      }      
      return;
    }

    //RESET FUNCTION
    //Description: Resets a sensor.
    void resetSensor()
    {
      m_energy = 1000;
      for (int k = 0; k < s_num_assigned; k++)
      {
        time_plan.pop_back();
      }
      s_num_assigned = 0;
    }
    
    //ACCESSOR FUNCTIONS
    float getSX()const{ return m_sx; }
    float getSY()const{ return m_sy; }
    int getENERGY()const{ return m_energy; }

};
    
/////////////
///NETWORK///
/////////////

class Network
{
  private:
    int m_num; //Number of sensors
    int m_mission_count; //Number of missions processed by Network
    int m_mission_satis; //Number of missions satisfied by Network
    vector<Sensor> m_sensor; //Vector of sensors
    
  public:
    
    //DEFAULT CONSTRUCTOR
    Network(int sensor_count)
    {
      m_mission_count = 0;
      m_mission_satis = 0;
      m_num = sensor_count;
      Sensor* temp;
      for (int i = 0; i < sensor_count; i++)
      {
        temp = new Sensor;
        m_sensor.push_back(*temp);
        delete temp;
      }
      temp = NULL;
    }
    
    //CLEAR FUNCTION
    //Description: Clears a Network completely, removing all sensors.
    void clearNetwork()
    {
      m_mission_count = 0;
      m_mission_satis = 0;
      for (int i = 0; i < m_num; i++)
      {
        m_sensor.pop_back();
      }
      m_num = 0;
    }
   
    //RESET FUNCTION
    //Description: Resets a Network back to its original state. This
    //retains all sensors used, and resets all of them as well.
    void resetNetwork()
    {
      m_mission_count = 0;
      m_mission_satis = 0;
      for (int i = 0; i < m_num; i++)
      {
        m_sensor[i].resetSensor();
      }
      return;
    }
    
    //ADD SENSORS FUNCTION
    //Description: Adds additional sensors to the network. Resets Network
    //automatically.
    void addSensors(const int sensor_count)
    {
      resetNetwork();
      m_num = m_num + sensor_count;
      Sensor* temp;
      for (int i = 0; i < sensor_count; i++)
      {
        temp = new Sensor;
        m_sensor.push_back(*temp);
        delete temp;
      }
      temp = NULL;
      return;
    }
    
    //ACCESSOR FUNCTIONS    
    int getNUM(){ return m_num; }
    int getMISSIONCOUNT(){ return m_mission_count; } 
    int getMISSIONSATIS(){ return m_mission_satis; }

    //COUNT SCHEDULABLE FUNCTION
    //Description: Used to count the number of sensors that are able to
    //satisfy the "isSchedulable" function with regards to some mission.
    int countSched(Mission & task)
    {
      int count_sch = 0; //Used to store amount of assignable sensors
      for (int i = 0; i < m_num; i++)
      {
        if (m_sensor[i].isSchedulable(task)) //If a sensor can be assigned
        {
          count_sch++; //Amount of assignable sensors incremented
        }
      }
      return count_sch;
    }

    //LOW-ENERGY-SENSOR COUNTING FUNCTION
    //Description: Returns the number of sensors that can no longer complete
    //missions.
    float calcLES(const int dura)const
    {
      int num_low_energy = 0;
      for (int i = 0; i < m_num; i++)
      {
        if (m_sensor[i].getENERGY() < dura)
        { //If the sensor has than "dura" units of energy left
          num_low_energy++;
        }
      }
      return num_low_energy;
    }       
    
    //RANDOM ASSIGNMENT FUNCTION
    //Description: This function first assesses whether or not a mission
    //can be completed by the Network's sensors. If it can be completed,
    //the function begins picking random sensors. If the chosen sensor
    //can be assigned to the mission, it is. Otherwise, the function
    //picks a new sensor. After the mission has had sufficient sensors
    //assigned, it is marked as satisfied.
    void randomAssign(Mission & task, const int  num_sensors)
    {
      task.attempted = true; //The mission is marked as attempted
      if (countSched(task) >= num_sensors) //If there are enough sensors
      { //available for scheduling, then mission can be assigned.
        int index = 0;
        int num_assigned = 0; //Stores sensors assigned
        while (num_assigned < num_sensors) //While more sensors are needed
        {
            index = rand()%m_num; //Choose random sensor
            if (m_sensor[index].isSchedulable(task))
            {
              m_sensor[index].enschedule(task);
              num_assigned++;
            }
        }
        m_mission_satis++; //Increments amount of missions satisfied
      }
      m_mission_count++; //Another mission attempt is recorded
      return;      
    }

    //MISSION ASSIGNMENT FUNCTION
    //Description: This function is used to assign a mission to a Network.
    //It first marks the mission as attempted, and then if there are enough
    //sensors available to complete the mission, then the sensor with the
    //most energy is assigned to the mission at every iteration until the
    //amount of sensors needed is reached. The network then records it as a
    //satisfied mission.
    void missionAssign(Mission & task, const int  num_sensors)
    {
      task.attempted = true; //The mission is marked as attempted
      if (countSched(task) >= num_sensors)
      {
        int c_en = 0; //Stores energy of evaluated sensor
        int c_max = 0; //Stores maximum found value amongst sensors
        int MES_index = 0; //Stores index of max energy sensor
        int num_assigned = 0; //Stores number of sensors assigned
        while (num_assigned < num_sensors) //While more sensors are needed
        {
          c_max = 0;
          MES_index = 0;
          for (int k = 0; k < m_num; k++) //For all sensors
          {
            c_en = m_sensor[k].getENERGY();
            if (m_sensor[k].isSchedulable(task) && c_en > c_max)
            {
              c_max = c_en;
              MES_index = k;
            }
          }
          m_sensor[MES_index].enschedule(task);
          num_assigned++;
        }
        m_mission_satis++; //Increments amount of missions satisfied
      }
      m_mission_count++; //Another mission attempt is recorded
      return;      
    }

    //TOTAL ENERGY CALCULATION FUNCTION
    //Description: Used in the Offline Algorithm. Let us define the "TE", or
    //Total Energy, of a mission as the sum of all remaining energy on all
    //sensors that the mission would use if it were accepted. So, this function
    //takes a mission and the amount of sensors needed as parameters, and then
    //sums up all the energy amongst the sensors that would be assigned were
    //that mission to be satisfied.
    int calcTE(Mission & task, const int num_sensors)
    {
      if (countSched(task) < num_sensors) //If there aren't enough sensors
      { //available for scheduling, TE must be zero.
        return 0;
      }
      int TE = 0; //Total energy of across all sensors that would be used for a
      //mission.
      int num_chosen = 0; //Number of chosen sensors
      vector<int> chosen_sensors; //Stores chosen sensor indexes
      for (int k = 0; k < num_sensors; k++) //Finds required amount of sensors
      {
        int max_energy = 0; //Greatest energy value of avail. sensors
        int temp = 0; //Stores current sensor's energy
        int maxTE_index = 0; //Stores index of greatest energy sensors
        for (int i = 0; i < m_num; i++) //Find maximum energy
        {    
          temp = 0;
          bool repeated_index = false; //Stores if index i was already chosen
          for (int h = 0; h < num_chosen; h++) //Tests if i was chosen
          {
            if (chosen_sensors[h] == i) //If i is an index already chosen
            {
              repeated_index = true;
            }
          }
          if (!repeated_index) //If this index is not already chosen
          {
            temp = m_sensor[i].getENERGY();
            if (m_sensor[i].isSchedulable(task) && temp > max_energy)
            { //Sensor can be assigned and it has more energy than max_energy
                max_energy = temp; //Update max_energy
                maxTE_index = i; //Stores index of max_energy
            }
          }
        }
        max_energy = 0;
        chosen_sensors.push_back(maxTE_index); //Marks sensor index as chosen
        num_chosen++;
      }
      for (int p = 0; p < num_sensors; p++)
      {
        TE += m_sensor[chosen_sensors[p]].getENERGY();
        //Adds total energy of chosen sensor to total
      }
      return TE;
    }      
};  



//////////////////////////////////////////////////////////////////////////
/////////////////////////////MAIN_PROGRAM/////////////////////////////////
//////////////////////////////////////////////////////////////////////////

int main()
{
  /*-----DECLARATIONS-----*/
  srand(time(NULL)); //Seeds random number generation
  vector<Mission> List; //An vector that stores all of the missions.
  Network WSN(0); //The WSN, which contains all the sensors.

  int randomSA = 0;  //The three variables on the left are used to store the
  int onlineSA = 0;  //Satisfaction Average, or the average amount of missions
  int offlineSA = 0; //that were satisfied each iteration.
  
  int randomLEA = 0; //These three variables store the percentage of low-energy
  int onlineLEA = 0; //sensors for the three algorithms on average after each
  int offlineLEA = 0;//iteration.
    
  int index = 0; //Current mission being considered
  
  /*-----PARAMETER INPUT-----*/
  int input;
  cout << "Enter the Mission Duration: ";
  cin >> input;
  int M_DURATION = input;
  cout << endl << "Enter the Number of Sensors: ";
  cin >> input;
  int NUM_SENSORS = input;
  cout << endl << "Enter the Sensors required per Mission: ";
  cin >> input;
  int REQ_SENS = input;
  cout << endl << endl;

  /*-----PROGRAM BEGIN-----*/
  for (int iterations = 0; iterations < NUM_TEST; iterations++)
  {
    /*-----NETWORK GENERATION-----*/
    WSN.addSensors(NUM_SENSORS);
     
    /*-----MISSION LIST GENERATION-----*/
    int st = 0; //Stores start time of last mission
    int dur = 0; //Stores duration
    Mission* mstor; //Stores missions
    for (int i = 0; i < M_COUNT; i++) //Mission Generation
    {
      st += (rand()%MSV);
      dur = M_DURATION;
      mstor = new Mission(st, dur);
      List.push_back(*mstor);
      delete mstor;
    }
    mstor = NULL;

    /*-----RANDOM APPROACH-----*/
    while (index < M_COUNT) //While there are more missions
    {
      WSN.randomAssign(List[index], REQ_SENS); //Attempts to complete mission
      index++;
    }
    randomSA += WSN.getMISSIONSATIS(); //Adds to total satisfied missions
    randomLEA += WSN.calcLES(M_DURATION);

    /*-----DATA PREP-----*/
    index = 0;
    WSN.resetNetwork();
    for (int q = 0; q < M_COUNT; q++)
    {
      List[q].attempted = false;
    }

    /*-----ONLINE APPROACH-----*/
    while (index < M_COUNT) //While there are more missions
    {
      WSN.missionAssign(List[index], REQ_SENS); //Attempts to complete mission
      index++;
    }
    onlineSA += WSN.getMISSIONSATIS(); //Adds to total satisfied missions
    onlineLEA += WSN.calcLES(M_DURATION);
    
    /*-----DATA PREP-----*/
    index = 0;
    WSN.resetNetwork();
    for (int q = 0; q < M_COUNT; q++)
    {
      List[q].attempted = false;
    }

    /*-----OFFLINE APPROACH-----*/
    int offcount = 0; //Used to count missions completed by offline algorithm
    while (offcount < M_COUNT) //While there are more missions
    {
      index = 0; //Index is reset
      int maxTE = 0; //Stores calculated max TE
      for (int i = 0; i < M_COUNT; i++) //This loop finds the maximum TE
      {
        int currentTE = WSN.calcTE(List[i], REQ_SENS); //Stores current TE
        if (currentTE >= maxTE && List[i].attempted == false)
        { //If the currentTE exceeds maxTE and that mission hasn't been chosen
          maxTE = currentTE; //Max TE updated
          index = i; //Index of mission with MAX TE updated
        }
      }
      WSN.missionAssign(List[index], REQ_SENS); //Attempts to complete mission
      offcount++; //Additional mission attempt recorded
    }
    offlineSA += WSN.getMISSIONSATIS(); //Adds to total satisfied missions
    offlineLEA += WSN.calcLES(M_DURATION);

    /*-----DATA CLEANUP-----*/
    for (int i = 0; i < M_COUNT; i++) //Mission Generation
    {
      List.pop_back();
    }
    WSN.clearNetwork(); //Deletes all sensors, resets data.  
  }  

  /*-----RESULTS-----*/
  float avg; //Used to output the calculated averages.
  avg = static_cast<float>(randomSA)/(static_cast<float>((NUM_TEST*M_COUNT)));
  cout << "The Random Algorithm's Satisfaction Rate was " << avg*100 << "%";
  cout << endl;
  avg = static_cast<float>(onlineSA)/(static_cast<float>((NUM_TEST*M_COUNT)));
  cout << "The Online Algorithm's Satisfaction Rate was " << avg*100 << "%";
  cout << endl;
  avg = static_cast<float>(offlineSA)/(static_cast<float>((NUM_TEST*M_COUNT)));
  cout << "The Offline Algorithm's Satisfaction Rate was " << avg*100 << "%";
  cout << endl;
  avg = static_cast<float>(randomLEA)/
        (static_cast<float>((NUM_TEST*NUM_SENSORS)));
  cout << "The Random Algorithm's Low Energy Percent was " << avg*100 << "%";
  cout << endl;
  avg = static_cast<float>(onlineLEA)/
        (static_cast<float>((NUM_TEST*NUM_SENSORS)));
  cout << "The Online Algorithm's Low Energy Percent was " << avg*100 << "%";
  cout << endl;
  avg = static_cast<float>(offlineLEA)/
        (static_cast<float>((NUM_TEST*NUM_SENSORS)));
  cout << "The Offline Algorithm's Low Energy Percent was " << avg*100 << "%";
  cout << endl;
  
  return 0;
}

