/*
* Huyen Tran
* COMP372
* Project 1 - GPS Navigation
* I have neither given nor received unauthorized aid on this program
*/

#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <map>
#include <cmath>
#include <limits>
#include <thread>
#include <chrono>

#include "pqueue.h"

using namespace std;

struct Location{
  string locID;
  double latitude;
  double longtitude;
};

struct Road{
  string startID;
  string endID;
  int speedLimit;
  string name;
};

struct Node{
  string state; //node's location ID
  string goal;
  string pLocID; //parent location ID
  string road;
  double g; //actual cost
  double h; //heuristic or estimated
  double f; //total g+h
};

map <string, Location> locations; // map from ID to location
map <string, vector <Road> > roads;
//split string based on char
vector<string> split(const string &s, char delim)
{
  stringstream ss(s);  // this requires #include <sstream>
  string item;
  vector<string> elems;
  while (getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

double getDist(Location a, Location b){
  double dist;
  double degrees_to_radians = M_PI/180;
  double phi1 = (90 - a.latitude)*degrees_to_radians;
  double phi2 = (90 - b.latitude)*degrees_to_radians;

  double theta1 = a.longtitude*degrees_to_radians;
  double theta2 = b.longtitude*degrees_to_radians;

  double arc = acos(sin(phi1)*sin(phi2)*cos(theta1 - theta2) +
  cos(phi1)*cos(phi2));
  dist = arc * 3960;
  return dist;
}
double driveTime(Location a, Location b, int sl){
  double time;
  double dist;
  dist = getDist(a, b);
  time = (dist*3600)/sl;
  return time;
}

void printNode(Node n){
  cout << "[State=" << n.state << ", parent=" << n.pLocID <<
  ", g=" << n.g << ", h=" << n.h << ", f=" << n.f << "]";
}

vector <Node> Expand(Node s, Location goal){
  vector <Node> cNodes; //child nodes
  Node c;
  c.pLocID = s.state;

  //for all roads of that location,
  vector <Road> locRoads = roads.find(s.state)->second;
  for(int i = 0; i < locRoads.size(); i++){
    Location a = locations.find(s.state)->second;
    Location b = locations.find(locRoads[i].endID)->second;
    c.state = b.locID;
    c.road = locRoads[i].name;
    c.g = s.g + driveTime(a, b, locRoads[i].speedLimit);
    c.h = driveTime(b, goal, 65);
    c.f = c.g + c.h;
    cNodes.push_back(c);
  }
  return cNodes;
}

double getBearing(Location a, Location b){
  double lat1 = a.latitude * M_PI/180;
  double lat2 = b.latitude * M_PI/180;
  double long1 = a.longtitude * M_PI/180;
  double long2 = b.longtitude * M_PI/180;

  double y = sin(long2-long1) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(long2-long1);
  double angle = atan2(y, x);
  double bearing = (int)(angle * 180/M_PI + 360) % 360;
  return bearing;
}

string getDir(double bearing){
  //determine direction
  string dir;
  if(bearing >= 337.5 && bearing <= 22.5 ){
    dir = "north";
  }
  else if (bearing >= 22.5 && bearing <= 67.5){
    dir = "northeast";
  }
  else if (bearing >= 67.5 && bearing <= 112.5){
    dir = "east";
  }
  else if (bearing >= 112.5 && bearing <= 157.5){
    dir = "southeast";
  }
  else if (bearing >= 157.5 && bearing <= 202.5){
    dir = "south";
  }
  else if (bearing >= 202.5 && bearing <= 247.5){
    dir = "southwest";
  }
  else if (bearing >= 247.5 && bearing <= 292.5){
    dir = "west";
  }
  else {
    dir = "northwest";
  }
  return dir;
}

void GPS(vector <Node> route){
  //provide initial direction + name of road
  string dir; //direction
  double bearing;
  string turn;
  double allDist = 0;   //total distance on the same road
  double allTime = 0;
  string cRoad; //current road
  bool initial = true;
  Location a,b;

  for (int i = 0; i < route.size(); i++){
    //handles the initial direction
    if(initial){
      cRoad = route[i+1].road;
      a = locations.find(route[i].state)->second;
      b = locations.find(route[i+1].state)->second;
      bearing = getBearing(a, b);
      dir = getDir(bearing);
      while(cRoad == route[i+1].road){
        a = locations.find(route[i].state)->second;
        b = locations.find(route[i+1].state)->second;
        double time = route[i+1].g - route[i].g;
        double currDist = getDist(a, b);
        allDist += currDist;
        allTime += time;
        cRoad = route[i+1].road;
        i++;
      }
      cout << "Head " << dir << " on " << cRoad << endl;
      cout << "     Drive for " << setprecision(2) << fixed << allDist
                  << " miles (" << allTime << " seconds)" << endl;

      initial = false;
    }
    //all roads after initial are turns
    else{
      //determines if it's a right or left turn
      a = locations.find(route[i-2].state)->second;
      b = locations.find(route[i-1].state)->second;
      bearing = getBearing(a,b);
      a = locations.find(route[i-1].state)->second;
      b = locations.find(route[i].state)->second;
      double afterBearing = getBearing(a, b);
      if(bearing > afterBearing && (bearing-afterBearing < 180)){
        turn = "left";
      }
      else{
        turn = "right";
      }

      //last 2 roads, where the two roads are not the same name
      if(i == route.size()-2 && route[i].road != route[i+1].road){
        a = locations.find(route[i].state)->second;
        b = locations.find(route[i+1].state)->second;
        double time = route[i+1].g - route[i].g;
        double currDist = getDist(a, b);
        allDist += currDist;
        allTime += time;
        cRoad = route[i+1].road;
        //print turn info
        cout << "Turn " << turn << " onto " << cRoad << endl;
        cout << "     Drive for " <<  setprecision(2) << fixed << allDist << " miles (" << allTime << " seconds)" << endl;
        i++;
      }
      else{
      //handles last road with a diff name to this current road travel dist and time
      //since our while loop doesn't handle this condition
        cRoad = route[i].road;
        a = locations.find(route[i].state)->second;
        b = locations.find(route[i-1].state)->second;
        double time = route[i].g - route[i-1].g;
        double currDist = getDist(a, b);
        allDist += currDist;
        allTime += time;

        //all travel dist +  time on this road
        while(cRoad == route[i+1].road){
          a = locations.find(route[i].state)->second;
          b = locations.find(route[i+1].state)->second;
          double time = route[i+1].g - route[i].g;
          double currDist = getDist(a, b);
          allDist += currDist;
          allTime += time;
          cRoad = route[i+1].road;
          i++;
        }
        //print turn info
        cout << "Turn " << turn << " onto " << cRoad << endl;
        cout << "     Drive for " <<  setprecision(2) << fixed << allDist << " miles (" << allTime << " seconds)" << endl;
      }
    }
    //reset sums for that road
    allDist = 0;
    allTime = 0;
  }
  //final
  cout << "You have arrive!" << endl;

}

void printOutput(int ncount, Node f, map<string, Node> v){
  vector <Node> route;
  cout << endl;
  cout << "Total travel time in seconds: " << f.f << endl;
  cout << "Number of nodes visited: " << ncount << endl << endl;

  //keep tracing back to the start node (where the parent is null)
  while(!f.pLocID.empty()){
    route.insert(route.begin(), f);
    f = v.find(f.pLocID)->second;
  }
  route.insert(route.begin(), f);

  //print out route
  cout << "Route found is:\n";
  for(int i = 0; i < route.size(); i++){
    cout << route[i].state;
    cout << " (" << route[i].road << ")" << endl;
  }
  cout << endl;

  //get GPS directions
  GPS(route);

}

Node runAStar(Location start, Location end, bool debug){
  int ncount = 1; //count number of visited nodes
  map <string,Node> visited;
  Node init;
  init.state = start.locID;
  init.road = "starting location";
  init.g = 0;
  init.h = driveTime(start, end, 65);
  init.f = init.g + init.h; //max mph to calc heustric

  //frontier with only inital node
  pqueue<Node, double> frontier;
  frontier.enqueue(init, init.f);
  //reached maps states to nodes
  map<string, Node> reached;
  reached.insert(pair<string , Node>(init.state, init));

  while(!frontier.empty()){
    //removes lowest cost node from the frontier
    Node s = frontier.dequeue();
    //print node if debug is true
    if(debug){
      cout << "Visiting ";
      printNode(s);
      cout << endl;
    }

    //reached our goal, return
    if (s.state == end.locID){
      printOutput(ncount, s, visited);
      return s;
    }
    vector <Node> expand = Expand(s, end);
    //for each child in expand
    string cstate = expand[0].state; //child state
    Node child = expand[0];
    for(int i = 0; i < expand.size(); i++){
      string cstate = expand[i].state; //child state
      Node child = expand[i];
      //if s is not in reached or child.path-cost < reached[s].path-cost
      if(reached.find(cstate) == reached.end()
      || (child.f < reached.find(cstate)->second.f))
      {
          reached.insert(pair<string, Node> (cstate, child));
          frontier.enqueue(child, child.f);
          if(debug){
            cout << "     Adding ";
            printNode(child);
            cout << " to frontier. \n";
          }
        }
      else{
        if(debug){
          cout << "     Skipping";
          printNode(child);
          cout << endl;
        }
      }
    }
  visited.insert(pair<string, Node>(s.state, s));
  ncount++;
  }
  return init;
}

int main(){
  fstream newfile;
  string file;
  vector <Road> allR;

  //ask user to input file name
  cout << "Enter a filename: ";
  cin >> file;

  //open file and read from it
  newfile.open(file, ios::in);
  if(newfile.is_open()){
      string s;
      while(getline(newfile, s)){
        vector<string> line =  split(s, '|');

        //if it's a location, add location info as an vertex in graph
        if(line[0] == "location"){
          Location loc;
          loc.locID = line[1];
          loc.latitude = stod(line[2]);
          loc.longtitude = stod(line[3]);
          //insert location elements to map
          locations.insert(pair<string, Location>(loc.locID, loc));
        }

        //if it's a road, add an edge between the 2 locations
        else{
          Road r;
          r.startID = line[1];
          r.endID = line[2];
          r.speedLimit = stoi(line[3]);
          r.name = line[4];

          //road going other way
          Road r2;
          r2.startID = line[2];
          r2.endID = line[1];
          r2.speedLimit = stoi(line[3]);
          r2.name = line[4];

          //start loc doesn't exist and end loc doesn't exist
          if(roads.find(r.startID) == roads.end() && roads.find(r.endID) == roads.end()){
            roads.insert(pair<string, vector <Road> >(r.startID, allR));
            roads.insert(pair<string, vector <Road> >(r.endID, allR));
            roads.find(r.startID)->second.push_back(r);
            roads.find(r.endID)->second.push_back(r2);

          }

          //start loc doesn't exist but end loc exist
          else if(roads.find(r.startID) == roads.end() && roads.find(r.endID) != roads.end()){
            roads.insert(pair<string, vector <Road> >(r.startID, allR));
            //roads.insert(pair<string, vector <Road> >(r.endID, allR));
            roads.find(r.startID)->second.push_back(r);
            roads.find(r.endID)->second.push_back(r2);
          }

          //start loc exist but end loc doesn't exist
          else if(roads.find(r.startID) != roads.end() && roads.find(r.endID) == roads.end()){
            roads.find(r.startID)->second.push_back(r);
            roads.insert(pair<string, vector <Road> >(r.endID, allR));
            roads.find(r.endID)->second.push_back(r2);
          }

          //both locations exist
          else{
            roads.find(r.startID)->second.push_back(r);
            roads.find(r.endID)->second.push_back(r2);
          }
        }
      }
      newfile.close();
  }

  cout << endl;

  //get user input
  cout << "Enter starting location ID: ";
  string slocID;
  cin >> slocID;
  cout << "Enter ending location ID: ";
  string elocID;
  cin >> elocID;
  cout << "Do you want debugging information (y/n)? ";
  string debug;
  cin >> debug;

  //create frontier
  Location start = locations.find(slocID)->second;
  Location end = locations.find(elocID)->second;

  if(debug == "y"){
    runAStar(start, end, true);
  }
  else{
    runAStar(start, end, false);
  }

  return 0;
}
