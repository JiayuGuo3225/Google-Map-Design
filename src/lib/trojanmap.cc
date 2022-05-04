#include "trojanmap.h"
// #include "mapui.h"
// using namespace std;
#include <random>
#include <time.h>
#include <chrono>

//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(const std::string& id) {
    if(data.count(id) == 1){
      return data[id].lat;
    }
    else{
      return -1;
    }
}

/**
 * GetLon: Get the longitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(const std::string& id) { 
    if(data.count(id) == 1){
      return data[id].lon;
    }
    else{
      return -1;
    }
}

/**
 * GetName: Get the name of a Node given its id. If id does not exist, return "NULL".
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(const std::string& id) { 
    if(data.count(id) == 1){
      return data[id].name;
    }
    else{
      return "";
    }
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node. If id does not exist, return an empty vector.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(const std::string& id) {
    if(data.count(id) == 1){
      return data[id].neighbors;
    }
    else{
      return {};
    }
}

/**
 * GetID: Given a location name, return the id. 
 * If the node does not exist, return an empty string. 
 *
 * @param  {std::string} name          : location name
 * @return {int}  : id
 */
std::string TrojanMap::GetID(const std::string& name) {
  std::string res;
  for(auto n : data){
    if(n.second.name == name){
      res = n.first;
      return res;
    }
  }
  return "";
}

/**
 * GetPosition: Given a location name, return the position. If id does not exist, return (-1, -1).
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  std::pair<double, double> res;
  for(auto n : data){
    if(n.second.name == name){
      res.first = n.second.lat;
      res.second = n.second.lon;
      return res;
    }
  }
  std::pair<double, double> results(-1, -1);
  return results;
}


/**
 * CalculateEditDistance: Calculate edit distance between two location names
 * 
 */
int TrojanMap::CalculateEditDistance(std::string a, std::string b){
    int m = a.size();
    int n = b.size();
    int R[m+1][n+1];
    for(auto i=0;i<=m;i++){
      R[i][0] = i;
    }
    for(auto j=0;j<=n;j++){
      R[0][j] = j;
    }
    for(auto i=1;i<=m;i++){
      for(auto j=1;j<=n;j++){
        if(a[i-1]==b[j-1]){
          R[i][j] = R[i-1][j-1];
        }
        else{
          R[i][j] = 1 + std::min(R[i-1][j-1],std::min(R[i][j-1],R[i-1][j]));
        }
      }
    }
    return R[m][n];
}

/**
 * FindClosestName: Given a location name, return the name with smallest edit distance.
 *
 * @param  {std::string} name          : location name
 * @return {std::string} tmp           : similar name
 */
std::string TrojanMap::FindClosestName(std::string name) {
  if(name.size()==0){
    return "";
  }
  int min_distance = INT_MAX;
  std::string min_name;
  for(auto n:data){
    int temp = CalculateEditDistance(name,n.second.name);
    if(temp < min_distance){
      min_distance = temp;
      min_name = n.second.name;
    }
  }
  return min_name;
}


/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix. The function should be case-insensitive.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name){ 
  if(name.empty()!= true){
    name.erase(0,name.find_first_not_of(' '));
    name.erase(name.find_last_not_of(' ')+1);
  }
  if(name.size()==0){
    return {""};
  }
  else{
    std::vector<std::string> results;
    std::transform(name.begin(),name.end(),name.begin(),::tolower);
    for(auto n : data){
      std::string current = n.second.name;
      std::transform(current.begin(),current.end(),current.begin(),::tolower);
      if(current.find(name) != current.npos && current.find(name) == 0){
        results.push_back(n.second.name); 
      }
    }
    return results;
  }
}


/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {std::string} a  : a_id
 * @param  {std::string} b  : b_id
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const std::string &a_id, const std::string &b_id) {
  // Do not change this function
  Node a = data[a_id];
  Node b = data[b_id];
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2),2.0) + cos(a.lat * M_PI / 180.0) * cos(b.lat * M_PI / 180.0) * pow(sin(dlon / 2),2.0);
  double c = 2 * asin(std::min(1.0,sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  // Do not change this function
  double sum = 0;
  for (int i = 0;i < int(path.size())-1; i++) {
    sum += CalculateDistance(path[i], path[i+1]);
  }
  return sum;
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Use priority queue.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
  std::string location1_name, std::string location2_name) {
  std::vector<std::string> path;
  std::priority_queue<std::pair<double,std::string>, std::vector<std::pair<double,std::string>>, std::greater<std::pair<double,std::string>>> q;
  std::unordered_map<std::string,double> distance;
  std::unordered_map<std::string,std::string> prev;
  std::string start = GetID(location1_name);
  std::string end = GetID(location2_name);
  for(auto node:data){
    distance.insert({node.first,DBL_MAX});
  }
  distance[start] = 0;
  q.push(std::make_pair(0,start));
  while(!q.empty()){
    std::string curr = q.top().second;
    q.pop();
    //std::map<std::string, Node>::iterator iter;
    for(auto neighbor:GetNeighborIDs(curr)){
      neighbor = neighbor.substr(0,10);
      double neighborDist = CalculateDistance(neighbor,curr);

      if(distance[neighbor] > distance[curr] + neighborDist){
        distance[neighbor] = distance[curr] + neighborDist;
        prev[neighbor] = curr;
        q.push(std::make_pair(distance[neighbor], neighbor));
      }
    }
  }
  std::string toPath = end;
  if(prev.find(toPath)!=prev.end()){
    while(toPath!=start){
      path.push_back(toPath);
      toPath = prev.at(toPath);
    }
    path.push_back(start);
    std::reverse(path.begin(),path.end());
  }
  return path;
}


/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Do the early termination when there is no change on distance.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
  std::string location1_name, std::string location2_name){
  std::vector<std::string> path;
  std::unordered_map<std::string,double> distance;
  std::string start;
  std::string end;
  std::unordered_map<std::string,std::string> pre;
  int len = data.size();
  for(auto &i:data){
    distance[i.first] = INT_MAX;
    if(i.second.name == location1_name){
      start = i.first;
    }
    if(i.second.name == location2_name){
      end = i.first;
    }
  }
  distance[start] = 0;
  for(auto i=0;i<len-1;i++){
    int flag = 0;
    for(auto &v:data){
      std::string node = v.first;
      for(auto &neigh:v.second.neighbors){
        double d = sqrt(pow(v.second.lat-data[neigh].lat,2)+pow(v.second.lon-data[neigh].lon,2));
        if(distance[node]>distance[neigh]+d){
          distance[node] = distance[neigh]+d;
          pre[node] = neigh;
          flag =1;
        }
      }
    }
    if(flag == 0){
      break;
    }
  }
  if(pre.find(end)!=pre.end()){
    std::string temp = end;
    while(temp!=start){
      path.push_back(temp);
      temp = pre[temp];
    }
    path.push_back(start);
    std::reverse(path.begin(),path.end());
  }
  return path;
}

/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Brute_force(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  std::vector<std::vector<std::string>> res_vec;
  std::vector<std::string> optimal_path;
  double path_length = INT_MAX; //minimal length
  location_ids.push_back(location_ids[0]); //route needs to go back to the starting point
  //backtrack will totally be invoked O(n!), push_back O(n). tatal:O(n*n!)
  Brute_Force_helper(location_ids, res_vec, 1, path_length, optimal_path);
  res_vec.push_back(optimal_path);
  records = std::make_pair(path_length, res_vec);
  return records;
}

void TrojanMap::Brute_Force_helper(std::vector<std::string> &points, std::vector<std::vector<std::string>> &res, 
                          int current, double &path_length, std::vector<std::string> &optimal_path){
  //reference - lc46
  //stable state
  if (current == points.size()-1){
    double temp_length = CalculatePathLength(points);
    if (temp_length < path_length){
      path_length = temp_length;
      optimal_path = points;
      res.push_back(points); //records O(n)
    }
    return;
  }
  for (int i=current; i<points.size()-1; i++){
    std::swap(points[current], points[i]); //O(1), swap two elements
    Brute_Force_helper(points, res, current+1, path_length, optimal_path);
    std::swap(points[current], points[i]); //revoke swap
  }
}
  
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Backtracking(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  std::vector<std::vector<std::string>> res_vec;
  std::vector<std::string> optimal_path;
  //std::vector<std::string> current_path;
  double path_length = INT_MAX, cur_length = 0; //minimal length
  //current_path.push_back(location_ids[0]);
  //location_ids.push_back(location_ids[0]); //route needs to go back to the starting point
  //backtrack will totally be invoked O(n!), push_back O(n). tatal:O(n*n!)
  //backtrack(location_ids, res_vec, pathlen, optimal_path, current_path);
  Backtracking_helper(location_ids, res_vec, 1, cur_length, path_length, optimal_path);
  res_vec.push_back(optimal_path);
  records = std::make_pair(path_length, res_vec);
  return records;
}

void TrojanMap::Backtracking_helper(std::vector<std::string> &points, std::vector<std::vector<std::string>> &res, 
                          int current, double &cur_length, double &path_length, std::vector<std::string> &optimal_path){
  //stable state
  if (current == points.size()){
    double templen = CalculateDistance(points[current-1], points[0]); //it's a circle!
    if (templen+cur_length < path_length){
      path_length = templen + cur_length;
      optimal_path = points;
      optimal_path.push_back(points[0]);
      res.push_back(optimal_path); //records O(n)
    }
    return;
  }
  for (int i=current; i<points.size(); i++){
    double temp_length2 = CalculateDistance(points[current-1], points[i]);
    if (cur_length + temp_length2 < path_length){ //early backtrack
      cur_length += temp_length2; //add the next distance
      std::swap(points[current], points[i]); //O(1), swap two elements
      Backtracking_helper(points, res, current+1, cur_length, path_length, optimal_path);
      std::swap(points[current], points[i]); //revoke swap
      cur_length -= temp_length2; //revoke the add
    }
  }
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;
  location_ids.push_back(location_ids[0]);
  std::vector<std::vector<std::string>> res_vec; //all the path
  std::vector<std::string> optimal_path = location_ids;
  int len=location_ids.size();
  double current_dist = CalculatePathLength(location_ids);
  double path_length = current_dist;
  while (true){
    int flag = 0;
    for (int i=1; i<len-2; i++){
      for (int k=i+1; k<len-1; k++){
        std::reverse(location_ids.begin()+i, location_ids.begin()+k+1);
        current_dist = CalculatePathLength(location_ids);
        if (current_dist < path_length){
          path_length = current_dist;
          optimal_path = location_ids;
          flag = 1;
          res_vec.push_back(location_ids);
        }
        else{std::reverse(location_ids.begin()+i, location_ids.begin()+k+1);}
      }
    }
    if (flag == 0){break;}
  }
  res_vec.push_back(optimal_path);
  records = std::make_pair(path_length, res_vec);
  return records;
}

// 3-opt TSP 
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_3opt(
      std::vector<std::string> &location_ids){
  // reference1: http://tsp-basics.blogspot.com/2017/03/3-opt-move.html 
  // reference2: http://tsp-basics.blogspot.com/2017/03/3-opt-iterative-general-idea.html

  std::pair<double, std::vector<std::vector<std::string>>> results;
  std::vector<std::vector<std::string>> res_all; //all the path
  if (location_ids.size() == 2){
    location_ids.push_back(location_ids[0]);
    results.second.push_back(location_ids);
    results.first = CalculatePathLength(results.second[0]);
  } 
  int optCase;
  int N = location_ids.size();
  double gain_length; //the minimal length of the path
  std::string X1, X2, Y1, Y2, Z1, Z2;
  std::string start = location_ids[0];
  int locallyOptimal = 1;
  int i;
  int j;
  int k;
  // according to the reference--3opt general
  while (locallyOptimal){
    locallyOptimal = 0;
    for (auto counter_1=0; counter_1<=N-1; counter_1++){
      i = counter_1;
      X1 = location_ids[i];
      X2 = location_ids[(i+1) % N];

      for (int counter_2=1; counter_2<= N-3; counter_2++){
        j = (i+counter_2) % N;
        Y1 = location_ids[j];
        Y2 = location_ids[(j+1)%N];

        for (int counter_3=counter_2+1; counter_3<=N-1; counter_3++){
          k = (i+counter_3) % N;
          Z1 = location_ids[k];
          Z2 = location_ids[(k+1)%N];

          for (auto optCase=0; optCase<=7; optCase++){
            gain_length = Gain_from_3opt(X1, X2, Y1, Y2, Z1, Z2, optCase);
            // if the gain is more than 1e-15, update the path
            if (gain_length > 1e-15){
              Move_3_opt(location_ids, i,j,k,optCase,N);
              res_all.push_back(location_ids);
              res_all[res_all.size()-1].push_back(location_ids[0]);
              locallyOptimal = 1;
              break;
            }
            
          }
          if(locallyOptimal == 1){
            break;
          }
        }
        if(locallyOptimal == 1){
          break;
        }
      }
      if(locallyOptimal == 1){
        break;
      }
    }
  }
  int count = 0;
  for (auto n:location_ids){
    if (n==start){
      break;
    }
    count ++;
  }
  std::vector<std::string> final_res;
  for (int i=count; i<count+N; i++){
    final_res.push_back(location_ids[i%N]);
  }
  final_res.push_back(start);
  res_all.push_back(final_res);
  results = std::make_pair(CalculatePathLength(final_res), res_all);
  return results;
}

// get the id's index in the location(original)
int TrojanMap::Tourindex(std::vector<std::string> &location_ids, std::string name_id){
  for (auto i=0; i<location_ids.size(); i++){
    if (location_ids[i] == name_id){
      return i;
    }
  }
  return 0;
}

// Reverse the location, but remember to check the start index is smaller than end index
void TrojanMap::Reverse_segment(std::vector<std::string> &locationids, int start_index, int end_index){
  std::reverse(locationids.begin()+start_index,locationids.begin()+end_index);
}

//move function of 3-opt--according to the 3opt move reference
void TrojanMap::Move_3_opt(std::vector<std::string> &location_ids, int i, int j, int k, int opt3_case, int M){
  int x1,x2,y1,y2,z1,z2;
  int N = location_ids.size();
  switch (opt3_case)
  {
  case 0:
    break;
  case 1:
    if((k+1)%N<=i){
      Reverse_segment(location_ids,(k+1)%N,i+1);
    }
    else{
      Reverse_segment(location_ids,(i+1)%N,(k+1)%N);
    }
    break;

  case 2:
    if((j+1)%N<=k){
      Reverse_segment(location_ids,(j+1)%N,(k+1));
    }
    else{
      Reverse_segment(location_ids,(k+1)%N,(j+1)%N);
    }
    break;

  case 3:
    if((i+1)%N<=j){
      Reverse_segment(location_ids,(i+1)%N,j+1);
    }
    else{
      Reverse_segment(location_ids,(j+1)%N,(i+1)%N);
    }
    break;

  case 4:
    if((j+1)%N<=k){
      Reverse_segment(location_ids,(j+1)%N,k+1);
    }
    else{
      Reverse_segment(location_ids,(k+1)%N,(j+1)%N);
    }
    if((i+1)%N<=j){
      Reverse_segment(location_ids,(i+1)%N,j+1);
    }
    else{
      Reverse_segment(location_ids,j,(i+1)%N+1);
    }
    break;

  case 5:
    if((k+1)%N<=i){
      Reverse_segment(location_ids,(k+1)%N,i+1);
    }
    else{
      Reverse_segment(location_ids,(i+1)%N,(k+1)%N);
    }
    if((i+1)%N<=j){
      Reverse_segment(location_ids,(i+1)%N,j+1);
    }
    else{
      Reverse_segment(location_ids,j,(i+1)%N+1);
    }
    break;

  case 6:
    if((k+1)%N<=i){
      Reverse_segment(location_ids,(k+1)%N,i+1);
    }
    else{
      Reverse_segment(location_ids,(i+1)%N,(k+1)%N);
    }
    if((j+1)%N<=k){
      Reverse_segment(location_ids,(j+1)%N,k+1);
    }
    else{
      Reverse_segment(location_ids,k,(j+1)%N+1);
    }
    break;

  case 7:
    if((k+1)%N<=i){
      Reverse_segment(location_ids,(k+1)%N,i+1);
    }
    else{
      Reverse_segment(location_ids,(i+1)%N,(k+1)%N);
    }
    if((i+1)%N<=j){
      Reverse_segment(location_ids,(i+1)%N,j+1);
    }
    else{
      Reverse_segment(location_ids,j,(i+1)%N+1);
    }
    if((j+1)%N<=k){
      Reverse_segment(location_ids,(j+1)%N,k+1);
    }
    else{
      Reverse_segment(location_ids,k,(j+1)%N+1);
    }
    break;
  }
}

//gain function of 3-opt which is according to the reference
double TrojanMap::Gain_from_3opt(std::string X1, std::string X2, std::string Y1, std::string Y2, 
                  std::string Z1, std::string Z2, int opt_Case){
  double del_Length, add_Length; 
  switch (opt_Case)
  {
  case 0:
    return 0;
  case 1:
    del_Length = CalculateDistance(X1, X2) + CalculateDistance(Z1, Z2);
    add_Length = CalculateDistance(X1, Z1) + CalculateDistance(X2, Z2);
    break;
  case 2:
    del_Length = CalculateDistance(Y1, Y2) + CalculateDistance(Z1, Z2);
    add_Length = CalculateDistance(Y1, Z1) + CalculateDistance(Y2, Z2);
    break;
  case 3:
    del_Length = CalculateDistance(X1, X2) + CalculateDistance(Y1, Y2);
    add_Length = CalculateDistance(X1, Y1) + CalculateDistance(X2, Y2);
    break;
  case 4:
    del_Length = CalculateDistance(X1,X2) + CalculateDistance(Y1,Y2) + CalculateDistance(Z1,Z2);
    add_Length = CalculateDistance(X1,Y1) + CalculateDistance(X2,Z1) + CalculateDistance(Y2,Z2);
    break;
  case 5:
    del_Length = CalculateDistance(X1,X2) + CalculateDistance(Y1,Y2) + CalculateDistance(Z1,Z2);
    add_Length = CalculateDistance(X1,Z1) + CalculateDistance(Y2,X2) + CalculateDistance(Y1,Z2);
    break;
  case 6:
    del_Length = CalculateDistance(X1,X2) + CalculateDistance(Y1,Y2) + CalculateDistance(Z1,Z2);
    add_Length = CalculateDistance(X1,Y2) + CalculateDistance(Z1,Y1) + CalculateDistance(X2,Z2);
    break;
  case 7:
    del_Length = CalculateDistance(X1,X2) + CalculateDistance(Y1,Y2) + CalculateDistance(Z1,Z2);
    add_Length = CalculateDistance(X1,Y2) + CalculateDistance(Z1,X2) + CalculateDistance(Y1,Z2);
    break;
  }
  double result = del_Length-add_Length;
  return result; 
}

/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 *
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations 
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(std::string locations_filename){
  std::vector<std::string> location_names_from_csv;
  std::fstream fin;
  fin.open(locations_filename, std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      location_names_from_csv.push_back(word);
    }
  }
  fin.close();
  return location_names_from_csv;
}


/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 *
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(std::string dependencies_filename){
  std::vector<std::vector<std::string>> dependencies_from_csv;
  std::string father_location;
  std::string child_location;
  std::fstream fin;
  fin.open(dependencies_filename, std::ios::in);
  std::string line, word;
  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);
    std::vector<std::string> tem;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      if (count == 0){
        father_location = word;
        tem.push_back(father_location);
      }
      else if (count == 1){
        child_location = word;
        tem.push_back(child_location);
      }
      count+=1;
    }
    dependencies_from_csv.push_back(tem);
  }
  fin.close();
  return dependencies_from_csv;
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a sorting of nodes
 * that satisfies the given dependencies. If there is no way to do it, return a empty vector.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan(std::vector<std::string> &locations,
                                                     std::vector<std::vector<std::string>> &dependencies){
  std::vector<std::string> result;
  std::unordered_multiset<std::string> que;
  std::map<std::string, std::vector<std::string>> adj;
  std::map<std::string, int> indegree;
  std::map<std::string, bool> visited;
  bool flag = false;
  int count = 0;
  // initialize
  for(auto n:locations){
    indegree[n] = 0;
    visited[n] = false;
  }
  // get the adjacency list and get the indegree
  for(auto i=0;i<dependencies.size();i++){
    std::string pre_requ = dependencies[i][0];
    adj[pre_requ].push_back(dependencies[i][1]);
    indegree[dependencies[i][1]] += 1;
  }
  // initialize the queue and if there exists a circle return an empty vector
  for(auto n:locations){
    if(indegree[n]==0){
      que.insert(n);
      visited[n]=true;
    }
  }
  for(auto n:indegree){
    if(n.second != 0){
      flag = true;
    }
  }
  if(flag == false){
    return {};
  }
  // topological sort
  while (que.empty()!=true)
  {
    std::string cur = *que.begin();
    que.erase(que.begin());
    result.push_back(cur);
    for(auto n:adj[cur]){
      indegree[n] -= 1;
      if(indegree[n] ==0 && visited[n] == false ){
        visited[n]==true;
        que.insert(n);
      }
    }
  }
  return result;                                                     
}

/**
 * inSquare: Give a id retunr whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square) {
  if(GetLon(id) >= square[0] && GetLon(id) <= square[1] && GetLat(id) <= square[2] && GetLat(id) >= square[3]){
    return true;
  }
  return false;
}

/**
 * GetSubgraph: Give four vertexes of the square area, return a list of location ids in the squares
 *
 * @param  {std::vector<double>} square         : four vertexes of the square area
 * @return {std::vector<std::string>} subgraph  : list of location ids in the square
 */
std::vector<std::string> TrojanMap::GetSubgraph(std::vector<double> &square) {
  // include all the nodes in subgraph
  std::vector<std::string> subgraph;
  double longitude_left = square[0];
  double longitude_right = square[1];
  double latitude_up = square[2];
  double latitude_down = square[3];
  for(auto n:data){
    if(n.second.lon >= longitude_left && n.second.lon <= longitude_right && n.second.lat >= latitude_down && n.second.lat <= latitude_up){
      subgraph.push_back(n.first);
    }
  }
  return subgraph;
}

/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true if there
 * is a cycle path inside the square, false otherwise.
 * 
 * @param {std::vector<std::string>} subgraph: list of location ids in the square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square) {
  std::unordered_map<std::string,bool> visited;
  for(auto i = 0; i<subgraph.size(); i++){
    for(auto n:subgraph){
      visited[n] = false;
    }
    bool flag = false;
    std::string cur_id = subgraph[i];
    std::string par_id = cur_id;
    hasCycle(cur_id,visited,par_id,square,flag);
    if(flag == true){
      return true;
    }
  }
  return false;
}
void TrojanMap::hasCycle(std::string cur_id,std::unordered_map<std::string,bool>&visited, std::string par_id,std::vector<double> &square,bool &flag){
  if(inSquare(cur_id,square)==true){
    if(visited[cur_id]==true){
      flag = true;
      // PlotPath();
      return;
    }
    else{
      visited[cur_id] = true;
    }
  }
  else{
    return;
  }
  for(auto n:GetNeighborIDs(cur_id)){
    if(n != par_id){
      hasCycle(n,visited,cur_id,square,flag);
    }
  }
}
/**
 * FindNearby: Given a class name C, a location name L and a number r, 
 * find all locations in class C on the map near L with the range of r and return a vector of string ids
 * 
 * @param {std::string} className: the name of the class
 * @param {std::string} locationName: the name of the location
 * @param {int} r: search radius
 * @param {int} k: search numbers
 * @return {std::vector<std::string>}: location name that meets the requirements
 */

bool Location::operator<(const Location& rhs)const{
        return dis < rhs.dis;
}

std::vector<std::string> TrojanMap::FindNearby(std::string attributesName, std::string name, double r, int k) {
  std::vector<std::string> res;
  std::priority_queue<Location> que;
  std::string cur_id = GetID(name);
  for(const auto &n:data){
    if(n.second.id==cur_id){
      continue;
    }
    if(n.second.attributes.count(attributesName)>0){
      double cur_distance = CalculateDistance(n.second.id,cur_id);
      if(cur_distance<=r){
        if(que.size()<k){
          que.push({n.second.id,cur_distance});
        } 
        if(que.size()>=k && cur_distance<que.top().dis){
          que.pop();
          que.push({n.second.id,cur_distance});
        }
      }
    } 
  }
  while (que.size()!=0){
    auto n = que.top();
    que.pop();
    res.push_back(n.id);
  }
  std::reverse(res.begin(),res.end());
  return res;
}

/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile() {
  // Do not change this function
  std::fstream fin;
  fin.open("src/lib/data.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        if (isalpha(word[0]))
          n.attributes.insert(word);
        if (isdigit(word[0]))
          n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}