#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"

// Test the Autocomplete
// Test the lower case
TEST(TrojanMapStudentTest1, Test1) {
  TrojanMap m;
  auto names = m.Autocomplete("cha");
  std::unordered_set<std::string> gt = {"Chase", "Chase Plaza Heliport"}; // groundtruth for "Ch"
  int success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
}
// test the upper case
TEST(TrojanMapStudentTest1, Test2) {
  TrojanMap m;
  auto names = m.Autocomplete("CHA");
  std::unordered_set<std::string> gt = {"Chase", "Chase Plaza Heliport"}; // groundtruth for "Ch"
  int success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
}
// test the space and the lower and upper case
TEST(TrojanMapStudentTest1, Test3) {
  TrojanMap m;
  auto names = m.Autocomplete(" Cha ");
  std::unordered_set<std::string> gt = {"Chase", "Chase Plaza Heliport"}; // groundtruth for "Ch"
  int success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
}


// Test the Getposition
// Test Subway
TEST(TrojanMapTest2, Test1) {
  TrojanMap m;
  auto position = m.GetPosition("Subway");
  std::pair<double, double> gt1(34.0118217, -118.2822980); // groundtruth for "Subway"
  EXPECT_EQ(position, gt1);
}
// Test Burger King
TEST(TrojanMapTest2, Test2) {
  TrojanMap m;
  auto position = m.GetPosition("Burger King");
  std::pair<double, double> gt1(34.0034629, -118.2824020); // groundtruth for "Burger King"
  EXPECT_EQ(position, gt1);
}
// Test the null
TEST(TrojanMapTest2, Test3) {
  TrojanMap m;
  auto position = m.GetPosition(" ");
  std::pair<double, double> gt1(-1, -1); // groundtruth for null
  EXPECT_EQ(position, gt1);
}

// Test CalculateEditDistance function
// test house to ros
TEST(TrojanMapTest3, CalculateEditDistance1) {
  TrojanMap m;
  EXPECT_EQ(m.CalculateEditDistance("house", "ros"), 3);
}
// Test extend to extension
TEST(TrojanMapTest3, CalculateEditDistance2) {
  TrojanMap m;
  EXPECT_EQ(m.CalculateEditDistance("extend", "extension"), 4);
}
// Test rose to rise
TEST(TrojanMapTest3, CalculateEditDistance3) {
  TrojanMap m;
  EXPECT_EQ(m.CalculateEditDistance("rose", "rise"), 1);
}

// Test FindClosestName function
TEST(TrojanMapTest4, FindClosestName1) {
  TrojanMap m;
  EXPECT_EQ(m.FindClosestName("Chick-fil"), "Chick-fil-A");
}
TEST(TrojanMapTest4, FindClosestName2) {
  TrojanMap m;
  EXPECT_EQ(m.FindClosestName("Ros"), "Ross");
}
TEST(TrojanMapTest4, FindClosestName3) {
  TrojanMap m;
  EXPECT_EQ(m.FindClosestName("KDC"), "KFC");
}

// Test the CalculateShortestPath_Dijkstra
// Test ralphs to target
TEST(TrojanMapTest5, CalculateShortestPath_Dijkstra1) {
  TrojanMap m;
  auto path = m.CalculateShortestPath_Dijkstra("Ralphs", "Target");
  std::vector<std::string> gt{
  "2578244375","4380040154","4380040158","4380040167","6805802087","8410938469","6813416131","7645318201",
  "6813416130","6813416129","123318563","452688940","6816193777","123408705","6816193774","452688933",
  "452688931","123230412","6816193770","6787470576","4015442011","6816193692","6816193693","6816193694",
  "4015377691","544693739","6816193696","6804883323","6807937309","6807937306","6816193698","4015377690",
  "4015377689","122814447","6813416159","6813405266","4015372488","4015372487","6813405229","122719216",
  "6813405232","4015372486","7071032399","4015372485","6813379479","6813379584","6814769289","5237417650"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  // Reverse the input
  path = m.CalculateShortestPath_Dijkstra("Target", "Ralphs");
  std::reverse(gt.begin(),gt.end()); // Reverse the path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}
// Test Shall Gas to Rite Aid
TEST(TrojanMapTest5, CalculateShortestPath_Dijkstra2) {
  TrojanMap m;
  auto path = m.CalculateShortestPath_Dijkstra("Shall Gas", "Rite Aid");
  std::vector<std::string> gt{
  "3577173133", "1873056046", "72092030", "123699398", "1873055883", "1873055895", "72092041", 
  "250874495", "1778692886", "250874494", "1873055917", "1873055960", "250874498", "2199140379", 
  "1873055966", "1873055921", "1873055992", "67619040", "1732340082", "4012693767", "1873056002", 
  "6790384659", "1732340072", "1873055991", "9053766706", "3573974854", "3573974879", "3573974877", 
  "3577173167"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  // Reverse the input
  path = m.CalculateShortestPath_Dijkstra("Rite Aid", "Shall Gas");
  std::reverse(gt.begin(),gt.end()); // Reverse the path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}
// Test Junipero Serra Library to Holy Cross Church
TEST(TrojanMapTest5, CalculateShortestPath_Dijkstra3) {
  TrojanMap m;
  auto path = m.CalculateShortestPath_Dijkstra("Junipero Serra Library", "Holy Cross Church");
  std::vector<std::string> gt{
  "3699294513", "4343588843", "1716288048", "4343588841", "4343588840", "1716287963", "1773954244", "7425911443", 
  "7425911444", "7425911448", "3699278867"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  // Reverse the input
  path = m.CalculateShortestPath_Dijkstra("Holy Cross Church", "Junipero Serra Library");
  std::reverse(gt.begin(),gt.end()); // Reverse the path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

//Test CalculateShortestPath_Bellman_Ford
//Test  Ralphs to Target
TEST(TrojanMapTest6, CalculateShortestPath_Bellman_Ford1) {
  TrojanMap m;
  auto path = m.CalculateShortestPath_Bellman_Ford("Ralphs", "Target");
  std::vector<std::string> gt{
  "2578244375","4380040154","4380040158","4380040167","6805802087","8410938469","6813416131","7645318201",
  "6813416130","6813416129","123318563","452688940","6816193777","123408705","6816193774","452688933","452688931",
  "123230412","6816193770","6787470576","4015442011","6816193692","6816193693","6816193694","4015377691","544693739",
  "6816193696","6804883323","6807937309","6807937306","6816193698","4015377690","4015377689","122814447","6813416159",
  "6813405266","4015372488","4015372487","6813405229","122719216","6813405232","4015372486","7071032399","4015372485",
  "6813379479","6813379584","6814769289","5237417650"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  // Reverse the input 
  path = m.CalculateShortestPath_Bellman_Ford("Target", "Ralphs");
  std::reverse(gt.begin(),gt.end()); // Reverse the path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}
// Test Shall Gas to Rite Aid
TEST(TrojanMapTest6, CalculateShortestPath_Bellman_Ford2) {
  TrojanMap m;
  auto path = m.CalculateShortestPath_Bellman_Ford("Shall Gas", "Rite Aid");
  std::vector<std::string> gt{
  "3577173133", "1873056046", "72092030", "123699398", "1873055883", "1873055895", "72092041", 
  "250874495", "1778692886", "250874494", "1873055917", "1873055960", "250874498", "2199140379", 
  "1873055966", "1873055921", "1873055992", "67619040", "1732340082", "4012693767", "1873056002", 
  "6790384659", "1732340072", "1873055991", "9053766706", "3573974854", "3573974879", "3573974877", 
  "3577173167"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  // Reverse the input
  path = m.CalculateShortestPath_Bellman_Ford("Rite Aid", "Shall Gas");
  std::reverse(gt.begin(),gt.end()); // Reverse the path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}
// Test Junipero Serra Library to Holy Cross Church
TEST(TrojanMapTest6, CalculateShortestPath_Bellman_Ford3) {
  TrojanMap m;
  auto path = m.CalculateShortestPath_Bellman_Ford("Junipero Serra Library", "Holy Cross Church");
  std::vector<std::string> gt{
  "3699294513", "4343588843", "1716288048", "4343588841", "4343588840", "1716287963", "1773954244", "7425911443", 
  "7425911444", "7425911448", "3699278867"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  // Reverse the input
  path = m.CalculateShortestPath_Bellman_Ford("Holy Cross Church", "Junipero Serra Library");
  std::reverse(gt.begin(),gt.end()); // Reverse the path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test cycle detection function
TEST(TrojanMapTest7, CycleDetection1) {
  TrojanMap m;
  std::vector<double> square2 = {-118.289, -118.288, 34.029, 34.028};
  auto sub2 = m.GetSubgraph(square2);
  bool result2 = m.CycleDetection(sub2, square2);
  EXPECT_EQ(result2, false);
}
TEST(TrojanMapTest7, CycleDetection2) {
  TrojanMap m;
  std::vector<double> square1 = {-118.295, -118.265, 34.033, 34.010};
  auto sub1 = m.GetSubgraph(square1);
  bool result1 = m.CycleDetection(sub1, square1);
  EXPECT_EQ(result1, true);
}
TEST(TrojanMapTest7, CycleDetection3) {
  TrojanMap m;
  std::vector<double> square1 = {-118.293, -118.263, 34.031, 34.012};
  auto sub1 = m.GetSubgraph(square1);
  bool result1 = m.CycleDetection(sub1, square1);
  EXPECT_EQ(result1, true);
}

// Test the topological sort
TEST(TrojanMapTest8, TopologicalSort1) {
  TrojanMap m;
  std::vector<std::string> location_names = {"Ralphs", "Chick-fil-A", "Target"};
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs","Target"}, {"Ralphs","Chick-fil-A"}, {"Target","Chick-fil-A"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={"Ralphs", "Target","Chick-fil-A"};
  EXPECT_EQ(result, gt);
}
TEST(TrojanMapTest8, TopologicalSort2) {
  TrojanMap m;
  std::vector<std::string> location_names = {"Ralphs", "Chick-fil-A", "Target"};
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs","Target"}, {"Chick-fil-A","Ralphs"}, {"Target","Chick-fil-A"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={};
  EXPECT_EQ(result, gt);
}
TEST(TrojanMapTest8, TopologicalSort3) {
  TrojanMap m;
  std::vector<std::string> location_names = {"Ralphs", "KFC", "Holy Cross Church"};
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs","KFC"}, {"Holy Cross Church","Ralphs"}, {"KFC","Holy Cross Church"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={};
  EXPECT_EQ(result, gt);
}

// Test TravellingTrojan_Brute_force
TEST(TrojanMapTest9, TSP1) {
  TrojanMap m;
  std::vector<std::string> input{"3700479860","3762334037","3810143804","4009673261","3762337072","6816180153","4009709276","4009734480"}; // Input location ids 
  auto result = m.TravellingTrojan_Brute_force(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"3700479860", "3810143804", "6816180153", "4009709276", "4009673261", "4009734480", "3762334037", "3762337072", "3700479860"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
   EXPECT_EQ(flag, true);
}
TEST(TrojanMapTest9, TSP2) {
  TrojanMap m;
  std::vector<std::string> input{"4010228208","3762334037","3810143804","4010228190","4010228224","4010252822","4009709276","4009734480","4011837228","4009691191"}; // Input location ids 
  auto result = m.TravellingTrojan_Brute_force(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"4010228208", "4010228190", "4010252822", "4010228224", "4009734480", "4009691191", "4009709276", "3762334037", "3810143804", "4011837228", "4010228208"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
   EXPECT_EQ(flag, true);
}
TEST(TrojanMapTest9, TSP3) {
  TrojanMap m;
  std::vector<std::string> input{"6787827520","6807272299","6787803631","7362236509","2871010078","5002237823","7861033574","1773954300","6804926838","6787827520"}; // Input location ids 
  auto result = m.TravellingTrojan_Brute_force(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6787827520", "6807272299", "6787803631", "7362236509", "2871010078", "5002237823", "7861033574", "1773954300", "6804926838", "6787827520", "6787827520" }; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
   EXPECT_EQ(flag, true);
}

// Test TravellingTrojan_Backtracking
TEST(TrojanMapTest10, TSP1) {
  TrojanMap m;
  std::vector<std::string> input{"3700479860","3762334037","3810143804","4009673261","3762337072","6816180153","4009709276","4009734480"}; // Input location ids 
  auto result = m.TravellingTrojan_Backtracking(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"3700479860", "3810143804", "6816180153", "4009709276", "4009673261", "4009734480", "3762334037", "3762337072", "3700479860"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
   EXPECT_EQ(flag, true);
}
TEST(TrojanMapTest10, TSP2) {
  TrojanMap m;
  std::vector<std::string> input{"4010228208","3762334037","3810143804","4010228190","4010228224","4010252822","4009709276","4009734480","4011837228","4009691191"}; // Input location ids 
  auto result = m.TravellingTrojan_Backtracking(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"4010228208", "4010228190", "4010252822", "4010228224", "4009734480", "4009691191", "4009709276", "3762334037", "3810143804", "4011837228", "4010228208"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
   EXPECT_EQ(flag, true);
}
TEST(TrojanMapTest10, TSP3) {
  TrojanMap m;
  std::vector<std::string> input{"6787827520","6807272299","6787803631","7362236509","2871010078","5002237823","7861033574","1773954300","6804926838","6787827520"}; // Input location ids 
  auto result = m.TravellingTrojan_Backtracking(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6787827520", "6807272299", "6787803631", "7362236509", "2871010078", "5002237823", "7861033574", "1773954300", "6804926838", "6787827520", "6787827520" }; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
   EXPECT_EQ(flag, true);
}

// Test TravellingTrojan_2opt
TEST(TrojanMapTest11, TSP1) {
  TrojanMap m;
  std::vector<std::string> input{"3700479860","3762334037","3810143804","4009673261","3762337072","6816180153","4009709276","4009734480"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"3700479860", "3810143804", "6816180153", "4009709276", "4009673261", "4009734480", "3762334037", "3762337072", "3700479860"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
   EXPECT_EQ(flag, true);
}
TEST(TrojanMapTest11, TSP2) {
  TrojanMap m;
  std::vector<std::string> input{"4010228208","3762334037","3810143804","4010228190","4010228224","4010252822","4009709276","4009734480","4011837228","4009691191"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"4010228208", "4010228190", "4010252822", "4010228224", "4009734480", "4009691191", "4009709276", "3762334037", "3810143804", "4011837228", "4010228208"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
   EXPECT_EQ(flag, true);
}
TEST(TrojanMapTest11, TSP3) {
  TrojanMap m;
  std::vector<std::string> input{"6787827520","6807272299","6787803631","7362236509","2871010078","5002237823","7861033574","1773954300","6804926838","6787827520"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6787827520", "6807272299", "6787803631", "7362236509", "2871010078", "5002237823", "7861033574", "1773954300", "6804926838", "6787827520", "6787827520" }; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
   EXPECT_EQ(flag, true);
}

// Test FindNearby points
TEST(TrojanMapTest12, FindNearby1) {
  TrojanMap m;
  auto result = m.FindNearby("bank", "Ralphs", 10, 10);
  std::vector<std::string> ans{"9591449465", "5237417651", "9591449441"};
  EXPECT_EQ(result, ans);
}
TEST(TrojanMapTest12, FindNearby2) {
  TrojanMap m;
  auto result = m.FindNearby("bar", "Ralphs", 10, 10);
  std::vector<std::string> ans{"5567714035", "6045035789", "6045038065"};
  EXPECT_EQ(result, ans);
}
TEST(TrojanMapTest12, FindNearby3) {
  TrojanMap m;
  auto result = m.FindNearby("bicycle", "Ralphs", 10, 10);
  std::vector<std::string> ans{"6047197531", "2956626162"};
  EXPECT_EQ(result, ans);
}


// Test TravellingTrojan_3opt
TEST(TrojanMapTest13, TSP1) {
  TrojanMap m;
  std::vector<std::string> input{"6805134326","4630504671","1855166430","6716185893","6805606049","7559181321","6807381239","6804831748"}; // Input location ids 
  auto result = m.TravellingTrojan_3opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6805134326","4630504671","7559181321","1855166430","6804831748","6805606049","6807381239","6716185893","6805134326"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
   EXPECT_EQ(flag, true);
}
TEST(TrojanMapTest13, TSP2) {
  TrojanMap m;
  std::vector<std::string> input{"4835551222","6818427931","3398621867","304903415","6512491097","122827901","613372848","122742337"}; // Input location ids 
  auto result = m.TravellingTrojan_3opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"4835551222","3398621867","304903415","6818427931","122827901","6512491097","122742337","613372848","4835551222"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
   EXPECT_EQ(flag, true);
}
TEST(TrojanMapTest13, TSP3) {
  TrojanMap m;
  std::vector<std::string> input{"8383157989","123459651","5618025975","1855170159","6813565302","123408741","7591500901","3403035585","4640058001","7863404962"}; // Input location ids 
  auto result = m.TravellingTrojan_3opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"8383157989","5618025975","1855170159","7591500901","123408741","6813565302","3403035585","7863404962","123459651","4640058001","8383157989" }; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
   EXPECT_EQ(flag, true);
}