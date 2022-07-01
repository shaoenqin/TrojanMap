#include <map>
#include <vector>
#include <unordered_set>

#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"

TEST(TrojanMapStudentTest, Autocomplete) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test the upper case 
  auto names = m.Autocomplete("USC");
  std::unordered_set<std::string> gt1 = {"USC Village Gym", "USC Village Dining Hall", "USC Parking", "USC Fisher Museum of Art"}; // groundtruth for "USC"
  int success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt1.count(n) > 0, true);
    if (gt1.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt1.size());
  // Test the lower case
  names = m.Autocomplete("usc");
  std::unordered_set<std::string> gt2 = {"USC Village Gym", "USC Village Dining Hall", "USC Parking", "USC Fisher Museum of Art"}; // groundtruth for "USC"
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt2.count(n) > 0, true);
    if (gt2.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt2.size());
  // Test the lower and upper case 
  names = m.Autocomplete("uSc"); 
  std::unordered_set<std::string> gt3 = {"USC Village Gym", "USC Village Dining Hall", "USC Parking", "USC Fisher Museum of Art"}; // groundtruth for "USC"
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt3.count(n) > 0, true);
    if (gt3.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt3.size());
  // Test the upper and lower case 
  names = m.Autocomplete("UsC"); 
  std::unordered_set<std::string> gt4 = {"USC Village Gym", "USC Village Dining Hall", "USC Parking", "USC Fisher Museum of Art"}; // groundtruth for "USC"
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt4.count(n) > 0, true);
    if (gt4.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt4.size());
}

// Test CalculateShortestPath_Dijkstra function 1
TEST(TrojanMapStudentTest, CalculateShortestPath_Dijkstra) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test from Ralphs to Target
  auto path = m.CalculateShortestPath_Bellman_Ford("Ralphs", "Target");
  std::vector<std::string> gt{
      "2578244375","5559640911","6787470571","6808093910","6808093913","6808093919","6816831441","6813405269","6816193784",
      "6389467806","6816193783","123178876","2613117895","122719259","6807243574","6807243576","213332111","441895337",
      "441895335","122719255","2613117893","6813405231","122719216","6813405232","4015372486","7071032399","4015372485",
      "6813379479","5237417650"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);

  // Reverse the input from Ralphs to Target
  path = m.CalculateShortestPath_Dijkstra("Target", "Ralphs");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test CalculateShortestPath_Dijkstra function 2
TEST(TrojanMapStudentTest, CalculateShortestPath_Dijkstra2) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto path = m.CalculateShortestPath_Bellman_Ford("USC Parking", "Ralphs");
  // Test from USC Parking to Ralphs
  std::vector<std::string> gt{
      "6045067407","6285409682","6813379531","4141790934","123327636","6807200379","123327627","6813379521","6813379517",
      "6813379513","123005253","4399698008","4399698009","123044712","4399698010","4399698011","4399698013","4399698012",
      "5237381975","6813379479","4015372485","7071032399","4015372486","6813405232","122719216","6813405231","2613117893",
      "122719255","441895335","441895337","213332111","6807243576","6807243574","122719259","2613117895","123178876",
      "6816193783","6389467806","6816193784","6813405269","6816831441","6808093919","6808093913","6808093910","6787470571",
      "5559640911","2578244375"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);

  // Reverse the input from USC Parking to Ralphs
  path = m.CalculateShortestPath_Dijkstra("Ralphs", "USC Parking");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test CalculateShortestPath_Dijkstra function 3
TEST(TrojanMapStudentTest, CalculateShortestPath_Dijkstra3) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto path = m.CalculateShortestPath_Bellman_Ford("PED", "CVS");
  // Test from PED to CVS
  std::vector<std::string> gt{}; // Expected path, no route between PED and CVS
  EXPECT_EQ(path, gt);

  // Reverse the input from PED to CVS
  path = m.CalculateShortestPath_Dijkstra("CVS", "PED");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  EXPECT_EQ(path, gt);
}

// Test cycle detection function
TEST(TrojanMapStudentTest, CycleDetection) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test case 1
  std::vector<double> square1 = {-118.290, -118.270, 34.030, 34.015};
  bool result1 = m.CycleDetection(square1);
  EXPECT_EQ(result1, true);

  // Test case 2
  std::vector<double> square2 = {-118.288818, -118.282123, 34.023054, 34.018356};
  bool result2 = m.CycleDetection(square2);
  EXPECT_EQ(result2, false);

  // Test case 3
  std::vector<double> square3 = {-118.270, -118.264, 34.016, 34.011};
  bool result3 = m.CycleDetection(square3);
  EXPECT_EQ(result3, true);
}

TEST(TrojanMapTest, FindPosition) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test ChickfilA
  auto position = m.GetPosition("University SDA Church Food Pantry");
  std::pair<double, double> gt1(34.0111333, -118.2957046); // groundtruth for "University SDA Church Food Pantry"
  EXPECT_EQ(position, gt1);
  // Test Ralphs
  position = m.GetPosition("Traveler");
  std::pair<double, double> gt2(34.0211573, -118.2868678); // groundtruth for "Ralphs"
  EXPECT_EQ(position, gt2);
  // Test Target
  position = m.GetPosition("XX");
  std::pair<double, double> gt3(-1, -1); // groundtruth for "Target"
  EXPECT_EQ(position, gt3);
  // Test Unknown
  position = m.GetPosition("Department of Motor Vehicles");
  std::pair<double, double> gt4(34.0180476, -118.2791616);
  EXPECT_EQ(position, gt4);
}

TEST(TrojanMapTest, TSP) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"5768963646", "6807553006", "5680945529", "6813405278", "7424581964"}; // Input location ids 
  auto result = m.TravellingTrojan(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"5768963646", "6807553006", "5680945529", "6813405278", "7424581964", "5768963646"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP2) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"122845013", "6818390151", "122659207", "4835551074", "4060105089", "4011837221", "7424270441", "123380346", "216153383", "1841849484"}; // Input location ids 
  auto result = m.TravellingTrojan(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"122845013", "6818390151", "122659207", "4835551074", "4060105089", "4011837221", "7424270441", "123380346", "216153383", "1841849484", "122845013"}; // Expected order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the groundtruth path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) // counterclockwise
    flag = true;
  EXPECT_EQ(flag, true);
}

// Test TSP function 3
TEST(TrojanMapTest, TSP3) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"6807222049", "1732243549", "1377683421", "122827955", "1838284628", "1716461276", "123015367", "4147530482", "5231970322"}; // Input location ids 
  auto result = m.TravellingTrojan(input);
  std::cout << "My path length: " <<result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6807222049", "1732243549", "1377683421", "122827955", "1838284628", "1716461276", "123015367", "4147530482", "5231970322", "6807222049"}; // Expected order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the groundtruth path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) // counterclockwise
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP_2opt) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"5768963646", "5680945529", "6807553006", "6813405278", "7424581964"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"5768963646", "6807553006", "5680945529", "6813405278", "7424581964", "5768963646"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}
TEST(TrojanMapTest, TSP_2opt_3points) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"5768963646", "7424581964"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"5768963646", "7424581964", "5768963646"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP2_2opt) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"122845013", "6818390151", "122659207", "4835551074", "4060105089", "4011837221", "7424270441", "123380346", "216153383", "1841849484"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"122845013", "6818390151", "122659207", "4835551074", "4060105089", "4011837221", "7424270441", "123380346", "216153383", "1841849484", "122845013"}; // Expected order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the groundtruth path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) // counterclockwise
    flag = true;
  EXPECT_EQ(flag, true);
}

// Test TSP function 3
TEST(TrojanMapTest, TSP2_3opt) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"122845013", "6818390151", "122659207", "4835551074", "4060105089", "4011837221", "7424270441", "123380346", "216153383", "1841849484"}; // Input location ids 
  auto result = m.TravellingTrojan_3opt(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"122845013", "6818390151", "122659207", "4835551074", "4060105089", "4011837221", "7424270441", "123380346", "216153383", "1841849484", "122845013"}; // Expected order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the groundtruth path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) // counterclockwise
    flag = true;
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP3_2opt) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"6807222049", "122827955", "1838284628", "123015367","1716461276", "1377683421", "1732243549", "4147530482", "5231970322"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: " <<result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6807222049", "1732243549", "1377683421", "122827955", "1838284628", "1716461276", "123015367", "4147530482", "5231970322", "6807222049"}; // Expected order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the groundtruth path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) // counterclockwise
    flag = true;
  //can not get the shortest path in this example
  EXPECT_EQ(flag, false);
}


TEST(TrojanMapTest, TSP4_2opt) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{
    "7137906862","3088548446","4060015482","6815813018","6819170150","6820982916","4015477527","6819217384","4019292085","123316793","123153849","5237388100","6047218633","6807243576","4015405540","123058551","4020099349","1862312572","6987230634","7544324483","6818427911","6813405225","5567721536","1849116069","122719216","3438433459","6816193808","4835549589","4944439089","269636772","4060116190","29477081","5695236162","4835551066","6813513564","2613117890","6813565294","1878000349","1771091137","6807934448","7861033558","6805603628","123302797","123015369","7561883968","6793073226","6805054066","6807295173","2613117877","6816193804"
  }; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: " <<result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{
    "7137906862","3088548446","4015405540","123058551","4020099349","1862312572","4060015482","6815813018","6819170150","6820982916","4015477527","6819217384","4019292085","123316793","123153849","5237388100","6047218633","6807243576","6987230634","7544324483","6818427911","6813405225","5567721536","1849116069","122719216","3438433459","6816193808","4835549589","4944439089","269636772","4060116190","29477081","5695236162","4835551066","6813513564","2613117890","6813565294","1878000349","1771091137","6807934448","7861033558","6805603628","123302797","123015369","7561883968","6793073226","6805054066","6807295173","2613117877","6816193804","7137906862"
  }; // Expected order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the groundtruth path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) // counterclockwise
    flag = true;
  //can not get the shortest path in this example
  EXPECT_EQ(flag, false);
}

//Test for 3opt
TEST(TrojanMapTest, TSP3_3opt) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"6807222049", "122827955", "1838284628", "123015367","1716461276", "1377683421", "1732243549", "4147530482", "5231970322"}; // Input location ids 
  auto result = m.TravellingTrojan_3opt(input);
  std::cout << "My path length: " <<result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6807222049", "1732243549", "1377683421", "122827955", "1838284628", "1716461276", "123015367", "4147530482", "5231970322", "6807222049"}; // Expected order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the groundtruth path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) // counterclockwise
    flag = true;
  
  EXPECT_EQ(flag, true);
}


TEST(TrojanMapTest, TSP4_3opt) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{
     "123096999","6789994791","6045038065","4399698008","6813565311","1738419610","6807935208","6788584374","216153383","4020099336","6805827714"
  }; // Input location ids 
  auto result = m.TravellingTrojan_3opt(input);
  std::cout << "My path length: " <<result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{
    "123096999","4399698008","6045038065","6813565311","6788584374","6789994791","1738419610","6807935208","216153383","4020099336","6805827714","123096999"
  }; // Expected order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the groundtruth path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) // counterclockwise
    flag = true;
  
  EXPECT_EQ(flag, false);
}

// Test topological sort
TEST(TrojanMapTest, TopologicalSort) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> location_names = {"Cardinal Gardens", "Coffee Bean1", "Aldewaniah", "CVS"};
  std::vector<std::vector<std::string>> dependencies = {{"Cardinal Gardens","Coffee Bean1"}, {"Cardinal Gardens","CVS"}, {"Coffee Bean1","CVS"}, {"CVS", "Aldewaniah"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={"Cardinal Gardens", "Coffee Bean1","CVS", "Aldewaniah"};
  EXPECT_EQ(result, gt);
}

TEST(TrojanMapTest, TopologicalSort1) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> location_names = {"Cardinal Gardens", "Coffee Bean1", "Aldewaniah", "CVS"};
  std::vector<std::vector<std::string>> dependencies = {{"Cardinal Gardens","Coffee Bean1"}, {"Cardinal Gardens","CVS"}, {"Coffee Bean1","CVS"}, {"CVS", "Aldewaniah"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={"Cardinal Gardens", "Coffee Bean1","CVS", "Aldewaniah"};
  EXPECT_EQ(result, gt);
}

TEST(TrojanMapTest, TopologicalSort2) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> location_names = {"Cardinal Gardens", "Coffee Bean1", "Aldewaniah", "CVS", "Birnkrant", "Ralphs"};
  std::vector<std::vector<std::string>> dependencies = {{"Cardinal Gardens","Coffee Bean1"}, {"Cardinal Gardens","CVS"}, {"Coffee Bean1","CVS"}, {"CVS", "Aldewaniah"}, {"Birnkrant", "Ralphs"}, {"Birnkrant", "Cardinal Gardens"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={"Birnkrant", "Ralphs", "Cardinal Gardens", "Coffee Bean1","CVS", "Aldewaniah"};
  EXPECT_EQ(result, gt);
}
// Test K closest points
TEST(TrojanMapTest, FindKClosestPoints) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto result = m.FindKClosestPoints("Cardinal Gardens", 5);
  std::vector<std::string> gt{
  "6396635921", "6047234446", "6047234444", "5567714035", "5567721536"};
  EXPECT_EQ(result, gt);
}

TEST(TrojanMapTest, FindKClosestPoints1) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto result = m.FindKClosestPoints("Coffee Bean1", 9);
  std::vector<std::string> gt{
  "732641023", "4547476733", "4162647227", "4577908517", "6206425701", "4089614984", "269633667", "4399693644", "2817034894"};
  EXPECT_EQ(result, gt);
}

TEST(TrojanMapTest, FindKClosestPoints2) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto result = m.FindKClosestPoints("Aldewaniah", 10);
  std::vector<std::string> gt{
  "5514004014", "2193435039","5695236162", "5695236163", "3732301010", "5695236161", "5695236160", "368173251", "5695236164", "5555281222"};
  EXPECT_EQ(result, gt);
}