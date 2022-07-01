#include <iostream>
#include "src/lib/trojanmap.h"
#include <stdio.h>

int main() {
  TrojanMap x;
  x.PrintMenu();
  // x.CreateGraphFromCSVFile();

/* API test
  std::string res = x.GetID("Ralphs");
  double lat = x.GetLat("2578244375");
  // std::cout<< lat << std::endl;
  printf("%f", lat); //REMEMBER to include stdio.h
  std::cout << std::endl;

  double lon = x.GetLon("2578244375");
  // std::cout<< lat << std::endl;
  printf("%f", lon); //REMEMBER to include stdio.h
  std::cout << std::endl;
*/
/*
//test GetPosition
  std::pair<double, double> pos = x.GetPosition("ab");
  std::cout << "lat:";
  printf("%f", pos.first);
  std::cout << std::endl;
*/


 //test TravellingTrojan()
  // std::vector<std::string> location_ids = {"1873056015", "6905329551", "213332060", "1931345270"};
  // std::pair<double, std::vector<std::vector<std::string>>> res = x.TravellingTrojan(location_ids);
  // std::cout << "distance:"<< res.first << std::endl;
  // std::vector<std::vector<std::string>> second = res.second;
  // for(int i = 0; i < second.size(); i++){
  //   std::vector<std::string> cur = second[i];
  //   for(int j = 0; j < cur.size(); j++){
  //     std::cout << cur[j] << ",";
  //   }
  //   std::cout << std::endl;
  // }

//test TravellingTrojan_2opt()
  // std::vector<std::string> locations_id = {"1873056015", "6905329551", "213332060", "1931345270", "1873056015"};
  // std::pair<double, std::vector<std::vector<std::string>>> ans = x.TravellingTrojan_2opt(locations_id);
  // double dis = ans.first;
  // std::vector<std::vector<std::string>> route = ans.second;
  // std::vector<std::string> tail = route[route.size() - 1];
  // std::cout<<"dis: " << dis<< std::endl;
  // for(int i = 0; i < route.size(); i++){
  //   std::vector<std::string> sub = route[i];
  //   for(int j = 0; j < sub.size(); j++){
  //     std::cout << sub[j] <<",";
  //   }
  //   std::cout << std::endl;
  // }
  // for(int i = 0; i < tail.size(); i++){
  //   std::cout << tail[i] << ",";
  // }
  // std::cout << std::endl;
//test for DeliveringTrojan

  // std::vector<std::string> locations = {"Cardinal Gardens", "Coffee Bean1", "CVS"};
  // std::vector<std::vector<std::string>> dependencies = {{"Cardinal Gardens","Coffee Bean1"}, {"Cardinal Gardens","CVS"}, {"Coffee Bean1","CVS"}};
  // std::vector<std::string> res = x.DeliveringTrojan(locations, dependencies);
  // for(int i = 0; i < res.size(); i++){
  //   std::cout<< res[i] << ",";
  // }
  // std::cout << std::endl;
  
  // x.ReadLocationsFromCSVFile("/Users/songchanjuan/Desktop/USC-Courses/EE-538/project/final-project-Chanjuan/input/topologicalsort_locations.csv");
  // x.ReadDependenciesFromCSVFile("/Users/songchanjuan/Desktop/USC-Courses/EE-538/project/final-project-Chanjuan/input/topologicalsort_dependencies.csv");
  // x.CreateGraphFromCSVFile();
  // std::string res = x.GetID("Ralphs");
  // std::cout << "res:" << res << std::endl;
  // std::vector<std::string> res= x.FindKClosestPoints("Aldewaniah",10);
  // for(auto it = res.begin(); it != res.end(); it++){
  //   std::cout << *it << std::endl;
  // }
  
  // std::vector<std::string> locations_id = {"6807222049", "1732243549", "1377683421", "122827955", "1838284628", "1716461276", "123015367", "4147530482", "5231970322"};
  // std::pair<double, std::vector<std::vector<std::string>>> ans = x.TravellingTrojan_3opt(locations_id);
  // double dis = ans.first;
  // std::vector<std::vector<std::string>> route = ans.second;
  // std::vector<std::string> tail = route[route.size() - 1];
  // std::cout<<"dis: " << dis<< std::endl;
  // for(int i = 0; i < route.size(); i++){
  //   std::vector<std::string> sub = route[i];
  //   for(int j = 0; j < sub.size(); j++){
  //     std::cout << sub[j] <<",";
  //   }
  //   std::cout << std::endl;
  // }
  // for(int i = 0; i < tail.size(); i++){
  //   std::cout << tail[i] << ",";
  // }
  // std::cout << std::endl;
  return 0;
}