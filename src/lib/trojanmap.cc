#include "trojanmap.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <algorithm>
#include <fstream>
#include <locale>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <sstream>
#include <string>
#include <utility>
// #include <bits/stdc++.h>
#include <cmath>
#include <iostream>
#include <cctype>
#include <unordered_set>
#include <stack>
#include <chrono>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
typedef std::pair<double, std::string> iPair;

//-----------------------------------------------------
// TODO (Students): You do not and should not change the following functions:
//-----------------------------------------------------

/**
 * PrintMenu: Create the menu
 * 
 */
void TrojanMap::PrintMenu() {
  CreateGraphFromCSVFile();
  std::string menu =
      "**************************************************************\n"
      "* Select the function you want to execute.                    \n"
      "* 1. Autocomplete                                             \n"
      "* 2. Find the position                                        \n"
      "* 3. CalculateShortestPath                                    \n"
      "* 4. Travelling salesman problem                              \n"
      "* 5. Cycle Detection                                          \n"
      "* 6. Topological Sort                                         \n"
      "* 7. Find K Closest Points                                    \n"
      "* 8. Exit                                                     \n"
      "**************************************************************\n";
  std::cout << menu << std::endl;
  std::string input;
  getline(std::cin, input);
  char number = input[0];
  switch (number)
  {
  case '1':
  {
    menu =
        "**************************************************************\n"
        "* 1. Autocomplete                                             \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a partial location:";
    std::cout << menu;
    getline(std::cin, input);
    auto start = std::chrono::high_resolution_clock::now();
    auto results = Autocomplete(input);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0) {
      for (auto x : results) std::cout << x << std::endl;
    } else {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '2':
  {
    menu =
        "**************************************************************\n"
        "* 2. Find the position                                        \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a location:";
    std::cout << menu;
    getline(std::cin, input);
    auto start = std::chrono::high_resolution_clock::now();
    auto results = GetPosition(input);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.first != -1) {
      std::cout << "Latitude: " << results.first
                << " Longitude: " << results.second << std::endl;
      PlotPoint(results.first, results.second);
    } else {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '3':
  {
    menu =
        "**************************************************************\n"
        "* 3. CalculateShortestPath                                    \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input the start location:";
    std::cout << menu;
    std::string input1;
    getline(std::cin, input1);
    menu = "Please input the destination:";
    std::cout << menu;
    std::string input2;
    getline(std::cin, input2);
    auto start = std::chrono::high_resolution_clock::now();
    auto results = CalculateShortestPath_Dijkstra(input1, input2);
    // auto results = CalculateShortestPath_Bellman_Ford(input1, input2);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0) {
      for (auto x : results) std::cout << x << std::endl;
      std::cout << "The distance of the path is:" << CalculatePathLength(results) << " miles" << std::endl;
      PlotPath(results);
    } else {
      std::cout << "No route from the start point to the destination."
                << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '4':
  {
    menu =
        "**************************************************************\n"
        "* 4. Traveling salesman problem                              \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "In this task, we will select N random points on the map and you need to find the path to travel these points and back to the start point.";
    std::cout << menu << std::endl << std::endl;
    menu = "Please input the number of the places:";
    std::cout << menu;
    getline(std::cin, input);
    int num = std::stoi(input);
    std::vector<std::string> keys;
    for (auto x : data) {
      keys.push_back(x.first);
    }
    std::vector<std::string> locations;
    srand(time(NULL));
    for (int i = 0; i < num; i++)
      locations.push_back(keys[rand() % keys.size()]);
    PlotPoints(locations);
    std::cout << "Calculating ..." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    auto results = TravellingTrojan_3opt(locations);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    CreateAnimation(results.second);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.second.size() != 0) {
      for (auto x : results.second[results.second.size()-1]) std::cout << x << std::endl;
      menu = "**************************************************************\n";
      std::cout << menu;
      std::cout << "The distance of the path is:" << results.first << " miles" << std::endl;
      PlotPath(results.second[results.second.size()-1]);
    } else {
      std::cout << "The size of the path is 0" << std::endl;
    }
    menu = "**************************************************************\n"
           "You could find your animation at src/lib/output.avi.          \n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '5':
  {
    menu =
        "**************************************************************\n"
        "* 5. Cycle Detection                                          \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input the left bound longitude(between -118.299 and -118.264):";
    std::cout << menu;
    getline(std::cin, input);
    std::vector<double> square;
    square.push_back(atof(input.c_str()));

    menu = "Please input the right bound longitude(between -118.299 and -118.264):";
    std::cout << menu;
    getline(std::cin, input);
    square.push_back(atof(input.c_str()));

    menu = "Please input the upper bound latitude(between 34.011 and 34.032):";
    std::cout << menu;
    getline(std::cin, input);
    square.push_back(atof(input.c_str()));

    menu = "Please input the lower bound latitude(between 34.011 and 34.032):";
    std::cout << menu;
    getline(std::cin, input);
    square.push_back(atof(input.c_str()));

    auto start = std::chrono::high_resolution_clock::now();
    auto results = CycleDetection(square);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results == true)
      std::cout << "there exists cycle in the subgraph " << std::endl;
    else
      std::cout << "there exist no cycle in the subgraph " << std::endl;
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '6':
  {
    menu =
        "**************************************************************\n"
        "* 6. Topological Sort                                         \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input the locations filename:";
    std::cout << menu;
    std::string locations_filename;
    getline(std::cin, locations_filename);
    menu = "Please input the dependencies filename:";
    std::cout << menu;
    std::string dependencies_filename;
    getline(std::cin, dependencies_filename);
    
    // Read location names from CSV file
    std::vector<std::string> location_names;
    if (locations_filename == "") 
      location_names = {"Cardinal Gardens", "Coffee Bean1","CVS"};
    else
      location_names = ReadLocationsFromCSVFile(locations_filename);
    
    // Read dependencies from CSV file
    std::vector<std::vector<std::string>> dependencies;
    if (dependencies_filename == "")
      dependencies = {{"Coffee Bean1","Cardinal Gardens"}, {"CVS","Cardinal Gardens"}, {"CVS","Coffee Bean1"}};
    else
      dependencies = ReadDependenciesFromCSVFile(dependencies_filename);

    // std::vector<std::string> location_names = {"Cardinal Gardens", "Coffee Bean1","CVS"};
    // std::vector<std::vector<std::string>> dependencies = {{"Coffee Bean1","Cardinal Gardens"}, {"CVS","Cardinal Gardens"}, {"CVS","Coffee Bean1"}};
    auto start = std::chrono::high_resolution_clock::now();
    auto result = DeliveringTrojan(location_names, dependencies);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************";
    std::cout << menu << std::endl;
    std::cout << "Topological Sorting Results:" << std::endl;
    for (auto x : result) std::cout << x << std::endl;
    std::vector<std::string> node_ids;
    for (auto x: result) {
      std::string id = GetID(x);
      node_ids.push_back(id);
    }
    PlotPointsOrder(node_ids);
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
    case '7':
  {
    menu =
        "**************************************************************\n"
        "* 7. Find K Closest Points                                    \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    
    menu = "Please input the locations:";
    std::cout << menu;
    std::string origin;
    getline(std::cin, origin);
    menu = "Please input k:";
    std::cout << menu;
    getline(std::cin, input);
    int k = std::stoi(input);
    
    auto start = std::chrono::high_resolution_clock::now();
    auto result = FindKClosestPoints(origin, k);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************";
    std::cout << menu << std::endl;
    std::cout << "Find K Closest Points Results:" << std::endl;
    int cnt = 1;
    for (auto x : result) { 
      std::cout << cnt << " " << data[x].name << std::endl;
      cnt++;
    }
    PlotPointsLabel(result, GetID(origin));
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '8':
    break;
  default:
  {
    std::cout << "Please select 1 - 8." << std::endl;
    PrintMenu();
    break;
  }
  }
}


/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile() { 
  std::fstream fin;
  fin.open("src/lib/map.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '['), word.end());
      word.erase(std::remove(word.begin(), word.end(), ']'), word.end());
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
        n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}

/**
 * PlotPoint: Given a location id, plot the point on the map
 * 
 * @param  {std::string} id : location id
 */
void TrojanMap::PlotPoint(std::string id) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto result = GetPlotLocation(data[id].lat, data[id].lon);
  cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}
/**
 * PlotPoint: Given a lat and a lon, plot the point on the map
 * 
 * @param  {double} lat : latitude
 * @param  {double} lon : longitude
 */
void TrojanMap::PlotPoint(double lat, double lon) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto result = GetPlotLocation(lat, lon);
  cv::circle(img, cv::Point(int(result.first), int(result.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPath: Given a vector of location ids draws the path (connects the points)
 * 
 * @param  {std::vector<std::string>} location_ids : path
 */
void TrojanMap::PlotPath(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
  cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  for (auto i = 1; i < location_ids.size(); i++) {
    auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
    auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
    cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    cv::line(img, cv::Point(int(start.first), int(start.second)),
             cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
             LINE_WIDTH);
  }
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPoints: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points
 */
void TrojanMap::PlotPoints(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  for (auto x : location_ids) {
    auto result = GetPlotLocation(data[x].lat, data[x].lon);
    cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
  }
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPoints: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points inside square
 * @param  {std::vector<double>} square : boundary
 */
void TrojanMap::PlotPointsandEdges(std::vector<std::string> &location_ids, std::vector<double> &square) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto upperleft = GetPlotLocation(square[2], square[0]);
  auto lowerright = GetPlotLocation(square[3], square[1]);
  cv::Point pt1(int(upperleft.first), int(upperleft.second));
  cv::Point pt2(int(lowerright.first), int(lowerright.second));
  cv::rectangle(img, pt2, pt1, cv::Scalar(0, 0, 255));
  for (auto x : location_ids) {
    auto result = GetPlotLocation(data[x].lat, data[x].lon);
    cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    for(auto y : data[x].neighbors) {
      auto start = GetPlotLocation(data[x].lat, data[x].lon);
      auto end = GetPlotLocation(data[y].lat, data[y].lon);
      cv::line(img, cv::Point(int(start.first), int(start.second)),
              cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
              LINE_WIDTH);
    }
  }
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPointsOrder: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points
 */
void TrojanMap::PlotPointsOrder(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  for (auto x : location_ids) {
    auto result = GetPlotLocation(data[x].lat, data[x].lon);
    cv::putText(img, data[x].name, cv::Point(result.first, result.second), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 0, 0), 2);
  }
  // Plot dots and lines
  auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
  cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  for (auto i = 1; i < location_ids.size(); i++) {
    auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
    auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
    cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    cv::arrowedLine(img, cv::Point(int(start.first), int(start.second)),
             cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
             LINE_WIDTH);
  }
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPoints: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points
 */
void TrojanMap::PlotPointsLabel(std::vector<std::string> &location_ids, std::string origin) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  int cnt = 1;
  auto result = GetPlotLocation(data[origin].lat, data[origin].lon);
  cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 255, 0), cv::FILLED);
  for (auto x : location_ids) {
    auto result = GetPlotLocation(data[x].lat, data[x].lon);
    cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    cv::putText(img, std::to_string(cnt), cv::Point(result.first, result.second), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 0, 0), 2);
    cnt++;
  }
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * CreateAnimation: Create the videos of the progress to get the path
 * 
 * @param  {std::vector<std::vector<std::string>>} path_progress : the progress to get the path
 */
void TrojanMap::CreateAnimation(std::vector<std::vector<std::string>> path_progress){
  cv::VideoWriter video("src/lib/output.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, cv::Size(1248,992));
  for(auto location_ids: path_progress) {
    std::string image_path = cv::samples::findFile("src/lib/input.jpg");
    cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
    cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
              cv::Scalar(0, 0, 255), cv::FILLED);
    for (auto i = 1; i < location_ids.size(); i++) {
      auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
      auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
      cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
                cv::Scalar(0, 0, 255), cv::FILLED);
      cv::line(img, cv::Point(int(start.first), int(start.second)),
              cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
              LINE_WIDTH);
    }
    video.write(img);
    cv::imshow("TrojanMap", img);
    cv::waitKey(1);
  }
	video.release();
}
/**
 * GetPlotLocation: Transform the location to the position on the map
 * 
 * @param  {double} lat         : latitude 
 * @param  {double} lon         : longitude
 * @return {std::pair<double, double>}  : position on the map
 */
std::pair<double, double> TrojanMap::GetPlotLocation(double lat, double lon) {
  std::pair<double, double> bottomLeft(34.01001, -118.30000);
  std::pair<double, double> upperRight(34.03302, -118.26502);
  double h = upperRight.first - bottomLeft.first;
  double w = upperRight.second - bottomLeft.second;
  std::pair<double, double> result((lon - bottomLeft.second) / w * 1248,
                                   (1 - (lat - bottomLeft.first) / h) * 992);
  return result;
}

//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(std::string id) {
    Node node = data[id];
    return node.lat;
}


/**
 * GetLon: Get the longitude of a Node given its id. 
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(std::string id) { 
  Node node = data[id];
  return node.lon;
}

/**
 * GetName: Get the name of a Node given its id.
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(std::string id) { 
  Node node = data[id];
  return node.name;
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(std::string id) {
  Node node = data[id];
  std::vector<std::string> neib = node.neighbors;
  return neib;
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
  // TODO: Use Haversine Formula:
  // dlon = lon2 - lon1;
  // dlat = lat2 - lat1;
  // a = (sin(dlat / 2)) ^ 2 + cos(lat1) * cos(lat2) * (sin(dlon / 2)) ^ 2;
  // c = 2 * arcsin(min(1, sqrt(a)));
  // distances = 3961 * c;

  // where 3961 is the approximate radius of the earth at the latitude of
  // Washington, D.C., in miles
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
  for (int i = 0;i < path.size()-1; i++) {
    sum += CalculateDistance(path[i], path[i+1]);
  }
  return sum;
}

/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name){
  std::vector<std::string> results;
   for(auto node: data) {
    if(name.size() > node.second.name.size()) {
      continue;
    }

    std::string substr = node.second.name.substr(0, name.size());
    std::transform(name.begin(), name.end(), name.begin(), ::tolower);
    std::transform(substr.begin(), substr.end(), substr.begin(), ::tolower);
    if(name == substr) {
      results.push_back(node.second.name);
    }
  }
  return results;
}

/**
 * GetPosition: Given a location name, return the position.
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  /*
  1. get id of the name
  2. if exist, get lon and lat of id; else return (-1,-1)
  3.construct the pair and return
  */
  std::string id = GetID(name);
  if(id == ""){
    std::pair<double, double> results(-1, -1);
    return results;
  }
  
  double lat = GetLat(id);
  double lon = GetLon(id);

  std::pair<double, double> results(lat, lon);
  return results;
}

/**
 * GetID: Given a location name, return the id. 
 * If the node does not exist, return an empty string. 
 *
 * @param  {std::string} name          : location name
 * @return {int}  : id
 */
std::string TrojanMap::GetID(std::string name) {
  for(auto it = data.begin(); it != data.end(); it++){
    Node node = it->second;
    if(node.name == name){
      return node.id;
    }
  }
  return "";
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path which is a
 * list of id.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
    std::string location1_name, std::string location2_name) {
      std::string start_node = GetID(location1_name);
  std::string end_node = GetID(location2_name);
  std::priority_queue<iPair, std::vector<iPair> , std::greater<iPair> > pq;
  pq.push(make_pair(0, start_node));

  std::map<std::string, double> shortest_map;
  for(auto it = data.begin(); it != data.end(); it++){
    shortest_map[it->second.id] = INT_MAX;
  }
  shortest_map[start_node] = 0;

  std::map<std::string, std::string> predecessor_map;
  std::vector<std::string> path;
  while(!pq.empty()) {
    double cur_dist = pq.top().first;
    std::string cur_node = pq.top().second;
    pq.pop();

    if(cur_node == end_node) {
      // std::cout<< "break" << std::endl;
      break;
    } 

    if(cur_dist > shortest_map[cur_node]) {
      // std::cout<< "continue" << std::endl;
      continue;
    } //This node has already been visited, this pair of data is outdated

    for(auto neighbor : GetNeighborIDs(cur_node)){
      double new_dist = cur_dist + CalculateDistance(cur_node, neighbor);
      if(shortest_map[neighbor] > new_dist) {
        shortest_map[neighbor] = new_dist;
        predecessor_map[neighbor] = cur_node;
        pq.push(make_pair(new_dist, neighbor));
      }
    }
  }

  if(shortest_map[end_node] != INT_MAX) {
    std::string cur_node = end_node;
    while(cur_node != start_node) {
      path.push_back(cur_node);
      cur_node = predecessor_map[cur_node];
    }

    path.push_back(start_node);
    std::reverse(std::begin(path), std::end(path));
  }
  return path;
}

/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest path which is a
 * list of id.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
    std::string location1_name, std::string location2_name){

  std::string start_node = GetID(location1_name);
  std::string end_node = GetID(location2_name);

  std::map<std::string, std::vector<std::string>> neighbor_map;
  neighbor_map[start_node] = GetNeighborIDs(start_node);

  std::map<std::string, double> shortest_map;
  for(auto it = data.begin(); it != data.end(); it++){
    shortest_map[it->second.id] = INT_MAX;
  }
  shortest_map[start_node] = 0;

  std::map<std::string, std::string> predecessor_map;
  std::vector<std::string> path;
  bool stop = false;

  while(!stop){
    stop = true;
    for(auto it = neighbor_map.begin(); it != neighbor_map.end();) {
      for(auto neighbor : it->second){
        double new_dist = shortest_map[it->first] + CalculateDistance(it->first, neighbor);
        if(shortest_map[neighbor] > new_dist) {
          shortest_map[neighbor] = new_dist;
          predecessor_map[neighbor] = it->first;
          neighbor_map.insert(make_pair(neighbor, GetNeighborIDs(neighbor)));
          stop = false;
        }
      }
      neighbor_map.erase(it++);
    }
  }

  if(shortest_map[end_node] != INT_MAX) {
    std::string cur_node = end_node;
    while(cur_node != start_node) {
      path.push_back(cur_node);
      cur_node = predecessor_map[cur_node];
    }

    path.push_back(start_node);
    std::reverse(std::begin(path), std::end(path));
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
                                    std::vector<std::string> &location_ids) {

  std::vector<std::vector<std::string>> path;
  double min = DBL_MAX;
  std::vector<std::string> min_path;
  std::string start = location_ids[0];
  std::vector<std::string> cur_path = {start};
  bruteForceHelper(path, cur_path, min, min_path, location_ids, start, 0, start);
  std::reverse(path.begin(), path.end());
  std::pair<double, std::vector<std::vector<std::string>>> results(min, path);
  
  return results;
}
void TrojanMap::bruteForceHelper(std::vector<std::vector<std::string>> &path, std::vector<std::string> &sub_path,double &min, std::vector<std::string> &min_path, std::vector<std::string> &location_ids, std::string &cur_id, double cur_cost, std::string &start){
  //if we are at a leaf
  std::vector<std::string> cur_path = sub_path;
  if(cur_path.size() == location_ids.size()){
    cur_path.push_back(start);//come back to the start position
    cur_cost += CalculateDistance(cur_id, start);
    if(cur_cost < min){
      min = cur_cost;
      min_path.clear();
      min_path = cur_path;
      path.insert(path.begin(), min_path);//put in new min_path
    }
    return;
  }

  for(int i = 0; i < location_ids.size(); i++){
    //if we find curt node is already in cur_path, we skip it
    if(std::find(cur_path.begin(),
          cur_path.end(), location_ids[i]) != cur_path.end()){
            continue;
          }
    cur_path.push_back(location_ids[i]);
    double newCost = CalculateDistance(cur_id, location_ids[i]);
    bruteForceHelper(path, cur_path, min, min_path, location_ids, location_ids[i], cur_cost + newCost, start);
    cur_path.pop_back();
  }
}
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan(
                                    std::vector<std::string> &location_ids){
  std::vector<std::vector<std::string>> path;
  double min = DBL_MAX;
  std::vector<std::string> min_path;
  std::string start = location_ids[0];
  std::vector<std::string> cur_path = {start};
  bruteForceHelper(path, cur_path, min, min_path, location_ids, start, 0, start);
  std::reverse(path.begin(), path.end());
  std::pair<double, std::vector<std::vector<std::string>>> results(min, path);
  
  return results;
}

void TrojanMap::travelHelper(std::vector<std::vector<std::string>> &path, std::vector<std::string> &sub_path,double &min, std::vector<std::string> &min_path, std::vector<std::string> &location_ids, std::string &cur_id, double cur_cost, std::string &start){
  //if we are at a leaf
  std::vector<std::string> cur_path = sub_path;
  if(cur_path.size() == location_ids.size()){
    cur_path.push_back(start);//come back to the start position
    cur_cost += CalculateDistance(cur_id, start);
    if(cur_cost < min){
      min = cur_cost;
      min_path.clear();
      min_path = cur_path;
      path.insert(path.begin(), min_path);//put in new min_path
      return;
    }
    path.push_back(cur_path);
    return;
  }

  for(int i = 0; i < location_ids.size(); i++){
    //if we find curt node is already in cur_path, we skip it
    if(std::find(cur_path.begin(),
          cur_path.end(), location_ids[i]) != cur_path.end()){
            continue;
          }
    //else backtracking
    cur_path.push_back(location_ids[i]);
    double newCost = CalculateDistance(cur_id, location_ids[i]);
    travelHelper(path, cur_path, min, min_path, location_ids, location_ids[i], cur_cost + newCost, start);
    cur_path.pop_back();
  }
}

std::vector<std::string> TrojanMap::towOptSwap(std::vector<std::string> &location_ids, int i, int j){
  std::vector<std::string> ans = location_ids;
  iter_swap(ans.begin() + i, ans.begin() + j);// REMEMBER to include <algorithm> header file
  return ans;
}
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(
      std::vector<std::string> &location_ids){
  // initialize the min cost without swap locations
  // std::vector<std::string> cur_route = location_ids;
  std::string start = location_ids[0];
  location_ids.push_back(start);
  double min_cost = CalculatePathLength(location_ids);
  std::vector<std::vector<std::string>> paths;
  paths.push_back(location_ids);
  //swap the order of location_ids, make sure we will not switch the start posion with others
  double minChange;
  do{
    LABEL:
    minChange = 0;
    for(int i = 1; i <= location_ids.size() - 3; i++){
        for(int j = i + 1; j <= location_ids.size() - 2; j++){
        std::vector<std::string> newRoute = towOptSwap(location_ids, i, j);
        double newDistance = CalculatePathLength(newRoute);
        //If we get smaller cost, update min_cost and existing route
        if(newDistance < min_cost){
          minChange = newDistance - min_cost;
          min_cost = newDistance;
          location_ids = newRoute;
          paths.insert(paths.begin(), newRoute);
          goto LABEL;
        }
      }
    }
  } while(minChange < 0);
  std::reverse(paths.begin(), paths.end());
  std::pair<double, std::vector<std::vector<std::string>>> results(min_cost, paths);
  return results;
}

std::vector<std::vector<std::string>> TrojanMap::threeOptSwap(std::vector<std::string> &location_ids, int i, int j, int k){
  std::vector<std::vector<std::string>> result;
  for(int a = 0; a < 3; a++){
    for(int b = 0; b < 2; b++){
      std::vector<std::string> newRoute = towOptSwap(location_ids, i, j);
      newRoute = towOptSwap(newRoute, j, k);
      result.push_back(newRoute);
    }
  }
  return result;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_3opt(
      std::vector<std::string> &location_ids){
  // initialize the min cost without swap locations
  // std::vector<std::string> cur_route = location_ids;
  std::string start = location_ids[0];
  location_ids.push_back(start);
  double min_cost = CalculatePathLength(location_ids);
  std::vector<std::vector<std::string>> paths;
  paths.push_back(location_ids);
  //swap the order of location_ids, make sure we will not switch the start posion with others
  double minChange;
  do{
    LABEL:
    minChange = 0;
    for(int i = 1; i <= location_ids.size() - 4; i++){
        for(int j = i + 1; j <= location_ids.size() - 3; j++){
          for(int k = j + 1; k <=location_ids.size() - 2; k++){
            std::vector<std::vector<std::string>> newRoutes = threeOptSwap(location_ids, i, j, k);
            // std::vector<std::string> newRoute = towOptSwap(location_ids, i, j);
            for(int idx = 0; idx < newRoutes.size(); idx++){
              std::vector<std::string> newRoute = newRoutes[idx];
              double newDistance = CalculatePathLength(newRoute);
              //If we get smaller cost, update min_cost and existing route
              if(newDistance < min_cost){
                minChange = newDistance - min_cost;
                min_cost = newDistance;
                location_ids = newRoute;
                paths.insert(paths.begin(), newRoute);
                goto LABEL;
              }
            }
          }
      }
    }
  } while(minChange < 0);
  std::reverse(paths.begin(), paths.end());
  std::pair<double, std::vector<std::vector<std::string>>> results(min_cost, paths);
  return results;
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
  
  std::ifstream inFile(locations_filename, std::ios::in);
    if (!inFile)
    {
        std::cout << "Cannot find the file" << std::endl;
        exit(1);
    }
    std::string line;
    std::string field;
    getline(inFile, line);
    while (getline(inFile, line))//get data from CSV file line by line
    {
        std::string field;
        std::istringstream sin(line); 
        getline(sin, field, ','); //seperate by comma
        location_names_from_csv.push_back(field.c_str());
    }
    inFile.close();

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
  std::ifstream inFile(dependencies_filename, std::ios::in);
    if (!inFile)
    {
        std::cout << "Cannot find the file" << std::endl;
        exit(1);
    }
    std::string line;
    std::string field;

    while (getline(inFile, line))//get data from CSV file line by line
    {
        std::string field;
        std::istringstream sin(line); 
        std::vector<std::string> sub;

        getline(sin, field, ','); //seperate by comma
        // std::cout << field.c_str() << ",";
        sub.push_back(field.c_str());
        
        getline(sin, field, ',');
        // std::cout << field.c_str() << std::endl;
        sub.push_back(field.c_str());
        dependencies_from_csv.push_back(sub);
    }
    inFile.close();

  return dependencies_from_csv;
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a sorting of nodes
 * that satisfies the given dependencies.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan(std::vector<std::string> &locations,
                                                     std::vector<std::vector<std::string>> &dependencies){
  //Build the DAG and indegree vector
  std::unordered_map<std::string, int> degree;
  for(int i = 0; i < locations.size(); i++){
    std::string loc = locations[i];
    degree[loc] = 0;
  }
  std::unordered_map<std::string, std::vector<std::string>> graph;
  for(int i = 0; i < dependencies.size(); i++){
    std::string from = dependencies[i][0];
    std::string to = dependencies[i][1];
    // std::cout<<"to:" <<to<<std::endl;
    std::vector<std::string> neibourghs = graph[from];
    neibourghs.push_back(to);
    graph[from] = neibourghs;
    degree[to]++;
  }
  //push all nodes that with indegree equalts to 0
  std::queue<std::string> q;
  for(auto it = degree.begin(); it != degree.end(); it++){
    int indegree = it->second;
    std::string location = it->first;
    // std::cout<<"loc:" <<location<<", degree:" << indegree<< std::endl;
    if(indegree == 0){
      q.push(location);
    }
  }
  // do BFS
  std::vector<std::string> topo;
  while(!q.empty()){
    int size = q.size();
    for(int i = 0; i < size; i++){
      std::string cur = q.front();
      // std::cout<< cur<< std::endl;
      q.pop();
      topo.push_back(cur);
      std::vector<std::string> neibourghs = graph[cur];
      for(int j = 0; j < neibourghs.size(); j++){
        std::string neibourgh = neibourghs[j];
        degree[neibourgh]--;
        if(degree[neibourgh] == 0){
          q.push(neibourgh);
        }
      }
    }
  }

  return topo;                                                     
}

/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true if there
 * is a cycle path inside the square, false otherwise.
 * 
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
bool TrojanMap::CycleDetection(std::vector<double> &square) {
  std::map<std::string, bool> visited_map;
  std::map<std::string, std::string> predecessor_map;
  std::vector<std::string> path;

  for(auto it = data.begin(); it != data.end(); it++){
    if(it->second.lat <= square[2] && it->second.lat >= square[3] &&
       it->second.lon >= square[0] && it->second.lon <= square[1]) {
         visited_map[it->first] = false;
       }
  }

  std::string start_node;
  std::string stop_node;

  for(auto it = visited_map.begin(); it != visited_map.end(); it++) {
    if(it->second == false) {
      if(hasCycle(it->first, visited_map, predecessor_map, square, start_node, stop_node) == true) {
        std::string cur_node = stop_node;
        while(cur_node != start_node) {
          path.push_back(cur_node);
          cur_node = predecessor_map[cur_node];
        }
        path.push_back(start_node);
        PlotPointsandEdges(path, square);
        return true;
      }
    }
  }

  return false;
}

bool TrojanMap::hasCycle(std::string current_id, std::map<std::string, bool> &visited,
                         std::map<std::string, std::string> &predecessor_map, std::vector<double> &square, 
                         std::string &start_node, std::string &stop_node) {
  visited[current_id] = true;
  for(auto neighbor: GetNeighborIDs(current_id)) {

    if(data[neighbor].lat <= square[2] && data[neighbor].lat >= square[3] &&
       data[neighbor].lon >= square[0] && data[neighbor].lon <= square[1]) {

      if(visited[neighbor] == false) {
        predecessor_map[neighbor] = current_id;
        if(hasCycle(neighbor, visited, predecessor_map, square, start_node, stop_node) == true) {
          return true;
        } 
      } else {
        if(neighbor != predecessor_map[current_id]) {
          start_node = neighbor;
          stop_node = current_id;
          return true;
          }
      }
    }
  }             
  return false;
}

/**
 * FindKClosetPoints: Given a location id and k, find the k closest points on the map
 * 
 * @param {std::string} name: the name of the location
 * @param {int} k: number of closest points
 * @return {std::vector<std::string>}: k closest points
 */


//return the IDs of k closest points
std::vector<std::string> TrojanMap::FindKClosestPoints(std::string name, int k) {
  std::priority_queue<Point, std::vector<Point>, cmp> pq;
  for(auto it = data.begin(); it != data.end(); it++){
    std::string locId = it->first;
    std::string loc = GetName(locId);
    if(loc == name) continue;//If traverse to same point of name, we skip it
    //Calculate distance
    double dis = CalculateDistance(GetID(name), GetID(loc));
    Point *closePoint = new Point(locId, dis);
    //push into the pq
    pq.push(*closePoint);
    //check the size of pq
    if(pq.size() > k){
      pq.pop();
    }
  }
  //get the result from pq
  std::vector<std::string> res;
  while(!pq.empty()){
    Point cur = pq.top();
    pq.pop();
    std::string sub = cur.name;
    std::cout << sub << std::endl;
    res.push_back(sub);
  }
  std::reverse(res.begin(), res.end());
  return res;
}