#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <sys/stat.h>

#include "KDTree.hpp"

std::vector<std::string> split(std::string line, char delimiter) {
  std::vector<std::string> answer;
  std::stringstream ss(line);
  std::string temp;

  while (std::getline(ss, temp, delimiter)) {
      answer.push_back(temp);
  }

  return answer;
}

// read the mesh file and save the vertices and vertex normals 1-to-1
std::tuple<pointVec, pointVec> readMesh (const std::string file_name) {
  pointVec points;
  pointVec normals;
  pointVec vns;

  point_t pt;
  point_t n;

  bool assigned = false;

  // read file
  std::ifstream infile(file_name);
  std::string line_;  // line read
  while (std::getline(infile, line_))
  {
    std::vector<std::string> line = split(line_, ' ');  // splitted line
    if (line[0] == "v") { // vertex
      pt = {std::stod(line[1]), std::stod(line[2]), std::stod(line[3])};
      points.push_back(pt);
    }
    else if (line[0] == "vn") { // vertex normal
      n = {std::stod(line[1]), std::stod(line[2]), std::stod(line[3])};
      vns.push_back(n);
    }
    if (line[0] == "f") { // face
      if (!assigned) {  // assign vector
        normals.assign(points.size(), point_t(3, 0.0));
        assigned = true;
      }
      line.erase(line.begin()); // erase f
      for (auto element: line) {
        std::vector<std::string> vvn = split(element, '/');
        int v_index = std::stoi(vvn[0])-1;
        int vn_index = std::stoi(vvn[2])-1;
        normals[v_index] = vns[vn_index];
      }
    }
  }
  return {points, normals};
}

int main() {
  pointVec points;
  pointVec normals;

  // read mesh file
  std::string file_name = "../../input/curved_surface.obj"; // assume that file is executed in /bin folder
  struct stat buffer;   
  if(stat (file_name.c_str(), &buffer) != 0){
    std::cout << "FILE NOT FOUND\n";
    return 0;
  };

  tie(points, normals) = readMesh(file_name);

  // make KDTree
  KDTree tree(points, normals);

  // print out the points and normals
  // for (size_t i = 0 ; i < points.size() ; i ++) {
  //   std::cout << i << "th :" ;
  //   std::cout << "point :" << points[i][0] << ", " << points[i][1] << ", " << points[i][2] << std::endl;
  //   std::cout << "normal :" << normals[i][0] << ", " << normals[i][1] << ", " << normals[i][2] << std::endl;
  // }

  tree.print_tree();

  std::cout << "\nnearest test\n";
  point_t pt = {0.0, 0.0, 3.0};
  auto res = tree.nearest_point(pt);
  for (double b : res) {
      std::cout << b << " ";
  }

  std::cout << "\n\nneighborhood\n";
  auto res2 = tree.neighborhood_points(pt, .55);
  for (point_t a : res2) {
      for (double b : a) {
          std::cout << b << " ";
      }
      std::cout << '\n';
  }

  return 0;
}
