#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <sys/stat.h>

#include "KDTree.hpp"
#include "opencv2/opencv.hpp"

#define POINT_NUM 1000000
#define WALL_OBJ "../input/curved_surface.obj"
#define DRAWING_TXT "../input/drawing/heart_path_m.txt"
#define OUTPUT_TXT "../output/curved_surface_heart_path_m.txt"

using namespace std;
using namespace cv;

std::vector<std::string> split(std::string line, char delimiter) {
  std::vector<std::string> answer;
  std::stringstream ss(line);
  std::string temp;

  while (std::getline(ss, temp, delimiter)) {
      answer.push_back(temp);
  }

  return answer;
}

std::vector<double> calcNormal(pointVec &P){
  std::vector<double> normal;
  std::vector<double> P1, P2;

  // get 2 vectors from 3 points
  P1.push_back(P[0][0] - P[1][0]);
  P1.push_back(P[0][1] - P[1][1]);
  P1.push_back(P[0][2] - P[1][2]);
  P2.push_back(P[0][0] - P[2][0]);
  P2.push_back(P[0][1] - P[2][1]);
  P2.push_back(P[0][2] - P[2][2]);

  // calculate normal vector
  normal.push_back(P1[1]*P2[2] - P1[2]*P2[1]);
  normal.push_back(P1[2]*P2[0] - P1[0]*P2[2]);
  normal.push_back(P1[0]*P2[1] - P1[1]*P2[0]);

  // find d (x + y + z + d = 0)
  double d = -normal[0]*P[0][0] - normal[1]*P[0][1] - normal[2]*P[0][2];
  normal.push_back(d);

  return normal;
}

// read the mesh file and save the vertices and vertex normals 1-to-1
std::tuple<pointVec, pointVec, pointVec> readMesh (const std::string file_name) {
  pointVec points;
  pointVec normals;
  pointVec vns;
  pointVec faces;

  point_t pt;
  point_t n;
  point_t f;

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
      line.erase(line.begin()); // erase "f"
      for (auto element: line) {
        std::vector<std::string> vvn = split(element, '/');
        int v_index = std::stoi(vvn[0])-1;
        int vn_index = std::stoi(vvn[2])-1;
        normals[v_index] = vns[vn_index];
        f.push_back(v_index); // save vertexes in same face
      }
      faces.push_back({f[0],f[1],f[2],f[3]});
      f.clear();
    }
  }

  infile.close();

  return {points, normals, faces};
}

pointIndexArr getSurfaceNormal(const pointVec &points, const pointVec &faces){
  pointIndexArr surface_n;
  surface_n.assign(points.size()*2, make_pair(point_t(),POINT_NUM));

  for (size_t i = 0 ; i < faces.size() ; i ++) {
    pointVec coor = {points[faces[i][0]], points[faces[i][1]], points[faces[i][2]]}; // points[faces[i][3]]

    point_t sn = calcNormal(coor);

    for(size_t j = 0; j < 4; j++) {
      if(surface_n[faces[i][j]*2].second == POINT_NUM) {
        surface_n[faces[i][j]*2]=make_pair(sn, i);
      }
      else surface_n[faces[i][j]*2+1]=make_pair(sn, i);
    }
  }

  return surface_n;
}

cv::Mat homographyMat(pointVec &pt){
    // Four corners of the book in source image
    std::vector<cv::Point2f> pts_src;
    pts_src.push_back(cv::Point2f(pt[0][1], pt[0][2]));
    pts_src.push_back(cv::Point2f(pt[1][1], pt[1][2]));
    pts_src.push_back(cv::Point2f(pt[2][1], pt[2][2]));
    pts_src.push_back(cv::Point2f(pt[3][1], pt[3][2]));

    // Four corners of the book in destination image.
    std::vector<cv::Point2f> pts_dst;
    pts_dst.push_back(cv::Point2f(0, 0));
    pts_dst.push_back(cv::Point2f(1, 0));
    pts_dst.push_back(cv::Point2f(1, 1));
    pts_dst.push_back(cv::Point2f(0, 1));

    // Calculate Homography
    cv::Mat h = cv::findHomography(pts_src, pts_dst);

    return h;
}



int main() {
  pointVec points;
  pointVec normals;
  pointVec faces;

  // read mesh file
  std::string mesh_name = WALL_OBJ; // assume that file is executed in /build folder
  struct stat buffer;   
  if(stat (mesh_name.c_str(), &buffer) != 0){
    std::cout << "FILE NOT FOUND\n";
    return 0;
  };

  tie(points, normals, faces) = readMesh(mesh_name);

  // calculate surface normal
  pointIndexArr surface_n = getSurfaceNormal(points, faces);

  // make KDTree
  KDTree tree(points, surface_n, POINT_NUM);

  // read drawing file
  std::string drawing_name = DRAWING_TXT; // assume that file is executed in /build folder
  if(stat (drawing_name.c_str(), &buffer) != 0){
    std::cout << "FILE NOT FOUND\n";
    return 0;
  };

  // read file
  std::ifstream infile(drawing_name);
  std::string output_name = OUTPUT_TXT;
  std::ofstream outfile(output_name);
  std::string line_;  // line read

  std::getline(infile, line_); // ignore first line, which contains image size data
  outfile << line_ << "\n";

  while (std::getline(infile, line_)){
    if(line_ == "End") outfile << line_ << "\n";
    else{
      std::vector<std::string> line = split(line_, ' ');  // splitted line

      point_t pt = {0.0, std::stod(line[0]), std::stod(line[1])};
      
      int f_idx = tree.find_sn_index(pt);
      if(f_idx == -1){
        std::cout << "Face not found...\n";
        continue;
      }

      pointVec coor = {points[faces[f_idx][3]], points[faces[f_idx][2]], points[faces[f_idx][1]], points[faces[f_idx][0]]};
      pointVec nv = {normals[faces[f_idx][3]], normals[faces[f_idx][2]], normals[faces[f_idx][1]], normals[faces[f_idx][0]]};
      
      // matrix to convert quad to rectangle 
      cv::Mat h = homographyMat(coor);  // 3X3 matrix
      cv::Mat h_Inv=h.inv();

      // bilinear interpolation
      point_t pt_t;
      for(int i = 0; i < 3; i++){
          pt_t.push_back(h.at<double>(i,0)*pt[1]+h.at<double>(i,1)*pt[2]+h.at<double>(i,2));
      }

      double dy = 1-pt_t[0];
      double dz = pt_t[0];

      double A = dy*(1-dz);
      double B = (1-dy)*(1-dz);
      double C = (1-dy)*(dz);
      double D = dy*(dz);

      // point_t normal_vector;
      double x = A*coor[0][0]+B*coor[1][0]+C*coor[2][0]+D*coor[3][0];
      // for(int i = 0; i < 3; i++){
      //   normal_vector.push_back(A*nv[0][i]+B*nv[1][i]+C*nv[2][i]+D*nv[3][i]);
      // }

      outfile << std::to_string(x) << " " << line[0] << " " << line[1] << "\n";
    }
  }

  infile.close();
  outfile.close();

  return 0;
}
