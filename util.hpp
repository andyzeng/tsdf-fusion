#include <ctime>
#include <string>
#include <random>
#include <algorithm>
#include <iostream>
#include <vector>
#include <fstream>
#include <functional>
#include <numeric>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <limits>
#include <cstring>
#include <dirent.h>
#include <sstream>
#include <iomanip>
#include <thread>
#include <pwd.h>
#include <cstdlib>
#include <cerrno>
//#include <libio.h>
#include <sys/types.h>
#include <sys/stat.h>

///////////////////////////////////////////////////////////////////////
// Load a MxN matrix from a text file
std::vector<float> load_matrix_from_file(std::string filename, int M, int N) {
  std::vector<float> matrix;
  FILE *fp = fopen(filename.c_str(), "r");
  for (int i = 0; i < M * N; i++) {
    float tmp;
    int iret = fscanf(fp, "%f", &tmp);
    matrix.push_back(tmp);
  }
  fclose(fp);
  return matrix;
}

///////////////////////////////////////////////////////////////////////
// Timer
std::clock_t tic_toc_timer;
void tic() {
  tic_toc_timer = clock();
}
void toc() {
  std::clock_t toc_timer = clock();
  printf("Elapsed time is %f seconds.\n", double(toc_timer - tic_toc_timer) / CLOCKS_PER_SEC);
}

///////////////////////////////////////////////////////////////////////
// Run a system command
void sys_command(std::string str) {
  if (system(str.c_str()))
    return;
}

///////////////////////////////////////////////////////////////////////
// Generate random float
float gen_random_float(float min, float max) {
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<double> dist(min, max - 0.0001);
  return dist(mt);
}

///////////////////////////////////////////////////////////////////////
// Generate random string
std::string gen_rand_str(size_t len) {
  auto randchar = []() -> char {
    // const char charset[] =
    //   "0123456789"
    //   "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    //   "abcdefghijklmnopqrstuvwxyz";
    const char charset[] =
    "0123456789"
    "abcdefghijklmnopqrstuvwxyz";
    // const size_t max_index = (sizeof(charset) - 1);
    // return charset[rand() % max_index];
    return charset[((int) std::floor(gen_random_float(0.0f, (float) sizeof(charset) - 1)))];
  };
  std::string str(len, 0);
  std::generate_n(str.begin(), len, randchar);
  return str;
}

///////////////////////////////////////////////////////////////////////
// Check if file exists
bool file_exists(const std::string &filename) {
  std::ifstream file(filename);
  return (!file.fail());
}

///////////////////////////////////////////////////////////////////////
// Return all files in directory using search string
void get_files_in_directory(const std::string &directory, std::vector<std::string> &file_list, const std::string &search_string) {
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir (directory.c_str())) != NULL) {
    while ((ent = readdir (dir)) != NULL) {
      std::string filename(ent->d_name);
      if (filename.find(search_string) != std::string::npos && filename != "." && filename != "..")
        file_list.push_back(filename);
    }
    closedir (dir);
  } else {
    perror ("Error: could not look into directory!");
  }
}

///////////////////////////////////////////////////////////////////////
// Return indices of all occurances of substr in str
std::vector<size_t> find_substring(const std::string &str, const std::string &substr) {
  std::vector<size_t> substr_idx; 
  size_t tmp_idx = str.find(substr, 0);
  while(tmp_idx != std::string::npos) {
      substr_idx.push_back(tmp_idx);
      tmp_idx = str.find(substr, tmp_idx+1);
  }
  return substr_idx;
}

///////////////////////////////////////////////////////////////////////
// Load a 4x4 identity matrix
// Matrices are stored from left-to-right, top-to-bottom
void load_identity_matrix(float mOut[16]) {
  mOut[0] = 1.0f;  mOut[1] = 0.0f;  mOut[2] = 0.0f;  mOut[3] = 0.0f;
  mOut[4] = 0.0f;  mOut[5] = 1.0f;  mOut[6] = 0.0f;  mOut[7] = 0.0f;
  mOut[8] = 0.0f;  mOut[9] = 0.0f;  mOut[10] = 1.0f; mOut[11] = 0.0f;
  mOut[12] = 0.0f; mOut[13] = 0.0f; mOut[14] = 0.0f; mOut[15] = 1.0f;
}

///////////////////////////////////////////////////////////////////////
// 4x4 matrix multiplication
// Matrices are stored from left-to-right, top-to-bottom
void multiply_matrix(const float m1[16], const float m2[16], float mOut[16]) {
  mOut[0]  = m1[0] * m2[0]  + m1[1] * m2[4]  + m1[2] * m2[8]   + m1[3] * m2[12];
  mOut[1]  = m1[0] * m2[1]  + m1[1] * m2[5]  + m1[2] * m2[9]   + m1[3] * m2[13];
  mOut[2]  = m1[0] * m2[2]  + m1[1] * m2[6]  + m1[2] * m2[10]  + m1[3] * m2[14];
  mOut[3]  = m1[0] * m2[3]  + m1[1] * m2[7]  + m1[2] * m2[11]  + m1[3] * m2[15];

  mOut[4]  = m1[4] * m2[0]  + m1[5] * m2[4]  + m1[6] * m2[8]   + m1[7] * m2[12];
  mOut[5]  = m1[4] * m2[1]  + m1[5] * m2[5]  + m1[6] * m2[9]   + m1[7] * m2[13];
  mOut[6]  = m1[4] * m2[2]  + m1[5] * m2[6]  + m1[6] * m2[10]  + m1[7] * m2[14];
  mOut[7]  = m1[4] * m2[3]  + m1[5] * m2[7]  + m1[6] * m2[11]  + m1[7] * m2[15];

  mOut[8]  = m1[8] * m2[0]  + m1[9] * m2[4]  + m1[10] * m2[8]  + m1[11] * m2[12];
  mOut[9]  = m1[8] * m2[1]  + m1[9] * m2[5]  + m1[10] * m2[9]  + m1[11] * m2[13];
  mOut[10] = m1[8] * m2[2]  + m1[9] * m2[6]  + m1[10] * m2[10] + m1[11] * m2[14];
  mOut[11] = m1[8] * m2[3]  + m1[9] * m2[7]  + m1[10] * m2[11] + m1[11] * m2[15];

  mOut[12] = m1[12] * m2[0] + m1[13] * m2[4] + m1[14] * m2[8]  + m1[15] * m2[12];
  mOut[13] = m1[12] * m2[1] + m1[13] * m2[5] + m1[14] * m2[9]  + m1[15] * m2[13];
  mOut[14] = m1[12] * m2[2] + m1[13] * m2[6] + m1[14] * m2[10] + m1[15] * m2[14];
  mOut[15] = m1[12] * m2[3] + m1[13] * m2[7] + m1[14] * m2[11] + m1[15] * m2[15];
}

////////////////////////////////////////////////////////////////////////////////
// 4x4 matrix inversion
// Matrices are stored from left-to-right, top-to-bottom
bool invert_matrix(const float m[16], float invOut[16]) {
  float inv[16], det;
  int i;
  inv[0] = m[5]  * m[10] * m[15] -
           m[5]  * m[11] * m[14] -
           m[9]  * m[6]  * m[15] +
           m[9]  * m[7]  * m[14] +
           m[13] * m[6]  * m[11] -
           m[13] * m[7]  * m[10];

  inv[4] = -m[4]  * m[10] * m[15] +
           m[4]  * m[11] * m[14] +
           m[8]  * m[6]  * m[15] -
           m[8]  * m[7]  * m[14] -
           m[12] * m[6]  * m[11] +
           m[12] * m[7]  * m[10];

  inv[8] = m[4]  * m[9] * m[15] -
           m[4]  * m[11] * m[13] -
           m[8]  * m[5] * m[15] +
           m[8]  * m[7] * m[13] +
           m[12] * m[5] * m[11] -
           m[12] * m[7] * m[9];

  inv[12] = -m[4]  * m[9] * m[14] +
            m[4]  * m[10] * m[13] +
            m[8]  * m[5] * m[14] -
            m[8]  * m[6] * m[13] -
            m[12] * m[5] * m[10] +
            m[12] * m[6] * m[9];

  inv[1] = -m[1]  * m[10] * m[15] +
           m[1]  * m[11] * m[14] +
           m[9]  * m[2] * m[15] -
           m[9]  * m[3] * m[14] -
           m[13] * m[2] * m[11] +
           m[13] * m[3] * m[10];

  inv[5] = m[0]  * m[10] * m[15] -
           m[0]  * m[11] * m[14] -
           m[8]  * m[2] * m[15] +
           m[8]  * m[3] * m[14] +
           m[12] * m[2] * m[11] -
           m[12] * m[3] * m[10];

  inv[9] = -m[0]  * m[9] * m[15] +
           m[0]  * m[11] * m[13] +
           m[8]  * m[1] * m[15] -
           m[8]  * m[3] * m[13] -
           m[12] * m[1] * m[11] +
           m[12] * m[3] * m[9];

  inv[13] = m[0]  * m[9] * m[14] -
            m[0]  * m[10] * m[13] -
            m[8]  * m[1] * m[14] +
            m[8]  * m[2] * m[13] +
            m[12] * m[1] * m[10] -
            m[12] * m[2] * m[9];

  inv[2] = m[1]  * m[6] * m[15] -
           m[1]  * m[7] * m[14] -
           m[5]  * m[2] * m[15] +
           m[5]  * m[3] * m[14] +
           m[13] * m[2] * m[7] -
           m[13] * m[3] * m[6];

  inv[6] = -m[0]  * m[6] * m[15] +
           m[0]  * m[7] * m[14] +
           m[4]  * m[2] * m[15] -
           m[4]  * m[3] * m[14] -
           m[12] * m[2] * m[7] +
           m[12] * m[3] * m[6];

  inv[10] = m[0]  * m[5] * m[15] -
            m[0]  * m[7] * m[13] -
            m[4]  * m[1] * m[15] +
            m[4]  * m[3] * m[13] +
            m[12] * m[1] * m[7] -
            m[12] * m[3] * m[5];

  inv[14] = -m[0]  * m[5] * m[14] +
            m[0]  * m[6] * m[13] +
            m[4]  * m[1] * m[14] -
            m[4]  * m[2] * m[13] -
            m[12] * m[1] * m[6] +
            m[12] * m[2] * m[5];

  inv[3] = -m[1] * m[6] * m[11] +
           m[1] * m[7] * m[10] +
           m[5] * m[2] * m[11] -
           m[5] * m[3] * m[10] -
           m[9] * m[2] * m[7] +
           m[9] * m[3] * m[6];

  inv[7] = m[0] * m[6] * m[11] -
           m[0] * m[7] * m[10] -
           m[4] * m[2] * m[11] +
           m[4] * m[3] * m[10] +
           m[8] * m[2] * m[7] -
           m[8] * m[3] * m[6];

  inv[11] = -m[0] * m[5] * m[11] +
            m[0] * m[7] * m[9] +
            m[4] * m[1] * m[11] -
            m[4] * m[3] * m[9] -
            m[8] * m[1] * m[7] +
            m[8] * m[3] * m[5];

  inv[15] = m[0] * m[5] * m[10] -
            m[0] * m[6] * m[9] -
            m[4] * m[1] * m[10] +
            m[4] * m[2] * m[9] +
            m[8] * m[1] * m[6] -
            m[8] * m[2] * m[5];

  det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

  if (det == 0)
    return false;

  det = 1.0 / det;

  for (i = 0; i < 16; i++)
    invOut[i] = inv[i] * det;

  return true;
}