#include "util.hpp"
#include <png++/png.hpp>

////////////////////////////////////////////////////////////////////////////////

void get_frustum_bounds(float* K, std::vector<std::vector<float>> &extrinsic_poses, int base_frame, int curr_frame, float* camera_relative_pose, float* view_bounds,
                        float vox_unit, int* vox_size, float* vox_range_cam) {

  // Use two extrinsic matrices to compute relative rotations between current frame and first frame
  std::vector<float> ex_pose1 = extrinsic_poses[base_frame];
  std::vector<float> ex_pose2 = extrinsic_poses[curr_frame];

  float * ex_mat1 = &ex_pose1[0];
  float * ex_mat2 = &ex_pose2[0];

  float ex_mat1_inv[16] = {0};
  invert_matrix(ex_mat1, ex_mat1_inv);
  multiply_matrix(ex_mat1_inv, ex_mat2, camera_relative_pose);

  // Init cam view frustum
  float max_depth = 0.8;
  float cam_view_frustum[15] =
  { 0, -320 * max_depth / K[0], -320 * max_depth / K[0], 320 * max_depth / K[0],  320 * max_depth / K[0],
    0, -240 * max_depth / K[0],  240 * max_depth / K[0], 240 * max_depth / K[0], -240 * max_depth / K[0],
    0,               max_depth,               max_depth,              max_depth,              max_depth
  };

  // Rotate cam view frustum wrt Rt
  for (int i = 0; i < 5; i++) {
    float tmp_arr[3] = {0};
    tmp_arr[0] = camera_relative_pose[0 * 4 + 0] * cam_view_frustum[0 + i] + camera_relative_pose[0 * 4 + 1] * cam_view_frustum[5 + i] + camera_relative_pose[0 * 4 + 2] * cam_view_frustum[2 * 5 + i];
    tmp_arr[1] = camera_relative_pose[1 * 4 + 0] * cam_view_frustum[0 + i] + camera_relative_pose[1 * 4 + 1] * cam_view_frustum[5 + i] + camera_relative_pose[1 * 4 + 2] * cam_view_frustum[2 * 5 + i];
    tmp_arr[2] = camera_relative_pose[2 * 4 + 0] * cam_view_frustum[0 + i] + camera_relative_pose[2 * 4 + 1] * cam_view_frustum[5 + i] + camera_relative_pose[2 * 4 + 2] * cam_view_frustum[2 * 5 + i];
    cam_view_frustum[0 * 5 + i] = tmp_arr[0] + camera_relative_pose[3];
    cam_view_frustum[1 * 5 + i] = tmp_arr[1] + camera_relative_pose[7];
    cam_view_frustum[2 * 5 + i] = tmp_arr[2] + camera_relative_pose[11];
  }

  // Compute frustum endpoints
  float range2test[3][2] = {0};
  for (int i = 0; i < 3; i++) {
    range2test[i][0] = *std::min_element(&cam_view_frustum[i * 5], &cam_view_frustum[i * 5] + 5);
    range2test[i][1] = *std::max_element(&cam_view_frustum[i * 5], &cam_view_frustum[i * 5] + 5);
  }

  // Compute frustum bounds wrt volume
  for (int i = 0; i < 3; i++) {
    view_bounds[i * 2 + 0] = std::max(0.0f, std::floor((range2test[i][0] - vox_range_cam[i * 2 + 0]) / vox_unit));
    view_bounds[i * 2 + 1] = std::min((float)(vox_size[i]), std::ceil((range2test[i][1] - vox_range_cam[i * 2 + 0]) / vox_unit + 1));
  }
}

////////////////////////////////////////////////////////////////////////////////

void save_volume_to_ply(const std::string &file_name, int* vox_size, float* vox_tsdf, float* vox_weight) {
  float tsdf_threshold = 0.2f;
  float weight_threshold = 1.0f;
  // float radius = 5.0f;

  // Count total number of points in point cloud
  int num_points = 0;
  for (int i = 0; i < vox_size[0] * vox_size[1] * vox_size[2]; i++)
    if (std::abs(vox_tsdf[i]) < tsdf_threshold && vox_weight[i] > weight_threshold)
      num_points++;

  // Create header for ply file
  FILE *fp = fopen(file_name.c_str(), "w");
  fprintf(fp, "ply\n");
  fprintf(fp, "format binary_little_endian 1.0\n");
  fprintf(fp, "element vertex %d\n", num_points);
  fprintf(fp, "property float x\n");
  fprintf(fp, "property float y\n");
  fprintf(fp, "property float z\n");
  fprintf(fp, "end_header\n");

  // Create point cloud content for ply file
  for (int i = 0; i < vox_size[0] * vox_size[1] * vox_size[2]; i++) {

    // If TSDF value of voxel is less than some threshold, add voxel coordinates to point cloud
    if (std::abs(vox_tsdf[i]) < tsdf_threshold && vox_weight[i] > weight_threshold) {

      // Compute voxel indices in int for higher positive number range
      int z = floor(i / (vox_size[0] * vox_size[1]));
      int y = floor((i - (z * vox_size[0] * vox_size[1])) / vox_size[0]);
      int x = i - (z * vox_size[0] * vox_size[1]) - (y * vox_size[0]);

      // Convert voxel indices to float, and save coordinates to ply file
      float float_x = (float) x;
      float float_y = (float) y;
      float float_z = (float) z;
      fwrite(&float_x, sizeof(float), 1, fp);
      fwrite(&float_y, sizeof(float), 1, fp);
      fwrite(&float_z, sizeof(float), 1, fp);
    }
  }
  fclose(fp);
}

bool read_depth_data(const std::string &file_name, unsigned short * data) {
  png::image< png::gray_pixel_16 > img(file_name.c_str(), png::require_color_space< png::gray_pixel_16 >());
  int index = 0;
  for (int i = 0; i < 480; ++i) {
    for (int j = 0; j < 640; ++j) {
      unsigned short s = img.get_pixel(j, i);
      *(data + index) = s;
      ++index;
    }
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////

__global__
void integrate(float* K, unsigned short* depth_data, float* view_bounds, float* camera_relative_pose,
               float vox_unit, float vox_mu, int* vox_size, float* vox_range_cam, float* vox_tsdf, float* vox_weight) {

  int z = blockIdx.x;
  int y = threadIdx.x;

  if (z < (int)view_bounds[2 * 2 + 0] || z >= (int)view_bounds[2 * 2 + 1])
    return;
  if (y < (int)view_bounds[1 * 2 + 0] || y >= (int)view_bounds[1 * 2 + 1])
    return;
  for (int x = view_bounds[0 * 2 + 0]; x < view_bounds[0 * 2 + 1]; x++) {

    // grid to world coords
    float tmp_pos[3] = {0};
    tmp_pos[0] = (x + 1) * vox_unit + vox_range_cam[0 * 2 + 0];
    tmp_pos[1] = (y + 1) * vox_unit + vox_range_cam[1 * 2 + 0];
    tmp_pos[2] = (z + 1) * vox_unit + vox_range_cam[2 * 2 + 0];

    // transform
    float tmp_arr[3] = {0};
    tmp_arr[0] = tmp_pos[0] - camera_relative_pose[3];
    tmp_arr[1] = tmp_pos[1] - camera_relative_pose[7];
    tmp_arr[2] = tmp_pos[2] - camera_relative_pose[11];
    tmp_pos[0] = camera_relative_pose[0 * 4 + 0] * tmp_arr[0] + camera_relative_pose[1 * 4 + 0] * tmp_arr[1] + camera_relative_pose[2 * 4 + 0] * tmp_arr[2];
    tmp_pos[1] = camera_relative_pose[0 * 4 + 1] * tmp_arr[0] + camera_relative_pose[1 * 4 + 1] * tmp_arr[1] + camera_relative_pose[2 * 4 + 1] * tmp_arr[2];
    tmp_pos[2] = camera_relative_pose[0 * 4 + 2] * tmp_arr[0] + camera_relative_pose[1 * 4 + 2] * tmp_arr[1] + camera_relative_pose[2 * 4 + 2] * tmp_arr[2];
    if (tmp_pos[2] <= 0)
      continue;

    int px = roundf(K[0] * (tmp_pos[0] / tmp_pos[2]) + K[2]);
    int py = roundf(K[4] * (tmp_pos[1] / tmp_pos[2]) + K[5]);
    if (px < 1 || px > 640 || py < 1 || py > 480)
      continue;

    float p_depth = *(depth_data + (py - 1) * 640 + (px - 1)) / 1000.f;
    if (p_depth < 0.2 || p_depth > 0.8)
      continue;
    if (roundf(p_depth * 1000.0f) == 0)
      continue;

    float eta = (p_depth - tmp_pos[2]) * sqrtf(1 + powf((tmp_pos[0] / tmp_pos[2]), 2) + powf((tmp_pos[1] / tmp_pos[2]), 2));
    if (eta <= -vox_mu)
      continue;

    // Integrate
    int volumeIDX = z * vox_size[0] * vox_size[1] + y * vox_size[0] + x;
    float sdf = fmin(1.0f, eta / vox_mu);
    float w_old = vox_weight[volumeIDX];
    float w_new = w_old + 1.0f;
    vox_weight[volumeIDX] = w_new;
    vox_tsdf[volumeIDX] = (vox_tsdf[volumeIDX] * w_old + sdf) / w_new;
  }
}

void vol2bin() {
  // Write data to binary file
  // std::string volume_filename = "volume.tsdf.bin";
  // std::ofstream out_file(volume_filename, std::ios::binary | std::ios::out);
  // for (int i = 0; i < vox_size[0] * vox_size[1] * vox_size[2]; i++)
  //   out_file.write((char*)&vox_tsdf[i], sizeof(float));
  // out_file.close();
}

void FatalError(const int lineNumber = 0) {
  std::cerr << "FatalError";
  if (lineNumber != 0) std::cerr << " at LINE " << lineNumber;
  std::cerr << ". Program Terminated." << std::endl;
  cudaDeviceReset();
  exit(EXIT_FAILURE);
}

void checkCUDA(const int lineNumber, cudaError_t status) {
  if (status != cudaSuccess) {
    std::cerr << "CUDA failure at LINE " << lineNumber << ": " << status << std::endl;
    FatalError();
  }
}

int main(int argc, char **argv) {

  std::string data_directory = "data";
  std::string sequence_directory = data_directory + "/sample";

  // Get file list of color images
  std::vector<std::string> file_list_color;
  std::string color_regex = ".color.png";
  get_files_in_directory(sequence_directory, file_list_color, color_regex);
  std::sort(file_list_color.begin(), file_list_color.end());

  // Get file list of depth images
  std::vector<std::string> file_list_depth;
  std::string depth_regex = ".depth.png";
  get_files_in_directory(sequence_directory, file_list_depth, depth_regex);
  std::sort(file_list_depth.begin(), file_list_depth.end());

  // Get file list of intrinsics
  std::vector<std::string> file_list_intrinsics;
  std::string intrinsics_regex = ".K.txt";
  get_files_in_directory(sequence_directory, file_list_intrinsics, intrinsics_regex);
  std::sort(file_list_intrinsics.begin(), file_list_intrinsics.end());

  // Get file list of extrinsics
  std::vector<std::string> file_list_extrinsics;
  std::string extrinsics_regex = ".pose.txt";
  get_files_in_directory(sequence_directory, file_list_extrinsics, extrinsics_regex);
  std::sort(file_list_extrinsics.begin(), file_list_extrinsics.end());

  // Load intrinsics (3x3 matrix)
  std::string intrinsic_filename = sequence_directory + "/intrinsics.K.txt";
  std::vector<float> K_vec = load_matrix_from_file(intrinsic_filename, 3, 3);
  float K[9];
  for (int i = 0; i < 9; i++)
    K[i] = K_vec[i];

  // Load extrinsics (4x4 matrices)
  std::vector<std::vector<float>> extrinsics;
  for (std::string &curr_filename : file_list_extrinsics) {
    std::string curr_extrinsic_filename = sequence_directory + "/" + curr_filename;
    std::vector<float> curr_extrinsic = load_matrix_from_file(curr_extrinsic_filename, 4, 4);
    extrinsics.push_back(curr_extrinsic);
  }

  // Init voxel volume params
  float vox_unit = 0.001; // in meters
  float vox_mu_grid = 5;
  float vox_mu = vox_unit * vox_mu_grid;
  int vox_size[3];
  float vox_range_cam[6];
  float * vox_tsdf;
  float * vox_weight;
  vox_size[0] = 512;
  vox_size[1] = 512;
  vox_size[2] = 512;
  vox_range_cam[0 * 2 + 0] = -(float)(vox_size[0]) * vox_unit / 2;
  vox_range_cam[0 * 2 + 1] = vox_range_cam[0 * 2 + 0] + (float)(vox_size[0]) * vox_unit;
  vox_range_cam[1 * 2 + 0] = -(float)(vox_size[1]) * vox_unit / 2;
  vox_range_cam[1 * 2 + 1] = vox_range_cam[1 * 2 + 0] + (float)(vox_size[1]) * vox_unit;
  vox_range_cam[2 * 2 + 0] = -50.0f * vox_unit;
  vox_range_cam[2 * 2 + 1] = vox_range_cam[2 * 2 + 0] + (float)(vox_size[2]) * vox_unit;
  vox_tsdf = new float[vox_size[0] * vox_size[1] * vox_size[2]];
  vox_weight = new float[vox_size[0] * vox_size[1] * vox_size[2]];
  memset(vox_weight, 0, sizeof(float) * vox_size[0] * vox_size[1] * vox_size[2]);
  for (int i = 0; i < vox_size[0] * vox_size[1] * vox_size[2]; i++)
    vox_tsdf[i] = 1.0f;

  // Copy voxel volume to GPU
  int * d_vox_size;
  float * d_vox_tsdf;
  float * d_vox_weight;
  cudaMalloc(&d_vox_size, 3 * sizeof(float));
  cudaMalloc(&d_vox_tsdf, vox_size[0] * vox_size[1] * vox_size[2] * sizeof(float));
  cudaMalloc(&d_vox_weight, vox_size[0] * vox_size[1] * vox_size[2] * sizeof(float));
  checkCUDA(__LINE__, cudaGetLastError());
  cudaMemcpy(d_vox_size, vox_size, 3 * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(d_vox_tsdf, vox_tsdf, vox_size[0] * vox_size[1] * vox_size[2] * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(d_vox_weight, vox_weight, vox_size[0] * vox_size[1] * vox_size[2] * sizeof(float), cudaMemcpyHostToDevice);
  checkCUDA(__LINE__, cudaGetLastError());

  // Allocate GPU to hold fusion params
  float * d_K;
  unsigned short * d_depth_data;
  float * d_view_bounds;
  float * d_camera_relative_pose;
  float * d_vox_range_cam;
  cudaMalloc(&d_K, 9 * sizeof(float));
  cudaMalloc(&d_depth_data, 480 * 640 * sizeof(unsigned short));
  cudaMalloc(&d_view_bounds, 6 * sizeof(float));
  cudaMalloc(&d_camera_relative_pose, 16 * sizeof(float));
  cudaMalloc(&d_vox_range_cam, 6 * sizeof(float));
  checkCUDA(__LINE__, cudaGetLastError());

  // Set first frame of sequence as base coordinate frame
  int base_frame = 0;

  // Fuse frames
  for (int curr_frame = 0; curr_frame < file_list_depth.size(); curr_frame++) {
    std::cerr << "Fusing frame " << curr_frame << "...";

    // Load image/depth/extrinsic data for current frame
    unsigned short * depth_data = (unsigned short *) malloc(480 * 640 * sizeof(unsigned short));
    std::string curr_filename = sequence_directory + "/" + file_list_depth[curr_frame];
    read_depth_data(curr_filename, depth_data);

    // Compute relative camera pose transform between current frame and base frame
    // Compute camera view frustum bounds within the voxel volume
    float camera_relative_pose[16] = {0};
    float view_bounds[6] = {0};
    get_frustum_bounds(K, extrinsics, base_frame, curr_frame, camera_relative_pose, view_bounds,
                       vox_unit, vox_size, vox_range_cam);

    // Copy fusion params to GPU
    cudaMemcpy(d_K, K, 9 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_depth_data, depth_data, 480 * 640 * sizeof(unsigned short), cudaMemcpyHostToDevice);
    cudaMemcpy(d_view_bounds, view_bounds, 6 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_camera_relative_pose, camera_relative_pose, 16 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_vox_range_cam, vox_range_cam, 6 * sizeof(float), cudaMemcpyHostToDevice);
    checkCUDA(__LINE__, cudaGetLastError());

    // Integrate
    // integrateCPU(K, depth_data, view_bounds, camera_relative_pose,
    //           vox_unit, vox_mu, vox_range_cam, vox_tsdf, vox_weight);
    int CUDA_NUM_BLOCKS = vox_size[2];
    int CUDA_NUM_THREADS = vox_size[1];
    integrate <<< CUDA_NUM_BLOCKS, CUDA_NUM_THREADS >>>(d_K, d_depth_data, d_view_bounds, d_camera_relative_pose,
                                vox_unit, vox_mu, d_vox_size, d_vox_range_cam, d_vox_tsdf, d_vox_weight);
    checkCUDA(__LINE__, cudaGetLastError());

    // Clear memory
    free(depth_data);
    std::cerr << " done!" << std::endl;

    // If at the last frame, save current TSDF volume to point cloud visualization
    std::string scene_ply_name = "volume.pointcloud.ply";
    if (curr_frame == file_list_depth.size() - 1) {

      // Copy data back to memory
      cudaMemcpy(vox_tsdf, d_vox_tsdf, vox_size[0] * vox_size[1] * vox_size[2] * sizeof(float), cudaMemcpyDeviceToHost);
      cudaMemcpy(vox_weight, d_vox_weight, vox_size[0] * vox_size[1] * vox_size[2] * sizeof(float), cudaMemcpyDeviceToHost);
      checkCUDA(__LINE__, cudaGetLastError());
      save_volume_to_ply(scene_ply_name, vox_size, vox_tsdf, vox_weight);
    }

  }
  return 0;
}

