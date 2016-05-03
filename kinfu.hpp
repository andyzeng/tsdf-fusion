





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

void save_volume_to_ply(const std::string &file_name, int* vox_size, float* vox_tsdf) {
  float tsdf_threshold = 0.2f;
  // float weight_threshold = 1.0f;
  // float radius = 5.0f;

  // Count total number of points in point cloud
  int num_points = 0;
  for (int i = 0; i < vox_size[0] * vox_size[1] * vox_size[2]; i++)
    if (std::abs(vox_tsdf[i]) < tsdf_threshold)
      num_points++;

  // Create header for ply file
  FILE *fp = fopen(file_name.c_str(), "w");
  fprintf(fp, "ply\n");
  fprintf(fp, "format binary_little_endian 1.0\n");
  fprintf(fp, "element vertex %d\n", num_points);
  fprintf(fp, "property float x\n");
  fprintf(fp, "property float y\n");
  fprintf(fp, "property float z\n");
  fprintf(fp, "property uchar red\n");
  fprintf(fp, "property uchar green\n");
  fprintf(fp, "property uchar blue\n");
  fprintf(fp, "end_header\n");

  // Create point cloud content for ply file
  for (int i = 0; i < vox_size[0] * vox_size[1] * vox_size[2]; i++) {

    // If TSDF value of voxel is less than some threshold, add voxel coordinates to point cloud
    if (std::abs(vox_tsdf[i]) < tsdf_threshold) {

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

      if (vox_tsdf[i] < 0) {
        unsigned char r = (unsigned char)0;
        unsigned char g = (unsigned char)0;
        unsigned char b = (unsigned char)255;
        fwrite(&r, sizeof(unsigned char), 1, fp);
        fwrite(&g, sizeof(unsigned char), 1, fp);
        fwrite(&b, sizeof(unsigned char), 1, fp);
      } else {
        unsigned char r = (unsigned char)255;
        unsigned char g = (unsigned char)0;
        unsigned char b = (unsigned char)0;
        fwrite(&r, sizeof(unsigned char), 1, fp);
        fwrite(&g, sizeof(unsigned char), 1, fp);
        fwrite(&b, sizeof(unsigned char), 1, fp);
      }
    }
  }
  fclose(fp);
}

////////////////////////////////////////////////////////////////////////////////

__global__
void integrate(float* tmp_K, unsigned short* tmp_depth_data, float* tmp_view_bounds, float* tmp_camera_relative_pose,
               float tmp_vox_unit, float tmp_vox_mu, int* tmp_vox_size, float* tmp_vox_range_cam, float* tmp_vox_tsdf, float* tmp_vox_weight) {

  int z = (int)tmp_view_bounds[2 * 2 + 0] + blockIdx.x;
  int y = (int)tmp_view_bounds[1 * 2 + 0] + threadIdx.x;

  // if (z < (int)tmp_view_bounds[2 * 2 + 0] || z >= (int)tmp_view_bounds[2 * 2 + 1])
  //   return;
  // if (y < (int)tmp_view_bounds[1 * 2 + 0] || y >= (int)tmp_view_bounds[1 * 2 + 1])
  //   return;
  for (int x = tmp_view_bounds[0 * 2 + 0]; x < tmp_view_bounds[0 * 2 + 1]; x++) {

    // grid to world coords
    float tmp_pos[3] = {0};
    tmp_pos[0] = (x + 1) * tmp_vox_unit + tmp_vox_range_cam[0 * 2 + 0];
    tmp_pos[1] = (y + 1) * tmp_vox_unit + tmp_vox_range_cam[1 * 2 + 0];
    tmp_pos[2] = (z + 1) * tmp_vox_unit + tmp_vox_range_cam[2 * 2 + 0];

    // transform
    float tmp_arr[3] = {0};
    tmp_arr[0] = tmp_pos[0] - tmp_camera_relative_pose[3];
    tmp_arr[1] = tmp_pos[1] - tmp_camera_relative_pose[7];
    tmp_arr[2] = tmp_pos[2] - tmp_camera_relative_pose[11];
    tmp_pos[0] = tmp_camera_relative_pose[0 * 4 + 0] * tmp_arr[0] + tmp_camera_relative_pose[1 * 4 + 0] * tmp_arr[1] + tmp_camera_relative_pose[2 * 4 + 0] * tmp_arr[2];
    tmp_pos[1] = tmp_camera_relative_pose[0 * 4 + 1] * tmp_arr[0] + tmp_camera_relative_pose[1 * 4 + 1] * tmp_arr[1] + tmp_camera_relative_pose[2 * 4 + 1] * tmp_arr[2];
    tmp_pos[2] = tmp_camera_relative_pose[0 * 4 + 2] * tmp_arr[0] + tmp_camera_relative_pose[1 * 4 + 2] * tmp_arr[1] + tmp_camera_relative_pose[2 * 4 + 2] * tmp_arr[2];
    if (tmp_pos[2] <= 0)
      continue;

    int px = roundf(tmp_K[0] * (tmp_pos[0] / tmp_pos[2]) + tmp_K[2]);
    int py = roundf(tmp_K[4] * (tmp_pos[1] / tmp_pos[2]) + tmp_K[5]);
    if (px < 1 || px > 640 || py < 1 || py > 480)
      continue;

    float p_depth = *(tmp_depth_data + (py - 1) * 640 + (px - 1)) / 1000.f;
    if (p_depth < 0.2 || p_depth > 0.8)
      continue;
    if (roundf(p_depth * 1000.0f) == 0)
      continue;

    float eta = (p_depth - tmp_pos[2]) * sqrtf(1 + powf((tmp_pos[0] / tmp_pos[2]), 2) + powf((tmp_pos[1] / tmp_pos[2]), 2));
    if (eta <= -tmp_vox_mu)
      continue;

    // Integrate
    int volumeIDX = z * tmp_vox_size[0] * tmp_vox_size[1] + y * tmp_vox_size[0] + x;
    float sdf = fmin(1.0f, eta / tmp_vox_mu);
    float w_old = tmp_vox_weight[volumeIDX];
    float w_new = w_old + 1.0f;
    tmp_vox_weight[volumeIDX] = w_new;
    tmp_vox_tsdf[volumeIDX] = (tmp_vox_tsdf[volumeIDX] * w_old + sdf) / w_new;
  }
}

////////////////////////////////////////////////////////////////////////////////

void vol2bin() {
  // Write data to binary file
  // std::string volume_filename = "volume.tsdf.bin";
  // std::ofstream out_file(volume_filename, std::ios::binary | std::ios::out);
  // for (int i = 0; i < vox_size[0] * vox_size[1] * vox_size[2]; i++)
  //   out_file.write((char*)&vox_tsdf[i], sizeof(float));
  // out_file.close();
}

////////////////////////////////////////////////////////////////////////////////

void kFatalError(const int lineNumber = 0) {
  std::cerr << "FatalError";
  if (lineNumber != 0) std::cerr << " at LINE " << lineNumber;
  std::cerr << ". Program Terminated." << std::endl;
  cudaDeviceReset();
  exit(EXIT_FAILURE);
}

////////////////////////////////////////////////////////////////////////////////

void kCheckCUDA(const int lineNumber, cudaError_t status) {
  if (status != cudaSuccess) {
    std::cerr << "CUDA failure at LINE " << lineNumber << ": " << status << std::endl;
    kFatalError();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Fusion: always keep a TSDF volume active in GPU

// TSDF volume in CPU memory
float vox_unit;
float vox_mu_grid;
float vox_mu;
int vox_size[3];
float vox_range_cam[6];
float * vox_tsdf;
float * vox_weight;

// TSDF volume in GPU memory
int * d_vox_size;
float * d_vox_tsdf;
float * d_vox_weight;

// Fusion params in GPU memory
float * d_K;
unsigned short * d_depth_data;
float * d_view_bounds;
float * d_camera_relative_pose;
float * d_vox_range_cam;

// Training data in GPU memory
float * d_pos_crop_2D;
float * d_neg_crop_2D;
float * d_pos_crop_3D;
float * d_neg_crop_3D;

// Initialize existing cropped TSDF volume in GPU memory
__global__
void reset_vox_crop_GPU(float* tmp_view_bounds, int* tmp_vox_size, float* tmp_vox_tsdf, float* tmp_vox_weight) {
  // int z = blockIdx.x;
  // int y = threadIdx.x;
  int z = (int)tmp_view_bounds[2 * 2 + 0] + blockIdx.x;
  int y = (int)tmp_view_bounds[1 * 2 + 0] + threadIdx.x;
  for (int x = tmp_view_bounds[0 * 2 + 0]; x < tmp_view_bounds[0 * 2 + 1]; x++) {
    tmp_vox_tsdf[z * tmp_vox_size[0] * tmp_vox_size[1] + y * tmp_vox_size[0] + x] = 1.0f;
    tmp_vox_weight[z * tmp_vox_size[0] * tmp_vox_size[1] + y * tmp_vox_size[0] + x] = 0;
  }
}

// Initialize existing TSDF volume in GPU memory
__global__
void reset_vox_whole_GPU(int* tmp_vox_size, float* tmp_vox_tsdf, float* tmp_vox_weight) {
  int z = blockIdx.x;
  int y = threadIdx.x;
  for (int x = 0; x < tmp_vox_size[0]; x++) {
    tmp_vox_tsdf[z * tmp_vox_size[0] * tmp_vox_size[1] + y * tmp_vox_size[0] + x] = 1.0f;
    tmp_vox_weight[z * tmp_vox_size[0] * tmp_vox_size[1] + y * tmp_vox_size[0] + x] = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////

// Initialize TSDF volume and fusion params
void init_fusion_GPU() {

  // Init voxel volume params
  vox_unit = 0.0025;
  vox_mu_grid = 5;
  vox_mu = vox_unit * vox_mu_grid;
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
  // memset(vox_tsdf, 0, sizeof(float) * vox_size[0] * vox_size[1] * vox_size[2]);
  for (int i = 0; i < vox_size[0] * vox_size[1] * vox_size[2]; i++)
    vox_tsdf[i] = 1.0f;

  // Copy voxel volume to GPU
  cudaMalloc(&d_vox_size, 3 * sizeof(float));
  cudaMalloc(&d_vox_tsdf, vox_size[0] * vox_size[1] * vox_size[2] * sizeof(float));
  cudaMalloc(&d_vox_weight, vox_size[0] * vox_size[1] * vox_size[2] * sizeof(float));
  kCheckCUDA(__LINE__, cudaGetLastError());
  cudaMemcpy(d_vox_size, vox_size, 3 * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(d_vox_tsdf, vox_tsdf, vox_size[0] * vox_size[1] * vox_size[2] * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(d_vox_weight, vox_weight, vox_size[0] * vox_size[1] * vox_size[2] * sizeof(float), cudaMemcpyHostToDevice);
  kCheckCUDA(__LINE__, cudaGetLastError());

  // // Init volume in GPU
  // int CUDA_NUM_BLOCKS = vox_size[2];
  // int CUDA_NUM_THREADS = vox_size[1];
  // reset_vox_GPU <<< CUDA_NUM_BLOCKS, CUDA_NUM_THREADS >>>(d_vox_size, d_vox_tsdf, d_vox_weight);
  // kCheckCUDA(__LINE__, cudaGetLastError());

  // Allocate GPU memory to hold fusion params
  cudaMalloc(&d_K, 9 * sizeof(float));
  cudaMalloc(&d_depth_data, 480 * 640 * sizeof(unsigned short));
  cudaMalloc(&d_view_bounds, 6 * sizeof(float));
  cudaMalloc(&d_camera_relative_pose, 16 * sizeof(float));
  cudaMalloc(&d_vox_range_cam, 6 * sizeof(float));
  kCheckCUDA(__LINE__, cudaGetLastError());

  // Allocate GPU memory to hold training data

}

////////////////////////////////////////////////////////////////////////////////

// void realloc_fusion_params() {

//   if (d_K != NULL)
//     cudaFree(d_K);
//   if (d_depth_data != NULL)
//     cudaFree(d_depth_data);
//   if (d_view_bounds != NULL)
//     cudaFree(d_view_bounds);
//   if (d_camera_relative_pose != NULL)
//     cudaFree(d_camera_relative_pose);
//   if (d_vox_range_cam != NULL)
//     cudaFree(d_vox_range_cam);
//   kCheckCUDA(__LINE__, cudaGetLastError());

//   cudaMalloc(&d_K, 9 * sizeof(float));
//   cudaMalloc(&d_depth_data, 480 * 640 * sizeof(unsigned short));
//   cudaMalloc(&d_view_bounds, 6 * sizeof(float));
//   cudaMalloc(&d_camera_relative_pose, 16 * sizeof(float));
//   cudaMalloc(&d_vox_range_cam, 6 * sizeof(float));
//   kCheckCUDA(__LINE__, cudaGetLastError());
// }

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////