# pointcloud_downsample

## Purpose

After receiving the raw point cloud, this module first performs preprocessing, and then it can be sent to the localization module for ndt registration.

The preprocess is divided into three steps:
crop_box: removes points with in a given box region;
voxel_grid_downsample: points in each voxel are approximated with their centroid;
random_downsample: points are sampled with uniform probability.


## Inputs / Outputs

### Input

| Name         | Type                      | Description           |
| ------------ | ------------------------- | --------------------- |
| `input/pointcloud` | `sensor_msgs::msg::PointCloud2` | raw pointcloud |

### Output

| Name              | Type                 |Description            |
| ----------------- | ---------------------| -----------------------|
| `downsample/pointcloud` | `sensor_msgs::msg::PointCloud2` | downsampled pointcloud |


## Additional Remark
The code in the filters subdirectory is the original code after the indices mechanism is stripped. But the inheritance relationship between them is canceled.
