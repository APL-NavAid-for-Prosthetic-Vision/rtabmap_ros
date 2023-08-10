# point_cloud                   set of obstacle points in map coordinates computed within a local grid
# point_connected_component     connected component label associated with each point in point_cloud
# point_row                     the row position in the local grid associated with each point in point_cloud
# point_col                     the column position in the local grid associated with each point in point_cloud
# grid_resolution_per_meter     the grid resolution at which point_cloud was constructed (pixels or points per meter)
# grid_cols                     the number of columns in the local grid used to construct point_cloud
# grid_rows                     the number of rows in the local grid used to construct point_cloud
# grid_origin_x                 the x map coordinate of pixel (0,0) in the local grid used to construct point_cloud
# grid_origin_y                 the y map coordinate of pixel (0,0) in the local grid used to construct point_cloud

sensor_msgs/PointCloud point_cloud
int32[] point_connected_component
int32[] point_row
int32[] point_col
float32 grid_resolution_per_meter
int32 grid_cols
int32 grid_rows
float32 grid_origin_x
float32 grid_origin_y
