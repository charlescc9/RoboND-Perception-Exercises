import pcl

# Load Point Cloud file
cloud = pcl.load_XYZRGB('tabletop.pcd')

# Voxel Grid filter
vox = cloud.make_voxel_grid_filter()
leaf_size = 0.01
vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
cloud_filtered = vox.filter()
pcl.save(cloud_filtered, 'voxel_downsampled.pcd')

# PassThrough filter
passthrough = cloud_filtered.make_passthrough_filter()
filter_axis = 'z'
passthrough.set_filter_field_name(filter_axis)
axis_min = 0.6
axis_max = 1.1
passthrough.set_filter_limits(axis_min, axis_max)
cloud_filtered = passthrough.filter()
pcl.save(cloud_filtered, 'pass_through_filtered.pcd')

# RANSAC plane segmentation
seg = cloud_filtered.make_segmenter()
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
max_distance = 0.01
seg.set_distance_threshold(max_distance)
inliners, coefficients = seg.segment()

# Extract inliers
extracted_inliners = cloud_filtered.extract(inliners, negative=False)
pcl.save(extracted_inliners, 'extracted_inliers.pcd')

# Extract outliers
extracted_outliers = cloud_filtered.extract(inliners, negative=True)
pcl.save(extracted_outliers, 'extracted_outliers.pcd')
