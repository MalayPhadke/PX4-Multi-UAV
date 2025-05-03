/*
	FILE: occupancyMap.cpp
	--------------------------------------
	function definition of occupancy map
*/
#include <px4_map_manager/occupancy_map.hpp>
#include <functional>
#include <sensor_msgs/image_encodings.hpp>
using namespace std::placeholders;
#include <chrono>

namespace px4_map_manager{
	occMap::occMap()
		: rclcpp::Node("occupancy_map", rclcpp::NodeOptions()) {
		this->ns_ = "occupancy_map";
		this->hint_ = "[OccMap]";
	}
	occMap::occMap(const rclcpp::NodeOptions & options)
	: rclcpp::Node("occupancy_map", options) {
		this->ns_ = "occupancy_map";
		this->hint_ = "[OccMap]";
		// Declare all parameters in the constructor
		this->declareParameters();
		// Then initialize and read parameters
		this->initParam();
		this->initPrebuiltMap();
		this->registerPub();
		this->registerCallback();
	}

	void occMap::initMap(const rclcpp::NodeOptions & options) {
		this->ns_ = "occupancy_map";
		this->hint_ = "[OccMap]";
		// Don't declare parameters again, they should already be declared in the constructor
		// Just initialize based on existing parameters
		// this->initParam();
		// this->initPrebuiltMap();
		// this->registerPub();
		// this->registerCallback();
	}
	
	void occMap::declareParameters() {
		// Declare all parameters with default values
		this->declare_parameter("sensor_input_mode", 0);
		this->declare_parameter("localization_mode", 0);
		this->declare_parameter("depth_image_topic", "/camera/depth/image_raw");
		this->declare_parameter("point_cloud_topic", "/camera/depth/points");
		this->declare_parameter("pose_topic", "/px4_visualizer/px4_1/vehicle_pose");
		this->declare_parameter("odom_topic", "/px4_1/fmu/out/vehicle_odometry");
		this->declare_parameter("robot_size", std::vector<double>{0.5, 0.5, 0.3});  // Float array
		this->declare_parameter("depth_intrinsics", std::vector<double>{432.496042035043, 432.496042035043, 320.0, 240.0});
		this->declare_parameter("depth_scale_factor", 10.0);  // Declare as double
		this->declare_parameter("depth_min_value", 0.5);
		this->declare_parameter("depth_max_value", 5.0);
		this->declare_parameter("depth_filter_margin", 2);
		this->declare_parameter("depth_skip_pixel", 2);
		this->declare_parameter("image_cols", 640);
		this->declare_parameter("image_rows", 480);
		this->declare_parameter("body_to_camera", std::vector<double>{1.0, 0.0, 0.0, 0.12, 0.0, 1.0, 0.0, 0.03, 0.0, 0.0, 1.0, 0.242, 0.0, 0.0, 0.0, 1.0});
		this->declare_parameter("raycast_max_length", 5.0);
		this->declare_parameter("p_hit", 0.7);
		this->declare_parameter("p_miss", 0.35);
		this->declare_parameter("p_min", 0.12);
		this->declare_parameter("p_max", 0.97);
		this->declare_parameter("p_occ", 0.8);
		this->declare_parameter("map_resolution", 0.1);
		this->declare_parameter("ground_height", -0.1);
		this->declare_parameter("map_size", std::vector<double>{40.0, 40.0, 3.0});  // Float array
		this->declare_parameter("local_update_range", std::vector<double>{5.0, 5.0, 5.0});  // Float array
		this->declare_parameter("local_bound_inflation", 3.0);
		this->declare_parameter("clean_local_map", false);
		this->declare_parameter("local_map_size", std::vector<double>{40.0, 40.0, 6.0});  // Float array
		this->declare_parameter("max_height_visualization", 2.5);
		this->declare_parameter("visualize_global_map", true);
		this->declare_parameter("verbose", false);
		this->declare_parameter("prebuilt_map_directory", "No");
		
		std::cout << this->hint_ << ": Parameters declared" << std::endl;
}

void occMap::initParam(){
		// Log parameter reading
		// std::cout << this->hint_ << ": Reading parameters..." << std::endl;
		// auto param_list = this->list_parameters({}, 100);
		// for (auto &n : param_list.names) {
		//   std::cout << this->hint_ << ":  " << n << std::endl;
		// }

		// sensor input mode
		if (not this->get_parameter("sensor_input_mode", this->sensorInputMode_)){
			this->sensorInputMode_ = 0;
			cout << this->hint_ << ": No sensor input mode option. Use default: depth image" << endl;
		}
		else{
			cout << this->hint_ << ": Sensor input mode: depth image (0)/pointcloud (1). Your option: " << this->sensorInputMode_ << endl;
		}		

		// localization mode
		if (not this->get_parameter("localization_mode", this->localizationMode_)){
			this->localizationMode_ = 0;
			cout << this->hint_ << ": No localization mode option. Use default: pose" << endl;
		}
		else{
			cout << this->hint_ << ": Localizaiton mode: pose (0)/odom (1). Your option: " << this->localizationMode_ << endl;
		}

		// depth topic name
		if (not this->get_parameter("depth_image_topic", this->depthTopicName_)){
			this->depthTopicName_ = "/world/simple_tunnel_03/model/x500_depth_1/link/OakDLite/base_link/sensor/StereoOV7251/depth_image";
			cout << this->hint_ << ": No depth image topic name. Use default: /camera/depth/image_raw" << endl;
		}
		else{
			cout << this->hint_ << ": Depth topic: " << this->depthTopicName_ << endl;
		}

		// pointcloud topic name
		if (not this->get_parameter("point_cloud_topic", this->pointcloudTopicName_)){
			this->pointcloudTopicName_ = "/world/simple_tunnel_03/model/x500_depth_1/link/OakDLite/base_link/sensor/StereoOV7251/depth_image/points";
			cout << this->hint_ << ": No poincloud topic name. Use default: /camera/depth/points" << endl;
		}
		else{
			cout << this->hint_ << ": Pointcloud topic: " << this->pointcloudTopicName_ << endl;
		}

		if (this->localizationMode_ == 0){
			// odom topic name
			if (not this->get_parameter("pose_topic", this->poseTopicName_)){
				this->poseTopicName_ = "/px4_visualizer/px4_1/vehicle_pose";
				cout << this->hint_ << ": No pose topic name. Use default: /px4_visualizer/px4_1/vehicle_pose" << endl;
			}
			else{
				cout << this->hint_ << ": Pose topic: " << this->poseTopicName_ << endl;
			}			
		}

		if (this->localizationMode_ == 1){
			// pose topic name
			if (not this->get_parameter("odom_topic", this->odomTopicName_)){
				this->odomTopicName_ = "/px4_2/fmu/out/vehicle_odometry";
				cout << this->hint_ << ": No odom topic name. Use default: /px4_2/fmu/out/vehicle_odometry" << endl;
			}
			else{
				cout << this->hint_ << ": Odom topic: " << this->odomTopicName_ << endl;
			}
		}

		std::vector<double> robotSizeVec (3);
		if (not this->get_parameter("robot_size", robotSizeVec)){
			robotSizeVec = std::vector<double>{0.5, 0.5, 0.3};
		}
		else{
			cout << this->hint_ << ": robot size: " << "[" << robotSizeVec[0]  << ", " << robotSizeVec[1] << ", "<< robotSizeVec[2] << "]" << endl;
		}
		this->robotSize_(0) = robotSizeVec[0]; this->robotSize_(1) = robotSizeVec[1]; this->robotSize_(2) = robotSizeVec[2];

		std::vector<double> depthIntrinsics (4);
		if (not this->get_parameter("depth_intrinsics", depthIntrinsics)){
			cout << this->hint_ << ": Please check camera intrinsics!" << endl;
			exit(0);
		}
		else{
			this->fx_ = depthIntrinsics[0];
			this->fy_ = depthIntrinsics[1];
			this->cx_ = depthIntrinsics[2];
			this->cy_ = depthIntrinsics[3];
			cout << this->hint_ << ": fx, fy, cx, cy: " << "["  << this->fx_ << ", " << this->fy_  << ", " << this->cx_ << ", "<< this->cy_ << "]" << endl;
		}

		// depth scale factor
		if (not this->get_parameter("depth_scale_factor", this->depthScale_)){
			this->depthScale_ = 1000;
			cout << this->hint_ << ": No depth scale factor. Use default: 1000." << endl;
		}
		else{
			cout << this->hint_ << ": Depth scale factor: " << this->depthScale_ << endl;
		}

		// depth min value
		if (not this->get_parameter("depth_min_value", this->depthMinValue_)){
			this->depthMinValue_ = 0.2;
			cout << this->hint_ << ": No depth min value. Use default: 0.2 m." << endl;
		}
		else{
			cout << this->hint_ << ": Depth min value: " << this->depthMinValue_ << endl;
		}

		// depth max value
		if (not this->get_parameter("depth_max_value", this->depthMaxValue_)){
			this->depthMaxValue_ = 5.0;
			cout << this->hint_ << ": No depth max value. Use default: 5.0 m." << endl;
		}
		else{
			cout << this->hint_ << ": Depth depth max value: " << this->depthMaxValue_ << endl;
		}

		// depth filter margin
		if (not this->get_parameter("depth_filter_margin", this->depthFilterMargin_)){
			this->depthFilterMargin_ = 0;
			cout << this->hint_ << ": No depth filter margin. Use default: 0." << endl;
		}
		else{
			cout << this->hint_ << ": Depth filter margin: " << this->depthFilterMargin_ << endl;
		}

		// depth skip pixel
		if (not this->get_parameter("depth_skip_pixel", this->skipPixel_)){
			this->skipPixel_ = 1;
			cout << this->hint_ << ": No depth skip pixel. Use default: 1." << endl;
		}
		else{
			cout << this->hint_ << ": Depth skip pixel: " << this->skipPixel_ << endl;
		}

		// ------------------------------------------------------------------------------------
		// depth image columns
		if (not this->get_parameter("image_cols", this->imgCols_)){
			this->imgCols_ = 640;
			cout << this->hint_ << ": No depth image columns. Use default: 640." << endl;
		}
		else{
			cout << this->hint_ << ": Depth image columns: " << this->imgCols_ << endl;
		}

		// depth skip pixel
		if (not this->get_parameter("image_rows", this->imgRows_)){
			this->imgRows_ = 480;
			cout << this->hint_ << ": No depth image rows. Use default: 480." << endl;
		}
		else{
			cout << this->hint_ << ": Depth image rows: " << this->imgRows_ << endl;
		}
		this->projPoints_.resize(this->imgCols_ * this->imgRows_ / (this->skipPixel_ * this->skipPixel_));
		// ------------------------------------------------------------------------------------


		// transform matrix: body to camera
		std::vector<double> body2CamVec (16);
		if (not this->get_parameter("body_to_camera", body2CamVec)){
			RCLCPP_ERROR(this->get_logger(), "[OccMap]: Please check body to camera matrix!");
		}
		else{
			for (int i=0; i<4; ++i){
				for (int j=0; j<4; ++j){
					this->body2Cam_(i, j) = body2CamVec[i * 4 + j];
				}
			}
			// cout << this->hint_ << ": from body to camera: " << endl;
			// cout << this->body2Cam_ << endl;
		}

		// Raycast max length
		if (not this->get_parameter("raycast_max_length", this->raycastMaxLength_)){
			this->raycastMaxLength_ = 5.0;
			cout << this->hint_ << ": No raycast max length. Use default: 5.0." << endl;
		}
		else{
			cout << this->hint_ << ": Raycast max length: " << this->raycastMaxLength_ << endl;
		}

		// p hit
		double pHit;
		if (not this->get_parameter("p_hit", pHit)){
			pHit = 0.70;
			cout << this->hint_ << ": No p hit. Use default: 0.70." << endl;
		}
		else{
			cout << this->hint_ << ": P hit: " << pHit << endl;
		}
		this->pHitLog_ = this->logit(pHit);

		// p miss
		double pMiss;
		if (not this->get_parameter("p_miss", pMiss)){
			pMiss = 0.35;
			cout << this->hint_ << ": No p miss. Use default: 0.35." << endl;
		}
		else{
			cout << this->hint_ << ": P miss: " << pMiss << endl;
		}
		this->pMissLog_ = this->logit(pMiss);

		// p min
		double pMin;
		if (not this->get_parameter("p_min", pMin)){
			pMin = 0.12;
			cout << this->hint_ << ": No p min. Use default: 0.12." << endl;
		}
		else{
			cout << this->hint_ << ": P min: " << pMin << endl;
		}
		this->pMinLog_ = this->logit(pMin);

		// p max
		double pMax;
		if (not this->get_parameter("p_max", pMax)){
			pMax = 0.97;
			cout << this->hint_ << ": No p max. Use default: 0.97." << endl;
		}
		else{
			cout << this->hint_ << ": P max: " << pMax << endl;
		}
		this->pMaxLog_ = this->logit(pMax);

		// p occ
		double pOcc;
		if (not this->get_parameter("p_occ", pOcc)){
			pOcc = 0.80;
			cout << this->hint_ << ": No p occ. Use default: 0.80." << endl;
		}
		else{
			cout << this->hint_ << ": P occ: " << pOcc << endl;
		}
		this->pOccLog_ = this->logit(pOcc);


		// map resolution
		if (not this->get_parameter("map_resolution", this->mapRes_)){
			this->mapRes_ = 0.1;
			cout << this->hint_ << ": No map resolution. Use default: 0.1." << endl;
		}
		else{
			cout << this->hint_ << ": Map resolution: " << this->mapRes_ << endl;
		}

		// ground height
		if (not this->get_parameter("ground_height", this->groundHeight_)){
			this->groundHeight_ = 0.0;
			cout << this->hint_ << ": No ground height. Use default: 0.0." << endl;
		}
		else{
			cout << this->hint_ << ": Ground height: " << this->groundHeight_ << endl;
		}


		// map size
		std::vector<double> mapSizeVec (3);
		if (not this->get_parameter("map_size", mapSizeVec)){
			mapSizeVec[0] = 20; mapSizeVec[1] = 20; mapSizeVec[2] = 3;
			cout << this->hint_ << ": No map size. Use default: [20, 20, 3]." << endl;
		}
		else{
			this->mapSize_(0) = mapSizeVec[0];
			this->mapSize_(1) = mapSizeVec[1];
			this->mapSize_(2) = mapSizeVec[2];

			// init min max
			this->mapSizeMin_(0) = -mapSizeVec[0]/2; this->mapSizeMax_(0) = mapSizeVec[0]/2;
			this->mapSizeMin_(1) = -mapSizeVec[1]/2; this->mapSizeMax_(1) = mapSizeVec[1]/2;
			this->mapSizeMin_(2) = this->groundHeight_; this->mapSizeMax_(2) = this->groundHeight_ + mapSizeVec[2];
			
			// min max for voxel
			this->mapVoxelMin_(0) = 0; this->mapVoxelMax_(0) = ceil(mapSizeVec[0]/this->mapRes_);
			this->mapVoxelMin_(1) = 0; this->mapVoxelMax_(1) = ceil(mapSizeVec[1]/this->mapRes_);
			this->mapVoxelMin_(2) = 0; this->mapVoxelMax_(2) = ceil(mapSizeVec[2]/this->mapRes_);

			// reserve vector for variables
			int reservedSize = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
			this->countHitMiss_.resize(reservedSize, 0);
			this->countHit_.resize(reservedSize, 0);
			this->occupancy_.resize(reservedSize, this->pMinLog_-this->UNKNOWN_FLAG_);
			this->occupancyInflated_.resize(reservedSize, false);
			this->flagTraverse_.resize(reservedSize, -1);
			this->flagRayend_.resize(reservedSize, -1);

			cout << this->hint_ << ": Map size: " << "[" << mapSizeVec[0] << ", " << mapSizeVec[1] << ", " << mapSizeVec[2] << "]" << endl;
		}

		// local update range
		std::vector<double> localUpdateRangeVec;
		if (not this->get_parameter("local_update_range", localUpdateRangeVec)){
			localUpdateRangeVec = std::vector<double>{5.0, 5.0, 3.0};
			cout << this->hint_ << ": No local update range. Use default: [5.0, 5.0, 3.0] m." << endl;
		}
		else{
			cout << this->hint_ << ": Local update range: " << "[" << localUpdateRangeVec[0] << ", " << localUpdateRangeVec[1] << ", " << localUpdateRangeVec[2] << "]" << endl;
		}
		this->localUpdateRange_(0) = localUpdateRangeVec[0]; this->localUpdateRange_(1) = localUpdateRangeVec[1]; this->localUpdateRange_(2) = localUpdateRangeVec[2];


		// local bound inflate factor
		if (not this->get_parameter("local_bound_inflation", this->localBoundInflate_)){
			this->localBoundInflate_ = 0.0;
			cout << this->hint_ << ": No local bound inflate. Use default: 0.0 m." << endl;
		}
		else{
			cout << this->hint_ << ": Local bound inflate: " << this->localBoundInflate_ << endl;
		}

		// whether to clean local map
		if (not this->get_parameter("clean_local_map", this->cleanLocalMap_)){
			this->cleanLocalMap_ = true;
			cout << this->hint_ << ": No clean local map option. Use default: true." << endl;
		}
		else{
			cout << this->hint_ << ": Clean local map option is set to: " << this->cleanLocalMap_ << endl; 
		}

		// absolute dir of prebuilt map file (.pcd)
		if (not this->get_parameter("prebuilt_map_directory", this->prebuiltMapDir_)){
			this->prebuiltMapDir_ = "";
			cout << this->hint_ << ": Not using prebuilt map." << endl;
		}
		else{
			cout << this->hint_ << ": the prebuilt map absolute dir is found: " << this->prebuiltMapDir_ << endl;
		}

		// local map size (visualization)
		std::vector<double> localMapSizeVec;
		if (not this->get_parameter("local_map_size", localMapSizeVec)){
			localMapSizeVec = std::vector<double>{10.0, 10.0, 2.0};
			cout << this->hint_ << ": No local map size. Use default: [10.0, 10.0, 3.0] m." << endl;
		}
		else{
			cout << this->hint_ << ": Local map size: " << "[" << localMapSizeVec[0] << ", " << localMapSizeVec[1] << ", " << localMapSizeVec[2] << "]" << endl;
		}
		this->localMapSize_(0) = localMapSizeVec[0]/2; this->localMapSize_(1) = localMapSizeVec[1]/2; this->localMapSize_(2) = localMapSizeVec[2]/2;
		this->localMapVoxel_(0) = int(ceil(localMapSizeVec[0]/(2*this->mapRes_))); this->localMapVoxel_(1) = int(ceil(localMapSizeVec[1]/(2*this->mapRes_))); this->localMapVoxel_(2) = int(ceil(localMapSizeVec[2]/(2*this->mapRes_)));

		// max vis height
		if (not this->get_parameter("max_height_visualization", this->maxVisHeight_)){
			this->maxVisHeight_ = 3.0;
			cout << this->hint_ << ": No max visualization height. Use default: 3.0 m." << endl;
		}
		else{
			cout << this->hint_ << ": Max visualization height: " << this->maxVisHeight_ << endl;
		}

		// visualize global map
		if (not this->get_parameter("visualize_global_map", this->visGlobalMap_)){
			this->visGlobalMap_ = false;
			cout << this->hint_ << ": No visualize map option. Use default: visualize local map." << endl;
		}
		else{
			cout << this->hint_ << ": Visualize map option. local (0)/global (1): " << this->visGlobalMap_ << endl;
		}

		// verbose
		if (not this->get_parameter("verbose", this->verbose_)){
			this->verbose_ = true;
			cout << this->hint_ << ": No verbose option. Use default: check update info." << endl;
		}
		else{
			if (not this->verbose_){
				cout << this->hint_ << ": Not display messages" << endl;
			}
			else{
				cout << this->hint_ << ": Display messages" << endl;
			}
		}


	}

	void occMap::initPrebuiltMap(){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

		if (pcl::io::loadPCDFile<pcl::PointXYZ> (this->prebuiltMapDir_, *cloud) == -1) //* load the file
		{
			cout << this->hint_ << ": No prebuilt map found/not using the prebuilt map." << endl;
		}
		else {
			cout << this->hint_ << ": Map loaded with " << cloud->size() << " data points. " << endl;
			int address;
			Eigen::Vector3i pointIndex;
			Eigen::Vector3d pointPos;
			Eigen::Vector3i inflateIndex;
			int inflateAddress;

			// update occupancy info
			int xInflateSize = ceil(this->robotSize_(0)/(2*this->mapRes_));
			int yInflateSize = ceil(this->robotSize_(1)/(2*this->mapRes_));
			int zInflateSize = ceil(this->robotSize_(2)/(2*this->mapRes_));

			Eigen::Vector3d currMapRangeMin (0.0, 0.0, 0.0);
			Eigen::Vector3d currMapRangeMax (0.0, 0.0, 0.0);

			const int  maxIndex = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
			for (const auto& point: *cloud)
			{
				address = this->posToAddress(point.x, point.y, point.z);
				pointPos(0) = point.x; pointPos(1) = point.y; pointPos(2) = point.z;
				this->posToIndex(pointPos, pointIndex);

				this->occupancy_[address] = this->pMaxLog_;
				// update map range
				if (pointPos(0) < currMapRangeMin(0)){
					currMapRangeMin(0) = pointPos(0);
				}

				if (pointPos(0) > currMapRangeMax(0)){
					currMapRangeMax(0) = pointPos(0);
				}

				if (pointPos(1) < currMapRangeMin(1)){
					currMapRangeMin(1) = pointPos(1);
				}

				if (pointPos(1) > currMapRangeMax(1)){
					currMapRangeMax(1) = pointPos(1);
				}

				if (pointPos(2) < currMapRangeMin(2)){
					currMapRangeMin(2) = pointPos(2);
				}

				if (pointPos(2) > currMapRangeMax(2)){
					currMapRangeMax(2) = pointPos(2);
				}

				for (int ix=-xInflateSize; ix<=xInflateSize; ++ix){
					for (int iy=-yInflateSize; iy<=yInflateSize; ++iy){
						for (int iz=-zInflateSize; iz<=zInflateSize; ++iz){
							inflateIndex(0) = pointIndex(0) + ix;
							inflateIndex(1) = pointIndex(1) + iy;
							inflateIndex(2) = pointIndex(2) + iz;
							inflateAddress = this->indexToAddress(inflateIndex);
							if ((inflateAddress < 0) or (inflateAddress > maxIndex)){
								continue; // those points are not in the reserved map
							} 
							this->occupancyInflated_[inflateAddress] = true;
						}
					}
				}
			}
			this->currMapRangeMin_ = currMapRangeMin;
			this->currMapRangeMax_ = currMapRangeMax;
		}
	}

	void occMap::registerCallback(){
		RCLCPP_INFO(this->get_logger(), "[RegisterCallback] Set up point cloud and pose synchronization with relaxed timing");
		// RCLCPP_INFO(this->get_logger(), "LocalizationMode: %d", this->localizationMode_);

		// Add direct depth image subscription without synchronization for debug
		if (this->sensorInputMode_ == 0) {
			RCLCPP_INFO(this->get_logger(), "[RegisterCallback] Setting up direct depth image subscription to: %s", this->depthTopicName_.c_str());
			
			// Create a direct subscription to depth images that doesn't require synchronization
			this->directDepthSub_ = this->create_subscription<sensor_msgs::msg::Image>(
				this->depthTopicName_,
				rclcpp::QoS(10),
				[this](const sensor_msgs::msg::Image::SharedPtr msg) {
					RCLCPP_INFO(this->get_logger(), "[DirectDepthCB] Received depth image with timestamp %d.%d, size %dx%d",
						   msg->header.stamp.sec, msg->header.stamp.nanosec, msg->width, msg->height);
					
					try {
						// Convert depth image from ROS message
						cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
						this->depthImage_ = cv_ptr->image;
						
						// For testing - set flag to update occupancy map
						RCLCPP_INFO(this->get_logger(), "[DirectDepthCB] Depth image converted successfully, triggering map update");
						this->occNeedUpdate_ = true;
						
					} catch (cv_bridge::Exception& e) {
						RCLCPP_ERROR(this->get_logger(), "[DirectDepthCB] cv_bridge exception: %s", e.what());
					} catch (std::exception& e) {
						RCLCPP_ERROR(this->get_logger(), "[DirectDepthCB] exception: %s", e.what());
					}
				});
		}

		if (this->sensorInputMode_ == 0){
			RCLCPP_INFO(this->get_logger(), "SensorInput: %d", this->sensorInputMode_);
			
			// depth pose callback
			this->depthSub_.reset(new message_filters::Subscriber<sensor_msgs::msg::Image>(this, this->depthTopicName_, rclcpp::QoS(50).get_rmw_qos_profile()));
			if (this->localizationMode_ == 0){
				this->poseSub_.reset(new message_filters::Subscriber<geometry_msgs::msg::PoseStamped>(this, this->poseTopicName_, rclcpp::QoS(25).get_rmw_qos_profile()));
				this->depthPoseSync_.reset(new message_filters::Synchronizer<depthPoseSync>(depthPoseSync(100), *this->depthSub_, *this->poseSub_));
				this->depthPoseSync_->registerCallback(std::bind(&occMap::depthPoseCB, this, _1, _2));
			}
			else if (this->localizationMode_ == 1){
				this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::msg::Odometry>(this, this->odomTopicName_, rclcpp::QoS(25).get_rmw_qos_profile()));
				this->depthOdomSync_.reset(new message_filters::Synchronizer<depthOdomSync>(depthOdomSync(100), *this->depthSub_, *this->odomSub_));
				this->depthOdomSync_->registerCallback(std::bind(&occMap::depthOdomCB, this, _1, _2));
			}
			else{
				RCLCPP_ERROR(this->get_logger(), "[OccMap]: Invalid localization mode!");
				exit(0);
			}
		}
		else if (this->sensorInputMode_ == 1){
			RCLCPP_INFO(this->get_logger(), "SensorInput: %d", this->sensorInputMode_);

			// pointcloud callback
			this->pointcloudSub_.reset(new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(this, this->pointcloudTopicName_, rclcpp::QoS(50).get_rmw_qos_profile()));
			if (this->localizationMode_ == 0){
				this->poseSub_.reset(new message_filters::Subscriber<geometry_msgs::msg::PoseStamped>(this, this->poseTopicName_, rclcpp::QoS(25).get_rmw_qos_profile()));
				this->pointcloudPoseSync_.reset(new message_filters::Synchronizer<pointcloudPoseSync>(pointcloudPoseSync(100), *this->pointcloudSub_, *this->poseSub_));
						// Create a synchronizer with a larger queue size and more relaxed time constraints
				
				this->pointcloudPoseSync_->registerCallback(std::bind(&occMap::pointcloudPoseCB, this, _1, _2));
			}
			else if (this->localizationMode_ == 1){
				this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::msg::Odometry>(this, this->odomTopicName_, rclcpp::QoS(25).get_rmw_qos_profile()));
				this->pointcloudOdomSync_.reset(new message_filters::Synchronizer<pointcloudOdomSync>(pointcloudOdomSync(100), *this->pointcloudSub_, *this->odomSub_));
				this->pointcloudOdomSync_->registerCallback(std::bind(&occMap::pointcloudOdomCB, this, _1, _2));
			}
			else{
				RCLCPP_ERROR(this->get_logger(), "[OccMap]: Invalid localization mode!");
				exit(0);
			}
		}
		else{
			RCLCPP_ERROR(this->get_logger(), "[OccMap]: Invalid sensor input mode!");
			exit(0);
		}

		// occupancy update callback
		this->occTimer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&occMap::updateOccupancyCB, this));

		// map inflation callback
		this->inflateTimer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&occMap::inflateMapCB, this));

		// visualization callback
		this->visTimer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&occMap::visCB, this));
		this->visWorker_ = std::thread(&occMap::startVisualization, this);
		this->visWorker_.detach();
		// this->projPointsVisTimer_ = this->nh_.createTimer(ros::Duration(0.1), &occMap::projPointsVisCB, this);
		// this->mapVisTimer_ = this->nh_.createTimer(ros::Duration(0.15), &occMap::mapVisCB, this);
		// this->inflatedMapVisTimer_ = this->nh_.createTimer(ros::Duration(0.15), &occMap::inflatedMapVisCB, this);
		// this->map2DVisTimer_ = this->nh_.createTimer(ros::Duration(0.15), &occMap::map2DVisCB, this);
	}

	void occMap::registerPub(){
		// Don't prefix with this->ns_ as namespace is already provided in the launch file
		this->depthCloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("depth_cloud", rclcpp::QoS(10));
		this->mapVisPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_map", rclcpp::QoS(10));
		this->inflatedMapVisPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("inflated_voxel_map", rclcpp::QoS(10));
		this->map2DPub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("twoD_occupancy_map", rclcpp::QoS(10));
		this->mapExploredPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("explored_voxel_map",rclcpp::QoS(10));
		// publish service
		this->collisionCheckServer_ = this->create_service<px4_map_manager::srv::CheckPosCollision>(
			"check_pos_collision",
			std::function<void(std::shared_ptr<px4_map_manager::srv::CheckPosCollision::Request>, std::shared_ptr<px4_map_manager::srv::CheckPosCollision::Response>)>(
				[this](std::shared_ptr<px4_map_manager::srv::CheckPosCollision::Request> req,
					std::shared_ptr<px4_map_manager::srv::CheckPosCollision::Response> res){
					this->checkCollision(*req, *res);
				}
			)
		);
		this->raycastServer_ = this->create_service<px4_map_manager::srv::RayCast>(
			"raycast",
			std::function<void(std::shared_ptr<px4_map_manager::srv::RayCast::Request>, std::shared_ptr<px4_map_manager::srv::RayCast::Response>)>(
				[this](std::shared_ptr<px4_map_manager::srv::RayCast::Request> req,
					std::shared_ptr<px4_map_manager::srv::RayCast::Response> res){
					this->getRayCast(*req, *res);
				}
			)
		);
	}

	bool occMap::checkCollision(px4_map_manager::srv::CheckPosCollision::Request& req, px4_map_manager::srv::CheckPosCollision::Response& res){
		if (req.inflated){
			res.occupied = this->isInflatedOccupied(Eigen::Vector3d (req.x, req.y, req.z));
		}
		else{
			res.occupied = this->isOccupied(Eigen::Vector3d (req.x, req.y, req.z));
		}

		return true;
	}

	bool occMap::getRayCast(px4_map_manager::srv::RayCast::Request& req, px4_map_manager::srv::RayCast::Response& res){
		double hres = req.hres * M_PI/180.0;
		int numHbeams = int(360/req.hres);
		double vres = double(((req.vfov_max - req.vfov_min)* M_PI/180.0)/(req.vbeams-1));
		double vStartAngle = req.vfov_min * M_PI/180.0;
		int numVbeams = req.vbeams;
		double range = req.range;
		Eigen::Vector3d start = this->position_;
		double starthAngle = req.start_angle;
		for (int h=0; h<numHbeams; ++h){
			double hAngle = starthAngle + double(h) * hres;
			Eigen::Vector3d hdirection (cos(hAngle), sin(hAngle), 0.0); // horizontal direction 
			for (int v=0; v<numVbeams; ++v){
				// get hit points
				double vAngle = vStartAngle + double(v) * vres;
				double vup = tan(vAngle);
				Eigen::Vector3d direction = hdirection;
				direction(2) += vup;
				direction /= direction.norm();
				Eigen::Vector3d hitPoint;
				bool success = this->castRay(start, direction, hitPoint, range, true);
				if (not success){
					hitPoint = start + range * direction;
				}
				for (int i=0; i<3; ++i){
					res.points.push_back(hitPoint(i));
				}
			}
		}
		return true;
	}
	
	void occMap::depthPoseCB(const sensor_msgs::msg::Image::ConstSharedPtr& img, const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose){
		// Log when callback is called
		RCLCPP_INFO(this->get_logger(), "[DepthPoseCB] CALLBACK TRIGGERED - Received depth image (%dx%d) and pose", 
		                img->width, img->height);
		
		// store current depth image
		cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
		if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
			(imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
		}
		RCLCPP_INFO(this->get_logger(), "[DepthPoseCB] Depth image encoding: %s, copied to depth_image_", img->encoding.c_str());
		imgPtr->image.copyTo(this->depthImage_);

		// store current position and orientation (camera)
		Eigen::Matrix4d camPoseMatrix;
		this->getCameraPose(pose, camPoseMatrix);

		this->position_(0) = camPoseMatrix(0, 3);
		this->position_(1) = camPoseMatrix(1, 3);
		this->position_(2) = camPoseMatrix(2, 3);
		this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);

		if (this->isInMap(this->position_)){
			this->occNeedUpdate_ = true;
			RCLCPP_INFO(this->get_logger(), "[DepthPoseCB] Position (%.2f, %.2f, %.2f) is within map bounds. occNeedUpdate = true", 
                      this->position_(0), this->position_(1), this->position_(2));
		}
		else{
			this->occNeedUpdate_ = false;
			RCLCPP_WARN(this->get_logger(), "[DepthPoseCB] Position (%.2f, %.2f, %.2f) is OUTSIDE map bounds! occNeedUpdate = false", 
                      this->position_(0), this->position_(1), this->position_(2));
			// Output map bounds for reference
			RCLCPP_INFO(this->get_logger(), "[DepthPoseCB] Map bounds: Min (%.2f, %.2f, %.2f), Max (%.2f, %.2f, %.2f)", 
                      this->mapSizeMin_(0), this->mapSizeMin_(1), this->mapSizeMin_(2),
                      this->mapSizeMax_(0), this->mapSizeMax_(1), this->mapSizeMax_(2));
		}
	}

	void occMap::depthOdomCB(const sensor_msgs::msg::Image::ConstSharedPtr& img, const nav_msgs::msg::Odometry::ConstSharedPtr& odom){
		// store current depth image
		cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
		if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
			(imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
		}
		imgPtr->image.copyTo(this->depthImage_);

		// store current position and orientation (camera)
		Eigen::Matrix4d camPoseMatrix;
		this->getCameraPose(odom, camPoseMatrix);

		this->position_(0) = camPoseMatrix(0, 3);
		this->position_(1) = camPoseMatrix(1, 3);
		this->position_(2) = camPoseMatrix(2, 3);
		this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);

		if (this->isInMap(this->position_)){
			this->occNeedUpdate_ = true;
		}
		else{
			this->occNeedUpdate_ = false;
		}
	}

	void occMap::pointcloudPoseCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud, const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose){
		RCLCPP_INFO(this->get_logger(), "[PointcloudPoseCB] Received pointcloud with %u points and pose", pointcloud->width * pointcloud->height);
		
		// directly get the point cloud
		pcl::PCLPointCloud2 pclPC2;
		pcl_conversions::toPCL(*pointcloud, pclPC2); // convert ros pointcloud2 to pcl pointcloud2
		pcl::fromPCLPointCloud2(pclPC2, this->pointcloud_);

		// store current position and orientation (camera)
		Eigen::Matrix4d camPoseMatrix;
		this->getCameraPose(pose, camPoseMatrix);

		this->position_(0) = camPoseMatrix(0, 3);
		this->position_(1) = camPoseMatrix(1, 3);
		this->position_(2) = camPoseMatrix(2, 3);
		this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);

		RCLCPP_INFO(this->get_logger(), "[PointcloudPoseCB] Position (%.2f, %.2f, %.2f)", 
			   this->position_(0), this->position_(1), this->position_(2));
		
		if (this->isInMap(this->position_)){
			this->occNeedUpdate_ = true;
			RCLCPP_INFO(this->get_logger(), "[PointcloudPoseCB] Position is within map bounds, occNeedUpdate = true");
		}
		else{
			this->occNeedUpdate_ = false;
			RCLCPP_WARN(this->get_logger(), "[PointcloudPoseCB] Position is OUTSIDE map bounds, occNeedUpdate = false");
			// Output map bounds for reference
			RCLCPP_INFO(this->get_logger(), "[PointcloudPoseCB] Map bounds: Min (%.2f, %.2f, %.2f), Max (%.2f, %.2f, %.2f)", 
				   this->mapSizeMin_(0), this->mapSizeMin_(1), this->mapSizeMin_(2),
				   this->mapSizeMax_(0), this->mapSizeMax_(1), this->mapSizeMax_(2));
		}
	}

	void occMap::pointcloudOdomCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud, const nav_msgs::msg::Odometry::ConstSharedPtr& odom){
		// directly get the point cloud
		pcl::PCLPointCloud2 pclPC2;
		pcl_conversions::toPCL(*pointcloud, pclPC2); // convert ros pointcloud2 to pcl pointcloud2
		pcl::fromPCLPointCloud2(pclPC2, this->pointcloud_);


		// store current position and orientation (camera)
		Eigen::Matrix4d camPoseMatrix;
		this->getCameraPose(odom, camPoseMatrix);

		this->position_(0) = camPoseMatrix(0, 3);
		this->position_(1) = camPoseMatrix(1, 3);
		this->position_(2) = camPoseMatrix(2, 3);
		this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);

		if (this->isInMap(this->position_)){
			this->occNeedUpdate_ = true;
		}
		else{
			this->occNeedUpdate_ = false;
		}
	}

	void occMap::updateOccupancyCB(){
		// Always log the status, to track if this timer callback is being called
		RCLCPP_INFO(this->get_logger(), "[UpdateOccupancyCB] Timer triggered. occNeedUpdate=%d, sensorInputMode=%d", 
			   this->occNeedUpdate_, this->sensorInputMode_);
		
		// Check the state of the depth image
		if (this->depthImage_.empty()) {
			RCLCPP_ERROR(this->get_logger(), "[UpdateOccupancyCB] depthImage_ is empty - no depth data received yet");
		}
		
		if (not this->occNeedUpdate_){
			RCLCPP_INFO(this->get_logger(), "[UpdateOccupancyCB] occNeedUpdate is false, returning without processing");
			return;
		}
		
		RCLCPP_INFO(this->get_logger(), "[UpdateOccupancyCB] Processing occupancy update, sensorInputMode = %d", this->sensorInputMode_);
		
		rclcpp::Time startTime, endTime;
		
		startTime = this->get_clock()->now();
		if (this->sensorInputMode_ == 0){
			// project 3D points from depth map
			RCLCPP_INFO(this->get_logger(), "[UpdateOccupancyCB] Calling projectDepthImage()");
			this->projectDepthImage();
			RCLCPP_INFO(this->get_logger(), "[UpdateOccupancyCB] projectDepthImage() completed, projPointsNum = %d", this->projPointsNum_);
		}
		else if (this->sensorInputMode_ == 1){
			// directly get pointcloud
			RCLCPP_INFO(this->get_logger(), "[UpdateOccupancyCB] Calling getPointcloud()");
			this->getPointcloud();
		}

		// raycasting and update occupancy
		this->raycastUpdate();


		// clear local map
		if (this->cleanLocalMap_){
			this->cleanLocalMap();
		}
		
		// inflate map
		// this->inflateLocalMap();
		// endTime = rclcpp::Time::now();
		endTime = this->get_clock()->now();
		if (this->verbose_){
			cout << this->hint_ << ": Occupancy update time: " << (endTime - startTime).seconds() << " s." << endl;
		}
		this->occNeedUpdate_ = false;
		this->mapNeedInflate_ = true;
	}

	void occMap::inflateMapCB(){
		// inflate local map:
		if (this->mapNeedInflate_){
			this->inflateLocalMap();
			this->mapNeedInflate_ = false;
			this->esdfNeedUpdate_ = true;
		}
	}


	void occMap::projectDepthImage(){
		RCLCPP_INFO(this->get_logger(), "[ProjectDepthImage] Starting depth image processing");
		
		this->projPointsNum_ = 0;

		// First check if depthImage_ is valid
		if (this->depthImage_.empty()) {
			RCLCPP_ERROR(this->get_logger(), "[ProjectDepthImage] ERROR: depthImage_ is empty!");
			return;
		}

		int cols = this->depthImage_.cols;
		int rows = this->depthImage_.rows;
		RCLCPP_INFO(this->get_logger(), "[ProjectDepthImage] Depth image size: %dx%d", cols, rows);
		
		uint16_t* rowPtr;

		Eigen::Vector3d currPointCam, currPointMap;
		double depth;
		const double inv_factor = 1.0 / this->depthScale_;
		const double inv_fx = 1.0 / this->fx_;
		const double inv_fy = 1.0 / this->fy_;

		// Log camera parameters
		RCLCPP_INFO(this->get_logger(), "[ProjectDepthImage] Camera params: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f, scale=%.2f",
			   this->fx_, this->fy_, this->cx_, this->cy_, this->depthScale_);

		int pointsProcessed = 0;
		int validPointsFound = 0;

		// iterate through each pixel in the depth image
		for (int v=this->depthFilterMargin_; v<rows-this->depthFilterMargin_; v=v+this->skipPixel_){ // row
			rowPtr = this->depthImage_.ptr<uint16_t>(v) + this->depthFilterMargin_;
			for (int u=this->depthFilterMargin_; u<cols-this->depthFilterMargin_; u=u+this->skipPixel_){ // column
				pointsProcessed++;
				depth = (*rowPtr) * inv_factor;
				
				if (*rowPtr == 0) {
					depth = this->raycastMaxLength_ + 0.1;
				} else if (depth < this->depthMinValue_) {
					rowPtr = rowPtr + this->skipPixel_;
					continue;
				} else if (depth > this->depthMaxValue_ ) {
					depth = this->raycastMaxLength_ + 0.1;
				}

				rowPtr = rowPtr + this->skipPixel_;

				// get 3D point in camera frame
				currPointCam(0) = (u - this->cx_) * depth * inv_fx;
				currPointCam(1) = (v - this->cy_) * depth * inv_fy;
				currPointCam(2) = depth;
				currPointMap = this->orientation_ * currPointCam + this->position_; // transform to map coordinate

				if (this->useFreeRegions_){ // this region will not be updated and directly set to free
					if (this->isInHistFreeRegions(currPointMap)){
						continue;
					}
				}

				// store current point
				this->projPoints_[this->projPointsNum_] = currPointMap;
				this->projPointsNum_ = this->projPointsNum_ + 1;
				validPointsFound++;
			}
		}
		
		RCLCPP_INFO(this->get_logger(), "[ProjectDepthImage] Processed %d points, found %d valid points, final projPointsNum_=%d", 
			   pointsProcessed, validPointsFound, this->projPointsNum_);
	}

	void occMap::getPointcloud(){
		int nPoints = this->pointcloud_.points.size();
		RCLCPP_INFO(this->get_logger(), "[GetPointcloud] Processing %d points from pointcloud", nPoints);
		
		// Set projPointsNum_ to the number of points in the cloud
		this->projPointsNum_ = nPoints;

		// Make sure we have enough space in projPoints_ array
		if (this->projPoints_.size() < this->projPointsNum_) {
			RCLCPP_INFO(this->get_logger(), "[GetPointcloud] Resizing projPoints_ array to %d", this->projPointsNum_);
			this->projPoints_.resize(this->projPointsNum_);
		}

		for (int i=0; i<this->projPointsNum_; ++i){
			Eigen::Vector3d point;
			point(0) = (double)this->pointcloud_.points[i].x;
			point(1) = (double)this->pointcloud_.points[i].y;
			point(2) = (double)this->pointcloud_.points[i].z;
			Eigen::Vector3d currPointMap = this->orientation_ * point + this->position_;
			this->projPoints_[i] = currPointMap;
		}
		
		RCLCPP_INFO(this->get_logger(), "[GetPointcloud] Set projPointsNum_ = %d", this->projPointsNum_);
	}

	void occMap::raycastUpdate(){
		if (this->projPointsNum_ == 0){
			return;
		}
		this->raycastNum_ += 1;

		// record local bound of update
		double xmin, xmax, ymin, ymax, zmin, zmax;
		xmin = xmax = this->position_(0);
		ymin = ymax = this->position_(1);
		zmin = zmax = this->position_(2);

		// iterate through each projected points, perform raycasting and update occupancy
		Eigen::Vector3d currPoint;
		bool pointAdjusted;
		int rayendVoxelID, raycastVoxelID;
		double length;
		for (int i=0; i<this->projPointsNum_; ++i){
			currPoint = this->projPoints_[i];
			if (std::isnan(currPoint(0)) or std::isnan(currPoint(1)) or std::isnan(currPoint(2))){
				continue; // nan points can happen when we are using pointcloud as input
			}

			pointAdjusted = false;
			// check whether the point is in reserved map range
			if (not this->isInMap(currPoint)){
				currPoint = this->adjustPointInMap(currPoint);
				pointAdjusted = true;
			}

			// check whether the point exceeds the maximum raycasting length
			length = (currPoint - this->position_).norm();
			if (length > this->raycastMaxLength_){
				currPoint = this->adjustPointRayLength(currPoint);
				pointAdjusted = true;
			}


			// update local bound
			if (currPoint(0) < xmin){xmin = currPoint(0);}
			if (currPoint(1) < ymin){ymin = currPoint(1);}
			if (currPoint(2) < zmin){zmin = currPoint(2);}
			if (currPoint(0) > xmax){xmax = currPoint(0);}
			if (currPoint(1) > ymax){ymax = currPoint(1);}
			if (currPoint(2) > zmax){zmax = currPoint(2);}

			// update occupancy itself update information
			rayendVoxelID = this->updateOccupancyInfo(currPoint, not pointAdjusted); // point adjusted is free, not is occupied

			// check whether the voxel has already been updated, so no raycasting needed
			// rayendVoxelID = this->posToAddress(currPoint);
			if (this->flagRayend_[rayendVoxelID] == this->raycastNum_){
				continue; // skip
			}
			else{
				this->flagRayend_[rayendVoxelID] = this->raycastNum_;
			}



			// raycasting for update occupancy
			this->raycaster_.setInput(currPoint/this->mapRes_, this->position_/this->mapRes_);
			Eigen::Vector3d rayPoint, actualPoint;
			while (this->raycaster_.step(rayPoint)){
				actualPoint = rayPoint;
				actualPoint(0) += 0.5;
				actualPoint(1) += 0.5;
				actualPoint(2) += 0.5;
				actualPoint *= this->mapRes_;
				raycastVoxelID = this->updateOccupancyInfo(actualPoint, false);

				// raycastVoxelID = this->posToAddress(actualPoint);
				if (this->flagTraverse_[raycastVoxelID] == this->raycastNum_){
					break;
				}
				else{
					this->flagTraverse_[raycastVoxelID] = this->raycastNum_;
				}

			}
		}

		// store local bound and inflate local bound (inflate is for ESDF update)
		this->posToIndex(Eigen::Vector3d (xmin, ymin, zmin), this->localBoundMin_);
		this->posToIndex(Eigen::Vector3d (xmax, ymax, zmax), this->localBoundMax_);
		this->localBoundMin_ -= int(ceil(this->localBoundInflate_/this->mapRes_)) * Eigen::Vector3i(1, 1, 0); // inflate in x y direction
		this->localBoundMax_ += int(ceil(this->localBoundInflate_/this->mapRes_)) * Eigen::Vector3i(1, 1, 0); 
		this->boundIndex(this->localBoundMin_); // since inflated, need to bound if not in reserved range
		this->boundIndex(this->localBoundMax_);


		// update occupancy in the cache
		double logUpdateValue;
		int cacheAddress, hit, miss;
		while (not this->updateVoxelCache_.empty()){
			Eigen::Vector3i cacheIdx = this->updateVoxelCache_.front();
			this->updateVoxelCache_.pop();
			cacheAddress = this->indexToAddress(cacheIdx);

			hit = this->countHit_[cacheAddress];
			miss = this->countHitMiss_[cacheAddress] - hit;

			if (hit >= miss and hit != 0){
				logUpdateValue = this->pHitLog_;
			}
			else{
				logUpdateValue = this->pMissLog_;
			}
			this->countHit_[cacheAddress] = 0; // clear hit
			this->countHitMiss_[cacheAddress] = 0; // clear hit and miss

			// check whether point is in the local update range
			if (not this->isInLocalUpdateRange(cacheIdx)){
				continue; // do not update if not in the range
			}

			if (this->useFreeRegions_){ // current used in simulation, this region will not be updated and directly set to free
				Eigen::Vector3d pos;
				this->indexToPos(cacheIdx, pos);
				if (this->isInHistFreeRegions(pos)){
					this->occupancy_[cacheAddress] = this->pMinLog_;
					continue;
				}
			}

			// update occupancy info
			if ((logUpdateValue >= 0) and (this->occupancy_[cacheAddress] >= this->pMaxLog_)){
				continue; // not increase p if max clamped
			}
			else if ((logUpdateValue <= 0) and (this->occupancy_[cacheAddress] == this->pMinLog_)){
				continue; // not decrease p if min clamped
			}
			else if ((logUpdateValue <= 0) and (this->occupancy_[cacheAddress] < this->pMinLog_)){
				this->occupancy_[cacheAddress] = this->pMinLog_; // if unknown set it free (prior), 
				continue;
			}

			this->occupancy_[cacheAddress] = std::min(std::max(this->occupancy_[cacheAddress]+logUpdateValue, this->pMinLog_), this->pMaxLog_);

			// update the entire map range (if it is not unknown)
			if (not this->isUnknown(cacheIdx)){
				Eigen::Vector3d cachePos;
				this->indexToPos(cacheIdx, cachePos);
				if (cachePos(0) > this->currMapRangeMax_(0)){
					this->currMapRangeMax_(0) = cachePos(0);
				}
				else if (cachePos(0) < this->currMapRangeMin_(0)){
					this->currMapRangeMin_(0) = cachePos(0);
				}

				if (cachePos(1) > this->currMapRangeMax_(1)){
					this->currMapRangeMax_(1) = cachePos(1);
				}
				else if (cachePos(1) < this->currMapRangeMin_(1)){
					this->currMapRangeMin_(1) = cachePos(1);
				}

				if (cachePos(2) > this->currMapRangeMax_(2)){
					this->currMapRangeMax_(2) = cachePos(2);
				}
				else if (cachePos(2) < this->currMapRangeMin_(2)){
					this->currMapRangeMin_(2) = cachePos(2);
				}
			}
		}

	}

	void occMap::cleanLocalMap(){
		Eigen::Vector3i posIndex;
		this->posToIndex(this->position_, posIndex);
		Eigen::Vector3i innerMinBBX = posIndex - this->localMapVoxel_;
		Eigen::Vector3i innerMaxBBX = posIndex + this->localMapVoxel_;
		Eigen::Vector3i outerMinBBX = innerMinBBX - Eigen::Vector3i(5, 5, 5);
		Eigen::Vector3i outerMaxBBX = innerMaxBBX + Eigen::Vector3i(5, 5, 5);		this->boundIndex(innerMinBBX);
		this->boundIndex(innerMaxBBX);
		this->boundIndex(outerMinBBX);
		this->boundIndex(outerMaxBBX);

		// clear x axis
		for (int y=outerMinBBX(1); y<=outerMaxBBX(1); ++y){
			for (int z=outerMinBBX(2); z<=outerMaxBBX(2); ++z){
				for (int x=outerMinBBX(0); x<=innerMinBBX(0); ++x){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}

				for (int x=innerMaxBBX(0); x<=outerMaxBBX(0); ++x){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;					
				}
			}
		}

		// clear y axis
		for (int x=outerMinBBX(0); x<=outerMaxBBX(0); ++x){
			for (int z=outerMinBBX(2); z<=outerMaxBBX(2); ++z){
				for (int y=outerMinBBX(1); y<=innerMinBBX(1); ++y){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}

				for (int y=innerMaxBBX(1); y<=outerMaxBBX(1); ++y){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}
			}
		}

		// clear z axis
		for (int x=outerMinBBX(0); x<=outerMaxBBX(0); ++x){
			for (int y=outerMinBBX(1); y<=outerMaxBBX(1); ++y){
				for (int z=outerMinBBX(2); z<=innerMinBBX(2); ++z){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}

				for (int z=innerMaxBBX(2); z<=outerMaxBBX(2); ++z){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}
			}
		}
	}

	void occMap::inflateLocalMap(){
		int xmin = this->localBoundMin_(0);
		int xmax = this->localBoundMax_(0);
		int ymin = this->localBoundMin_(1);
		int ymax = this->localBoundMax_(1);
		int zmin = this->localBoundMin_(2);
		int zmax = this->localBoundMax_(2);
		Eigen::Vector3i clearIndex;
		// clear previous data in current data range
		for (int x=xmin; x<=xmax; ++x){
			for (int y=ymin; y<=ymax; ++y){
				for (int z=zmin; z<=zmax; ++z){
					clearIndex(0) = x; clearIndex(1) = y; clearIndex(2) = z;
					this->occupancyInflated_[this->indexToAddress(clearIndex)] = false;
				}
			}
		}

		int xInflateSize = ceil(this->robotSize_(0)/(2*this->mapRes_));
		int yInflateSize = ceil(this->robotSize_(1)/(2*this->mapRes_));
		int zInflateSize = ceil(this->robotSize_(2)/(2*this->mapRes_));

		// inflate based on current occupancy
		Eigen::Vector3i pointIndex, inflateIndex;
		int inflateAddress;
		const int  maxIndex = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
		for (int x=xmin; x<=xmax; ++x){
			for (int y=ymin; y<=ymax; ++y){
				for (int z=zmin; z<=zmax; ++z){
					pointIndex(0) = x; pointIndex(1) = y; pointIndex(2) = z;
					if (this->isOccupied(pointIndex)){
						for (int ix=-xInflateSize; ix<=xInflateSize; ++ix){
							for (int iy=-yInflateSize; iy<=yInflateSize; ++iy){
								for (int iz=-zInflateSize; iz<=zInflateSize; ++iz){
									inflateIndex(0) = pointIndex(0) + ix;
									inflateIndex(1) = pointIndex(1) + iy;
									inflateIndex(2) = pointIndex(2) + iz;
									inflateAddress = this->indexToAddress(inflateIndex);
									if ((inflateAddress < 0) or (inflateAddress > maxIndex)){
										continue; // those points are not in the reserved map
									} 
									this->occupancyInflated_[inflateAddress] = true;
								}
							}
						}
					}
				}
			}
		}
	}



	void occMap::visCB(){
		RCLCPP_DEBUG(this->get_logger(), "[VisCB] Visualization callback triggered");
		// Add the missing publish calls
		// this->publishProjPoints();
		// this->publishMap();
		this->publishInflatedMap();
		// this->publish2DOccupancyGrid();
	}

	void occMap::projPointsVisCB(){
		this->publishProjPoints();
	}

	void occMap::mapVisCB(){
		this->publishMap();
	}
	void occMap::inflatedMapVisCB(){
		this->publishInflatedMap();
	}

	void occMap::map2DVisCB(){
		this->publish2DOccupancyGrid();
	}
	
	void occMap::startVisualization(){
		rclcpp::Rate r (10);
		while (rclcpp::ok()){
			// pcl::PointCloud<pcl::PointXYZ> mapCloud, inflatedMapCloud, exploredMapCloud, depthCloud;
			// this->getMapVisData(mapCloud, inflatedMapCloud, exploredMapCloud, depthCloud);
			// sensor_msgs::PointCloud2 mapCloudMsg,inflatedMapCloudMsg, exploredMapCloudMsg, depthCloudMsg;
			// pcl::toROSMsg(mapCloud, mapCloudMsg);
			// pcl::toROSMsg(inflatedMapCloud, inflatedMapCloudMsg);
			// pcl::toROSMsg(exploredMapCloud, exploredMapCloudMsg);
			// pcl::toROSMsg(depthCloud, depthCloudMsg);

			// this->inflatedMapVisPub_.publish(inflatedMapCloudMsg);
			// this->mapVisPub_.publish(mapCloudMsg);
			// this->mapExploredPub_.publish(exploredMapCloudMsg);
			// this->depthCloudPub_.publish(depthCloudMsg);
			this->publishProjPoints();
			this->publishMap();
			// this->publishInflatedMap();
			this->publish2DOccupancyGrid();
			r.sleep();	
		}
	}

	void occMap::getMapVisData(pcl::PointCloud<pcl::PointXYZ>& mapCloud, pcl::PointCloud<pcl::PointXYZ>& inflatedMapCloud, pcl::PointCloud<pcl::PointXYZ>& exploredMapCloud, pcl::PointCloud<pcl::PointXYZ>& depthCloud){
		pcl::PointXYZ pt;
		Eigen::Vector3d minRange, maxRange;
		if (this->visGlobalMap_){
			// minRange = this->mapSizeMin_;
			// maxRange = this->mapSizeMax_;
			minRange = this->currMapRangeMin_;
			maxRange = this->currMapRangeMax_;
		}
		else{
			minRange = this->position_ - localMapSize_;
			maxRange = this->position_ + localMapSize_;
			minRange(2) = this->groundHeight_;
		}
		Eigen::Vector3i minRangeIdx, maxRangeIdx;
		this->posToIndex(minRange, minRangeIdx);
		this->posToIndex(maxRange, maxRangeIdx);
		this->boundIndex(minRangeIdx);
		this->boundIndex(maxRangeIdx);

		for (int i=0; i<this->projPointsNum_; ++i){
			pt.x = this->projPoints_[i](0);
			pt.y = this->projPoints_[i](1);
			pt.z = this->projPoints_[i](2);
			depthCloud.push_back(pt);
		}

		depthCloud.width = depthCloud.points.size();
		depthCloud.height = 1;
		depthCloud.is_dense = true;
		depthCloud.header.frame_id = "map";

		for (int x=minRangeIdx(0); x<=maxRangeIdx(0); ++x){
			for (int y=minRangeIdx(1); y<=maxRangeIdx(1); ++y){
				for (int z=minRangeIdx(2); z<=maxRangeIdx(2); ++z){
					Eigen::Vector3i pointIdx (x, y, z);

					if (this->isOccupied(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						if (point(2) <= this->maxVisHeight_){
							pt.x = point(0);
							pt.y = point(1);
							pt.z = point(2);
							mapCloud.push_back(pt);
						}
					}
					
					if (this->isInflatedOccupied(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						if (point(2) <= this->maxVisHeight_){
							pt.x = point(0);
							pt.y = point(1);
							pt.z = point(2);
							inflatedMapCloud.push_back(pt);
						}
					}

					// publish explored voxel map
					if(!this->isUnknown(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						pt.x = point(0);
						pt.y = point(1);
						pt.z = point(2);
						exploredMapCloud.push_back(pt);
					}
				}
			}
		}

		mapCloud.width = mapCloud.points.size();
		mapCloud.height = 1;
		mapCloud.is_dense = true;
		mapCloud.header.frame_id = "map";

		inflatedMapCloud.width = inflatedMapCloud.points.size();
		inflatedMapCloud.height = 1;
		inflatedMapCloud.is_dense = true;
		inflatedMapCloud.header.frame_id = "map";

		exploredMapCloud.width = exploredMapCloud.points.size();
		exploredMapCloud.height = 1;
		exploredMapCloud.is_dense = true;
		exploredMapCloud.header.frame_id = "map";
	}

	void occMap::publishProjPoints(){
		pcl::PointXYZ pt;
		pcl::PointCloud<pcl::PointXYZ> cloud;

		// RCLCPP_INFO(this->get_logger(), "[PublishProjPoints] Publishing depth cloud with %d points", this->projPointsNum_);
		
		for (int i=0; i<this->projPointsNum_; ++i){
			pt.x = this->projPoints_[i](0);
			pt.y = this->projPoints_[i](1);
			pt.z = this->projPoints_[i](2);
			cloud.push_back(pt);
		}

		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = true;
		cloud.header.frame_id = "map";
		// cloud.header.stamp = this->get_clock()->now().nanoseconds();

		sensor_msgs::msg::PointCloud2 cloudMsg;
		pcl::toROSMsg(cloud, cloudMsg);
		this->depthCloudPub_->publish(cloudMsg);
		
		// RCLCPP_INFO(this->get_logger(), "[PublishProjPoints] Published cloud.width = %d", cloud.width);
	}

	void occMap::publishMap(){
		pcl::PointXYZ pt;
		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::PointCloud<pcl::PointXYZ> exploredCloud;

		Eigen::Vector3d minRange, maxRange;
		if (this->visGlobalMap_){
			// minRange = this->mapSizeMin_;
			// maxRange = this->mapSizeMax_;
			minRange = this->currMapRangeMin_;
			maxRange = this->currMapRangeMax_;
		}
		else{
			minRange = this->position_ - localMapSize_;
			maxRange = this->position_ + localMapSize_;
			minRange(2) = this->groundHeight_;
		}
		Eigen::Vector3i minRangeIdx, maxRangeIdx;
		this->posToIndex(minRange, minRangeIdx);
		this->posToIndex(maxRange, maxRangeIdx);
		this->boundIndex(minRangeIdx);
		this->boundIndex(maxRangeIdx);

		for (int x=minRangeIdx(0); x<=maxRangeIdx(0); ++x){
			for (int y=minRangeIdx(1); y<=maxRangeIdx(1); ++y){
				for (int z=minRangeIdx(2); z<=maxRangeIdx(2); ++z){
					Eigen::Vector3i pointIdx (x, y, z);

					// if (this->occupancy_[this->indexToAddress(pointIdx)] > this->pMinLog_){
					if (this->isOccupied(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						if (point(2) <= this->maxVisHeight_){
							pt.x = point(0);
							pt.y = point(1);
							pt.z = point(2);
							cloud.push_back(pt);
						}
					}

					// publish explored voxel map
					if(!this->isUnknown(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						pt.x = point(0);
						pt.y = point(1);
						pt.z = point(2);
						exploredCloud.push_back(pt);
					}
				}
			}
		}

		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = true;
		cloud.header.frame_id = "map";
		// cloud.header.stamp = this->get_clock()->now().nanoseconds();

		exploredCloud.width = exploredCloud.points.size();
		exploredCloud.height = 1;
		exploredCloud.is_dense = true;
		exploredCloud.header.frame_id = "map";
		// exploredCloud.header.stamp = this->get_clock()->now().nanoseconds();

		sensor_msgs::msg::PointCloud2 cloudMsg;
		sensor_msgs::msg::PointCloud2 exploredCloudMsg;
		pcl::toROSMsg(cloud, cloudMsg);
		pcl::toROSMsg(exploredCloud, exploredCloudMsg);
		this->mapVisPub_->publish(cloudMsg);
		this->mapExploredPub_->publish(exploredCloudMsg);
	}

	void occMap::publishInflatedMap(){
		pcl::PointXYZ pt;
		pcl::PointCloud<pcl::PointXYZ> cloud;

		Eigen::Vector3d minRange, maxRange;
		if (this->visGlobalMap_){
			// minRange = this->mapSizeMin_;
			// maxRange = this->mapSizeMax_;
			minRange = this->currMapRangeMin_;
			maxRange = this->currMapRangeMax_;
		}
		else{
			minRange = this->position_ - localMapSize_;
			maxRange = this->position_ + localMapSize_;
			minRange(2) = this->groundHeight_;
		}
		Eigen::Vector3i minRangeIdx, maxRangeIdx;
		this->posToIndex(minRange, minRangeIdx);
		this->posToIndex(maxRange, maxRangeIdx);
		this->boundIndex(minRangeIdx);
		this->boundIndex(maxRangeIdx);

		for (int x=minRangeIdx(0); x<=maxRangeIdx(0); ++x){
			for (int y=minRangeIdx(1); y<=maxRangeIdx(1); ++y){
				for (int z=minRangeIdx(2); z<=maxRangeIdx(2); ++z){
					Eigen::Vector3i pointIdx (x, y, z);
					// if (this->occupancy_[this->indexToAddress(pointIdx)] > this->pMinLog_){
					if (this->isInflatedOccupied(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						if (point(2) <= this->maxVisHeight_){
							pt.x = point(0);
							pt.y = point(1);
							pt.z = point(2);
							cloud.push_back(pt);
						}
					}
				}
			}
		}

		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = true;
		cloud.header.frame_id = "map";
		// cloud.header.stamp = this->get_clock()->now().nanoseconds();

		sensor_msgs::msg::PointCloud2 cloudMsg;
		pcl::toROSMsg(cloud, cloudMsg);
		this->inflatedMapVisPub_->publish(cloudMsg);	
	}

	void occMap::publish2DOccupancyGrid(){
		Eigen::Vector3d minRange, maxRange;
		minRange = this->mapSizeMin_;
		maxRange = this->mapSizeMax_;
		minRange(2) = this->groundHeight_;
		Eigen::Vector3i minRangeIdx, maxRangeIdx;
		this->posToIndex(minRange, minRangeIdx);
		this->posToIndex(maxRange, maxRangeIdx);
		this->boundIndex(minRangeIdx);
		this->boundIndex(maxRangeIdx);

		nav_msgs::msg::OccupancyGrid mapMsg;
		for (int i=0; i<maxRangeIdx(0); ++i){
			for (int j=0; j<maxRangeIdx(1); ++j){
				mapMsg.data.push_back(0);
			}
		}

		double z = 0.5;
		int zIdx = int(z/this->mapRes_);
		for (int x=minRangeIdx(0); x<=maxRangeIdx(0); ++x){
			for (int y=minRangeIdx(1); y<=maxRangeIdx(1); ++y){
				Eigen::Vector3i pointIdx (x, y, zIdx);
				int map2DIdx = x  +  y * maxRangeIdx(0);
				if (this->isUnknown(pointIdx)){
					mapMsg.data[map2DIdx] = -1;
				}
				else if (this->isOccupied(pointIdx)){
					mapMsg.data[map2DIdx] = 100;
				}
				else{
					mapMsg.data[map2DIdx] = 0;
				}
			}
		}
		mapMsg.header.frame_id = "map";
		mapMsg.header.stamp = this->now();
		mapMsg.info.resolution = this->mapRes_;
		mapMsg.info.width = maxRangeIdx(0);
		mapMsg.info.height = maxRangeIdx(1);
		mapMsg.info.origin.position.x = minRange(0);
		mapMsg.info.origin.position.y = minRange(1);
		this->map2DPub_->publish(mapMsg);		
	}
}
