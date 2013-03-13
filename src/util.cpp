
void minmax3d(tf::Vector3 &min, tf::Vector3 &max, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    Eigen::Vector4f  	min_pt, max_pt;
    pcl::getMinMax3D 	( *cloud,min_pt,max_pt );
    min = tf::Vector3(min_pt.x(),min_pt.y(),min_pt.z());
    max = tf::Vector3(max_pt.x(),max_pt.y(),max_pt.z());
}

actionlib::SimpleActionClient<mc_graspable::FindGraspablesAction> *mcg_client_ = NULL;

void getGrasps(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<tf::Pose> &low, std::vector<tf::Pose> &high)
{

    if (!mcg_client_)
    {
        mcg_client_ = new actionlib::SimpleActionClient<mc_graspable::FindGraspablesAction> ("mc_graspable", true);

        ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        mcg_client_->waitForServer(); //will wait for infinite time
    }

    tf::Vector3 pmin,pmax;
    minmax3d(pmin,pmax,cloud);

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    mc_graspable::FindGraspablesGoal goal;

    goal.cloud_topic = "/mc_graspable_query";

    goal.frame_id = "/map";

    goal.aabb_min.x = pmin.x();
    goal.aabb_min.y = pmin.y();
    goal.aabb_min.z = pmin.z();
    goal.aabb_max.x = pmax.x();
    goal.aabb_max.y = pmax.y();
    goal.aabb_max.z = pmax.z();
    goal.delta = 0.02;
    goal.scaling = 20;
    goal.pitch_limit = 0.4;
    goal.thickness = 0.04;
    mcg_client_->sendGoal(goal);



    ros::Rate rt(5);
    for (size_t k =0; k < 10; k ++)
    {
        std::cout << "Publishing cluster on " << goal.cloud_topic << std::endl;
        pubCloud(goal.cloud_topic, cloud, "/map");
        ros::spinOnce();
        rt.sleep();
    }


    //wait for the action to return
    bool finished_before_timeout = mcg_client_->waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state =mcg_client_->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());

        mc_graspable::FindGraspablesResultConstPtr result = mcg_client_->getResult();
        for (std::vector<geometry_msgs::Pose>::const_iterator it = result->high.poses.begin(); it != result->high.poses.end(); ++it)
        {
            tf::Pose act;
            tf::poseMsgToTF(*it, act);
            high.push_back(act);
        }
        for (std::vector<geometry_msgs::Pose>::const_iterator it = result->low.poses.begin(); it != result->low.poses.end(); ++it)
        {
            tf::Pose act;
            tf::poseMsgToTF(*it, act);
            low.push_back(act);
        }

    }
    else
        ROS_INFO("Action did not finish before the time out.");

    //return 0;
}

bool inside(tf::Vector3 point, tf::Vector3 bbmin, tf::Vector3 bbmax)
{
    if ((point.x() > bbmin.x()) && (point.x() < bbmax.x()) &&
            (point.y() > bbmin.y()) && (point.y() < bbmax.y()) &&
            (point.z() > bbmin.z()) && (point.z() < bbmax.z()))
        return true;
    else
        return false;
}

tf::TransformListener*listener_ = 0L;

ros::Publisher *vis_pub_ = 0L;

ros::NodeHandle *nh_ = 0L;

ros::Publisher *pose_ary_pub_ = 0L;


double dist_to_sensor = 1;


void init()
{
    if (!listener_)
        nh_ = new ros::NodeHandle();
    if (!listener_)
        listener_ = new tf::TransformListener();
    if (!vis_pub_)
    {
        vis_pub_ = new ros::Publisher();
        *vis_pub_ = nh_->advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    }
    if (!br)
        br = new tf::TransformBroadcaster();
}

gpc_polygon last;
bool have_last = false;

void pubPolygon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = rgb_optical_frame_;
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
//only if using a MESH_RESOURCE marker type:
    //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

    gpc_polygon subject, result;
    subject.num_contours = 1;
    subject.hole = new int[1];
    subject.contour = new gpc_vertex_list[1];
    subject.contour[0].num_vertices = cloud_hull->points.size();
    subject.contour[0].vertex = new gpc_vertex[cloud_hull->points.size()];
    for (size_t i = 0; i < cloud_hull->points.size(); ++i)
    {
        subject.contour[0].vertex[i].x = cloud_hull->points[i].y * 10;
        subject.contour[0].vertex[i].y = cloud_hull->points[i].z * 10;
    }

    gpc_tristrip tristrip;

    if (have_last)
    {
        gpc_polygon_clip(GPC_INT, &subject, &last, &result);
        gpc_polygon_to_tristrip(&result,&tristrip);
    }
    else
    {
        gpc_polygon_to_tristrip(&subject,&tristrip);
    }

    have_last = true;
    last = subject;

    for (int i = 0; i < tristrip.num_strips; ++i)
    {
        for (int j = 3; j < tristrip.strip[i].num_vertices; ++j)
        {
            //std::cout << "tri" << tristrip.strip[i].vertex[j-2].x << " " << tristrip.strip[i].vertex[j-2].y << std::endl;
            //std::cout << "   " << tristrip.strip[i].vertex[j-1].x << " " << tristrip.strip[i].vertex[j-1].y << std::endl;
            //std::cout << "   " << tristrip.strip[i].vertex[j-0].x << " " << tristrip.strip[i].vertex[j-0].y << std::endl;
            geometry_msgs::Point pt[3];
            if (j % 2 == 0)
            {
                pt[0].x = dist_to_sensor;
                pt[0].y = tristrip.strip[i].vertex[j-2].x;
                pt[0].z = tristrip.strip[i].vertex[j-2].y;
                pt[1].x = dist_to_sensor;
                pt[1].y = tristrip.strip[i].vertex[j-1].x;
                pt[1].z = tristrip.strip[i].vertex[j-1].y;
                pt[2].x = dist_to_sensor;
                pt[2].y = tristrip.strip[i].vertex[j-0].x;
                pt[2].z = tristrip.strip[i].vertex[j-0].y;
            }
            else
            {
                pt[0].x = dist_to_sensor;
                pt[0].y = tristrip.strip[i].vertex[j-1].x;
                pt[0].z = tristrip.strip[i].vertex[j-1].y;
                pt[1].x = dist_to_sensor;
                pt[1].y = tristrip.strip[i].vertex[j-2].x;
                pt[1].z = tristrip.strip[i].vertex[j-2].y;
                pt[2].x = dist_to_sensor;
                pt[2].y = tristrip.strip[i].vertex[j-0].x;
                pt[2].z = tristrip.strip[i].vertex[j-0].y;
            }
            marker.points.push_back(pt[0]);
            marker.points.push_back(pt[1]);
            marker.points.push_back(pt[2]);
            std_msgs::ColorRGBA color;
            color.r = .7;
            color.g = .2;
            color.b = .1;
            color.a = .7;
            marker.colors.push_back(color);
            marker.colors.push_back(color);
            marker.colors.push_back(color);
            //std::cout << "tri" << tristrip.strip[i].vertex[j].x << " " << tristrip.strip[i].vertex[j].y << std::endl;
        }
    }

    vis_pub_->publish( marker );

}


tf::Stamped<tf::Pose> getPoseIn(const std::string target_frame, tf::Stamped<tf::Pose>src)
{

    if (src.frame_id_ == "NO_ID_STAMPED_DEFAULT_CONSTRUCTION")
    {
        ROS_ERROR("Frame not in TF: %s", src.frame_id_.c_str());
        tf::Stamped<tf::Pose> pose;
        return pose;
    }

    if (!listener_)
        listener_ = new tf::TransformListener(ros::Duration(30));

    tf::Stamped<tf::Pose> transform;
    //this shouldnt be here TODO
    //src.stamp_ = ros::Time(0);

    listener_->waitForTransform(src.frame_id_, target_frame,
                                ros::Time(0), ros::Duration(30.0));
    bool transformOk = false;
    while (!transformOk)
    {
        try
        {
            transformOk = true;
            listener_->transformPose(target_frame, src, transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("getPoseIn %s",ex.what());
            // dirty:
            src.stamp_ = ros::Time(0);
            transformOk = false;
        }
        ros::spinOnce();
    }
    return transform;
}

tf::Stamped<tf::Pose> getPose(const std::string target_frame,const std::string lookup_frame, ros::Time tm = ros::Time(0))
{

    init();
    ros::Rate rate(50.0);

    tf::StampedTransform transform;
    bool transformOk = false;
    bool had_to_retry = false;

    //listener_->waitForTransform(target_frame, lookup_frame, tm, ros::Duration(0.5));
    while (ros::ok() && (!transformOk))
    {
        transformOk = true;
        try
        {
            listener_->lookupTransform(target_frame, lookup_frame,tm, transform);
        }
        catch (tf::TransformException ex)
        {
            std::string what = ex.what();
            what.resize(100);
            ROS_INFO("getPose: tf::TransformException ex.what()='%s', will retry",what.c_str());
            transformOk = false;
            had_to_retry = true;
            //listener_->waitForTransform(target_frame, lookup_frame, ros::Time(0), ros::Duration(0.5));
        }
        if (!transformOk)
            rate.sleep();
    }

    if (had_to_retry)
        ROS_INFO("Retry sucessful");

    tf::Stamped<tf::Pose> ret;
    ret.frame_id_ = transform.frame_id_;
    ret.stamp_ = transform.stamp_;
    ret.setOrigin(transform.getOrigin());
    ret.setRotation(transform.getRotation());

    return ret;
}

void getCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string frame_id, ros::Time after, ros::Time *tm = 0)
{

    sensor_msgs::PointCloud2 pc;
    bool found = false;
    while (!found)
    {
        pc  = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>(rgb_topic_));
        if ((after == ros::Time(0,0)) || (pc.header.stamp > after))
            found = true;
        else
        {
            //ROS_ERROR("getKinectCloudXYZ cloud too old : stamp %f , target time %f",pc.header.stamp.toSec(), after.toSec());
        }
    }
    if (tm)
        *tm = pc.header.stamp;

    tf::Stamped<tf::Pose> net_stamped = getPose(fixed_frame_.c_str(),pc.header.frame_id.c_str());
    tf::Transform net_transform;
    net_transform.setOrigin(net_stamped.getOrigin());
    net_transform.setRotation(net_stamped.getRotation());

    sensor_msgs::PointCloud2 pct; //in map frame

    pcl_ros::transformPointCloud(frame_id.c_str(),net_transform,pc,pct);
    pct.header.frame_id = frame_id.c_str();

    geometry_msgs::Transform t_msg;
    tf::transformTFToMsg(net_transform, t_msg);

    //std::cout << "CLOUD transf " << pc.header.frame_id << " to " << pct.header.frame_id << " : " << t_msg << std::endl;

    pcl::fromROSMsg(pct, *cloud);
}

std::map<std::string, ros::Publisher*> cloud_publishers;

//void pubCloud(const std::string &topic_name, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::string frame_id = "/map")
template <class T>
void pubCloud(const std::string &topic_name, const T &cloud, std::string frame_id)
{
    ros::Publisher *cloud_pub;

    if (cloud_publishers.find(topic_name) == cloud_publishers.end())
    {

        cloud_pub = new ros::Publisher();
        *cloud_pub = nh_->advertise<sensor_msgs::PointCloud2>(topic_name,0,true);

        cloud_publishers.insert(std::pair<std::string, ros::Publisher*>(topic_name, cloud_pub ));
        //std::cout << "created new publisher" << cloud_pub << std::endl;
    }
    else
    {
        cloud_pub = cloud_publishers.find(topic_name)->second;
        //std::cout << "found pub on " << cloud_pub->getTopic() << ", reusing it" << std::endl;
    }

    sensor_msgs::PointCloud2 out; //in map frame

    pcl::toROSMsg(*cloud,out);
    out.header.frame_id = frame_id;
    out.header.stamp = ros::Time::now();
    cloud_pub->publish(out);

    //ROS_INFO("published frame %s %i x %i points on %s", out.header.frame_id.c_str(), out.height, out.width, topic_name.c_str());
}


void pubCloudWait(const std::string &topic_name, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::string frame_id, ros::Duration waitTime)
{
    waitTime.sleep();
    return pubCloud(topic_name,cloud,frame_id);
}


void getPointsInBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr inBox, const tf::Vector3 min, const tf::Vector3 max)
{
    Eigen::Vector4f min_pt, max_pt;

    min_pt = Eigen::Vector4f(std::min(min.x(), max.x()),std::min(min.y(), max.y()),std::min(min.z(), max.z()), 1);
    max_pt = Eigen::Vector4f(std::max(min.x(), max.x()),std::max(min.y(), max.y()),std::max(min.z(), max.z()), 1);

    //ROS_INFO("min %f %f %f" ,min_pt[0],min_pt[1],min_pt[2]);
    //ROS_INFO("max %f %f %f" ,max_pt[0],max_pt[1],max_pt[2]);

    //ROS_INFO("cloud size : %zu", cloud->points.size());

    boost::shared_ptr<std::vector<int> > indices( new std::vector<int> );

    pcl::getPointsInBox(*cloud,min_pt,max_pt,*indices);

    //std::cout << "idx size" << indices->size() << std::endl;

    pcl::ExtractIndices<pcl::PointXYZ> ei;
    ei.setInputCloud(cloud);
    ei.setIndices(indices);
    ei.filter(*inBox);

    pubCloud("debug_cloud", inBox);

    ROS_INFO("cloud size after box filtering: %zu = %i * %i", inBox->points.size(), inBox->width, inBox->height);
}


//orthogonal projection
void projectToPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_projected)
//(tf::Vector3 planeNormal, double planeDist,
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // construct a plane parallel to the camera sensor
    tf::Stamped<tf::Pose> planePoint;
    // should work with three points but strangely doesnt
    int numpoints = 10;
    for (int i = 0; i < numpoints ; ++i )
    {
        planePoint.frame_id_ = rgb_optical_frame_;
        //planePoint.frame_id_ = fixed_frame_;
        planePoint.stamp_ = ros::Time(0);
        //planePoint.setOrigin(tf::Vector3(dist_to_sensor,sin(i*(360 / numpoints )* M_PI / 180.0f),cos(i*(360 / numpoints )* M_PI / 180.0f)));
        planePoint.setOrigin(tf::Vector3(dist_to_sensor,sin(i*(360 / numpoints )* M_PI / 180.0f),cos(i*(360 / numpoints )* M_PI / 180.0f)));
        //planePoint.setOrigin(tf::Vector3(sin(i*(360 / numpoints )* M_PI / 180.0f),cos(i*(360 / numpoints )* M_PI / 180.0f),1.0));
        planePoint.setRotation(tf::Quaternion(0,0,0,1));
        planePoint = getPoseIn(fixed_frame_.c_str(), planePoint);

        pcl::PointXYZ planePt;
        planePt.x = planePoint.getOrigin().x();
        planePt.y = planePoint.getOrigin().y();
        planePt.z = planePoint.getOrigin().z();

        //std::cout << i<< planePt.x << " " << planePt.y << " " << planePt.z << std::endl;
        plane_cloud->points.push_back(planePt);
        inliers->indices.push_back(i);
    }

    //we abuse pcl plane model
    //pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (plane_cloud);
    seg.segment (*inliers, *coefficients);
    std::cerr << "PointCloud after segmentation has: "
              << inliers->indices.size () << " inliers." << std::endl;


    std::cout << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);
    std::cerr << "PointCloud after projection has: "
              << cloud_projected->points.size () << " data points." << std::endl;

    pubCloud("cloud_projected", cloud_projected);

}

void calcHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_hull)
{

    // Create a Concave Hull representation of the projected inliers
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);

    tf::Stamped<tf::Pose> net_stamped = getPose(rgb_optical_frame_.c_str(),fixed_frame_.c_str());
    tf::Transform fixed_to_sensor;
    fixed_to_sensor.setOrigin(net_stamped.getOrigin());
    fixed_to_sensor.setRotation(net_stamped.getRotation());

    net_stamped = getPose(fixed_frame_.c_str(),rgb_optical_frame_.c_str());
    tf::Transform sensor_to_fixed;
    sensor_to_fixed.setOrigin(net_stamped.getOrigin());
    sensor_to_fixed.setRotation(net_stamped.getRotation());

    // transforming point cloud back to sensor frame to get chull to recognize that its 2d
    // this will not be needed anmore with groove
    pcl_ros::transformPointCloud(*cloud,*cloud,fixed_to_sensor);

    for (size_t i = 0; i < cloud->points.size(); i++)
        cloud->points[i].x = 0;

    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud);
    chull.setAlpha (.01);
    chull.setAlpha (.1);
    chull.setKeepInformation(true);
    chull.reconstruct (*cloud_hull);

    std::cerr << "Concave hull " << chull.getDim() << " has: " << cloud_hull->points.size ()
              << " data points." << std::endl;

    if (chull.getDim() > 2)
        ROS_ERROR("CONCAVE HULL IS 3D!");

    pubPolygon(cloud_hull);

    for (size_t  i = 0; i < cloud_hull->points.size(); i++)
        cloud_hull->points[i].x = dist_to_sensor;

    pcl_ros::transformPointCloud(*cloud_hull,*cloud_hull,sensor_to_fixed);

    pubCloud("cloud_hull", cloud_hull);

    //pcl::PCDWriter writer;
    //writer.write ("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);
}

void test_hull_calc()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_box (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_box_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_box_projected_hull (new pcl::PointCloud<pcl::PointXYZ>);

    cloud_in_box->width = 0;
    cloud_in_box->height = 0;

    ros::Time lookup_time;

    getCloud(cloud, fixed_frame_, ros::Time::now() - ros::Duration(1), &lookup_time);

    {
        tf::Vector3 bb_min(-1.9,1.6,.84);
        tf::Vector3 bb_max(-1.6,2.1,1.2);

        getPointsInBox(cloud, cloud_in_box, bb_min, bb_max);

        projectToPlane(cloud_in_box, cloud_in_box_projected);

        calcHull(cloud_in_box_projected, cloud_in_box_projected_hull);
    }

    {
        tf::Vector3 bb_min(-1.9,1.6,.75);
        tf::Vector3 bb_max(-1.6,2.1,.84);

        getPointsInBox(cloud, cloud_in_box, bb_min, bb_max);

        projectToPlane(cloud_in_box, cloud_in_box_projected);

        calcHull(cloud_in_box_projected, cloud_in_box_projected_hull);
    }

}
