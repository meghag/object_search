#include "../include/grasp_planning.h"
#include "../include/van_der_corput.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


int get_ik(const int arm, const tf::Pose targetPose, std::vector<double> &jointValues);

template <class T>
void pubCloud(const std::string &topic_name, const T &cloud, std::string frame_id = "tum_os_table");


#include <Eigen/Dense>

using namespace Eigen;

template <typename Derived, typename OtherDerived>
void calculateSampleCovariance(const MatrixBase<Derived>& x, const MatrixBase<Derived>& y, MatrixBase<OtherDerived> & C_)
{
    typedef typename Derived::Scalar Scalar;
    typedef typename internal::plain_row_type<Derived>::type RowVectorType;

    const Scalar num_observations = static_cast<Scalar>(x.rows());

    const RowVectorType x_mean = x.colwise().sum() / num_observations;
    const RowVectorType y_mean = y.colwise().sum() / num_observations;

    MatrixBase<OtherDerived>& C = const_cast< MatrixBase<OtherDerived>& >(C_);

    C.derived().resize(x.cols(),x.cols()); // resize the derived object
    C = (x.rowwise() - x_mean).transpose() * (y.rowwise() - y_mean) / num_observations;


}

MatrixXd pos_covar_xy(const std::vector<tf::Vector3> &points)
{
    MatrixXd esamples(points.size(),3);
    for (size_t n=0; n < points.size(); ++n)
    {
        VectorXd evec;
        evec.resize(3);
        evec(0) = points[n].x();
        evec(1) = points[n].y();
        evec(2) = points[n].z();
        esamples.row(n) = evec;
    }
    MatrixXd ret;
    calculateSampleCovariance(esamples,esamples,ret);
    return ret;
}

void swap_if_less(tf::Vector3 &vec_a, double &val_a, tf::Vector3 &vec_b, double &val_b)
{
    if (val_a < val_b)
    {
        double tmp = val_a;
        tf::Vector3 tmp_vec = vec_a;
        vec_a = vec_b;
        val_a = val_b;
        vec_b = tmp_vec;
        val_b = tmp;
    }
}

void pos_eigen_xy(const std::vector<tf::Vector3> &points, std::vector<tf::Vector3> &evec, std::vector<double> &eval)
{
    Matrix<double,3,3> covarMat = pos_covar_xy(points);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,3,3> >
    eigenSolver(covarMat);

    for (size_t c = 0; c < 3; c++)
    {
        evec.push_back(tf::Vector3(eigenSolver.eigenvectors().col(c)(0),eigenSolver.eigenvectors().col(c)(1),eigenSolver.eigenvectors().col(c)(2)));
        eval.push_back(eigenSolver.eigenvalues()(c));
    }

    //bubble sort eigenvectors after eigen values
    swap_if_less(evec[0], eval[0], evec[1], eval[1]);
    swap_if_less(evec[1], eval[1], evec[2], eval[2]);
    swap_if_less(evec[0], eval[0], evec[1], eval[1]);
}

GraspPlanning::GraspPlanning()
{
    markerArrayPub_ = 0L;
    nh_ = 0L;
    initGrasps();
}

bool GraspPlanning::inside(tf::Vector3 point, tf::Vector3 bbmin, tf::Vector3 bbmax)
{
    if ((point.x() > bbmin.x()) && (point.x() < bbmax.x()) &&
            (point.y() > bbmin.y()) && (point.y() < bbmax.y()) &&
            (point.z() > bbmin.z()) && (point.z() < bbmax.z()))
        return true;
    else
        return false;
}

void GraspPlanning::checkGrasps(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<tf::Pose> &unchecked, std::vector<tf::Pose> &checked, std::vector<int> *grasp_index, std::vector<tf::Vector3> *normals, std::vector<tf::Vector3> *centers)
{
    //caching coords doesnt seem to make anything faster
    // we probably fail in the first pass most often and making it more expensive
    // is not outweighted by speedups in the following passes, also, multiplying is faster than memory access?
    //std::vector<tf::Vector3> point_cache;
    //point_cache.resize(cloud->points.size());

    for (int gidx = 0; gidx < grasps.size(); gidx++)
    {
        GraspBoxSet &act = grasps[gidx];

        std::vector<size_t> bb_cnt;
        bb_cnt.resize(act.bb_min.size());

        std::vector<tf::Vector3> points_inside;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_normalvis(new pcl::PointCloud<pcl::PointXYZRGB>);

        // for each grasp
        for (std::vector<tf::Pose>::iterator it = unchecked.begin(); it!=unchecked.end(); ++it)
        {

            points_inside.clear();
            std::fill( bb_cnt.begin(), bb_cnt.end(), 0 );

            bool good = true;

            //for each point, points first so that we transform only once, do not transform full pointcloud as we might get lucky and hit a point early
            // and thus not need to transform all of them
            //for (int i = 0; (i < cloud->points.size()) && good; ++i)

            //transform points only once
            //bool firstpass = true;

            for (size_t k = 0; (k < act.bb_min.size()) && good; ++k)
            {
                for (size_t i = 0; (i < cloud->points.size()) && good; ++i)
                {

                    tf::Vector3 curr;
                    //if (first_pass)
                    //{
                    curr = tf::Vector3(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
                    // project point to gripper coordinates
                    curr = (*it).inverse() * curr;
                    // point_cache[i] = curr;
                    //}
                    //else
                    //  curr = point_cache[i];

                    if (inside(curr, act.bb_min[k], act.bb_max[k]))
                    {
                        bb_cnt[k]++;
                        if (!act.bb_full[k])
                            good = false;
                        else
                            points_inside.push_back(tf::Vector3(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
                    }
                }
                if ((act.bb_full[k]) && (points_inside.size() < 50))
                    good = false;

                //first_pass = false;
            }


            //std::cout << std::endl;
            //for (size_t j = 0; j < act.bb_min.size(); j++)
            //{
              //  //! arbitrary threshold 10 magix number, why ten points min?
                //if (act.bb_full[j] && (bb_cnt[j] < 10))
                  //  good = false;
            //}

            if (good)
            {
                std::vector<tf::Vector3> evec;
                std::vector<double> eval;

                checked.push_back(*it);

                //for (int j = 0; j < bb_min.size(); j++)
                //std::cout << "bb_cnt" << j << " : " << bb_cnt[j] << std::endl;

                if (normals)
                {
                    pos_eigen_xy(points_inside, evec, eval);
                    //tf::Vector3 normal = evec[0].cross(evec[1]);
                    tf::Vector3 normal = evec[2];

                    normal = normal.normalized();

                    //std::cout << "evec0" << evec[0].x() << " " << evec[0].y() << " " << evec[0].z() << std::endl;
                    //std::cout << "evec1" << evec[1].x() << " " << evec[1].y() << " " << evec[1].z() << std::endl;
                    //std::cout << "evec2" << evec[2].x() << " " << evec[2].y() << " " << evec[2].z() << std::endl;
                    //std::cout << "NORM                                                 " << normal.x() << " " << normal.y() << " " << normal.z() << " pt in " << points_inside.size() << std::endl;

                    //normals->push_back(normal);

                    tf::Vector3 avg(0,0,0);
                    for (size_t k = 0; k < points_inside.size(); ++k)
                    {
                        pcl::PointXYZRGB pt;
                        pt.x = points_inside[k].x();
                        pt.y = points_inside[k].y();
                        pt.z = points_inside[k].z();
                        pt.r = 250;
                        pt.g = 50;
                        pt.b = 50;
                        cloud_normalvis->points.push_back(pt);

                        avg+= points_inside[k];
                    }

                    if (points_inside.size() > 0)
                        avg = (1 /  (double)points_inside.size()) * avg;

                    tf::Vector3 norm = evec[2].normalize();

                    for (int coord = 0 ; coord < 3; coord++)
                        for (double len = 0; len < 0.05; len+= 0.001)
                        {
                            pcl::PointXYZRGB pt;
                            pt.x = avg.x() + evec[coord].x() * len;
                            pt.y = avg.y() + evec[coord].y() * len;
                            pt.z = avg.z() + evec[coord].z() * len;
                            //pt.x = avg.x() + norm.x() * len;
                            //pt.y = avg.y() + norm.y() * len;
                            //pt.z = avg.z() + norm.z() * len;
                            pt.r = ((coord == 0) ? 255 : 0);
                            pt.g = ((coord == 1) ? 255 : 0);
                            pt.b = ((coord == 2) ? 255 : 0);
                            cloud_normalvis->points.push_back(pt);
                        }

                    normals->push_back(avg + norm);
                    if (centers)
                        centers->push_back(avg);
                    if (grasp_index)
                        grasp_index->push_back(gidx);

                }

            }


        }

        if (normals)
        {
            //pubCloud("normals", cloud_normalvis, "tum_os_table");
            //finish();
        }

    }

    //ros::Duration(.5).sleep();
}


GraspBoxSet
GraspPlanning::graspBoxSetFromMsg(const tum_os::BoxSet &msg)
{
    GraspBoxSet ret;
    ret.name = msg.name;
    for (std::vector<std_msgs::Bool>::const_iterator it = msg.include_points.begin(); it != msg.include_points.end();  ++it)
        ret.bb_full.push_back(it->data);
    for (std::vector<geometry_msgs::Point>::const_iterator it = msg.min.begin(); it != msg.min.end();  ++it)
    {
        tf::Point pt;
        tf::pointMsgToTF(*it,pt);
        ret.bb_min.push_back(pt);
    }
    for (std::vector<geometry_msgs::Point>::const_iterator it = msg.max.begin(); it != msg.max.end();  ++it)
    {
        tf::Point pt;
        tf::pointMsgToTF(*it,pt);
        ret.bb_max.push_back(pt);
    }
    for (std::vector<geometry_msgs::Pose>::const_iterator it = msg.approach.begin(); it != msg.approach.end();  ++it)
    {
        tf::Pose ps;
        tf::poseMsgToTF(*it,ps);
        ret.approach.push_back(ps);
    }
    return ret;
}


tum_os::BoxSet
GraspPlanning::graspBoxSetToMsg(const GraspBoxSet &boxset)
{
    tum_os::BoxSet ret;
    ret.name = boxset.name;
    for (std::vector<bool>::const_iterator it = boxset.bb_full.begin(); it != boxset.bb_full.end(); ++it)
    {
        std_msgs::Bool bl;
        bl.data = *it;
        ret.include_points.push_back(bl);
    }
    for (std::vector<tf::Point>::const_iterator it = boxset.bb_min.begin(); it != boxset.bb_min.end(); ++it)
    {
        geometry_msgs::Point pt;
        tf::pointTFToMsg(*it, pt);
        ret.min.push_back(pt);
    }
    for (std::vector<tf::Point>::const_iterator it = boxset.bb_max.begin(); it != boxset.bb_max.end(); ++it)
    {
        geometry_msgs::Point pt;
        tf::pointTFToMsg(*it, pt);
        ret.max.push_back(pt);
    }
    for (std::vector<tf::Pose>::const_iterator it = boxset.approach.begin(); it != boxset.approach.end(); ++it)
    {
        geometry_msgs::Pose ps;
        tf::poseTFToMsg(*it, ps);
        ret.approach.push_back(ps);
    }

    return ret;
}


void GraspPlanning::minmax3d(tf::Vector3 &min, tf::Vector3 &max, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    Eigen::Vector4f  	min_pt, max_pt;
    pcl::getMinMax3D 	( *cloud,min_pt,max_pt );
    min = tf::Vector3(min_pt.x(),min_pt.y(),min_pt.z());
    max = tf::Vector3(max_pt.x(),max_pt.y(),max_pt.z());
}

void GraspPlanning::checkGraspsIK(int arm, tf::Stamped<tf::Pose> fixed_to_ik, std::vector<tf::Pose> &unchecked, std::vector<tf::Pose> &checked)
{
    std::vector<double> result;
    result.resize(7);
    std::fill( result.begin(), result.end(), 0 );

    for (std::vector<tf::Pose>::iterator it = unchecked.begin(); it!=unchecked.end(); ++it)
    {
        tf::Pose in_ik_frame = fixed_to_ik.inverseTimes(*it);
        if (get_ik(arm, in_ik_frame, result) == 1)
            checked.push_back(*it);
    }

}


template <class PointCloudPtr>
std::vector<tf::Pose> GraspPlanning::calcGrasps(const PointCloudPtr object_model, const PointCloudPtr environment_model, bool check_ik, tf::Stamped<tf::Pose> fixed_to_ik, size_t grasp_type, size_t sample_size)
{
    std::vector<tf::Pose> ik_checked,random,low,high,checked,filtered, reachable, collision_free, ret;
    tf::Vector3 cluster_min, cluster_max;

    minmax3d(cluster_min, cluster_max, object_model);
    cluster_min -= tf::Vector3(.15,.15,.15);
    cluster_max += tf::Vector3(.15,.15,.15);

    ret.resize(sample_size);
    for (int i = 0; i < sample_size; ++i)
        ret.push_back(VanDerCorput::vdc_pose_bound(cluster_min, cluster_max, i));

    return ret;
}

void GraspPlanning::initGrasps()
{
    GraspBoxSet act;

    act.bb_min.push_back(tf::Vector3(0.23,-0.01,-0.02));
    act.bb_max.push_back(tf::Vector3(0.26,0.01,0.02));
    act.bb_full.push_back(true);

    act.bb_min.push_back(tf::Vector3(0.18,-3.46945e-18,-0.02));
    act.bb_max.push_back(tf::Vector3(0.23,0.04,0.02));
    act.bb_full.push_back(false);

    act.bb_min.push_back(tf::Vector3(0.18,-0.04,-0.02));
    act.bb_max.push_back(tf::Vector3(0.23,3.46945e-18,0.02));
    act.bb_full.push_back(false);

    act.bb_min.push_back(tf::Vector3(-0.02,-0.06,-0.03));
    act.bb_max.push_back(tf::Vector3(0.18,0.06,0.03));
    act.bb_full.push_back(false);

    tf::Pose push;
    push.setOrigin(tf::Vector3(0.1,0,0));
    push.setRotation(tf::Quaternion(0,0,0,1));
    act.approach.push_back(push);

    act.name = "push_forward";

    grasps.push_back(act);
}

void GraspPlanning::visualizeGrasp(size_t grasp_type, std::string frame_id, int highlight)
{

    if (markerArrayPub_==0)
    {
        if (nh_ == 0)
        {
            nh_ = new ros::NodeHandle();
        }
        markerArrayPub_ = new ros::Publisher();
        *markerArrayPub_ = nh_->advertise<visualization_msgs::MarkerArray>( "grasp_visualization", 0 );
    }

    if (grasp_type > grasps.size())
    {
        ROS_ERROR("GRASP %zu not defined", grasp_type);
        return;
    }

    visualization_msgs::MarkerArray arr;

    GraspBoxSet &grasp = grasps[grasp_type];
    for (size_t i = 0; i < grasp.bb_max.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "grasps";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = (grasp.bb_max[i].x() + grasp.bb_min[i].x()) / 2.0f;
        marker.pose.position.y = (grasp.bb_max[i].y() + grasp.bb_min[i].y()) / 2.0f;
        marker.pose.position.z = (grasp.bb_max[i].z() + grasp.bb_min[i].z()) / 2.0f;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = grasp.bb_max[i].x() - grasp.bb_min[i].x();
        marker.scale.y = grasp.bb_max[i].y() - grasp.bb_min[i].y();
        marker.scale.z = grasp.bb_max[i].z() - grasp.bb_min[i].z();
        marker.color.a = 0.5;
        marker.color.r = grasp.bb_full[i] ? 0.0 : 1.0;
        marker.color.g = grasp.bb_full[i] ? 1.0 : 0.0;
        marker.color.b = 0.0;

        if (highlight == (int) i)
        {
            marker.color.a = 0.75;
        }

        arr.markers.push_back(marker);
    }

    markerArrayPub_->publish(arr);
}
