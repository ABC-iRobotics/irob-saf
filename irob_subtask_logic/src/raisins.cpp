#include <irob_subtask_logic/raisins.hpp>

namespace saf
{
    Raisins::Raisins(ros::NodeHandle nh, ros::NodeHandle priv_nh, std::vector<std::string> arm_names)
        : AutosurgAgent(nh, priv_nh, arm_names), vision(nh, "target")
    {
        std::vector<double> offset_arm_1;
        priv_nh.getParam("offset_arm_1", offset_arm_1);
        offs_x = offset_arm_1[0];
        offs_y = offset_arm_1[1];
        offs_z = offset_arm_1[2];
        priv_nh.getParam("offset_filename_arm_1", offset_filename_arm_1);
        priv_nh.getParam("measurement_filename", measurement_filename);

        ROS_INFO_STREAM("raisins_constructor");
    }

    Raisins::~Raisins() {}

    void Raisins::collectOne()
    {
        double compress_rate = 0.2;
        int object_idx = 0;
        int peg_idx_to = 6;
        int increment = 1;
        int grasp_pos_idx = 1;
        int place_grasp_pos_idx = 1;

        std::vector<Eigen::Affine3d> waypoints;

        irob_msgs::Environment e = makeNaN<irob_msgs::Environment>();
        ROS_INFO_STREAM("Waiting for data from vision...");
        while (isnan(e) && ros::ok())
        {
            e = vision.getResult();
            ros::Duration(1).sleep();
        }

        ROS_INFO_STREAM("Vision data received.");

        Eigen::Translation3d offset_cf = Eigen::Translation3d(Eigen::Vector3d(offs_x, offs_y, offs_z));
        //  Eigen::Translation3d offset_peg_h = Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, peg_h));
        //  Eigen::Translation3d offset_block_h = Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, object_h));
        Eigen::Quaternion<double> ori_world = BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::DOWN_SIDEWAYS;
        // Eigen::Quaternion<double> grasp_ori_world = BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::DOWN_FORWARD;

        // Grasp object

        Eigen::Affine3d grasp_pose(unwrapMsg<geometry_msgs::Pose, Eigen::Affine3d>(e.objects[object_idx].grasp_poses[0]));
        Eigen::Affine3d approach_pose(unwrapMsg<geometry_msgs::Pose, Eigen::Affine3d>(e.objects[object_idx].approach_poses[0]));
        approach_pose = offset_cf * approach_pose;

        // modify by the orientation
        grasp_pose = grasp_pose * ori_world;
        grasp_pose = offset_cf * grasp_pose;
        approach_pose = approach_pose * ori_world;
        ROS_INFO_STREAM("Approach pose");
        ROS_INFO_STREAM(approach_pose.translation().x() << ", " << approach_pose.translation().y() << ", " << approach_pose.translation().z());

        ROS_INFO_STREAM("Grasp pose");
        ROS_INFO_STREAM(grasp_pose.translation().x() << ", " << grasp_pose.translation().y() << ", " << grasp_pose.translation().z());

        waypoints.push_back(approach_pose);

        arms[0]->grasp(grasp_pose, approach_pose, 2.0, 0.95, 20.0, 20.0);

        while (!arms[0]->isSurgemeDone() && ros::ok())
        {
            ros::Duration(0.1).sleep();
        }

        ROS_INFO_STREAM("Grasped object on rod");
        ros::Duration(0.1).sleep();

        // Place object to new location
        ROS_INFO_STREAM("Place object");

        Eigen::Affine3d collectPose = grasp_pose;
        Eigen::Translation3d collectOffset(0.0, 50, 0.0);
        collectPose = collectOffset * collectPose;

        arms[0]->place(collectPose, collectPose, speed_cartesian, waypoints);
        while (!arms[0]->isSurgemeDone() && ros::ok())
        {
            ros::Duration(0.1).sleep();
        }

        ROS_INFO_STREAM("Release object...");
        arms[0]->release(collectPose, 5.0, speed_cartesian, speed_jaw);

        while (!arms[0]->isSurgemeDone() && ros::ok())
        {
            ros::Duration(0.1).sleep();
        }
    }

    void Raisins::collectRaisins()
    {
        Eigen::Quaternion<double> ori_world = BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::DOWN_SIDEWAYS;
        irob_msgs::Environment e = makeNaN<irob_msgs::Environment>();
        Eigen::Affine3d collect_pose = makeNaN<Eigen::Affine3d>();

        ROS_INFO_STREAM("Waiting for data from vision...");

        while (isnan(e) && ros::ok())
        {
            e = vision.getResult();
            ros::Duration(1).sleep();
        }

        ROS_INFO_STREAM("Vision data received.");

        for (irob_msgs::GraspObject obj : e.objects)
        {
            Eigen::Affine3d grasp_pose(unwrapMsg<geometry_msgs::Pose, Eigen::Affine3d>(obj.grasp_poses[0]));
            Eigen::Affine3d approach_pose(unwrapMsg<geometry_msgs::Pose, Eigen::Affine3d>(obj.approach_poses[0]));
            grasp_pose = grasp_pose * ori_world;
            approach_pose = approach_pose * ori_world;

            ROS_INFO_STREAM("Approach pose");
            ROS_INFO_STREAM(approach_pose.translation().x() << ", " << approach_pose.translation().y() << ", " << approach_pose.translation().z());

            ROS_INFO_STREAM("Grasp pose");
            ROS_INFO_STREAM(grasp_pose.translation().x() << ", " << grasp_pose.translation().y() << ", " << grasp_pose.translation().z());

            std::vector<Eigen::Affine3d> waypoints;
            waypoints.push_back(approach_pose);

            arms[0]->grasp(grasp_pose, approach_pose, 2.0, 0.95, 20.0, 20.0);
            while (!arms[0]->isSurgemeDone() && ros::ok())
            {
                ros::Duration(0.1).sleep();
            }
            ROS_INFO_STREAM("Grasped object on rod");
            ros::Duration(0.1).sleep();

            if (isnan(collect_pose))
            {
                collect_pose = approach_pose;

                // offset with 50
                Eigen::Translation3d collectOffset(0.0, 50, 0.0);
                collect_pose = collectOffset * collect_pose;
            }

            // Place object to new location
            ROS_INFO_STREAM("Place object");
            arms[0]->place(collect_pose, collect_pose, speed_cartesian, waypoints);
            while (!arms[0]->isSurgemeDone() && ros::ok())
            {
                ros::Duration(0.1).sleep();
            }

            ROS_INFO_STREAM("Release object...");
            arms[0]->release(collect_pose, 5.0, speed_cartesian, speed_jaw);
            while (!arms[0]->isSurgemeDone() && ros::ok())
            {
                ros::Duration(0.1).sleep();
            }
        }
    }
}

using namespace saf;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "raisins");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    std::vector<std::string> arm_names;
    priv_nh.getParam("arm_names", arm_names);

    std::string mode;
    priv_nh.getParam("mode", mode);

    try
    {
        Raisins raisins(nh, priv_nh, arm_names);
        raisins.collectRaisins();

        ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM(e.what());
        ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
    }

    ros::shutdown();
    return 0;
}
