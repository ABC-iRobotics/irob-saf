import rospy
import numpy as np
import math

from geometry_msgs.msg import TransformStamped, Transform
from sensor_msgs.msg import JointState, Joy


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
import rosbag

from irob_utils.rigid_transform_3D import rigid_transform_3D
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp
import yaml


class HandEyeRegistrator:


    def __init__(self):
        """Constructor."""
        print("Node started")
        rospy.init_node('hand_eye_registrator', anonymous=True)
        self.arm = rospy.get_param('~arm')
        self.camera_registration_filename = rospy.get_param(
                                            '~camera_registration_file')

        self.mode = rospy.get_param('~mode')    # simple, auto, save
        self.poses_filename = rospy.get_param('~poses_filename')

        self.poses_to_save = []
        self.robot_positions = np.zeros((0,3))
        self.fiducial_positions = np.zeros((0,3))

        # Subscribe to topics
        rospy.Subscriber("fiducial_tf", Transform, self.cb_fiducial)
        rospy.Subscriber("/" + self.arm + "/measured_cp",
                        TransformStamped, self.cb_measured_cp)
        rospy.Subscriber("/" + self.arm + "/jaw/measured_js",
                        JointState, self.cb_measured_jaw)
        rospy.Subscriber("/" + self.arm + "/manip_clutch",
                       Joy, self.cb_manip_clutch)



        # Advertise topics
        self.servo_cp_pub = rospy.Publisher(
                        "/" + self.arm + "/servo_cp",
                        TransformStamped, queue_size=10)
        self.servo_jaw_pub = rospy.Publisher(
                        "/" + self.arm + "/jaw/servo_jp",
                        JointState, queue_size=10)



    def cb_fiducial(self, msg):
        """Callback function for fiducial pose."""
        self.fiducial_tf = msg
        #ospy.loginfo(rospy.get_caller_id() + " I heard fiducial_tf %s", msg)



    def cb_measured_cp(self, msg):
        """Callback function for measured_cp."""
        self.measured_cp = msg
        #rospy.loginfo(rospy.get_caller_id() + " I heard measured_cp %s", msg)


    def cb_measured_jaw(self, msg):
        """Callback function jaw/measured_js"""
        self.measured_jaw = msg
        #rospy.loginfo(rospy.get_caller_id() + " I heard jaw %s", msg)

    def cb_manip_clutch(self, msg):
        """Callback function when clutch button is pressed.
        Collect one sample from the positions.
        """
        rospy.loginfo(rospy.get_caller_id() + " I heard clutch %s", msg)
        if True:    # TODO when
            self.gather_actual_position()


    def gather_actual_position(self):
        """Gather a single position from the camera and the robot."""
        rospy.sleep(0.5)
        robot_pos = np.array([self.measured_cp.transform.translation.x,
                              self.measured_cp.transform.translation.y,
                              self.measured_cp.transform.translation.z]).T
        fiducial_pos = np.array([self.fiducial_tf.translation.x,
                                 self.fiducial_tf.translation.y,
                                 self.fiducial_tf.translation.z]).T
        if self.mode == "save":
            self.poses_to_save.append(self.measured_cp)


        self.robot_positions = np.vstack((self.robot_positions, robot_pos))
        self.fiducial_positions = np.vstack((self.fiducial_positions, fiducial_pos))

        print("Positions collected: " + str(self.robot_positions.shape[0]))



    def collect_and_register(self):
        """Wait for data colection. When key pressed,
        do the registration.
        """
        input("Collect data for registration. Press Enter when done...")

        R, t = rigid_transform_3D(self.robot_positions.T, self.fiducial_positions.T)

        # Check transformation
        points_transformed = np.zeros(self.robot_positions.shape)
        for i in range(self.robot_positions.shape[1]):
            p = np.dot(R, self.robot_positions[i,:].T) + t.T
            points_transformed[i,:] = p

        # Draw plot
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')
        self.ax.scatter(self.fiducial_positions[0,:], self.fiducial_positions[1,:], self.fiducial_positions[2,:], marker='o')
        self.ax.scatter(points_transformed[0,:], points_transformed[1,:],
                                        points_transformed[2,:], marker='^')


        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        # Save robot poses for auto registration
        if self.mode == "save":
            self.save_robot_poses()

        return R, t


    def save_registration(self, R, t):
        """Save registration to config file.

        Keyword arguments:
        R -- rotation matrix
        t -- translation vector
        """
        data = dict(
            t = [float(t[0,0]), float(t[1,0]), float(t[1,0])],
            R = [float(R[0,0]), float(R[0,1]), float(R[0,2]),
                float(R[1,0]), float(R[1,1]), float(R[1,2]),
                float(R[2,0]), float(R[2,1]), float(R[2,2])]
        )

        with open(self.camera_registration_filename, 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)
            outfile.close()




    def save_robot_poses(self):
        """Save robot poses to config file for auto registration."""
        data = dict(
            p = []
        )
        for r in self.poses_to_save:
            data["p"].append([r.transform.translation.x, r.transform.translation.y,
                             r.transform.translation.z, r.transform.rotation.x,
                             r.transform.rotation.y, r.transform.rotation.z,
                             r.transform.rotation.w])

        with open(self.poses_filename, 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)
            outfile.close()

    def load_robot_poses(self):
        """Load robot poses from file for auto registration."""
        with open(self.poses_filename, "r") as file:
            documents = yaml.full_load(file)
            self.poses_for_reg = []
            for item, doc in documents.items():
                print(item, ":", doc)
                for p in doc:
                    t = Transform()
                    t.translation.x = p[0]
                    t.translation.y = p[1]
                    t.translation.z = p[2]
                    t.rotation.x = p[3]
                    t.rotation.y = p[4]
                    t.rotation.z = p[5]
                    t.rotation.w = p[6]
                    self.poses_for_reg.append(t)


    def do_auto_registration(self, v, dt):
        """Register arm with a predefined set of positions autonomously.

        Keyword arguments:
        v -- TCP linear velocity
        dt -- sampling time
        """
        for t in self.poses_for_reg:
            self.move_tcp_to(t, v, dt)
            self.gather_actual_position()


    def move_tcp_to(self, target, v, dt):
        """Move the TCP to the desired position on linear trajectory.

        Keyword arguments:
        target -- desired position
        v -- TCP linear velocity
        dt -- sampling time
        """
        # Calculate the linear trajectory
        pos_current_np = np.array([self.measured_cp.transform.translation.x,
                                self.measured_cp.transform.translation.y,
                                self.measured_cp.transform.translation.z])
        pos_target_np = np.array([target.translation.x,
                                    target.translation.y,
                                    target.translation.z])
        d = np.linalg.norm(pos_target_np - pos_current_np)
        T = d / v
        N = int(math.floor(T / dt))
        tx = np.linspace(pos_current_np[0], target[0], N)
        ty = np.linspace(pos_current_np[1], target[1], N)
        tz = np.linspace(pos_current_np[2], target[2], N)

        #SLERP
        rot_current = Rotation.from_quat([self.measured_cp.transform.rotation.x,
                                           self.measured_cp.transform.rotation.y,
                                           self.measured_cp.transform.rotation.z,
                                           self.measured_cp.transform.rotation.w])
        rot_target = Rotation.from_quat([target.rotation.x,
                                           target.rotation.y,
                                           target.rotation.z,
                                           target.rotation.w])
        slerp = Slerp([0,T], [rot_current,rot_target])
        times = np.linspace(0, T, N)
        interp_rots = slerp(times)

        # Set the rate of the loop
        rate = rospy.Rate(1.0 / dt)

        # Send the robot to the points of the calculated trajectory
        # with the desired rate
        for i in range(N):
            if rospy.is_shutdown():
                break # CTRL-C is pressed

            p = self.measured_cp
            p.transform.translation.x = tx[i]
            p.transform.translation.y = ty[i]
            p.transform.translation.z = tz[i]

            quat_helper = interp_rots[i].as_quat()
            p.transform.rotation.x = quat_helper[0]
            p.transform.rotation.y = quat_helper[1]
            p.transform.rotation.z = quat_helper[2]
            p.transform.rotation.w = quat_helper[3]

            #rospy.loginfo(p)
            self.servo_cp_pub.publish(p)
            rate.sleep()    # Sleep to run with the desired rate



    def move_jaw_to(self, target, omega, dt):
        """Move the jaw to the desired angle on linear trajectory.

        Keyword arguments:
        target -- desired jaw angle
        omega -- angular velocity
        dt -- sampling time
        """
        d = abs(target - self.measured_jaw.position[0])
        T = d / omega
        N = int(math.floor(T / dt))
        tj = np.linspace(self.measured_jaw.position[0], target, N)
        rate = rospy.Rate(1.0 / dt) # 10hz

        for i in range(N):
            if rospy.is_shutdown():
                break
            j = self.measured_jaw
            j.position = [tj[i]]
            #rospy.loginfo(j)
            self.servo_jaw_pub.publish(j)
            rate.sleep()



if __name__ == '__main__':
    # Init arm
    reg = HandEyeRegistrator()
    # Sleep is necessary to make sure, that a marker position is received
    rospy.sleep(1.0)

    #reg.move_tcp_to([0.0, 0.0, -0.12], 0.05, 0.01)
    #reg.move_jaw_to(0.0, 0.1, 0.01)
    if reg.mode == "auto":
        reg.load_robot_poses()
        reg.do_auto_registration(0.05, 0.01)
    elif reg.mode == "save" or reg.mode == "simple":
        R, t = reg.collect_and_register()
        reg.save_registration(R, t)
    else:
        print("Please define a correct mode (simple, save, auto). Exiting...")



