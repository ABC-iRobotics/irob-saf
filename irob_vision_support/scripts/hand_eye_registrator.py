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
import yaml


class HandEyeRegistrator:


    def __init__(self):
        """Constructor."""
        print("Node started")
        rospy.init_node('hand_eye_registrator', anonymous=True)
        self.arm = rospy.get_param('~arm')
        self.camera_registration_filename = rospy.get_param('~camera_registration_file')

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
            rospy.sleep(0.5)

            robot_pos = np.array([self.measured_cp.transform.translation.x,
                                  self.measured_cp.transform.translation.y,
                                  self.measured_cp.transform.translation.z]).T
            fiducial_pos = np.array([self.fiducial_tf.x,
                                     self.fiducial_tf.y,
                                     self.fiducial_tf.z]).T

            self.robot_positions = np.vstack((self.robot_positions, robot_pos))
            self.fiducial_positions = np.vstack((self.fiducial_positions, fiducial_pos))
            print("Positions collected: " + str(self.robot_positions.shape[1]))



    def collect_and_register(self):
        """Wait for data colection. When key pressed,
        do the registration.
        """
        input("Collect data for registration. Press Enter when done...")

        R, t = rigid_transform_3D(self.robot_positions, self.fiducial_positions)

        # Check transformation
        points_transformed = np.zeros(self.robot_positions.shape)
        for i in range(self.robot_positions.shape[1]):
            p = np.dot(R, self.robot_positions[:,i]) + t.T
            points_transformed[:,i] = p

        # Draw plot
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')
        self.ax.scatter(points[0,:], points[1,:], points[2,:], marker='o')
        self.ax.scatter(points_transformed[0,:], points_transformed[1,:],
                                        points_transformed[2,:], marker='^')


        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

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



    # TODO slerp
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
        pos_target_np = np.array(target)
        d = np.linalg.norm(pos_target_np - pos_current_np)
        T = d / v
        N = int(math.floor(T / dt))
        tx = np.linspace(pos_current_np[0], target[0], N)
        ty = np.linspace(pos_current_np[1], target[1], N)
        tz = np.linspace(pos_current_np[2], target[2], N)
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
    R, t = reg.collect_and_register()
    reg.save_registration(R, t)


