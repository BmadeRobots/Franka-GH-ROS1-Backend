#!/usr/bin/env python3
import rospy
from frankx import Affine, LinearMotion, Robot, JointMotion,MotionData,Reaction,Measure
from franka_compas_interface.msg import FrankaState
from franka_compas_interface.srv import OpenGripper, OpenGripperResponse, CloseGripper, CloseGripperResponse, MoveJoint, MoveJointResponse, MoveLinear, MoveLinearResponse,HomeRobot,HomeRobotResponse
from franka_compas_interface.srv import MoveReaction, MoveReactionResponse
import frankx
import math

class FrankaClient:
    def __init__(self):
        self.robot = Robot("172.16.0.2")
        self.robot.set_default_behavior()
        self.robot.recover_from_errors()
      
        # self.gripper = self.robot.gripper
        self.gripper = self.robot.get_gripper()

        self.home_robot()

        #define timer to publish robot state
        self.robot_state_pub = rospy.Publisher("/robot/state",FrankaState,queue_size=10)
        self.robot_state_timer = rospy.Timer(rospy.Duration(0.1), self.publish_robot_state)

        #Define srv 
        self.open_gripper_srv = rospy.Service("/robot/open_gripper",OpenGripper,self.open_gripper)
        self.close_gripper_srv = rospy.Service("/robot/close_gripper",CloseGripper,self.close_gripper)
        self.move_joint_srv = rospy.Service("/robot/move_joint",MoveJoint,self.move_jnt)
        self.move_linear_srv = rospy.Service("/robot/move_linear",MoveLinear,self.move_linear)
        self.home_robot_srv = rospy.Service("/robot/home",HomeRobot,self.home_robot_srv_cb)
        self.move_reaction_srv = rospy.Service("/robot/move_reaction",MoveReaction,self.move_reaction_srv_cb)

        print("Franka client initialized")
        
    def publish_robot_state(self,event):
        try:
            robot_state = self.robot.get_state(read_once=True)
        except frankx.InvalidOperationException:
            robot_state = self.robot.get_state(read_once=False)
        
        msg = FrankaState()
        msg.o_t_ee = robot_state.O_T_EE
        msg.o_t_ee_d = robot_state.O_T_EE_d
        msg.f_t_ee = robot_state.F_T_EE
        msg.ee_t_k = robot_state.EE_T_K
        msg.m_ee = robot_state.m_ee
        msg.i_ee = robot_state.I_ee
        msg.f_x_cee = robot_state.F_x_Cee
        msg.m_load = robot_state.m_load
        msg.f_x_cload = robot_state.F_x_Cload
        msg.m_total = robot_state.m_total
        msg.i_total = robot_state.I_total
        msg.f_x_ctotal = robot_state.F_x_Ctotal
        msg.elbow = robot_state.elbow
        msg.elbow_d = robot_state.elbow_d
        msg.elbow_c = robot_state.elbow_c
        msg.delbow_c = robot_state.delbow_c
        msg.ddelbow_c = robot_state.ddelbow_c
        msg.tau_j = robot_state.tau_J
        msg.tau_j_d = robot_state.tau_J_d
        msg.dtau_j = robot_state.dtau_J
        msg.q = robot_state.q
        msg.q_d = robot_state.q_d
        msg.dq = robot_state.dq
        msg.dq_d = robot_state.dq_d
        msg.ddq_d = robot_state.ddq_d
        msg.cartesian_contact = robot_state.cartesian_contact
        msg.joint_collision = robot_state.joint_collision
        msg.cartesian_collision = robot_state.cartesian_collision
        msg.tau_ext_hat_filtered = robot_state.tau_ext_hat_filtered
        msg.o_f_ext_hat_k = robot_state.O_F_ext_hat_K
        msg.k_f_ext_hat_k = robot_state.K_F_ext_hat_K
        msg.o_t_ee_c = robot_state.O_T_EE_c
        msg.o_dp_ee_c = robot_state.O_dP_EE_c
        msg.o_ddp_ee_c = robot_state.O_ddP_EE_c
        msg.theta = robot_state.theta
        msg.dtheta = robot_state.dtheta
        self.robot_state_pub.publish(msg)

    def open_gripper(self,req):
        print("Opening gripper: ",req.distance)
        self.gripper.move(req.distance)
        return OpenGripperResponse(True)
    
    def close_gripper(self,req):
        print("Closing gripper: ",req.gripper_speed,req.gripper_force)
        self.gripper.gripper_speed = req.gripper_speed
        self.gripper.gripper_force = req.gripper_force
        self.gripper.clamp()
        return CloseGripperResponse(True)

    def home_robot_srv_cb(self,req):
        print("Home robot request")
        self.home_robot()
        return HomeRobotResponse(True)
    
    def home_robot(self):
        self.robot.set_default_behavior()
        self.robot.recover_from_errors()
        self.robot.set_dynamic_rel(0.2)
        self.robot.move(JointMotion([0, -0.785, 0, -2.356, 0, 1.571, math.pi/4]))
        
    def move_linear(self,req):
        print("Receive move linear request",req)
        self.robot.set_default_behavior()
        self.robot.recover_from_errors()
        #clamp req.speed to 0 - 1
        speed = req.speed
        if speed < 0:
            speed = 0
        if speed > 1:
            speed = 1
        self.robot.set_dynamic_rel(speed)

        motion = LinearMotion(Affine(req.x,req.y,req.z,req.rz,req.ry,req.rx))
        self.robot.move(motion)

        return MoveLinearResponse(True)

    def move_jnt(self,req):
        print("Receive move joint request",req)
        joints = req.joint_positions
        
        if len(joints) != 7:
            print("Invalid number of joints")
            return MoveJointResponse(False)
        print("Moving joints to: ",joints)
        self.robot.move(JointMotion(joints))
        return MoveJointResponse(True)

    def move_reaction_srv_cb(self,req):
        print("Receive move reaction request",req)
        self.robot.set_default_behavior()
        self.robot.recover_from_errors()

        p = req.target_plane
        motion = LinearMotion(Affine(p.x,p.y,p.z,p.rz,p.ry,p.rx))

        rv = req.reaction_vector
        # motion_data = MotionData().with_reaction(Reaction(Measure.ForceXYZNorm > req.max_force), )
        reaction_motion = LinearMotion(Affine(rv.x,rv.y,rv.z,rv.rz,rv.ry,rv.rx))
        
        motion_data = MotionData().with_reaction(Reaction(Measure.ForceZ < -10.0, reaction_motion))
        self.robot.move(motion,motion_data)

        if motion_data.did_break:
            print(f"Warning!!! Force limit {req.max_force} N reached")
        return MoveReactionResponse(True)


if __name__ == "__main__":
    rospy.init_node("franka_client")
    client = FrankaClient()
    rospy.spin()