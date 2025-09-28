#!/usr/bin/env python3
# encoding: utf-8
from threading import Thread

import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult

class Segment:
    def __init__(self, num_joints):
        self.start_time = 0.0  # trajectory segment start time
        self.duration = 0.0  # trajectory segment duration
        self.positions = [0.0] * num_joints
        self.velocities = [0.0] * num_joints


class JointTrajectoryActionController:
    def __init__(self, controller_namespace, controllers):
        self.update_rate = 200
        self.state_update_rate = 20
        self.trajectory = []

        self.controller_namespace = controller_namespace
        self.joint_names = [c.joint_name for c in controllers]

        self.joint_to_controller = {}
        for c in controllers:
            self.joint_to_controller[c.joint_name] = c

        self.port_to_joints = {}
        for c in controllers:
            if c.port_id not in self.port_to_joints:
                self.port_to_joints[c.port_id] = []
            self.port_to_joints[c.port_id].append(c.joint_name)

        self.port_to_io = {}
        for c in controllers:
            if c.port_id in self.port_to_io:
                continue
            self.port_to_io[c.port_id] = c.servo_io

        self.joint_states = dict(zip(self.joint_names, [c.joint_state for c in controllers]))
        self.num_joints = len(self.joint_names)
        self.joint_to_idx = dict(zip(self.joint_names, range(self.num_joints)))

    def initialize(self):
        ns = self.controller_namespace + '/joint_trajectory_action_node/constraints'
        self.stopped_velocity_tolerance = rospy.get_param(ns + '/stopped_velocity_tolerance', 0.01)
        self.goal_constraints = []
        self.trajectory_constraints = []
        self.min_velocity = rospy.get_param(self.controller_namespace + '/joint_trajectory_action_node/min_velocity',
                                            0.1)

        for joint in self.joint_names:
            self.goal_constraints.append(rospy.get_param(ns + '/' + joint + '/goal', -1.0))
            self.trajectory_constraints.append(rospy.get_param(ns + '/' + joint + '/trajectory', -1.0))

        # Message containing current state for all controlled joints
        self.msg = FollowJointTrajectoryFeedback()
        self.msg.joint_names = self.joint_names
        self.msg.desired.positions = [0.0] * self.num_joints
        self.msg.desired.velocities = [0.0] * self.num_joints
        self.msg.desired.accelerations = [0.0] * self.num_joints
        self.msg.actual.positions = [0.0] * self.num_joints
        self.msg.actual.velocities = [0.0] * self.num_joints
        self.msg.error.positions = [0.0] * self.num_joints
        self.msg.error.velocities = [0.0] * self.num_joints

        return True

    def start(self):
        self.running = True

        self.command_sub = rospy.Subscriber(self.controller_namespace + '/command', JointTrajectory,
                                            self.process_command)
        self.state_pub = rospy.Publisher(self.controller_namespace + '/state', FollowJointTrajectoryFeedback,
                                         queue_size=1)
        self.action_server = actionlib.SimpleActionServer(self.controller_namespace + '/follow_joint_trajectory',
                                                          FollowJointTrajectoryAction,
                                                          execute_cb=self.process_follow_trajectory,
                                                          auto_start=False)
        self.action_server.start()
        Thread(target=self.update_state).start()

    def stop(self):
        self.running = False

    def process_command(self, msg):
        if self.action_server.is_active(): self.action_server.set_preempted()

        while self.action_server.is_active():
            rospy.sleep(0.01)

        self.process_trajectory(msg)

    def process_follow_trajectory(self, goal):
        self.process_trajectory(goal.trajectory)

    def process_trajectory(self, traj):
        num_points = len(traj.points)

        if num_points == 0:
            msg = 'Incoming trajectory is empty'
            rospy.logerr(msg)
            self.action_server.set_aborted(text=msg)
            return

        # correlate the joints we're commanding to the joints in the message
        # map from an index of joint in the controller to an index in the trajectory
        lookup = [traj.joint_names.index(joint) for joint in self.joint_names]
        durations = [0.0] * num_points

        # find out the duration of each segment in the trajectory
        durations[0] = traj.points[0].time_from_start.to_sec()

        for i in range(1, num_points):
            durations[i] = (traj.points[i].time_from_start - traj.points[i - 1].time_from_start).to_sec()

        if not traj.points[0].positions:
            res = FollowJointTrajectoryResult()
            res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            msg = 'First point of trajectory has no positions'
            rospy.logerr(msg)
            self.action_server.set_aborted(result=res, text=msg)
            return

        trajectory = []
        time = rospy.Time.now() + rospy.Duration(0.01)

        for i in range(num_points):
            seg = Segment(self.num_joints)

            if traj.header.stamp == rospy.Time(0.0):
                seg.start_time = (time + traj.points[i].time_from_start).to_sec() - durations[i]
            else:
                seg.start_time = (traj.header.stamp + traj.points[i].time_from_start).to_sec() - durations[i]

            seg.duration = durations[i]

            # Checks that the incoming segment has the right number of elements.
            # if traj.points[i].velocities and len(traj.points[i].velocities) != self.num_joints:
            #     res = FollowJointTrajectoryResult()
            #     res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            #     msg = 'Command point %d has %d elements for the velocities' % (i, len(traj.points[i].velocities))
            #     rospy.logerr(msg)
            #     self.action_server.set_aborted(result=res, text=msg)
            #     return
            #
            # if len(traj.points[i].positions) != self.num_joints:
            #     res = FollowJointTrajectoryResult()
            #     res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            #     msg = 'Command point %d has %d elements for the positions' % (i, len(traj.points[i].positions))
            #     rospy.logerr(msg)
            #     self.action_server.set_aborted(result=res, text=msg)
            #     return

            for j in range(self.num_joints):
                if traj.points[i].positions:
                    seg.positions[j] = traj.points[i].positions[lookup[j]]

            trajectory.append(seg)

        rospy.loginfo('Trajectory start requested at %.3lf, waiting...', traj.header.stamp.to_sec())
        rate = rospy.Rate(self.update_rate)

        while traj.header.stamp > time:
            time = rospy.Time.now()
            rate.sleep()

        end_time = traj.header.stamp + rospy.Duration(sum(durations))
        seg_end_times = [rospy.Time.from_sec(trajectory[seg].start_time + durations[seg]) for seg in
                         range(len(trajectory))]

        rospy.loginfo('Trajectory start time is %.3lf, end time is %.3lf, total duration is %.3lf', time.to_sec(),
                      end_time.to_sec(), sum(durations))

        self.trajectory = trajectory

        for seg in range(len(trajectory)):
            rospy.logdebug('current segment is %d time left %f cur time %f' % (
                seg, durations[seg] - (time.to_sec() - trajectory[seg].start_time), time.to_sec()))
            rospy.logdebug('goal positions are: %s' % str(trajectory[seg].positions))

            # first point in trajectories calculated by OMPL is current position with duration of 0 seconds, skip it
            if durations[seg] == 0:
                rospy.logdebug('skipping segment %d with duration of 0 seconds' % seg)
                continue

            multi_packet = {}

            for port, joints in self.port_to_joints.items():
                vals = []

                for joint in joints:
                    j = self.joint_names.index(joint)
                    desired_position = trajectory[seg].positions[j]
                    self.msg.desired.positions[j] = desired_position
                    servo_id = self.joint_to_controller[joint].servo_id
                    pos = self.joint_to_controller[joint].pos_rad_to_raw(desired_position)
                    vals.append((servo_id, pos))

                multi_packet[port] = vals

            for port, vals in multi_packet.items():
                # dur = durations[seg]
                # dur = dur if dur > 30 else 30
                for id_, pos_ in vals:
                    self.port_to_io[port].set_position(id_, pos_, durations[seg])

            while time < seg_end_times[seg]:
                # heck if new trajectory was received, if so abort current trajectory execution
                # by setting the goal to the current position c
                if self.action_server.is_preempt_requested():
                    msg = 'New trajectory received. Exiting.'
                    self.action_server.set_preempted(text=msg)
                    rospy.loginfo(msg)
                    return

                rate.sleep()
                time = rospy.Time.now()

            # Verifies trajectory constraints
            for j, joint in enumerate(self.joint_names):
                if self.trajectory_constraints[j] > 0 and self.msg.error.positions[j] > self.trajectory_constraints[j]:
                    res = FollowJointTrajectoryResult()
                    res.error_code = FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
                    msg = 'Unsatisfied position constraint for %s, trajectory point %d, %f is larger than %f' % \
                          (joint, seg, self.msg.error.positions[j], self.trajectory_constraints[j])
                    rospy.logwarn(msg)
                    self.action_server.set_aborted(result=res, text=msg)
                    return

        # Checks that we have ended inside the goal constraints
        for (joint, pos_error, pos_constraint) in zip(self.joint_names, self.msg.error.positions,
                                                      self.goal_constraints):
            if pos_constraint > 0 and abs(pos_error) > pos_constraint:
                res = FollowJointTrajectoryResult()
                res.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
                msg = 'Aborting because %s joint wound up outside the goal constraints, %f is larger than %f' % \
                      (joint, pos_error, pos_constraint)
                rospy.logwarn(msg)
                self.action_server.set_aborted(result=res, text=msg)
                break
        else:
            msg = 'Trajectory execution successfully completed'
            rospy.loginfo(msg)
            res = FollowJointTrajectoryResult()
            res.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            self.action_server.set_succeeded(result=res, text=msg)

    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():
            self.msg.header.stamp = rospy.Time.now()

            # Publish current joint state
            for i, joint in enumerate(self.joint_names):
                state = self.joint_states[joint]
                self.msg.actual.positions[i] = state.current_pos
                self.msg.actual.velocities[i] = abs(state.velocity)
                self.msg.error.positions[i] = self.msg.actual.positions[i] - self.msg.desired.positions[i]
                self.msg.error.velocities[i] = self.msg.actual.velocities[i] - self.msg.desired.velocities[i]

            self.state_pub.publish(self.msg)
            rate.sleep()
