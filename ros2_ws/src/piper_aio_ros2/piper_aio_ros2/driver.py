"""Single owner of all four CAN channels. Default startup sends no CAN frames."""
import fcntl
import json
import math
import os
import time
import can
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from piper_msgs.msg import PosCmd, PiperStatusMsg
from piper_sdk import C_PiperInterface
from .core import commandable, fresh, LOWER, UPPER

JOINT_IDS = (0x2A5, 0x2A6, 0x2A7, 0x2A8)
CONTROL_IDS = (0x155, 0x156, 0x157, 0x159)
POSE_IDS = (0x2A2, 0x2A3, 0x2A4)
LOW_IDS = tuple(range(0x261, 0x267))
HIGH_IDS = tuple(range(0x251, 0x257))
NAMES = [f'joint{i}' for i in range(1,7)]+['gripper']


class Arm:
    def __init__(self, name):
        self.name = name
        self.lock = open(f'/tmp/piper_aio_{name}.lock', 'w')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.sdk = C_PiperInterface(can_name=name)
        self.sdk.ConnectPort(piper_init=False, start_thread=False)
        self.bus = can.interface.Bus(channel=name, bustype='socketcan', receive_own_messages=False)
        self.stamps = {}

    def drain(self):
        for _ in range(512):
            msg = self.bus.recv(timeout=0)
            if msg is None:
                return
            if msg.is_error_frame or msg.is_remote_frame or msg.is_extended_id or msg.dlc != 8:
                continue
            self.sdk.ParseCANFrame(msg)
            # Preserve kernel receive age; draining an old queue cannot refresh feedback.
            self.stamps[msg.arbitration_id] = time.monotonic()-max(0, time.time()-msg.timestamp)
        raise RuntimeError(f'{self.name}: CAN receive backlog')

    def valid(self, ids, timeout=0.15):
        return fresh(self.stamps, ids, time.monotonic(), timeout)

    def positions(self, source='feedback'):
        if source == 'control':
            j = self.sdk.GetArmJointCtrl().joint_ctrl
            g = self.sdk.GetArmGripperCtrl().gripper_ctrl
        else:
            j = self.sdk.GetArmJointMsgs().joint_state
            g = self.sdk.GetArmGripperMsgs().gripper_state
        return [math.radians(getattr(j, f'joint_{i}')/1000) for i in range(1,7)]+[g.grippers_angle/1e6]

    def enabled(self):
        low = self.sdk.GetArmLowSpdInfoMsgs()
        return self.valid(LOW_IDS) and all(getattr(low, f'motor_{i}').foc_status.driver_enable_status for i in range(1,7))

    def close(self):
        self.bus.shutdown()
        self.sdk.DisconnectPort()
        self.lock.close()


from .preparation import PreparationMixin


class FourCan(PreparationMixin, Node):
    def __init__(self):
        super().__init__('piper_four_can')
        defaults = {'left_follower':'can_left_fol','right_follower':'can_right_fol',
                    'left_leader':'can_left_lea','right_leader':'can_right_lea',
                    'leader_source':'feedback','auto_teach':True,'rate':100.0,'speed_percent':100,
                    'angular_speed':0.3,'gripper_speed':0.02,'start_angle':0.15,
                    'start_gripper':0.015,'max_tracking_error':0.35, 'lower_limits':LOWER, 'upper_limits':UPPER, 'prepare_pose':[0.0,math.radians(10),math.radians(-10),0.0,0.0,0.0,0.08], 'prepare_speed':0.05, 'prepare_acceleration':0.1, 'recovery_margin':math.radians(3)}
        for key,value in defaults.items(): self.declare_parameter(key,value,ParameterDescriptor(read_only=True))
        self.p = lambda k: self.get_parameter(k).value
        if self.p('leader_source') not in ('feedback','control'): raise ValueError('Invalid leader_source')
        if not 20 <= self.p('rate') <= 200: raise ValueError('rate must be 20..200 Hz')
        if not 1 <= self.p('speed_percent') <= 100: raise ValueError('speed_percent must be 1..100')
        for key in ('angular_speed','gripper_speed','start_angle','start_gripper','max_tracking_error','prepare_speed','prepare_acceleration','recovery_margin'):
            if not math.isfinite(self.p(key)) or self.p(key) <= 0: raise ValueError(key)
        self.lower=list(self.p('lower_limits')); self.upper=list(self.p('upper_limits'))
        if len(self.lower)!=7 or len(self.upper)!=7 or any(not math.isfinite(lo) or not math.isfinite(hi) or lo>=hi for lo,hi in zip(self.lower,self.upper)):
            raise ValueError('Invalid joint limits')
        self.arms = {}; self.pubs = {}; self.pose_pubs = {}; self.status_pubs = {}; self.applied = {}
        self.state = 'READ_ONLY'; self.reason = ''; self.previous = {}; self.follow = {}; self.follow_log = {}; self.last_tick = time.monotonic()
        self.preparation_init()
        self.deadline = 0; self.last_enable = 0; self.teaching = False; self.auto_pending = True; self.auto_log = 0.0
        try:
            for side in ('left','right'):
                for role in ('leader','follower'):
                    key=f'{side}_{role}'; self.arms[key]=Arm(self.p(key))
                    self.pubs[key]=self.create_publisher(JointState,f'/{role}/joint_{side}',10)
                    self.status_pubs[key]=self.create_publisher(PiperStatusMsg,f'/{role}/arm_status_{side}',10)
                self.pose_pubs[side]=self.create_publisher(PosCmd,f'/follower/end_pose_euler_{side}',10)
                self.applied[side]=self.create_publisher(JointState,f'/follower_cmd/joint_{side}',10)
        except Exception:
            for arm in self.arms.values(): arm.close()
            raise
        self.diag=self.create_publisher(String,'/teleop/status',10)
        self.create_service(Trigger,'/teleop/prepare',self.prepare)
        self.create_service(Trigger,'/teleop/prepare_leaders',self.prepare_leaders)
        self.create_service(Trigger,'/teleop/start',self.start)
        self.create_service(Trigger,'/teleop/stop',self.stop)
        self.create_service(SetBool,'/teleop/teach',self.teach)
        self.create_timer(1/self.p('rate'),self.tick)
        self.create_timer(1.0,self.diagnostic)
        if self.p('auto_teach'): self.get_logger().info('Waiting for fresh leader state, then drag-teach and follow start.')
        else: self.get_logger().info('READ_ONLY. No automatic enable, home, mode changes or motion.')

    def maybe_auto(self, now):
        if self.state!='READ_ONLY' or not self.p('auto_teach') or not self.auto_pending: return
        for side in ('left','right'):
            leader=self.arms[f'{side}_leader']
            if not leader.valid((0x2A1,)) or not leader.valid(self.leader_ids(), 1.0):
                self.auto_wait(now,'Waiting for fresh leader state before drag-teach.'); return
            if leader.sdk.GetArmStatus().arm_status.err_code:
                self.auto_wait(now,f'{side} leader fault; drag-teach is waiting.'); return
        try:
            for side in ('left','right'):
                s=self.arms[f'{side}_leader'].sdk
                s.MasterSlaveConfig(0xFC if self.p('leader_source')=='feedback' else 0xFA,0,0,0)
                s.EnableArm(7)
                s.MotionCtrl_1(grag_teach_ctrl=0x01)
            self.teaching=True; self.state='ACTIVE'; self.reason=''; self.follow={}; self.follow_log={}; self.auto_pending=False
            self.get_logger().info('Drag-teach on. Followers track leader joints.')
        except Exception as exc:
            self.get_logger().error(str(exc))

    def auto_wait(self, now, text):
        if now-self.auto_log>=2:
            self.get_logger().info(text); self.auto_log=now

    def leader_ids(self):
        return CONTROL_IDS if self.p('leader_source')=='control' else JOINT_IDS

    def start(self, req, res):
        transitioning=False
        try:
            if self.state not in ('READ_ONLY','STOPPED','FAULT','READY'): raise ValueError('Already starting/running')
            transitioning=True; self.state='ACTIVE'; self.reason=''; self.follow={}; self.follow_log={}
            self.command_followers()
            res.success=True; res.message='Following leader joints'
        except Exception as exc:
            if transitioning: self.halt(str(exc))
            res.success=False; res.message=str(exc)
        return res

    def command_followers(self):
        now=time.monotonic(); speed=self.p('speed_percent')
        for side in ('left','right'):
            leader=self.arms[f'{side}_leader']
            if not leader.valid(self.leader_ids(), 1.0): continue
            follower=self.arms[f'{side}_follower']; s=follower.sdk
            status=s.GetArmStatus().arm_status
            # Teach-exit and e-stop resume share one frame. Repeating it after the arm is
            # already in CAN mode drops the mode back to standby and clears motor enable.
            blocked=status.teach_status==1 or status.arm_status==1
            if blocked or not follower.enabled():
                if now-self.follow.get(side,0)>=0.05:
                    if side not in self.follow: s.MasterSlaveConfig(0xFC,0,0,0)
                    if blocked: s.MotionCtrl_1(emergency_stop=0x02, grag_teach_ctrl=0x02)
                    s.MotionCtrl_2(0x01,0x01,speed)
                    s.EnableArm(7)
                    self.follow[side]=now
                continue
            q=commandable(leader.positions(self.p('leader_source')))
            s.MotionCtrl_2(0x01,0x01,speed)
            s.JointCtrl(*[round(math.degrees(x)*1000) for x in q[:6]])
            s.GripperCtrl(max(0,min(80000,round(q[6]*1e6))),1000,0x01,0)
            s.MotionCtrl_2(0x01,0x01,speed)
            msg=self.joint_msg(follower,q=q); msg.header.stamp=self.get_clock().now().to_msg()
            self.applied[side].publish(msg)
        self.log_angles(now)

    def format_angles(self, arm, source='feedback'):
        ids=CONTROL_IDS if source=='control' else JOINT_IDS
        if not arm.valid(ids, 1.0): return '无数据'
        q=arm.positions(source)
        joints=' '.join(f'J{i} {math.degrees(v):.1f}°' for i,v in enumerate(q[:6],1))
        return f'{joints} 夹爪 {q[6]*1000:.1f}mm'

    def log_angles(self, now):
        if now-self.follow_log.get('angles',0)<1.0: return
        self.follow_log['angles']=now
        source=self.p('leader_source')
        for side in ('left','right'):
            leader=self.format_angles(self.arms[f'{side}_leader'], source)
            follower=self.format_angles(self.arms[f'{side}_follower'])
            self.get_logger().info(f'{side} 主臂 {leader} | 从臂 {follower}')

    def halt(self, reason, state='FAULT'):
        active=self.state in ('ENABLING','ACTIVE','PREPARING','READY')
        targets=set(self.preparation_controlled)
        if self.state in ('ENABLING','ACTIVE','PREPARING'): targets.update(('left_follower','right_follower'))
        self.state=state; self.reason=reason; self.follow.clear(); self.follow_log.clear()
        if active:
            for key in targets:
                try: self.arms[key].sdk.MotionCtrl_1(emergency_stop=0x01)
                except Exception as exc: self.get_logger().error(str(exc))
        self.previous.clear()

    def stop(self, req, res):
        self.halt('Operator stop','STOPPED'); res.success=True; res.message='Stopped; explicit start required'
        return res

    def prepare_leaders(self, req, res):
        if self.state in ('ENABLING','ACTIVE','PREPARING'):
            res.message='Stop following before changing communication role'; return res
        if self.p('leader_source')!='feedback':
            res.message='prepare_leaders is for feedback source only'; return res
        try:
            for side in ('left','right'):
                self.arms[f'{side}_leader'].sdk.MasterSlaveConfig(0xFC,0,0,0)
            res.success=True; res.message='Feedback role requested; no enable or position command sent. Wait for fresh state.'
        except Exception as exc: res.message=str(exc)
        return res

    def teach(self, req, res):
        try:
            if self.state in ('ENABLING','ACTIVE','PREPARING'): raise ValueError('Stop following before changing teach mode')
            if req.data:
                for side in ('left','right'):
                    arm=self.arms[f'{side}_leader']
                    if not arm.valid((0x2A1,)): raise ValueError(f'{side} leader status missing; check power/CAN before teach')
                    if arm.sdk.GetArmStatus().arm_status.err_code: raise ValueError(f'{side} leader fault')
            for side in ('left','right'):
                s=self.arms[f'{side}_leader'].sdk
                if req.data:
                    s.MasterSlaveConfig(0xFC if self.p('leader_source')=='feedback' else 0xFA,0,0,0)
                    s.EnableArm(7)
                    s.MotionCtrl_1(grag_teach_ctrl=0x01)
                else:
                    s.MotionCtrl_1(grag_teach_ctrl=0x02)
            self.teaching=req.data
            res.success=True; res.message='Teach command sent; verify arm status before dragging'
        except Exception as exc:
            res.success=False; res.message=str(exc)
        return res

    def joint_msg(self, arm, source='feedback', q=None):
        msg=JointState(); msg.name=NAMES
        ids=CONTROL_IDS if source=='control' else JOINT_IDS
        age=max(0,time.monotonic()-min(arm.stamps.get(i,time.monotonic()) for i in ids))
        msg.header.stamp=rclpy.time.Time(nanoseconds=max(0,self.get_clock().now().nanoseconds-int(age*1e9))).to_msg()
        msg.position=list(arm.positions(source) if q is None else q)
        if source=='feedback' and arm.valid(HIGH_IDS):
            high=arm.sdk.GetArmHighSpdInfoMsgs()
            msg.velocity=[getattr(high,f'motor_{i}').motor_speed/1000 for i in range(1,7)]+[0.0]
            msg.effort=[float(getattr(high,f'motor_{i}').effort) for i in range(1,7)]+[arm.sdk.GetArmGripperMsgs().gripper_state.grippers_effort/1000]
        else:
            msg.velocity=[0.0]*7; msg.effort=[0.0]*7
        return msg

    def publish_states(self):
        for key,arm in self.arms.items():
            source=self.p('leader_source') if key.endswith('leader') else 'feedback'
            if arm.valid(CONTROL_IDS if source=='control' else JOINT_IDS):
                self.pubs[key].publish(self.joint_msg(arm,source))
            if arm.valid((0x2A1,)):
                raw=arm.sdk.GetArmStatus().arm_status; msg=PiperStatusMsg()
                for field in ('ctrl_mode','arm_status','teach_status','motion_status','trajectory_num','err_code'):
                    setattr(msg,field,getattr(raw,field))
                msg.mode_feedback=raw.mode_feed
                for i in range(1,7):
                    for field in (f'joint_{i}_angle_limit',f'communication_status_joint_{i}'):
                        setattr(msg,field,getattr(raw.err_status,field))
                self.status_pubs[key].publish(msg)
        for side in ('left','right'):
            arm=self.arms[f'{side}_follower']
            if arm.valid(POSE_IDS+(0x2A8,)):
                raw=arm.sdk.GetArmEndPoseMsgs().end_pose; msg=PosCmd()
                msg.x=raw.X_axis/1e6; msg.y=raw.Y_axis/1e6; msg.z=raw.Z_axis/1e6
                msg.roll=math.radians(raw.RX_axis/1000); msg.pitch=math.radians(raw.RY_axis/1000); msg.yaw=math.radians(raw.RZ_axis/1000)
                msg.gripper=arm.positions()[6]; self.pose_pubs[side].publish(msg)

    def tick(self):
        now=time.monotonic(); dt=now-self.last_tick; self.last_tick=now
        try:
            for arm in self.arms.values(): arm.drain()
            self.publish_states()
            if self.state=='PREPARING':
                self.preparation_tick(now,dt); return
            self.maybe_auto(now)
            if self.state!='ACTIVE': return
            self.command_followers()
        except Exception as exc:
            if self.state!='FAULT': self.get_logger().error(str(exc))
            self.halt(str(exc))

    def diagnostic(self):
        msg=String(); msg.data=json.dumps({'state':self.state,'reason':self.reason,'teach_requested':self.teaching,
            'preparation': None if self.preparation is None else {k:self.preparation[k] for k in ('index','order','phase')},
            'arms':{k:{'can':a.name,'feedback_fresh':a.valid(JOINT_IDS),'control_fresh':a.valid(CONTROL_IDS),'status_fresh':a.valid((0x2A1,))} for k,a in self.arms.items()}})
        self.diag.publish(msg)

    def close(self):
        self.halt('Driver shutdown','STOPPED')
        for arm in self.arms.values(): arm.close()


def main(args=None):
    rclpy.init(args=args); node=None
    try:
        node=FourCan(); rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if node: node.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
