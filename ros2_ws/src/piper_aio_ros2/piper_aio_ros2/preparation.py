"""Sequential, bounded recovery to a common pose. No calibration writes."""
import math
import time
from .core import validate


def duration(start, goal, velocity, acceleration):
    distance=max(abs(a-b) for a,b in zip(start,goal))
    return max(1.0,1.875*distance/velocity,math.sqrt(5.774*distance/acceleration))


def trajectory(start, goal, elapsed, seconds):
    u=max(0.0,min(1.0,elapsed/seconds)); blend=10*u**3-15*u**4+6*u**5
    return [a+(b-a)*blend for a,b in zip(start,goal)]


def recovery_start(q, lower, upper, margin):
    if len(q)!=7 or not all(math.isfinite(v) for v in q): raise ValueError('Invalid feedback')
    for i,(v,lo,hi) in enumerate(zip(q,lower,upper)):
        allowance=margin if i<6 else 0.002
        if v<lo-allowance or v>hi+allowance: raise ValueError(f'Joint {i+1} exceeds recovery allowance')
    return list(q)


class PreparationMixin:
    def preparation_init(self):
        self.preparation=None; self.preparation_controlled=set()

    def prepare(self,req,res):
        try:
            if self.state not in ('READ_ONLY','STOPPED','FAULT','READY'): raise ValueError('Stop current operation first')
            goal=list(self.p('prepare_pose')); validate(goal,self.lower,self.upper)
            order=['left_follower','left_leader','right_follower','right_leader']
            starts={}
            for key in order:
                arm=self.arms[key]
                if not arm.valid(self.preparation_ids()): raise ValueError(f'{key}: missing fresh feedback')
                status=arm.sdk.GetArmStatus().arm_status
                if status.err_code: raise ValueError(f'{key}: arm error {status.err_code}')
                starts[key]=recovery_start(arm.positions(),self.lower,self.upper,self.p('recovery_margin'))
            self.preparation={'order':order,'index':0,'phase':'configure','goal':goal,
                              'starts':starts,'elapsed':0.0,'settled':0.0}
            self.state='PREPARING'; self.reason=''; self.teaching=False
            res.success=True; res.message='Sequential preparation accepted; monitor PREPARING -> READY. Following stays off.'
        except Exception as exc: res.message=str(exc)
        return res

    @staticmethod
    def preparation_ids():
        return (0x2A1,0x2A5,0x2A6,0x2A7,0x2A8)+tuple(range(0x261,0x267))

    def preparation_tick(self,now,dt):
        p=self.preparation
        if dt<=0 or dt>0.1: raise ValueError('Preparation control deadline exceeded')
        key=p['order'][p['index']]; arm=self.arms[key]; s=arm.sdk
        # Monitor all four arms, including already-positioned arms.
        for name,a in self.arms.items():
            if not a.valid(self.preparation_ids()): raise ValueError(f'{name}: stale feedback during preparation')
            if a.sdk.GetArmStatus().arm_status.err_code: raise ValueError(f'{name}: error during preparation')
            recovery_start(a.positions(),self.lower,self.upper,self.p('recovery_margin'))
        q=arm.positions(); phase=p['phase']
        if phase=='configure':
            p['start']=recovery_start(q,self.lower,self.upper,self.p('recovery_margin'))
            self.preparation_controlled.add(key)
            s.MasterSlaveConfig(0xFC,0,0,0)
            # Seed current position before leaving teach mode; do not reuse an old target.
            s.JointCtrl(*[round(math.degrees(v)*1000) for v in q[:6]])
            s.MotionCtrl_1(grag_teach_ctrl=0x02)
            s.MotionCtrl_1(emergency_stop=0x02)
            s.MotionCtrl_2(0x01,0x01,5)
            s.EnableArm(7)
            p.update(phase='enable',deadline=now+5,last_enable=now)
            return
        if phase=='enable':
            if now>p['deadline']: raise ValueError(f'{key}: enable/mode timeout')
            if now-p['last_enable']>0.2:
                s.EnableArm(7); s.MotionCtrl_2(0x01,0x01,5); p['last_enable']=now
            if arm.enabled() and s.GetArmStatus().arm_status.ctrl_mode==1:
                p['start']=q; p['last_command']=q[:6]
                p.update(phase='joints',elapsed=0.0,settled=0.0,
                         seconds=duration(q[:6],p['goal'][:6],self.p('prepare_speed'),self.p('prepare_acceleration')))
            return
        if not arm.enabled(): raise ValueError(f'{key}: lost motor enable')
        if s.GetArmStatus().arm_status.ctrl_mode!=1: raise ValueError(f'{key}: lost CAN control mode')
        if phase=='joints':
            if max(abs(a-b) for a,b in zip(q[:6],p['last_command']))>0.12:
                raise ValueError(f'{key}: joint tracking error during preparation')
            p['elapsed']+=dt
            target=trajectory(p['start'][:6],p['goal'][:6],p['elapsed'],p['seconds'])
            # Outside nominal limits, only the initial-to-limit inward recovery segment is allowed.
            for i,v in enumerate(target):
                if v<min(p['start'][i],self.lower[i])-1e-9 or v>max(p['start'][i],self.upper[i])+1e-9:
                    raise ValueError('Recovery corridor violation')
            s.MotionCtrl_2(0x01,0x01,5)
            s.JointCtrl(*[round(math.degrees(v)*1000) for v in target]); p['last_command']=target
            if p['elapsed']>p['seconds']+8: raise ValueError(f'{key}: joint arrival timeout')
            arrived=p['elapsed']>=p['seconds'] and max(abs(a-b) for a,b in zip(q[:6],p['goal'][:6]))<0.02
            p['settled']=p['settled']+dt if arrived else 0.0
            if p['settled']>=0.5:
                p.update(phase='gripper',elapsed=0.0,settled=0.0,gripper_start=q[6],last_gripper=q[6],
                         seconds=duration([q[6]],[p['goal'][6]],0.01,0.02))
            return
        if phase=='gripper':
            if max(abs(a-b) for a,b in zip(q[:6],p['goal'][:6]))>0.04: raise ValueError(f'{key}: joint drift while opening gripper')
            if abs(q[6]-p['last_gripper'])>0.015: raise ValueError(f'{key}: gripper blocked/tracking error')
            p['elapsed']+=dt
            target=trajectory([p['gripper_start']],[p['goal'][6]],p['elapsed'],p['seconds'])[0]
            s.GripperCtrl(round(target*1e6),300,0x01,0); p['last_gripper']=target
            if p['elapsed']>p['seconds']+8: raise ValueError(f'{key}: gripper arrival timeout')
            arrived=p['elapsed']>=p['seconds'] and abs(q[6]-p['goal'][6])<0.002
            p['settled']=p['settled']+dt if arrived else 0.0
            if p['settled']>=0.5:
                p['index']+=1
                if p['index']==len(p['order']):
                    for name,a in self.arms.items():
                        actual=a.positions(); validate(actual,self.lower,self.upper)
                        if max(abs(x-y) for x,y in zip(actual[:6],p['goal'][:6]))>0.03 or abs(actual[6]-p['goal'][6])>0.003:
                            raise ValueError(f'{name}: final alignment failed')
                    self.state='READY'; self.reason='All four arms at preparation pose; follow remains off'
                    p['phase']='complete'
                else: p['phase']='configure'
