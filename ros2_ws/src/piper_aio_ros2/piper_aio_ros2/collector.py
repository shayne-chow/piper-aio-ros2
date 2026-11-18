"""ROS2 SFT recorder. Bounded queues and append-only temporary HDF5 episodes."""
from collections import deque
import json
import os
from pathlib import Path
import time
import h5py
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from piper_msgs.msg import PosCmd


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def synchronized(queues, now, max_age, max_skew):
    if any(not q for q in queues.values()): return None
    target=min(q[-1][0] for q in queues.values())
    chosen={key:min(q,key=lambda x:abs(x[0]-target)) for key,q in queues.items()}
    stamps=[v[0] for v in chosen.values()]
    if any(now-s > max_age or s-now > max_skew for s in stamps): return None
    if max(stamps)-min(stamps)>max_skew: return None
    return target,chosen


class Recorder(Node):
    def __init__(self):
        super().__init__('piper_sft_recorder')
        defaults={'dataset_dir':str(Path.home()/'data'),'task_name':'piper_ros2',
                  'camera_names':['cam_high','cam_left_wrist','cam_right_wrist'],
                  'image_topics':['/camera_f/color/image_raw/compressed','/camera_l/color/image_raw/compressed','/camera_r/color/image_raw/compressed'],
                  'depth_topics':['/camera_f/depth/image_raw','/camera_l/depth/image_raw','/camera_r/depth/image_raw'],
                  'use_depth':False,'frame_rate':30.0,'max_age':0.2,'max_skew':0.05}
        for key,value in defaults.items(): self.declare_parameter(key,value)
        self.p=lambda k:self.get_parameter(k).value
        self.names=list(self.p('camera_names'))
        if len(self.names)!=3 or len(set(self.names))!=3 or any('/' in n or not n for n in self.names): raise ValueError('Three unique camera names required')
        if len(self.p('image_topics'))!=3 or len(self.p('depth_topics'))!=3: raise ValueError('Three camera topics required')
        if any(not np.isfinite(self.p(k)) or self.p(k)<=0 for k in ('frame_rate','max_age','max_skew')): raise ValueError('Invalid timing parameters')
        self.queues={}; self.subs=[]; self.bridge=CvBridge(); self.file=None; self.frames=0; self.last_target=-1
        self.teleop=''; self.status_time=0; self.last_warning=0
        self.directory=Path(self.p('dataset_dir')).expanduser()/self.p('task_name'); self.directory.mkdir(parents=True,exist_ok=True)
        for side in ('left','right'):
            for role in ('leader','follower','follower_cmd'):
                self.subscribe(f'{role}_{side}',f'/{role}/joint_{side}',JointState,30)
            self.subscribe(f'pose_{side}',f'/follower/end_pose_euler_{side}',PosCmd,30)
        for name,topic in zip(self.names,self.p('image_topics')):
            image_type = CompressedImage if topic.endswith('/compressed') else Image
            self.subscribe(f'image_{name}',topic,image_type,10)
        if self.p('use_depth'):
            for name,topic in zip(self.names,self.p('depth_topics')): self.subscribe(f'depth_{name}',topic,Image,10)
        self.subs.append(self.create_subscription(String,'/teleop/status',self.status,10))
        self.create_service(Trigger,'/record/start',self.start)
        self.create_service(Trigger,'/record/save',self.save)
        self.create_service(Trigger,'/record/discard',self.discard)
        self.create_timer(1/self.p('frame_rate'),self.tick)
        self.get_logger().info('Ready: /record/start, /record/save, /record/discard. Recording requires ACTIVE teleop and all streams.')

    def subscribe(self,key,topic,kind,depth):
        self.queues[key]=deque(maxlen=depth)
        def callback(msg):
            stamp=stamp_seconds(msg.header.stamp) if hasattr(msg,'header') else self.get_clock().now().nanoseconds/1e9
            if stamp>0: self.queues[key].append((stamp,msg))
        self.subs.append(self.create_subscription(kind,topic,callback,qos_profile_sensor_data))

    def status(self,msg):
        try: self.teleop=json.loads(msg.data)['state']; self.status_time=time.monotonic()
        except (ValueError,KeyError): self.teleop=''

    def start(self,req,res):
        if self.file is not None: res.message='Already recording'; return res
        index=0
        while (self.directory/f'episode_{index}.hdf5').exists() or (self.directory/f'episode_{index}.partial.hdf5').exists(): index+=1
        self.path=self.directory/f'episode_{index}.partial.hdf5'
        self.file=h5py.File(self.path,'x'); self.file.attrs['action_semantics']='raw leader absolute target; action_applied is rate-limited command'
        self.file.attrs['image_encoding']='rgb8'; self.file.attrs['ros_version']=2
        self.file.attrs['joint_units']='rad for joints 1-6, m for gripper'
        self.file.attrs['camera_names']=json.dumps(self.names)
        self.file.attrs['frame_rate_requested']=self.p('frame_rate')
        self.frames=0; self.last_target=-1
        for queue in self.queues.values(): queue.clear()
        res.success=True; res.message=f'Recording to {self.path}'; return res

    def append(self,path,value):
        value=np.asarray(value)
        if path not in self.file:
            parent,_,name=path.rpartition('/')
            group=self.file.require_group(parent) if parent else self.file
            kwargs={'compression':'lzf'} if value.ndim>=2 else {}
            group.create_dataset(name,shape=(0,)+value.shape,maxshape=(None,)+value.shape,chunks=(1,)+value.shape,dtype=value.dtype,**kwargs)
        dataset=self.file[path]
        if dataset.shape[1:]!=value.shape: raise ValueError(f'{path}: shape changed during episode')
        dataset.resize(self.frames+1,axis=0); dataset[self.frames]=value

    def tick(self):
        if self.file is None: return
        if self.teleop!='ACTIVE' or time.monotonic()-self.status_time>2: return
        result=synchronized(self.queues,self.get_clock().now().nanoseconds/1e9,self.p('max_age'),self.p('max_skew'))
        if result is None:
            if time.monotonic()-self.last_warning>5:
                self.get_logger().warning('Waiting for fresh synchronized arm/command/camera streams'); self.last_warning=time.monotonic()
            return
        target,chosen=result
        if target<=self.last_target: return
        messages={k:v[1] for k,v in chosen.items()}
        try:
            row={}
            for field,path in [('position','qpos'),('velocity','qvel'),('effort','effort')]:
                values=[getattr(messages[f'follower_{s}'],field) for s in ('left','right')]
                if any(len(v)!=7 or not np.isfinite(v).all() for v in values): raise ValueError(f'Invalid {field}')
                row[f'observations/{path}']=np.concatenate(values)
            for role,path in [('leader','action'),('follower_cmd','action_applied')]:
                values=[messages[f'{role}_{s}'].position for s in ('left','right')]
                if any(len(v)!=7 or not np.isfinite(v).all() for v in values): raise ValueError(f'Invalid {role}')
                row[path]=np.concatenate(values)
            row['observations/eef_pose']=np.array([getattr(messages[f'pose_{s}'],f) for s in ('left','right') for f in ('x','y','z','roll','pitch','yaw','gripper')])
            for name in self.names:
                image = messages[f'image_{name}']
                if isinstance(image, CompressedImage):
                    rgb = self.bridge.compressed_imgmsg_to_cv2(image, 'rgb8')
                else:
                    rgb = self.bridge.imgmsg_to_cv2(image, 'rgb8')
                row[f'observations/images/{name}'] = rgb
                if self.p('use_depth'):
                    image=messages[f'depth_{name}']
                    if image.encoding!='16UC1': raise ValueError('Depth must be 16UC1 millimeters')
                    row[f'observations/images_depth/{name}']=self.bridge.imgmsg_to_cv2(image,'passthrough')
            row['timestamps/frame']=target
            for key,(stamp,_) in chosen.items(): row[f'timestamps/{key}']=stamp
            # Build/validate the entire frame before appending; partial files are never auto-published.
            for path,value in row.items(): self.append(path,value)
            if 'collect' not in self.file: self.file.create_dataset('collect',shape=(0,),maxshape=(None,),dtype=h5py.string_dtype())
            self.file['collect'].resize(self.frames+1,axis=0); self.file['collect'][self.frames]='teleop'
            self.frames+=1; self.last_target=target
            if self.frames%30==0:
                self.file.attrs['complete_frames']=self.frames; self.file.flush()
        except Exception as exc:
            self.get_logger().error(f'Recording stopped; partial file retained: {exc}')
            self.file.attrs['error']=str(exc); self.file.close(); self.file=None

    def save(self,req,res):
        if self.file is None or self.frames==0: res.message='No complete frames; use discard for an empty episode'; return res
        self.file.attrs['complete_frames']=self.frames; self.file.flush(); self.file.close(); self.file=None
        final=self.path.with_name(self.path.name.replace('.partial',''))
        # link is atomic and refuses to overwrite an existing episode.
        os.link(self.path,final); self.path.unlink()
        res.success=True; res.message=f'Saved {self.frames} frames: {final}'; return res

    def discard(self,req,res):
        if self.file is None: res.message='No open episode'; return res
        self.file.close(); self.file=None; self.path.unlink()
        res.success=True; res.message='Discarded current episode'; return res

    def close(self):
        if self.file is not None:
            self.file.attrs['complete_frames']=self.frames; self.file.close(); self.file=None


def main(args=None):
    rclpy.init(args=args); node=None
    try: node=Recorder(); rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if node: node.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
