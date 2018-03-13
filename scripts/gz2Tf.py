#!/usr/bin/env python
import rospy
import tf


from optparse import OptionParser
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import *
from tf2_msgs.msg import TFMessage

class StateListener():
  def __init__(self,fixed_frame="world",prefix="",whitelist=[],blacklist=[],interval=.2):
    self.br = tf.TransformBroadcaster()
    self.fixed_frame = fixed_frame
    self.prefix = prefix
    self.whitelist = whitelist
    self.blacklist = blacklist
    self.lasttime = rospy.Time()
    self.sleep_time = rospy.Duration(interval)
    self.tfpub = rospy.Publisher('/tf', TFMessage, queue_size=10)
    
  def check_time(self):
    now = rospy.Time.now()
    if self.sleep_time.to_sec() > now.to_sec():
      return False
    if (now-self.sleep_time > self.lasttime):
      self.lasttime = now
      return True
    return False
  
  def callback(self,modelStates):
      if not self.check_time():
          return
      i = 0 
      now = rospy.Time.now()
      for name in modelStates.name:
          p = modelStates.pose[i].position
          o = modelStates.pose[i].orientation
          i = i +1
          self.publish_pose(p,o,now,name)
          
          
  def wl_callback(self,modelStates):
      if not self.check_time():
          return
      i = 0 
      now = rospy.Time.now()
      for name in modelStates.name:
          if name in self.whitelist:
            p = modelStates.pose[i].position
            o = modelStates.pose[i].orientation
            self.publish_pose(p,o,now,name)
          i = i + 1
          
  def bl_callback(self,modelStates):
      if not self.check_time():
          return
      i = 0 
      now = rospy.Time.now()
      for name in modelStates.name:
          if name in self.blacklist:            
            i = i +1
            continue
          p = modelStates.pose[i].position
          o = modelStates.pose[i].orientation
          self.publish_pose(p,o,now,name)          
          i = i +1
    
  def callback2(self,modelStates):
      if not self.check_time():
          return
      i = 0 
      now = rospy.Time.now()
      tf_msg = TFMessage()
      
      for name in modelStates.name:
          p = modelStates.pose[i].position
          o = modelStates.pose[i].orientation
          i = i +1
          tf_msg.transforms.append(self.getTransform(p,o,now,name))   
          
      self.tfpub.publish(tf_msg)
      
  def getTransform(self,pos,orientation,time,name):
      t_st = TransformStamped()
      t_st.header.stamp = time
      t_st.header.frame_id = self.fixed_frame
      t_st.child_frame_id = name
      t_st.transform.translation = Vector3(pos.x,pos.y,pos.z)
      t_st.transform.rotation = Quaternion(orientation.x,orientation.y,orientation.z,orientation.w)
      return t_st

  def publish_pose(self,pos,orientation,time,name):
     self.br.sendTransform(
          (pos.x,pos.y,pos.z),
          (orientation.x,orientation.y,orientation.z,orientation.w),
          time,
          self.prefix + name,
          self.fixed_frame
        )

      
def listener():
    parser = OptionParser()
    parser.add_option("--whitelist", 
                      help="comma seperated list of models that should be parsed", dest="whitelist", 
                      metavar="LIST",
                      default=False)
    parser.add_option( "--blacklist", 
                      help="comma seperated list of models that should not be parsed. ignored if a whitelist is set",
                      dest="blacklist", 
                      metavar="LIST",
                      default=False)
    parser.add_option( "--fixed-frame", 
                      help="fixed_frame to be used",
                      dest="fixed_frame", 
                      metavar="FRAME",
                      default="world")
    parser.add_option( "--prefix", 
                      help="prefix to be used for all tf frames(Optional)",
                      dest="prefix", 
                      metavar="PREFIX",
                      default="")
    parser.add_option( "--interval", 
                      help="interval in which to publish tf-msgs in seconde(default 0.2)",
                      dest="rate", 
                      metavar="INTERVAL",
                      type="float",
                      default=0.2)

    (options, args) = parser.parse_args()
    
    rospy.init_node('gz_listener', anonymous=True)
    if options.whitelist:
      st = StateListener(options.fixed_frame,
                       options.prefix,
                       whitelist = options.whitelist.split(','),
                       interval=options.rate
                       )
      rospy.Subscriber("/gazebo/model_states", ModelStates, st.wl_callback,queue_size=1)
    elif options.blacklist:
      st = StateListener(options.fixed_frame,
                       options.prefix,
                       blacklist = options.blacklist.split(','),
                       interval=options.rate
                       )
      rospy.Subscriber("/gazebo/model_states", ModelStates, st.bl_callback,queue_size=1)
    else:
      st = StateListener(options.fixed_frame,
                       options.prefix,
                       interval=options.rate
                       )
      rospy.Subscriber("/gazebo/model_states", ModelStates, st.callback2,queue_size=1)
    #rate = rospy.Rate(100)
    #while not rospy.is_shutdown():
    #  rate.sleep()
    rospy.spin()

if __name__ == '__main__':
      listener()
