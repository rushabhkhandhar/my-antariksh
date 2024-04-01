import rospy
from geometry_msgs.msg import Quaternion

def callback(data):
    rospy.loginfo("Received Quaternion: x=%f, y=%f, z=%f, w=%f", data.x, data.y, data.z, data.w)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/chal_ja", Quaternion, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
    
