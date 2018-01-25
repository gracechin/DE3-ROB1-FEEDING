import rospy
from std_msgs.msg import String

# callback function used by the subscriber
def show_message(message):
    print message.data

# initialise the node
rospy.init_node("chat_subscriber_node", anonymous=True)

# name of the topic used to chat
chat_topic = "chat_topic"

# subscribe to the chat topic and attach the callback function
rospy.Subscriber(chat_topic, String, show_message)

print("Listening to " + chat_topic + "\n")

# loop forever
rospy.spin()
