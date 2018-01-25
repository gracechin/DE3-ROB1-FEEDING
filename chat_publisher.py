import rospy
from std_msgs.msg import String

# initialise the node
rospy.init_node("chat_publihser_node", anonymous=True)

# name of the topic used to chat
chat_topic = "chat_topic"

# create a publisher to send messages on the chat topic
publisher = rospy.Publisher(chat_topic, String, queue_size=100)

print("Publishing to " + chat_topic + "\n")

# main loop to publish messages
name = raw_input("please enter your name: ")
publisher.publish(name + " has joined the chat")
while not rospy.is_shutdown():
    message = name + ": " + raw_input("> ")
    publisher.publish(message)
