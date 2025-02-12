import agxROS2
import time

def main():
    # Create Publisher and Subscriber with specific Topic.
    #pub = agxROS2.PublisherStdMsgsFloat32("my_topic")
    sub = agxROS2.SubscriberGeometryMsgsVector3("usv_desired_force_on_cog")

    # Try to receive the message.
    msgReceive = agxROS2.GeometryMsgsVector3()
    while True:
        if sub.receiveMessage(msgReceive):
            print("Received message: x:{} y:{} z:{}".format(msgReceive.x, msgReceive.y, msgReceive.z))
        else:
            print("Did not receive a message.")
        time.sleep(0.5)

if __name__ == '__main__':
    main()
