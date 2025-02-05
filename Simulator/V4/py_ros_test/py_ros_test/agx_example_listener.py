import agxROS2

def main():
    # Create Publisher and Subscriber with specific Topic.
    #pub = agxROS2.PublisherStdMsgsFloat32("my_topic")
    sub = agxROS2.SubscriberStdMsgsFloat32("my_topic")

    # Try to receive the message.
    msgReceive = agxROS2.StdMsgsFloat32()
    while True:
        if sub.receiveMessage(msgReceive):
            print("Received message: {}".format(msgReceive.data))
        else:
            print("Did not receive a message.")

if __name__ == '__main__':
    main()
