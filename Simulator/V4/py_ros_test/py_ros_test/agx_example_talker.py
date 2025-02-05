import agxROS2

def main():
    # Create Publisher and Subscriber with specific Topic.
    pub = agxROS2.PublisherStdMsgsFloat32("my_topic")
    sub = agxROS2.SubscriberStdMsgsFloat32("my_topic")

    # Create a message and send it.
    msgSend = agxROS2.StdMsgsFloat32()
    msgSend.data = 3.141592

    while True:
        pub.sendMessage(msgSend)
        print("Sent message")

    # Try to receive the message.


if __name__ == '__main__':
    main()
