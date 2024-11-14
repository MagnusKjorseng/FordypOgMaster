import agxROS2
import agx

class UsvController:
    def __init__(self):
        super().__init__()
        self.qos = agxROS2.QOS()
        self.qos.historyDepth = 1  # We don't want to apply any old message data.
        self.ros_subscriber = agxROS2.SubscriberStdMsgsFloat32("usv_position", self.qos)
        self.position_received = agxROS2.StdMsgsFloat32()
        
        #self.ros_publisher = agxROS2.PublisherMsgsFloat32("usv_action_force", self.qos)
        
        #self.desired_position = agx.Vec3(10,10,0)
    
    def listen(self):
        print("Listening...")
        while True:
            if self.ros_subscriber.receiveMessage(self.position_received):
                print(self.position_received.data)

def main():
    controller = UsvController()
    controller.listen()
    
if __name__ == "__main__":
    main()