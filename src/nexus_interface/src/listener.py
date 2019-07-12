import pymongo
import datetime
import rclpy
from rclpy.node import Node
from nexus_interface.msg import Position


class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        self.client = pymongo.MongoClient('localhost', 27017)
        self.db = self.client.vicon
        self.collection = self.db.translations
        self.sub = self.create_subscription(
            Position, 'vicon', self.chatter_callback, 10)

    def chatter_callback(self, msg):
        document = { "x" : msg.x,
                     "y" : msg.y,
                     "z" : msg.z,
                     "frame_number" : msg.frame_number,
                     "segment_name" : msg.segment_name,
                     "trans_type" : msg.trans_type}
        self.collection = self.db[msg.subject_name + "_" + datetime.datetime.utcnow()]
        self.collection.insert_one(document)

def main(args=None):
    rclpy.init(args=args)

    node = Listener()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
