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
        self.date = datetime.datetime.utcnow()
        

    def chatter_callback(self, msg):
        document = { "x_trans" : msg.x_trans,
                     "y_trans" : msg.y_trans,
                     "z_trans" : msg.z_trans,
                     "x_rot" : msg.x_rot,
                     "y_rot" : msg.y_rot,
                     "z_rot" : msg.z_rot,
                     "w" : msg.w,
                     "frame_number" : msg.frame_number,
                     "segment_name" : msg.segment_name,
                     "translation_type" : msg.translation_type}
        self.collection = self.db[msg.subject_name + "_" + str(self.date)]
        self.collection.insert_one(document)

def main(args=None):
    rclpy.init(args=args)

    node = Listener()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
