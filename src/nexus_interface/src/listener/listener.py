import pymongo
import datetime
import rclpy
from rclpy.node import Node
from nexus_interface.msg import Position, RootSegment


class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        self.client = pymongo.MongoClient('localhost', 27017)
        self.db = self.client.vicon
        self.collection = self.db.translations
        self.sub_pos = self.create_subscription(
            Position, 'vicon', self.pos_callback, 10)
        self.sub_root = self.create_subscription(
            RootSegment, 'vicon', self.root_callback, 10)
        self.date = datetime.datetime.utcnow()
        

    def pos_callback(self, msg):
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

    def root_callback(self, msg):
        document = { "root_segment_name": msg.name,
                     "collection_name": msg.subject_name + "_" + str(self.date)}
        self.collection = self.db["root_segments"]
        self.collection.insert_one(document)

def main(args=None):
    rclpy.init(args=args)

    node = Listener()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
