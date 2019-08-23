import mysql.connector
import datetime
import rclpy
from rclpy.node import Node
from nexus_interface.msg import Position


class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        self.client = mysql.connector.connect(
            host="localhost",
            user="vicon",
            passwd="lol",
            database="vicon"
        )
        self.sub_pos = self.create_subscription(
            Position, 'vicon', self.pos_callback, 10)
        self.date = str(datetime.datetime.utcnow()).replace(' ', '_')

    def check_table_exist(self, subject_name):
        cursor = self.client.cursor()
        sql = "CREATE TABLE IF NOT EXISTS" + "`[" + subject_name + "_" + self.date + "]`" + \
            " (id INT AUTO_INCREMENT PRIMARY KEY, x_trans FLOAT(25), y_trans FLOAT(25), z_trans FLOAT(25), x_rot FLOAT(25), y_rot FLOAT(25), z_rot FLOAT(25), w FLOAT(25), frame_number INT(255), segment_name VARCHAR(255));"
        cursor.execute(sql)

    def pos_callback(self, msg):
        self.check_table_exist(msg.subject_name)
        sql = "INSERT INTO " + "`[" + msg.subject_name + "_" + self.date + "]`" + \
            " (`x_trans`, `y_trans`, `z_trans`, `x_rot`, `y_rot`, `z_rot`, `w`, `frame_number`, `segment_name`) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)"
        val = (msg.x_trans,
               msg.y_trans,
               msg.z_trans,
               msg.x_rot,
               msg.y_rot,
               msg.z_rot,
               msg.w,
               msg.frame_number,
               msg.segment_name)
        cursor = self.client.cursor()
        cursor.execute(sql, val)
        self.client.commit()


def main(args=None):
    rclpy.init(args=args)

    node = Listener()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
