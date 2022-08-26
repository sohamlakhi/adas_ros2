import rospy
from geometry_msgs.msg import Point

def get_bounding_box(car_id):
    return rospy.get_param('/car/{}/bounding_box'.format(car_id))

class CarGeometry:
    def __init__(self, front_length, back_length, width):
        self.front_length = front_length
        self.back_length = back_length
        self.width = width

    def __repr__(self):
        return '(Front Length: {}, Back Length: {}, Width: {})'.format(
            self.front_length,
            self.back_length,
            self.width)

    def total_length(self):
        return self.front_length + self.back_length

def get_geometry_to_center_of_turn(car_id):
    tag_to_car_center, _, length, width = get_bounding_box(car_id)
    rear_axle_from_tag = rospy.get_param('/car/{}/rear_axle_from_tag'.format(car_id))
    front_length = (tag_to_car_center - rear_axle_from_tag) + length / 2
    back_length = length - front_length
    return CarGeometry(front_length, back_length, width)

def get_rect_to_geometric_center(car_id):
    offset_x, offset_y, size_x, size_y = rospy.get_param('/car/{}/bounding_box'.format(car_id))
    return [
        Point(-size_x / 2, size_y / 2, 0),
        Point(-size_x / 2, -size_y / 2, 0),
        Point(size_x / 2, -size_y / 2, 0),
        Point(size_x / 2, size_y / 2, 0)]

def get_rect_to_tag_center(car_id):
    offset_x, offset_y, size_x, size_y = rospy.get_param('/car/{}/bounding_box'.format(car_id))
    return [
        Point(offset_x - size_x / 2, offset_y + size_y / 2, 0),
        Point(offset_x - size_x / 2, offset_y - size_y / 2, 0),
        Point(offset_x + size_x / 2, offset_y - size_y / 2, 0),
        Point(offset_x + size_x / 2, offset_y + size_y / 2, 0)]
