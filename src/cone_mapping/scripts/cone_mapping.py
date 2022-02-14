#!/usr/bin/env python3

from inspect import signature
import rospy
import math
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, Pose2D, PoseArray, Pose
from etdv_messages.msg import ConePositionArrayStamped


class ConeMapper:
	def __init__(self):
		#            [Not used  , Start/Orange, Right/Yellow, Left/Blue]
		#self.cones = [PoseArray(), PoseArray(), PoseArray(), PoseArray()]
		self.position = PoseStamped()

		self.detected_cones = []
		self.distance_threshold = 2
		self.confidence_threshold = 2

		# Subscribers
		self.sub_pose = rospy.Subscriber(
			"/pose_stamped", PoseStamped, callback=self.pose_callback
		)
		self.sub_cone = rospy.Subscriber(
			"/sensor_fusion/detections_xyz", ConePositionArrayStamped, callback=self.cone_callback
		)

		# Publishers
		self.pub_right = rospy.Publisher(
			"/cone_right", PoseArray, latch=False, queue_size=1
		)
		self.pub_left = rospy.Publisher(
			"/cone_left", PoseArray, latch=False, queue_size=1
		)
		self.pub_start = rospy.Publisher(
			"/cone_orange", PoseArray, latch=False, queue_size=1
		)
		self.pub_unknown = rospy.Publisher(
			"/cone_unknown", PoseArray, latch=False, queue_size=1
		)

	def pose_callback(self, msg: PoseStamped):
		self.position = msg

	def get_closer_cone(self, cone_x, cone_y):
		'''
		Given a cone, find in self.cone_detected the closer one with respect to the euclidean distance
		'''
		min_distance = None
		min_index = None
		closer_cone = None

		# loop through the detected cones
		for idx, detected_cone in enumerate(self.detected_cones):
			# compute the euclidean distance w.r.t. the given cone
			distance = math.dist([cone_x, cone_y], [detected_cone['x'], detected_cone['y']])

			 # if the distance is below a threshold and it is less than the minimum distance detected until now
			if distance < self.distance_threshold and (
				min_distance is None or distance < min_distance
			):
				# save this detected cone as the closer one
				min_distance = distance
				min_index = idx
				closer_cone = detected_cone

		return closer_cone, min_index, min_distance


	def cone_callback(self, msg: ConePositionArrayStamped):
		axis2index = {"x": 0, "y": 1, "z": 2, "c": 3}
		noise_threshold = 20

		# for each detection
		for detection in msg.positions:

			x = detection.position.x
			y = detection.position.y
			#rospy.loginfo("Analyzing detection at {} ".format((x,y)))

			# filter the object if it's too distant
			if x <= 0 or x > noise_threshold or y == 0 or abs(y) > noise_threshold:
				rospy.logwarn("Object {} too distant: discarded.".format((x,y)))
				continue

			# compute cone_x and cone_y that represents the absolute detected position
			cone_x = self.position.pose.position.x + x
			cone_y = self.position.pose.position.y + y

			#rospy.loginfo("Summed to car position: {} ".format((cone_x,cone_y)))

			color = detection.type

			# compute the closer cone in the list of already detected cones
			closer_cone, cone_index, _ = self.get_closer_cone(cone_x, cone_y) # distance is never used

			# if there is no detected cones in the nearby of this cone, insert the new cone in the detected cones data structure
			if closer_cone is None:
				#rospy.loginfo("No close cones are found")
				self.detected_cones.append({'x': cone_x, 'y': cone_y, 'colors': [color], 'detections': [(cone_x, cone_y)]})
			# otherwise append the color detected this time to the closer cone
			else:
				#rospy.loginfo("Close cone detected: {}".format((closer_cone['x'], closer_cone['y'])))
				self.detected_cones[cone_index]['colors'].append(color)
				self.detected_cones[cone_index]['detections'].append((cone_x, cone_y))

	def get_cone_color(self, detected_cone):
		'''
		Given a detected cone represented as a dictionary:
		{x: cone_x, y: cone_y, colors: [array of detected colors]}
		return the mode of the colors arrays.
		If two colors have the same frequency in the array of cones, the last color detected is used
		'''
		colors = detected_cone["colors"]
		# create a set starting from the colors in the detected cones
		set_colors = set(colors)

		most_frequent_color = None
		most_frequent_color_number = None

		# for each color contained in the array find its frequency and see if it is the color with higher frequency
		for color in set_colors:
			if (
				most_frequent_color is None
				or colors.count(color) >= most_frequent_color_number
			):
				most_frequent_color = color
				most_frequent_color_number = colors.count(color)

		return most_frequent_color

	def get_detected_cones(self):
		'''
		Build the pose arrays of the detected cones with their color
		The color of a given detected cone is computed by the function get_cone_color
		'''
		orange_cones = PoseArray()
		yellow_cones = PoseArray()
		blue_cones = PoseArray()
		unknown_cones = PoseArray()

		# for each cone in the detected cones data structure
		for detected_cone in self.detected_cones:
			# if the cone has been seen less then confidence threshold just ignore it
			if len(detected_cone['detections']) < self.confidence_threshold:
				rospy.loginfo("Cone {} below confidence threshold".format((detected_cone['x'], detected_cone['y'])))
				continue
			
			# compute avg x and y
			avg_x = sum([x for x,_ in detected_cone['detections']]) / len(detected_cone['detections'])
			avg_y = sum([y for _,y in detected_cone['detections']]) / len(detected_cone['detections'])

			# assign detected cone x and y as the average x and y over the detections
			detected_cone['x'] = avg_x
			detected_cone['y'] = avg_y

			# find the most frequent color
			detected_color = self.get_cone_color(detected_cone)
			#rospy.loginfo("Cone [{}] most frequent color: {}".format((detected_cone['x'], detected_cone['y']), detected_color))

			# create the cone (Pose object)
			cone = Pose()
			cone.position.x = detected_cone['x']
			cone.position.y = detected_cone['y']

			# append the cone to the right PoseArray based on the detected color
			if detected_color == 0 or detected_color == 1:
				orange_cones.poses.append(cone)
			elif detected_color == 2:
				yellow_cones.poses.append(cone)
			elif detected_color == 3:
				blue_cones.poses.append(cone)
			else:
				unknown_cones.poses.append(cone)
			
			print("Cone [{}] most frequent color: {}\nDetections: {}\n".format((avg_x, avg_y), detected_color, detected_cone['detections']))

		return orange_cones, yellow_cones, blue_cones, unknown_cones


def main():
	rospy.init_node("cone_mapping")
	print("Start node cone_mapping")
	cone_mapper = ConeMapper()
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		orange_cones, yellow_cones, blue_cones, unknown_cones = cone_mapper.get_detected_cones()
		cone_mapper.pub_left.publish(blue_cones)
		cone_mapper.pub_right.publish(yellow_cones)
		cone_mapper.pub_start.publish(orange_cones)
		cone_mapper.pub_unknown.publish(unknown_cones)

		rate.sleep()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
