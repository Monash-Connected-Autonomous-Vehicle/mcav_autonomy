from rclpy.node import Node

from collections import deque
import copy
from hashlib import new
import numpy as np
from scipy.optimize import linear_sum_assignment

from mcav_interfaces.msg import DetectedObjectArray


"""
TODO
* Use TrackedObject to keep track of which objects are still in the frame
* Base largely off the github code continue to develop it out
"""

class TrackedObject():

    def __init__(self, detected_object, object_id):
        """
        Defintion for frame to track object through time

        Parameters
        ----------
        detected_object : DetectedObject
            DetectedObject to track over time
        object_id : int
            Index to give to tracked object
        """
        self.detected_object = detected_object
        self.detected_object.object_id = object_id
        self.object_id = object_id
        self.centre = (detected_object.pose.position.x, detected_object.pose.position.y, detected_object.pose.position.z)
        self.skipped_frames = 0 # number of frames it has been skipped because undetected


class Tracker(Node):

    def __init__(self, dist_threshold, max_frames_before_forget, max_frames_length):
        """
        Class to track objects through frames using Hungarian Algorithm.

        Parameters
        ----------
        dist_threshold : float
            Threshold before unassigning the tracking between frames.
        max_frames_before_forget : int
            Number of frames to iterate over without seeing DetectedObject before forgetting it
        max_frames_length : int
            Maximum memory back in time through frames
        """
        super(Tracker, self).__init__('tracker_dummy_node')

        self.dist_threshold = dist_threshold
        self.max_frames_before_forget = max_frames_before_forget

        self.max_frames_length = max_frames_length
        self.frames = deque(maxlen=max_frames_length)

        self.tracked_objects = []
        self.tracked_id_count = 0

        self.updated_centres = []

    def update(self, new_detects):
        """
        Callback to update the tracked DetectedObject's when receiving a new frame.

        Works by:
        1. Calculating cost matrix for distance between DetectedObject's previous 2 frames
        2. Using scipy.optimize.linear_sum_assignment (Hungarian Algorithm) to assign new
            DetectedObject's to previously identified DetectedObject's
        3. 
        4. 

        Parameters
        ----------
        new_frame : DetectedObjectArray
            Frame representing the most recent DetectedObjectArray 
        Returns
        -------
        DetectedObjectArray

        """
        # 1. Create cost matrix
        try:
            prev_detects = [obj.detected_object for obj in self.frames[-1]]
        except IndexError: # if only 1 frame
            for i, detected_object in enumerate(new_detects.detected_objects):
                detected_object.object_id = self.tracked_id_count
                new_track = TrackedObject(detected_object, self.tracked_id_count)
                self.tracked_id_count += 1
                self.tracked_objects.append(new_track)
            new_frame = copy.deepcopy(self.tracked_objects)
            self.frames.append(new_frame)
            tracked_object_array = self.create_detected_object_array()
            return tracked_object_array

        distances = np.zeros(shape=(len(prev_detects), len(new_detects.detected_objects)))
        for i, old_detected_object in enumerate(prev_detects):
            old_pos = old_detected_object.pose.position
            for j, new_detected_object in enumerate(new_detects.detected_objects):
                new_pos = new_detected_object.pose.position
                distances[i][j] = np.sqrt(
                    (old_pos.x - new_pos.x)**2 + (old_pos.y - new_pos.y)**2 + 
                    (old_pos.z - new_pos.z)**2
                )

        # 2. Perform Hungarian Algorithm assignment
        row_ind, col_ind = linear_sum_assignment(cost_matrix=distances)
        col_assignment = [-1 for _ in range(len(prev_detects))]
        for row, col in zip(row_ind, col_ind):
            col_assignment[row] = col

        self.distances = distances
        self.row_ind = row_ind
        self.col_ind = col_ind
        self.get_logger().info(f"Row inds and col inds: {row_ind, col_ind}")
        self.get_logger().info(f"Column assignment: {col_assignment} with length {len(col_assignment)}")
        # see if any tracks are missing from the assignment
        for i in range(len(col_assignment)):
            if col_assignment[i] != -1: # if assigned
                # if assignment is above some distance threshold then 
                # un assign the tracked object
                if distances[i][col_assignment[i]] > self.dist_threshold:
                    col_assignment[i] = -1
                else:
                    self.tracked_objects[i].skipped_frames = 0
            else: # if not assigned, add a frame skipped
                self.tracked_objects[i].skipped_frames += 1
        
        # delete objects that shouldn't be tracked any more
        for i, tracked_object in enumerate(self.tracked_objects):
            if tracked_object.skipped_frames > self.max_frames_before_forget:
                self.tracked_objects.pop(i)
                col_assignment.pop(i)
        
        # start tracking new items
        for i, detected_object in enumerate(new_detects.detected_objects):
            if i not in col_assignment:
                detected_object.object_id = self.tracked_id_count
                new_track = TrackedObject(detected_object, self.tracked_id_count)
                self.tracked_id_count += 1
                self.tracked_objects.append(new_track)

        # update centre coordinates for detected objects that are tracked
        self.updated_centres = []
        for tracked_object in self.tracked_objects:
            for detected_object in new_detects.detected_objects:
                if tracked_object.object_id == detected_object.object_id:
                    old = tracked_object.detected_object.pose.position
                    new = detected_object.pose.position
                    self.updated_centres.append([old.x, old.y, old.z, new.x, new.y, new.z])
                    tracked_object.detected_object = detected_object

        # store previous frames for use in comparison
        tracked_copy = copy.deepcopy(self.tracked_objects)
        self.frames.append(tracked_copy)

        tracked_object_array = self.create_detected_object_array()

        return tracked_object_array

    def create_detected_object_array(self):
        """Create DetectedObjectArray from tracked objects"""
        # create DetectedObjectArray for sending to ROS topics
        tracked_object_array = DetectedObjectArray()
        tracked_object_array.detected_objects = [obj.detected_object for obj in self.tracked_objects]
        return tracked_object_array
