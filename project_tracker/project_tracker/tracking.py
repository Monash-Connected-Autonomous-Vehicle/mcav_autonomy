from rclpy.node import Node

import logging

from collections import deque
import copy
import numpy as np
from scipy.optimize import linear_sum_assignment

from mcav_interfaces.msg import DetectedObjectArray

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

    def __init__(self, max_frames_before_forget, max_frames_length, tracking_method="centre_distance", 
                 dist_threshold=5, iou_threshold=0.8
        ):
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
        tracking_method : str
            One of "centre_distance" or "iou". 
            - "centre_distance" measures distances between centre points
                of the detected objects. 
            - "iou" looks at a birds eye view and measures IOU (intersection over union)
                between the two rectangles. Note, it moves the bounding boxes to axis-
                aligned by taking the top left point and the width and height of the rectangle
        """
        super(Tracker, self).__init__('tracker_dummy_node')
        
        self.tracking_method = tracking_method
        if tracking_method == "centre_distance":
            self.threshold = dist_threshold
        elif tracking_method == "iou":
            self.threshold = iou_threshold

        self.max_frames_before_forget = max_frames_before_forget
        self.max_frames_length = max_frames_length
        self.frames = deque(maxlen=max_frames_length)

        self.tracked_objects = []
        self.tracked_id_count = 0

        self.updated_centres = []

        self.get_logger().set_level(logging.INFO)

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
        except IndexError: # if only frames empty
            self.get_logger().debug("Frame empty, creating initial frame")
            for i, detected_object in enumerate(new_detects.detected_objects):
                detected_object.object_id = self.tracked_id_count
                new_track = TrackedObject(detected_object, self.tracked_id_count)
                self.tracked_id_count += 1
                self.tracked_objects.append(new_track)
            new_frame = copy.deepcopy(self.tracked_objects)
            self.frames.append(new_frame)
            tracked_object_array = self.create_detected_object_array()
            return tracked_object_array

        metrics = np.zeros(shape=(len(prev_detects), len(new_detects.detected_objects)))
        for i, old_detected_object in enumerate(prev_detects):
            old_pos = old_detected_object.pose.position
            old_dim = old_detected_object.dimensions
            new_bbs = []
            for j, new_detected_object in enumerate(new_detects.detected_objects):
                new_pos = new_detected_object.pose.position
                new_dim = new_detected_object.dimensions
                if self.tracking_method == "centre_distance":
                    metrics[i][j] = np.sqrt(
                        (old_pos.x - new_pos.x)**2 + (old_pos.y - new_pos.y)**2 + 
                        (old_pos.z - new_pos.z)**2
                    )
                else:
                    top_l_x = new_pos.x - 0.5 * new_dim.x
                    top_l_y = new_pos.y - 0.5 * new_dim.y
                    new_bbs.append([top_l_x, top_l_y, new_dim.x, new_dim.y])
            if self.tracking_method == "iou":
                new_bbs = np.asarray(new_bbs, dtype=np.float64)
                top_l_x = old_pos.x - 0.5 * old_dim.x
                top_l_y = old_pos.y - 0.5 * old_dim.y
                old_bb = np.array([top_l_x, top_l_y, old_dim.x, old_dim.y], dtype=np.float64)
                metrics[i] = 1.0  - self.iou(old_bb, new_bbs)

        # 2. Perform Hungarian Algorithm assignment
        row_ind, col_ind = linear_sum_assignment(cost_matrix=metrics)
        col_assignment = [-1 for _ in range(len(prev_detects))]
        for row, col in zip(row_ind, col_ind):
            col_assignment[row] = col

        self.get_logger().debug(f"Row inds and col inds: {row_ind, col_ind}")
        self.get_logger().debug(f"Column assignment: {col_assignment} with length {len(col_assignment)}")
        # see if any tracks are missing from the assignment
        for i in range(len(col_assignment)):
            if col_assignment[i] != -1: # if assigned
                # if assignment is above some distance threshold then 
                # un assign the tracked object
                if metrics[i][col_assignment[i]] > self.threshold:
                    col_assignment[i] = -1
                else:
                    # reset skipped frames to 0
                    self.tracked_objects[i].skipped_frames = 0
                    # update the detected object reference e.g. update centre coordinates
                    new_detects.detected_objects[col_assignment[i]].object_id = self.tracked_objects[i].object_id
                    self.tracked_objects[i].detected_object = new_detects.detected_objects[col_assignment[i]]
            else: # if not assigned, add a frame skipped
                self.tracked_objects[i].skipped_frames += 1
        
        # delete objects that shouldn't be tracked any more
        self.deleted_ids = [] # store for use in removing markers from RViz in cluster.py
        for i, tracked_object in enumerate(self.tracked_objects):
            if tracked_object.skipped_frames > self.max_frames_before_forget:
                deleted_object = self.tracked_objects.pop(i)
                self.deleted_ids.append(deleted_object.object_id)
                col_assignment.pop(i)
        
        # start tracking new items
        old_tracked_id_count = self.tracked_id_count
        for i, detected_object in enumerate(new_detects.detected_objects):
            if i not in col_assignment:
                detected_object.object_id = self.tracked_id_count
                new_track = TrackedObject(detected_object, self.tracked_id_count)
                self.tracked_id_count += 1
                self.tracked_objects.append(new_track)
        self.get_logger().debug(f"Added {self.tracked_id_count - old_tracked_id_count} objects to tracking.")

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

    def iou(self, bbox, candidates):
        """
        FROM: https://github.com/nwojke/deep_sort/blob/master/deep_sort/iou_matching.py

        Computer intersection over union.
        Parameters
        ----------
        bbox : ndarray
            A bounding box in format `(top left x, top left y, width, height)`.
        
        candidates : ndarray
            A matrix of candidate bounding boxes (one per row) in the same format
            as `bbox`.
        Returns
        -------
        ndarray
            The intersection over union in [0, 1] between the `bbox` and each
            candidate. A higher score means a larger fraction of the `bbox` is
            occluded by the candidate.
        """
        bbox_tl, bbox_br = bbox[:2], bbox[:2] + bbox[2:]
        candidates_tl = candidates[:, :2]
        candidates_br = candidates[:, :2] + candidates[:, 2:]

        tl = np.c_[np.maximum(bbox_tl[0], candidates_tl[:, 0])[:, np.newaxis],
                np.maximum(bbox_tl[1], candidates_tl[:, 1])[:, np.newaxis]]
        br = np.c_[np.minimum(bbox_br[0], candidates_br[:, 0])[:, np.newaxis],
                np.minimum(bbox_br[1], candidates_br[:, 1])[:, np.newaxis]]
        wh = np.maximum(0., br - tl)

        area_intersection = wh.prod(axis=1)
        area_bbox = bbox[2:].prod()
        area_candidates = candidates[:, 2:].prod(axis=1)
        return area_intersection / (area_bbox + area_candidates - area_intersection)

