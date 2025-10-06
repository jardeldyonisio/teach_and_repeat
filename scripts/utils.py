#!/usr/bin/env python3
#coding: utf-8

import bezier
import numpy as np

from BezierFitDemo import BezierFitDemo
import os

class OrientedPoint:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

def pointToPoseStamped(x: float, 
                       y: float, 
                       yaw: float,
                       frame_id: str = "map"):
    '''
    @brief Convert x, y coordinates to a PoseStamped message.

    @param x (float): X coordinate.
    @param y (float): Y coordinate.
    @param frame_id (str): Frame of reference for the pose.

    @return geometry_msgs.msg.PoseStamped: The corresponding PoseStamped message.
    '''
    from geometry_msgs.msg import PoseStamped
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = float(yaw)
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 1.0

    return pose

def readPointsFromFile(file_path: str):
    '''
    @brief Reads 2D points from a file.

    @param file_path (str): Path to the file containing points in "x,y" format.

    @return numpy.ndarray: Array of points read from the file.
    '''

    file_points = np.empty((0, 2))

    with open(file_path, 'r') as f:
        for line in f:
            x, y = line.strip().split(',')
            point = np.array([[float(x), float(y)]])
            file_points = np.append(file_points, point, axis=0)
    return file_points

def readOrientedPointsFromFile(file_path: str):
    '''
    @brief Reads 2D points from a file.

    @param file_path (str): Path to the file containing points in "x,y, yaw" format.

    @return numpy.ndarray: Array of points read from the file.
    '''

    file_points = np.empty((0, 3))

    with open(file_path, 'r') as f:
        for line in f:
            x, y, yaw = line.strip().split(',')
            point = np.array([[float(x), float(y), float(yaw)]])
            file_points = np.append(file_points, point, axis=0)
    return file_points

def saveOrientedCoordsToFile(file_path: str, path_coords):
    '''
    @brief Saves oriented coordinates to a file.

    @param file_path (str): Path to the file where coordinates will be saved.
    @param path_coords (list): List of OrientedPoint objects to be saved.
    @return None
    '''
    
    directory = os.path.dirname(file_path)

    if not os.path.exists(directory):
        os.makedirs(directory)

    with open(file_path, 'w') as file:
        for point in path_coords:
            # Converte np.array para ponto se necessário
            if isinstance(point, np.ndarray):
                point = OrientedPoint(point[0], point[1], point[2])
            file.write('{},{},{}\n'.format(point.x, point.y, point.yaw))

def generateBezierCurve(ctrl_pts, 
                        step_size=0.001, 
                        max = 1.0, 
                        min = 0.0):
    '''
    Generate a Bezier curve from control points.

    @param ctrl_pts (numpy.ndarray): Control points for the Bezier curve.
    @param step_size (float): Step size for generating points along the curve.
    @param max (float): Maximum value for the parameter t.
    @param min (float): Minimum value for the parameter t.

    @return numpy.ndarray: Points along the generated Bezier curve.
    '''

    # TODO: Use step_size, max, min variables to customize point generation

    curve = bezier.Curve.from_nodes(ctrl_pts)

    # Cria pontos ao longo da curva com o espaçamento desejado
    vals = np.linspace(0.0, 1.0, num=1000)
    points = curve.evaluate_multi(vals).T
    return points

def filePathPointsToOrientedPoses(file_path: str, 
                                frame_id: str = "map"):
    '''
    @brief Reads oriented points from a file and converts them to PoseStamped messages.

    @param file_path (str): Path to the file containing control points in "x,y, yaw" format.
    @param frame_id (str): Frame of reference for the poses.

    @return list: List of PoseStamped messages representing the sampled points on the Bezier curve.
    '''

    # Read raw path coordinates from file
    path_points = readOrientedPointsFromFile(file_path)
    
    # Convert sampled points to PoseStamped messages
    oriented_poses = [pointToPoseStamped(x, y, yaw, frame_id) for x, y, yaw in path_points]
    
    return oriented_poses


def filePathPointsToBezierPoses(file_path: str, 
                                dist_btw_points: float = 0.1, 
                                frame_id: str = "map"):
    '''
    @brief Reads points from a file, generates a Bezier curve, samples points at specified intervals, 
    and converts them to PoseStamped messages.

    @param file_path (str): Path to the file containing control points in "x,y" format.
    @param dist_btw_points (float): Distance between consecutive points on the Bezier curve.
    @param frame_id (str): Frame of reference for the poses.

    @return list: List of PoseStamped messages representing the sampled points on the Bezier curve.
    '''

    # Read raw path coordinates from file
    path_points = readPointsFromFile(file_path)

    # Generate Bezier curve control points using BezierFitDemo
    ctrl_points = BezierFitDemo(path_points)
    ctrl_points_transpose = ctrl_points.T
    
    if path_points.shape[0] < 2:
        raise ValueError("At least two control points are required to generate a Bezier curve.")
    
    # # Generate Bezier curve from control points
    bezier_curve = generateBezierCurve(ctrl_points_transpose)
    
    # # Sample points along the Bezier curve at specified intervals
    sampled_points = bezierPointsDistance(bezier_curve, dist_btw_points)
    
    # Convert sampled points to PoseStamped messages
    bezier_poses = [pointToPoseStamped(x, y, frame_id) for x, y in sampled_points]
    
    return bezier_poses

def bezierPointsDistance(bezier_curve: np.ndarray, 
                        points_distance: float) -> np.ndarray:
    '''
    @brief Extracts points from a Bezier curve at specified intervals.

    @param bezier_curve (numpy.ndarray): The Bezier curve points.
    @param points_distance (float): The distance between consecutive points.

    @return numpy.ndarray: Points extracted at the specified intervals.
    '''
    interval_distance = points_distance
    num_points = bezier_curve.shape[0]
    points_at_interval = [bezier_curve[0]]

    total_distance = 0.0
    for i in range(1, num_points):
        segment_distance = np.linalg.norm(bezier_curve[i] - bezier_curve[i - 1])
        total_distance += segment_distance

        while total_distance >= interval_distance:
            overshoot = total_distance - interval_distance
            ratio = 1.0 - (overshoot / segment_distance)
            intermediate_point = bezier_curve[i - 1] + ratio * (bezier_curve[i] - bezier_curve[i - 1])
            points_at_interval.append(intermediate_point)
            total_distance -= interval_distance

    return np.array(points_at_interval)