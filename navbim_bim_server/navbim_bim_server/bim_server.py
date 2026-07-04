#!/usr/bin/env python3

"""
BIM Server - Provides IFC element information using ifcopenshell.

This lifecycle node loads an IFC file and provides a service to query
element information (type, name, origin) by GUID.
"""

import os
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from navbim_msgs.srv import GetIfcElementInfo, GetElementsByType
from geometry_msgs.msg import Point, Pose, Quaternion
import ifcopenshell


class BimServer(LifecycleNode):
    """Lifecycle node for BIM server providing IFC element information."""

    def __init__(self):
        super().__init__('bim_server')

        # Declare parameters
        self.declare_parameter('ifc_file', '')

        # Initialize variables
        self.ifc_file = None
        self.ifc_model = None
        self.element_cache = {}

        self.get_logger().info('BIM Server initialized')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure the BIM server - load IFC file."""
        self.get_logger().info('Configuring BIM Server...')

        # Get parameters
        ifc_file_path = self.get_parameter('ifc_file').get_parameter_value().string_value

        if not ifc_file_path:
            self.get_logger().error('No IFC file specified in ifc_file parameter')
            return TransitionCallbackReturn.FAILURE

        if not os.path.exists(ifc_file_path):
            self.get_logger().error(f'IFC file does not exist: {ifc_file_path}')
            return TransitionCallbackReturn.FAILURE

        # Load IFC file
        try:
            self.get_logger().info(f'Loading IFC file: {ifc_file_path}')
            self.ifc_model = ifcopenshell.open(ifc_file_path)
            self.ifc_file = ifc_file_path

            # Build element cache for faster lookups
            self._build_element_cache()

            self.get_logger().info(f'Successfully loaded IFC file with {len(self.element_cache)} elements')

        except Exception as e:
            self.get_logger().error(f'Failed to load IFC file: {str(e)}')
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info('BIM Server configured successfully')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate the BIM server - start service."""
        self.get_logger().info('Activating BIM Server...')

        # Create services
        self.element_info_service = self.create_service(
            GetIfcElementInfo,
            'bim_server/get_element_info',
            self.get_element_info_callback
        )

        self.elements_by_type_service = self.create_service(
            GetElementsByType,
            'bim_server/get_elements_by_type',
            self.get_elements_by_type_callback
        )

        self.get_logger().info('BIM Server activated and ready to serve element info')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate the BIM server - stop service."""
        self.get_logger().info('Deactivating BIM Server...')

        # Destroy services
        if hasattr(self, 'element_info_service'):
            self.destroy_service(self.element_info_service)
        if hasattr(self, 'elements_by_type_service'):
            self.destroy_service(self.elements_by_type_service)

        self.get_logger().info('BIM Server deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Cleanup the BIM server - unload IFC file."""
        self.get_logger().info('Cleaning up BIM Server...')

        # Clear cache and model
        self.element_cache.clear()
        self.ifc_model = None
        self.ifc_file = None

        self.get_logger().info('BIM Server cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shutdown the BIM server."""
        self.get_logger().info('Shutting down BIM Server...')

        # Clear cache and model
        self.element_cache.clear()
        self.ifc_model = None
        self.ifc_file = None

        self.get_logger().info('BIM Server shutdown complete')
        return TransitionCallbackReturn.SUCCESS

    def _build_element_cache(self):
        """Build a cache of elements by GUID for faster lookups."""
        if not self.ifc_model:
            return

        self.element_cache.clear()

        # Get all elements with a GlobalId
        for element in self.ifc_model.by_type('IfcElement'):
            if hasattr(element, 'GlobalId'):
                self.element_cache[element.GlobalId] = element

        # Also cache building elements
        for element in self.ifc_model.by_type('IfcBuildingElement'):
            if hasattr(element, 'GlobalId'):
                self.element_cache[element.GlobalId] = element

    def get_element_info_callback(self, request, response):
        """Service callback to get IFC element information by GUID."""
        guid = request.guid

        self.get_logger().info(f'Received request for element info: {guid}')

        # Check if model is loaded
        if not self.ifc_model or not self.element_cache:
            response.success = False
            response.message = 'IFC model not loaded'
            self.get_logger().warn('IFC model not loaded')
            return response

        # Try to find element by GUID
        element = self.element_cache.get(guid)

        if element is None:
            # Try direct lookup as fallback
            try:
                element = self.ifc_model.by_guid(guid)
            except Exception:
                pass

        if element is None:
            response.success = False
            response.message = f'Element with GUID {guid} not found in IFC model'
            self.get_logger().warn(f'Element {guid} not found')
            return response

        # Extract element information
        try:
            # Get element type
            response.element_type = element.is_a()

            # Get element name
            response.element_name = element.Name if hasattr(element, 'Name') and element.Name else 'Unnamed'

            # Get element origin/location
            response.pose = self._get_element_pose(element)

            response.success = True
            response.message = 'Element information retrieved successfully'

            self.get_logger().info(
                f'Element {guid}: type={response.element_type}, '
                f'name={response.element_name}, '
                f'pose=({response.pose.position.x:.2f}, {response.pose.position.y:.2f}, {response.pose.position.z:.2f})'
            )

        except Exception as e:
            response.success = False
            response.message = f'Error extracting element information: {str(e)}'
            self.get_logger().error(f'Error extracting info for {guid}: {str(e)}')

        return response

    def get_elements_by_type_callback(self, request, response):
        """Service callback to get all IFC elements of a specific type (building-wide)."""
        element_type = request.element_type

        self.get_logger().info(
            f'Received request for all elements of type: {element_type}'
        )

        # Check if model is loaded
        if not self.ifc_model or not self.element_cache:
            response.success = False
            response.message = 'IFC model not loaded'
            self.get_logger().warn('IFC model not loaded')
            return response

        try:
            # Get all elements of the specified type
            elements = self.ifc_model.by_type(element_type)

            if not elements:
                response.success = True
                response.message = f'No elements of type {element_type} found'
                self.get_logger().info(f'No elements of type {element_type} found')
                return response

            # Build response lists
            for element in elements:
                if not hasattr(element, 'GlobalId'):
                    continue

                # Get GUID
                response.guids.append(element.GlobalId)

                # Get name
                name = element.Name if hasattr(element, 'Name') and element.Name else 'Unnamed'
                response.names.append(name)

                # Get type
                response.types.append(element.is_a())

                # Get pose
                pose = self._get_element_pose(element)
                response.poses.append(pose)

            response.success = True
            response.message = f'Found {len(response.guids)} elements of type {element_type}'

            self.get_logger().info(
                f'Returning {len(response.guids)} elements of type {element_type}'
            )

        except Exception as e:
            response.success = False
            response.message = f'Error querying elements: {str(e)}'
            self.get_logger().error(f'Error querying elements of type {element_type}: {str(e)}')

        return response

    def _get_element_floor(self, element):
        """Get the floor/storey name that an element belongs to."""
        try:
            # Try to get ContainedInStructure relationship
            if hasattr(element, 'ContainedInStructure') and element.ContainedInStructure:
                for rel in element.ContainedInStructure:
                    if hasattr(rel, 'RelatingStructure'):
                        structure = rel.RelatingStructure
                        if structure.is_a('IfcBuildingStorey'):
                            return structure.Name if hasattr(structure, 'Name') else structure.LongName

            # Try to get Decomposes relationship (for spatial elements)
            if hasattr(element, 'Decomposes') and element.Decomposes:
                for rel in element.Decomposes:
                    if hasattr(rel, 'RelatingObject'):
                        obj = rel.RelatingObject
                        if obj.is_a('IfcBuildingStorey'):
                            return obj.Name if hasattr(obj, 'Name') else obj.LongName

        except Exception as e:
            self.get_logger().debug(f'Could not determine floor for element: {str(e)}')

        return None

    def _get_element_origin(self, element):
        """Extract the origin/location of an IFC element."""
        origin = Point()
        origin.x = 0.0
        origin.y = 0.0
        origin.z = 0.0

        try:
            # Try to get ObjectPlacement
            if hasattr(element, 'ObjectPlacement') and element.ObjectPlacement:
                placement = element.ObjectPlacement

                # Get relative placement
                if hasattr(placement, 'RelativePlacement') and placement.RelativePlacement:
                    rel_placement = placement.RelativePlacement

                    # Get location
                    if hasattr(rel_placement, 'Location') and rel_placement.Location:
                        location = rel_placement.Location

                        if hasattr(location, 'Coordinates') and location.Coordinates:
                            coords = location.Coordinates
                            if len(coords) >= 2:
                                origin.x = float(coords[0])
                                origin.y = float(coords[1])
                            if len(coords) >= 3:
                                origin.z = float(coords[2])

        except Exception as e:
            self.get_logger().debug(f'Could not extract origin for element: {str(e)}')

        return origin

    def _get_element_pose(self, element):
        """Extract the full pose (position + orientation) of an IFC element."""
        pose = Pose()
        pose.position = self._get_element_origin(element)
        pose.orientation = self._get_element_orientation(element)
        return pose

    def _get_element_orientation(self, element):
        """Extract the orientation of an IFC element as a quaternion."""
        # Initialize as identity quaternion
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        try:
            if hasattr(element, 'ObjectPlacement') and element.ObjectPlacement:
                placement = element.ObjectPlacement

                # Get relative placement for orientation
                if hasattr(placement, 'RelativePlacement') and placement.RelativePlacement:
                    rel_placement = placement.RelativePlacement

                    # Extract axis directions if available
                    x_axis = np.array([1.0, 0.0, 0.0])  # Default
                    y_axis = np.array([0.0, 1.0, 0.0])  # Default
                    z_axis = np.array([0.0, 0.0, 1.0])  # Default

                    # Get Z axis (Axis in IFC)
                    if hasattr(rel_placement, 'Axis') and rel_placement.Axis:
                        if hasattr(rel_placement.Axis, 'DirectionRatios'):
                            z_axis = np.array(rel_placement.Axis.DirectionRatios)
                            z_axis = z_axis / np.linalg.norm(z_axis)

                    # Get X axis (RefDirection in IFC)
                    if hasattr(rel_placement, 'RefDirection') and rel_placement.RefDirection:
                        if hasattr(rel_placement.RefDirection, 'DirectionRatios'):
                            x_axis = np.array(rel_placement.RefDirection.DirectionRatios)
                            x_axis = x_axis / np.linalg.norm(x_axis)

                            # Compute Y axis as cross product to ensure orthogonality
                            y_axis = np.cross(z_axis, x_axis)
                            if np.linalg.norm(y_axis) > 1e-6:
                                y_axis = y_axis / np.linalg.norm(y_axis)
                            else:
                                y_axis = np.array([0.0, 1.0, 0.0])

                    # Build rotation matrix (column vectors)
                    rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])

                    # Convert to quaternion using scipy
                    rot = Rotation.from_matrix(rotation_matrix)
                    quat = rot.as_quat()  # Returns [x, y, z, w]

                    orientation.x = quat[0]
                    orientation.y = quat[1]
                    orientation.z = quat[2]
                    orientation.w = quat[3]

        except Exception as e:
            self.get_logger().debug(f'Could not extract orientation for element: {str(e)}')

        return orientation


def main(args=None):
    """Main entry point for the BIM server node."""
    rclpy.init(args=args)

    bim_server = BimServer()

    try:
        rclpy.spin(bim_server)
    except KeyboardInterrupt:
        pass
    finally:
        bim_server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
