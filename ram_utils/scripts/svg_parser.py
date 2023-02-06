#!/usr/bin/env python
from ram_utils.srv import ParseSvgFile
from svgpathtools import svg2paths, wsvg, Line, Arc, Path
from xml.parsers.expat import ExpatError
import geometry_msgs.msg
import numpy
import ram_msgs.msg
import rospy
import unique_id
import uuid_msgs.msg


def parse_svg_file(req):
    if req.arc_points is 0:
        return [0, "Arc points must be > 0", ram_msgs.msg.AdditiveManufacturingTrajectory()]

    try:
        paths, attributes = svg2paths(req.file_name,
                                      False,
                                      True,
                                      True)
    except IOError:
        return [0, "Cannot read SVG file.", ram_msgs.msg.AdditiveManufacturingTrajectory()]

    except ExpatError:
        return [0, "ExpatError, invalid XML, error in the SVG file", ram_msgs.msg.AdditiveManufacturingTrajectory()]

    if len(paths) is 0:
        return [0, "No paths in the SVG file.", ram_msgs.msg.AdditiveManufacturingTrajectory()]

    if len(paths[0]) is 0:
        return [0, "Each path must contain at least one object.", ram_msgs.msg.AdditiveManufacturingTrajectory()]

    if req.closed_path and not paths[0].isclosed():
        return [0, "The path must be closed", ram_msgs.msg.AdditiveManufacturingTrajectory()]

    traj = ram_msgs.msg.AdditiveManufacturingTrajectory()
    traj.generated = rospy.Time.now()
    traj.modified = traj.generated
    traj.file = req.file_name
    traj.generation_info = "From SVG file"

    # Add first pose of the trajectory
    traj.poses.append(ram_msgs.msg.AdditiveManufacturingPose())
    traj.poses[0].layer_index = 0
    traj.poses[0].layer_level = 0
    traj.poses[0].polygon_start = False
    traj.poses[0].polygon_end = False
    traj.poses[0].entry_pose = False
    traj.poses[0].exit_pose = False
    traj.poses[0].pose.orientation.w = 1
    traj.poses[0].unique_id = unique_id.toMsg(unique_id.fromRandom())

    # https://github.com/mathandy/svgpathtools/issues/61#issuecomment-411936817
    if type(paths[0][0]) is Arc:
        pts = [paths[0][0].point(t) for t in numpy.linspace(0, 1, req.arc_points)]
        converted_path = Path(*[Line(pts[i - 1], pts[i]) for i in range(1, len(pts))])
        # Take the first point of the arc
        traj.poses[0].pose.position.x = converted_path[0][0].real / 1000.0
        traj.poses[0].pose.position.y = converted_path[0][0].imag / 1000.0
    elif type(paths[0][0] is Line):
        traj.poses[0].pose.position.x = paths[0][0][0].real / 1000.0
        traj.poses[0].pose.position.y = paths[0][0][0].imag / 1000.0
    else:
        return [0, "The first SVG path is neither an Arc nor a Line.", ram_msgs.msg.AdditiveManufacturingTrajectory()]

    for a, b in enumerate(paths):
        for c, d in enumerate(b):
            if type(d) is Arc:
                # Convert arc into multiple points (discretize)
                pts = [d.point(t) for t in numpy.linspace(0, 1, req.arc_points)]
                converted_path = Path(*[Line(pts[i - 1], pts[i]) for i in range(1, len(pts))])
                for e, f in enumerate(converted_path):
                    if type(f) is not Line:
                        return [0, "Each object in the path must be a line or an arc. Type = " + type(f).__name__,
                                ram_msgs.msg.AdditiveManufacturingTrajectory()]

                    if len(f) is not 2:
                        return [0, "Each line of the path must contain 2 points exactly.",
                                ram_msgs.msg.AdditiveManufacturingTrajectory()]

                    traj.poses.append(ram_msgs.msg.AdditiveManufacturingPose())
                    traj.poses[-1].pose.position.x = f[1].real / 1000.0
                    traj.poses[-1].pose.position.y = f[1].imag / 1000.0
                    traj.poses[-1].unique_id = unique_id.toMsg(unique_id.fromRandom())
                continue

            if type(d) is not Line:
                return [0, "Each object in the path must be a line or an arc. Type = " + type(d).__name__,
                        ram_msgs.msg.AdditiveManufacturingTrajectory()]

            if len(d) is not 2:
                return [0, "Each line of the path must contain 2 points exactly.",
                        ram_msgs.msg.AdditiveManufacturingTrajectory()]

            traj.poses.append(ram_msgs.msg.AdditiveManufacturingPose())
            traj.poses[-1].pose.position.x = d[1].real / 1000.0
            traj.poses[-1].pose.position.y = d[1].imag / 1000.0
            traj.poses[-1].unique_id = unique_id.toMsg(unique_id.fromRandom())
            # Polygon start and end management if a SVG file contains several polygons
            if len(paths) > 1:
                # Ignore the fist pose of the file, because it is managed above
                if a == 0 and c == 0:
                    continue
                # If a polygon contains only one pose
                if len(b) == 1:
                    traj.poses[-1].polygon_start = True
                    traj.poses[-1].polygon_end = True
                else:
                    # For other polygons
                    if c == 0:
                        # First pose of the Nth polygon
                        traj.poses[-1].polygon_start = True
                        traj.poses[-1].polygon_end = False
                    elif c == len(b) - 1:
                        # Last pose of the Nth polygon
                        traj.poses[-1].polygon_start = False
                        traj.poses[-1].polygon_end = True

    if len(traj.poses) is 0:
        return [0, "Trajectory has zero poses", ram_msgs.msg.AdditiveManufacturingTrajectory()]

    # Tweak polygon_start and polygon_end
    traj.poses[0].polygon_start = True
    traj.poses[-1].polygon_end = True

    return [len(paths), "", traj]


def services_servers():
    rospy.init_node('ram_svg_parser')
    service = rospy.Service('ram_utils/parse_svg_file', ParseSvgFile, parse_svg_file)
    rospy.spin()


if __name__ == '__main__':
    services_servers()
