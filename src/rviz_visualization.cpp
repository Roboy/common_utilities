#include "common_utilities/rviz_visualization.hpp"

rviz_visualization::rviz_visualization() {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "TrackedObject",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    visualization_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 100);
}

void rviz_visualization::publishMesh(Vector3d &pos, Vector4d &orientation, const char *modelname,
                                     const char *frame, const char *ns, int message_id, int duration) {
    visualization_msgs::Marker mesh;
    mesh.header.frame_id = frame;
    mesh.ns = ns;
    mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh.color.r = 1.0f;
    mesh.color.g = 1.0f;
    mesh.color.b = 1.0f;
    mesh.color.a = 0.5;
    mesh.scale.x = 1.0;
    mesh.scale.y = 1.0;
    mesh.scale.z = 1.0;
    mesh.lifetime = ros::Duration(duration);
    mesh.header.stamp = ros::Time::now();
    mesh.action = visualization_msgs::Marker::ADD;
    mesh.id = message_id;
    mesh.pose.position.x = pos[0];
    mesh.pose.position.y = pos[1];
    mesh.pose.position.z = pos[2];
    mesh.pose.orientation.x = orientation[0];
    mesh.pose.orientation.y = orientation[1];
    mesh.pose.orientation.z = orientation[2];
    mesh.pose.orientation.w = orientation[3];
    char meshpath[200];
    sprintf(meshpath, "package://darkroom/calibrated_objects/models/%s.dae", modelname);
    mesh.mesh_resource = meshpath;
    visualization_pub.publish(mesh);
};

void rviz_visualization::publishSphere(Vector3d &pos, const char *frame, const char *ns, int message_id, COLOR color,
                                       float radius, int duration) {
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = frame;
    sphere.ns = ns;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.color.r = color.r;
    sphere.color.g = color.g;
    sphere.color.b = color.b;
    sphere.color.a = color.a;
    sphere.lifetime = ros::Duration(duration);
    sphere.scale.x = radius;
    sphere.scale.y = radius;
    sphere.scale.z = radius;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.header.stamp = ros::Time::now();
    sphere.id = message_id;
    sphere.pose.position.x = pos(0);
    sphere.pose.position.y = pos(1);
    sphere.pose.position.z = pos(2);
    sphere.pose.position.z = pos(2);
    sphere.pose.orientation.x = 1;
    sphere.pose.orientation.y = 0;
    sphere.pose.orientation.z = 0;
    sphere.pose.orientation.w = 0;
    visualization_pub.publish(sphere);
};

void rviz_visualization::publishCube(Vector3d &pos, Vector4d &quat, const char *frame, const char *ns, int message_id,
                                     COLOR color, float radius, int duration) {
    visualization_msgs::Marker cube;
    cube.header.frame_id = frame;
    cube.ns = ns;
    cube.type = visualization_msgs::Marker::CUBE;
    cube.color.r = color.r;
    cube.color.g = color.g;
    cube.color.b = color.b;
    cube.color.a = color.a;
    cube.lifetime = ros::Duration(duration);
    cube.scale.x = radius;
    cube.scale.y = radius;
    cube.scale.z = radius;
    cube.action = visualization_msgs::Marker::ADD;
    cube.header.stamp = ros::Time::now();
    cube.id = message_id;
    cube.pose.position.x = pos(0);
    cube.pose.position.y = pos(1);
    cube.pose.position.z = pos(2);
    cube.pose.orientation.x = quat(0);
    cube.pose.orientation.y = quat(1);
    cube.pose.orientation.z = quat(2);
    cube.pose.orientation.w = quat(3);
    visualization_pub.publish(cube);
};

void rviz_visualization::publishRay(Vector3d &pos, Vector3d &dir, const char *frame, const char *ns, int message_id,
                                    COLOR color, int duration) {
    visualization_msgs::Marker arrow;
    arrow.ns = ns;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.color.r = color.r;
    arrow.color.g = color.g;
    arrow.color.b = color.b;
    arrow.color.a = color.a;
    arrow.lifetime = ros::Duration(duration);
    arrow.scale.x = 0.005;
    arrow.scale.y = 0.03;
    arrow.scale.z = 0.03;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.header.stamp = ros::Time::now();

    arrow.header.frame_id = frame;
    arrow.id = message_id;
    arrow.points.clear();
    geometry_msgs::Point p;
    p.x = pos(0);
    p.y = pos(1);
    p.z = pos(2);
    arrow.points.push_back(p);
    p.x += dir(0);
    p.y += dir(1);
    p.z += dir(2);
    arrow.points.push_back(p);
    visualization_pub.publish(arrow);
};

void rviz_visualization::publishText(Vector3d &pos, const char *text, const char *frame, const char *ns, int message_id,
                                     COLOR color, int duration, float height) {
    visualization_msgs::Marker text_msg;
    text_msg.header.frame_id = frame;
    text_msg.ns = ns;
    text_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_msg.color.r = color.r;
    text_msg.color.g = color.g;
    text_msg.color.b = color.b;
    text_msg.color.a = color.a;
    text_msg.lifetime = ros::Duration(duration);
    text_msg.scale.z = height;
    text_msg.action = visualization_msgs::Marker::ADD;
    text_msg.header.stamp = ros::Time::now();
    text_msg.id = message_id;
    text_msg.pose.position.x = pos(0);
    text_msg.pose.position.y = pos(1);
    text_msg.pose.position.z = pos(2);
    text_msg.text = text;
    visualization_pub.publish(text_msg);
};
