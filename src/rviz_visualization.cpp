#include "common_utilities/rviz_visualization.hpp"

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> rviz_visualization::interactive_marker_server;
interactive_markers::MenuHandler rviz_visualization::menu_handler;
bool rviz_visualization::first = true;

rviz_visualization::rviz_visualization() {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "rviz_visualization",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    visualization_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 1);

    if(first){
        first = false;
        interactive_marker_server = boost::shared_ptr<interactive_markers::InteractiveMarkerServer>(new interactive_markers::InteractiveMarkerServer("interactive_markers")) ;
        menu_handler.insert( "First Entry", &processFeedback );
        menu_handler.insert( "Second Entry", &processFeedback );
        interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Submenu" );
        menu_handler.insert( sub_menu_handle, "First Entry", &processFeedback );
        menu_handler.insert( sub_menu_handle, "Second Entry", &processFeedback );
    }

}

rviz_visualization::~rviz_visualization() {
    interactive_marker_server->clear();
    interactive_marker_server->applyChanges();
}

void rviz_visualization::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

    std::ostringstream mouse_point_ss;
    if( feedback->mouse_point_valid )
    {
        mouse_point_ss << " at " << feedback->mouse_point.x
                       << ", " << feedback->mouse_point.y
                       << ", " << feedback->mouse_point.z
                       << " in frame " << feedback->header.frame_id;
    }

    switch ( feedback->event_type )
    {
        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
            ROS_DEBUG_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
            ROS_DEBUG_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
            break;

        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            ROS_DEBUG_STREAM( s.str() << ": pose changed"
                                      << "\nposition = "
                                      << feedback->pose.position.x
                                      << ", " << feedback->pose.position.y
                                      << ", " << feedback->pose.position.z
                                      << "\norientation = "
                                      << feedback->pose.orientation.w
                                      << ", " << feedback->pose.orientation.x
                                      << ", " << feedback->pose.orientation.y
                                      << ", " << feedback->pose.orientation.z
                                      << "\nframe: " << feedback->header.frame_id
                                      << " time: " << feedback->header.stamp.sec << "sec, "
                                      << feedback->header.stamp.nsec << " nsec" );
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
            ROS_DEBUG_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
            ROS_DEBUG_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
            break;
    }

    interactive_marker_server->applyChanges();
}

Marker rviz_visualization::makeBox( InteractiveMarker &msg )
{
    Marker marker;

    marker.type = Marker::CUBE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    return marker;
}

InteractiveMarkerControl& rviz_visualization::makeBoxControl( InteractiveMarker &msg )
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeBox(msg) );
    msg.controls.push_back( control );

    return msg.controls.back();
}

void rviz_visualization::make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position,
                                         bool show_6dof, double scale, const char *frame, const char *name,
                                         const char *description )
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = frame;
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.pose.orientation.w = 1;
    int_marker.pose.orientation.x = 0;
    int_marker.pose.orientation.y = 0;
    int_marker.pose.orientation.z = 0;
    int_marker.scale = scale;

    int_marker.name = name;
    int_marker.description = description;

    // insert a box
    makeBoxControl(int_marker);

    if(interaction_mode==InteractiveMarkerControl::MOVE_PLANE){
        int_marker.controls[0].orientation.w = 0;
        int_marker.controls[0].orientation.x = 0;
        int_marker.controls[0].orientation.y = 1;
        int_marker.controls[0].orientation.z = 0;
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    }

    InteractiveMarkerControl control;
    int_marker.controls[0].interaction_mode = interaction_mode;

    if ( fixed )
    {
        int_marker.name += "_fixed";
        int_marker.description += "\n(fixed orientation)";
        control.orientation_mode = InteractiveMarkerControl::FIXED;
    }

    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    {
        std::string mode_text;
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
//        int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
        int_marker.description = name;
    }

    if(show_6dof)
    {
        control.orientation.w = 0;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 0;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 0;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_y";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }

    interactive_marker_server->insert(int_marker);
    interactive_marker_server->setCallback(int_marker.name, &processFeedback);
    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
        menu_handler.apply( *interactive_marker_server, int_marker.name );
    interactive_marker_server->applyChanges();
}

geometry_msgs::Vector3 rviz_visualization::convertEigenToGeometry(const Vector3d &vector_in){
    geometry_msgs::Vector3 vector_out;
    vector_out.x = vector_in[0];
    vector_out.y = vector_in[1];
    vector_out.z = vector_in[2];
    return vector_out;
}

Vector3d rviz_visualization::convertGeometryToEigen(const geometry_msgs::Vector3 &vector_in){
    Vector3d vector_out;
    vector_out[0] = vector_in.x;
    vector_out[1] = vector_in.y;
    vector_out[2] = vector_in.z;
    return vector_out;
}

void rviz_visualization::publishMesh(const char * package, const char* relative_path, const char *modelname, Vector3d &pos, Quaterniond &orientation,
                                     double scale, const char *frame, const char *ns, int message_id, double duration) {
    visualization_msgs::Marker mesh;
    mesh.header.frame_id = frame;
    mesh.ns = ns;
    mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh.color.r = 1.0f;
    mesh.color.g = 1.0f;
    mesh.color.b = 1.0f;
    mesh.color.a = 0.5;
    mesh.scale.x = scale;
    mesh.scale.y = scale;
    mesh.scale.z = scale;
    mesh.lifetime = ros::Duration(duration);
    mesh.header.stamp = ros::Time::now();
    mesh.action = visualization_msgs::Marker::ADD;
    mesh.id = message_id;
    mesh.pose.position.x = pos[0];
    mesh.pose.position.y = pos[1];
    mesh.pose.position.z = pos[2];
    mesh.pose.orientation.x = orientation.x();
    mesh.pose.orientation.y = orientation.y();
    mesh.pose.orientation.z = orientation.z();
    mesh.pose.orientation.w = orientation.w();
    char meshpath[200];
    sprintf(meshpath, "package://%s/%s/%s",package, relative_path, modelname);
    mesh.mesh_resource = meshpath;
    visualization_pub.publish(mesh);
};

void rviz_visualization::publishSphere(Vector3d &pos, const char *frame, const char *ns, int message_id, COLOR color,
                                       float radius, double duration) {
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
                                     COLOR color, float radius, double duration) {
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

void rviz_visualization::publishCylinder(Vector3d &pos, const char* frame, const char* ns, int message_id,
                                         COLOR color, float radius, double duration){
    visualization_msgs::Marker cylinder;
    cylinder.header.frame_id = frame;
    cylinder.ns = ns;
    cylinder.type = visualization_msgs::Marker::CYLINDER;
    cylinder.color.r = color.r;
    cylinder.color.g = color.g;
    cylinder.color.b = color.b;
    cylinder.color.a = color.a;
    cylinder.lifetime = ros::Duration(duration);
    cylinder.scale.x = radius;
    cylinder.scale.y = radius;
    cylinder.scale.z = radius;
    cylinder.action = visualization_msgs::Marker::ADD;
    cylinder.header.stamp = ros::Time::now();
    cylinder.id = message_id;
    cylinder.pose.position.x = pos(0);
    cylinder.pose.position.y = pos(1);
    cylinder.pose.position.z = pos(2);
    visualization_pub.publish(cylinder);
}

void rviz_visualization::publishRay(Vector3d &pos, Vector3d &dir, const char *frame, const char *ns, int message_id,
                                    COLOR color, double duration) {
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
                                     COLOR color, double duration, float height) {
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

void rviz_visualization::clearAll() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::DELETEALL;
    visualization_pub.publish(marker);
}
