#include "common_utilities/rviz_visualization.hpp"

std::shared_ptr<interactive_markers::InteractiveMarkerServer> rviz_visualization::interactive_marker_server;
interactive_markers::MenuHandler rviz_visualization::menu_handler;
bool rviz_visualization::first = true;

rviz_visualization::rviz_visualization() {
    if (!rclcpp::is_initialized()) {
        int argc = 0;
        char **argv = NULL;
        rclcpp::init(argc, argv); //, node_name, ros::init_options::NoSigintHandler);
    }

    nh = rclcpp::Node::make_shared("rviz_visualization");

    visualization_pub = nh->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 1);
    visualization_array_pub = nh->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 1);

    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tfBuffer = std::make_shared<tf2_ros::Buffer>(nh->get_clock());
    tfBuffer->setUsingDedicatedThread(true);
    listener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer, nh, false);//  new tf2_ros::TransformListener(buffer, nh, false);
    broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);

    if(first){
        first = false;
        interactive_marker_server = std::make_shared<interactive_markers::InteractiveMarkerServer>("interactive_markers", nh) ;
//        menu_handler.insert()
        auto process_callback_ = interactive_markers::InteractiveMarkerServer::FeedbackCallback(
                boost::bind(&rviz_visualization::processFeedback, this, _1));
        menu_handler.insert( "First Entry", process_callback_ );
        menu_handler.insert( "Second Entry", process_callback_ );
        interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Submenu" );
        menu_handler.insert( sub_menu_handle, "First Entry", process_callback_ );
        menu_handler.insert( sub_menu_handle, "Second Entry", process_callback_ );
    }

}

rviz_visualization::~rviz_visualization() {
    interactive_marker_server->clear();
    interactive_marker_server->applyChanges();
}

void rviz_visualization::processFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstPtr &feedback )
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
        case visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK:
            RCLCPP_DEBUG_STREAM(nh->get_logger(), s.str() << ": button click" << mouse_point_ss.str() << "." );
            break;

        case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
            RCLCPP_DEBUG_STREAM(nh->get_logger(), s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
            break;

        case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
            RCLCPP_DEBUG_STREAM(nh->get_logger(), s.str() << ": pose changed"
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
                                      << feedback->header.stamp.nanosec << " nsec" );
            break;

        case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN:
            RCLCPP_DEBUG_STREAM(nh->get_logger(), s.str() << ": mouse down" << mouse_point_ss.str() << "." );
            break;

        case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP:
            RCLCPP_DEBUG_STREAM(nh->get_logger(), s.str() << ": mouse up" << mouse_point_ss.str() << "." );
            break;
    }

    interactive_marker_server->applyChanges();
}

Marker rviz_visualization::makeBox( InteractiveMarker &msg )
{


    std::string path = "";//ros::package::getPath("robots");
    // TODO use ament_index_cpp::get_package_share_directory
    struct stat buffer;
    if(stat((path+"/common/meshes/visuals/roboy_head.stl").c_str(), &buffer) != 0) {
        Marker marker;
        marker.type = Marker::CUBE;
        marker.scale.x = msg.scale * 0.45;
        marker.scale.y = msg.scale * 0.45;
        marker.scale.z = msg.scale * 0.45;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 0.8;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        return marker;
    }else {
        Marker mesh;
        mesh.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        mesh.scale.x = 0.0003;
        mesh.scale.y = 0.0003;
        mesh.scale.z = 0.0003;
        mesh.color.r = 1.0;
        mesh.color.g = 0;
        mesh.color.b = 0;
        mesh.color.a = 1.0;
        mesh.pose.position.x = 0;
        mesh.pose.position.y = 0;
        mesh.pose.position.z = 0;
        mesh.pose.orientation.x = 0;
        mesh.pose.orientation.y = 0;
        mesh.pose.orientation.z = 0;
        mesh.pose.orientation.w = 1;
        mesh.mesh_resource = "package://robots/common/meshes/visuals/roboy_head.stl";
        return mesh;
    }
}

InteractiveMarkerControl& rviz_visualization::makeBoxControl( InteractiveMarker &msg )
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeBox(msg) );
    msg.controls.push_back( control );

    return msg.controls.back();
}

void rviz_visualization::make6DofMarker( bool fixed, unsigned int interaction_mode, const tf2::Vector3& position,
                                         bool show_6dof, double scale, const char *frame, const char *name,
                                         const char *description )
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = frame;
    int_marker.pose.position = geometry_msgs::msg::Point();
    int_marker.pose.position.x = position.x();
    int_marker.pose.position.y = position.y();
    int_marker.pose.position.z = position.z();
//    tf2::pointTFToMsg(position, int_marker.pose.position);
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

    if (interaction_mode != visualization_msgs::msg::InteractiveMarkerControl::NONE)
    {
        std::string mode_text;
        if( interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
        if( interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
        if( interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
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
        control.name = "rotate_y";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 0;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }

    interactive_marker_server->insert(int_marker);
    auto process_callback_ = interactive_markers::InteractiveMarkerServer::FeedbackCallback(
            boost::bind(&rviz_visualization::processFeedback, this, _1));
    interactive_marker_server->setCallback(int_marker.name, process_callback_);
    if (interaction_mode != visualization_msgs::msg::InteractiveMarkerControl::NONE)
        menu_handler.apply( *interactive_marker_server, int_marker.name );
    interactive_marker_server->applyChanges();
}

geometry_msgs::msg::Vector3 rviz_visualization::convertEigenToGeometry(const Vector3d &vector_in){
    geometry_msgs::msg::Vector3 vector_out;
    vector_out.x = vector_in[0];
    vector_out.y = vector_in[1];
    vector_out.z = vector_in[2];
    return vector_out;
}

Vector3d rviz_visualization::convertGeometryToEigen(const geometry_msgs::msg::Vector3 &vector_in){
    Vector3d vector_out;
    vector_out[0] = vector_in.x;
    vector_out[1] = vector_in.y;
    vector_out[2] = vector_in.z;
    return vector_out;
}

void rviz_visualization::PoseMsgToTF(const geometry_msgs::msg::Pose& msg, tf2::Transform& bt)
{
    bt = tf2::Transform(tf2::Quaternion(msg.orientation.x,
                              msg.orientation.y,
                              msg.orientation.z,
                              msg.orientation.w),
                       tf2::Vector3(msg.position.x, msg.position.y, msg.position.z));
}

void rviz_visualization::publishMesh(const char * package, const char* relative_path, const char *modelname, Vector3d &pos, Quaterniond &orientation,
                                     double scale, const char *frame, const char *ns, int message_id, double duration, COLOR color, bool update) {
    visualization_msgs::msg::Marker mesh;
    mesh.header.frame_id = frame;
    mesh.ns = ns;
    mesh.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    mesh.color.r = color.r;
    mesh.color.g = color.g;
    mesh.color.b = color.b;
    mesh.color.a = color.a;
    mesh.scale.x = scale;
    mesh.scale.y = scale;
    mesh.scale.z = scale;
    mesh.lifetime = rclcpp::Duration(duration);
    mesh.header.stamp = nh->now();
    if(!update)
        mesh.action = visualization_msgs::msg::Marker::ADD;
    else
        mesh.action = visualization_msgs::msg::Marker::MODIFY;
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
    if(publish_as_marker_array) {
        marker_array.markers.push_back(mesh);
        if(marker_array.markers.size()>number_of_markers_to_publish_at_once){
            visualization_array_pub->publish(marker_array);
            marker_array.markers.clear();
        }
    }else {
        visualization_pub->publish(mesh);
    }
};

void rviz_visualization::publishMesh(const char * package, const char* relative_path, const char *modelname, geometry_msgs::msg::Pose &pose,
                 double scale, const char *frame, const char *ns, int message_id, double duration, COLOR color, bool update){
    visualization_msgs::msg::Marker mesh;
    mesh.header.frame_id = frame;
    mesh.ns = ns;
    mesh.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    mesh.color.r = color.r;
    mesh.color.g = color.g;
    mesh.color.b = color.b;
    mesh.color.a = color.a;
    mesh.scale.x = scale;
    mesh.scale.y = scale;
    mesh.scale.z = scale;
    mesh.lifetime = rclcpp::Duration(duration);
    mesh.header.stamp = nh->now();
    if(!update)
        mesh.action = visualization_msgs::msg::Marker::ADD;
    else
        mesh.action = visualization_msgs::msg::Marker::MODIFY;
    mesh.id = message_id;
    mesh.pose = pose;
    char meshpath[200];
    sprintf(meshpath, "package://%s/%s/%s",package, relative_path, modelname);
    mesh.mesh_resource = meshpath;
    if(publish_as_marker_array) {
        marker_array.markers.push_back(mesh);
        if(marker_array.markers.size()>number_of_markers_to_publish_at_once){
            visualization_array_pub->publish(marker_array);
            marker_array.markers.clear();
        }
    }else {
        visualization_pub->publish(mesh);
    }
}

void rviz_visualization::publishSphere(Vector3d &pos, const char *frame, const char *ns, int message_id, COLOR color,
                                       float radius, double duration) {
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = frame;
    sphere.ns = ns;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.color.r = color.r;
    sphere.color.g = color.g;
    sphere.color.b = color.b;
    sphere.color.a = color.a;
    sphere.lifetime = rclcpp::Duration(duration);
    sphere.scale.x = radius;
    sphere.scale.y = radius;
    sphere.scale.z = radius;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.header.stamp = nh->now();
    sphere.id = message_id;
    sphere.pose.position.x = pos(0);
    sphere.pose.position.y = pos(1);
    sphere.pose.position.z = pos(2);
    sphere.pose.orientation.x = 0;
    sphere.pose.orientation.y = 0;
    sphere.pose.orientation.z = 0;
    sphere.pose.orientation.w = 1;
    if(publish_as_marker_array) {
        marker_array.markers.push_back(sphere);
        if(marker_array.markers.size()>number_of_markers_to_publish_at_once){
            visualization_array_pub->publish(marker_array);
            marker_array.markers.clear();
        }
    }else {
        visualization_pub->publish(sphere);
    }
};

void rviz_visualization::publishSphere(geometry_msgs::msg::Pose &pose, const char *frame, const char *ns, int message_id, COLOR color,
                                       float radius, double duration) {
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = frame;
    sphere.ns = ns;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.color.r = color.r;
    sphere.color.g = color.g;
    sphere.color.b = color.b;
    sphere.color.a = color.a;
    sphere.lifetime = rclcpp::Duration(duration);
    sphere.scale.x = radius;
    sphere.scale.y = radius;
    sphere.scale.z = radius;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.header.stamp = nh->now();
    sphere.id = message_id;
    sphere.pose.position = pose.position;
    sphere.pose.orientation.x = 0;
    sphere.pose.orientation.y = 0;
    sphere.pose.orientation.z = 0;
    sphere.pose.orientation.w = 1;
    if(publish_as_marker_array) {
        marker_array.markers.push_back(sphere);
        if(marker_array.markers.size()>number_of_markers_to_publish_at_once){
            visualization_array_pub->publish(marker_array);
            marker_array.markers.clear();
        }
    }else {
        visualization_pub->publish(sphere);
    }
};

void rviz_visualization::publishCube(Vector3d &pos, Vector4d &quat, const char *frame, const char *ns, int message_id,
                                     COLOR color, float radius, double duration) {
    visualization_msgs::msg::Marker cube;
    cube.header.frame_id = frame;
    cube.ns = ns;
    cube.type = visualization_msgs::msg::Marker::CUBE;
    cube.color.r = color.r;
    cube.color.g = color.g;
    cube.color.b = color.b;
    cube.color.a = color.a;
    cube.lifetime = rclcpp::Duration(duration);
    cube.scale.x = radius;
    cube.scale.y = radius;
    cube.scale.z = radius;
    cube.action = visualization_msgs::msg::Marker::ADD;
    cube.header.stamp = nh->now();
    cube.id = message_id;
    cube.pose.position.x = pos(0);
    cube.pose.position.y = pos(1);
    cube.pose.position.z = pos(2);
    cube.pose.orientation.x = quat(0);
    cube.pose.orientation.y = quat(1);
    cube.pose.orientation.z = quat(2);
    cube.pose.orientation.w = quat(3);
    if(publish_as_marker_array) {
        marker_array.markers.push_back(cube);
        if(marker_array.markers.size()>number_of_markers_to_publish_at_once){
            visualization_array_pub->publish(marker_array);
            marker_array.markers.clear();
        }
    }else {
        visualization_pub->publish(cube);
    }
};

void rviz_visualization::publishCube(geometry_msgs::msg::Pose &pose, const char* frame, const char* ns,
                                     int message_id, COLOR color,float radius, double duration){
    visualization_msgs::msg::Marker cube;
    cube.header.frame_id = frame;
    cube.ns = ns;
    cube.type = visualization_msgs::msg::Marker::CUBE;
    cube.color.r = color.r;
    cube.color.g = color.g;
    cube.color.b = color.b;
    cube.color.a = color.a;
    cube.lifetime = rclcpp::Duration(duration);
    cube.scale.x = radius;
    cube.scale.y = radius;
    cube.scale.z = radius;
    cube.action = visualization_msgs::msg::Marker::ADD;
    cube.header.stamp = nh->now();
    cube.id = message_id;
    cube.pose = pose;
    if(publish_as_marker_array) {
        marker_array.markers.push_back(cube);
        if(marker_array.markers.size()>number_of_markers_to_publish_at_once){
            visualization_array_pub->publish(marker_array);
            marker_array.markers.clear();
        }
    }else {
        visualization_pub->publish(cube);
    }
}

void rviz_visualization::publishCube(Vector3d &pos, Quaternionf &quat, const char *frame, const char *ns, int message_id,
                                     COLOR color, float dx, float dy, float dz, double duration){
    visualization_msgs::msg::Marker cube;
    cube.header.frame_id = frame;
    cube.ns = ns;
    cube.type = visualization_msgs::msg::Marker::CUBE;
    cube.color.r = color.r;
    cube.color.g = color.g;
    cube.color.b = color.b;
    cube.color.a = color.a;
    cube.lifetime = rclcpp::Duration(duration);
    cube.scale.x = dx;
    cube.scale.y = dy;
    cube.scale.z = dz;
    cube.action = visualization_msgs::msg::Marker::ADD;
    cube.header.stamp = nh->now();
    cube.id = message_id;
    cube.pose.position.x = pos(0);
    cube.pose.position.y = pos(1);
    cube.pose.position.z = pos(2);
    cube.pose.orientation.x = quat.x();
    cube.pose.orientation.y = quat.y();
    cube.pose.orientation.z = quat.z();
    cube.pose.orientation.w = quat.w();
    if(publish_as_marker_array) {
        marker_array.markers.push_back(cube);
        if(marker_array.markers.size()>number_of_markers_to_publish_at_once){
            visualization_array_pub->publish(marker_array);
            marker_array.markers.clear();
        }
    }else {
        visualization_pub->publish(cube);
    }
}

void rviz_visualization::publishCylinder(Vector3d &pos, const char* frame, const char* ns, int message_id,
                                         COLOR color, float radius, double duration){
    visualization_msgs::msg::Marker cylinder;
    cylinder.header.frame_id = frame;
    cylinder.ns = ns;
    cylinder.type = visualization_msgs::msg::Marker::CYLINDER;
    cylinder.color.r = color.r;
    cylinder.color.g = color.g;
    cylinder.color.b = color.b;
    cylinder.color.a = color.a;
    cylinder.lifetime = rclcpp::Duration(duration);
    cylinder.scale.x = radius;
    cylinder.scale.y = radius;
    cylinder.scale.z = radius;
    cylinder.action = visualization_msgs::msg::Marker::ADD;
    cylinder.header.stamp = nh->now();
    cylinder.id = message_id;
    cylinder.pose.position.x = pos(0);
    cylinder.pose.position.y = pos(1);
    cylinder.pose.position.z = pos(2);
    cylinder.pose.orientation.x = 0;
    cylinder.pose.orientation.y = 0;
    cylinder.pose.orientation.z = 0;
    cylinder.pose.orientation.w = 1;
    if(publish_as_marker_array) {
        marker_array.markers.push_back(cylinder);
        if(marker_array.markers.size()>number_of_markers_to_publish_at_once){
            visualization_array_pub->publish(marker_array);
            marker_array.markers.clear();
        }
    }else {
        visualization_pub->publish(cylinder);
    }
}

void rviz_visualization::publishRay(Vector3d &pos, Vector3d &dir, const char *frame, const char *ns, int message_id,
                                    COLOR color, double duration) {
    visualization_msgs::msg::Marker arrow;
    arrow.ns = ns;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.color.r = color.r;
    arrow.color.g = color.g;
    arrow.color.b = color.b;
    arrow.color.a = color.a;
    arrow.lifetime = rclcpp::Duration(duration);
    arrow.scale.x = 0.0025;
    arrow.scale.y = 0.015;
    arrow.scale.z = 0.015;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.header.stamp = nh->now();

    arrow.header.frame_id = frame;
    arrow.id = message_id;
    arrow.points.clear();
    geometry_msgs::msg::Point p;
    p.x = pos(0);
    p.y = pos(1);
    p.z = pos(2);
    arrow.points.push_back(p);
    p.x += dir(0);
    p.y += dir(1);
    p.z += dir(2);
    arrow.points.push_back(p);
    if(publish_as_marker_array) {
        marker_array.markers.push_back(arrow);
        if(marker_array.markers.size()>number_of_markers_to_publish_at_once){
            visualization_array_pub->publish(marker_array);
            marker_array.markers.clear();
        }
    }else {
        visualization_pub->publish(arrow);
    }
};

void rviz_visualization::publishText(Vector3d &pos, const char *text, const char *frame, const char *ns, int message_id,
                                     COLOR color, double duration, float height) {
    visualization_msgs::msg::Marker text_msg;
    text_msg.header.frame_id = frame;
    text_msg.ns = ns;
    text_msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_msg.color.r = color.r;
    text_msg.color.g = color.g;
    text_msg.color.b = color.b;
    text_msg.color.a = color.a;
    text_msg.lifetime = rclcpp::Duration(duration);
    text_msg.scale.z = height;
    text_msg.action = visualization_msgs::msg::Marker::ADD;
    text_msg.header.stamp = nh->now();
    text_msg.id = message_id;
    text_msg.pose.position.x = pos(0);
    text_msg.pose.position.y = pos(1);
    text_msg.pose.position.z = pos(2);
    text_msg.pose.orientation.x = 0;
    text_msg.pose.orientation.y = 0;
    text_msg.pose.orientation.z = 0;
    text_msg.pose.orientation.w = 1;
    text_msg.text = text;
    if(publish_as_marker_array) {
        marker_array.markers.push_back(text_msg);
        if(marker_array.markers.size()>number_of_markers_to_publish_at_once){
            visualization_array_pub->publish(marker_array);
            marker_array.markers.clear();
        }
    }else {
        visualization_pub->publish(text_msg);
    }
};

void rviz_visualization::clearMarker(int id) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.id = id;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    visualization_pub->publish(marker);
}

void rviz_visualization::clearAll() {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    visualization_pub->publish(marker);
}

bool rviz_visualization::getTransform(string from, string to, geometry_msgs::msg::Pose &transform){
//    tf2_ros::StampedTransform trans;
    geometry_msgs::msg::TransformStamped trans;
    try {
        trans = tfBuffer->lookupTransform(from.c_str(), to.c_str(), tf2::get_now());
        transform.position.x = trans.transform.translation.x;
        transform.position.y = trans.transform.translation.y;
        transform.position.z = trans.transform.translation.z;
        transform.orientation = trans.transform.rotation;
//        if (listener->waitForTransform(from.c_str(), to.c_str(),
//                                      ros::Time(0), rclcpp::Duration(0.0001))) {
//            listener.lookupTransform(from.c_str(), to.c_str(),
//                                     ros::Time(0), trans);
//            tf2::poseTFToMsg(trans,transform);
//        } else {
            rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(nh->get_logger(), steady_clock,10,"transform %s->%s is not available", from.c_str(), to.c_str());
//            return false;
//        }
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(nh->get_logger(),"%s", ex.what());
        return false;
    }
    return true;
}

bool rviz_visualization::getLighthouseTransform(bool lighthouse, const char *to, Matrix4d &transform){
    geometry_msgs::msg::TransformStamped trans;
    try {
        trans = tfBuffer->lookupTransform(to, (lighthouse?"lighthouse2":"lighthouse1"), tf2::get_now());
        //listener.lookupTransform(to, (lighthouse?"lighthouse2":"lighthouse1"), ros::Time(0), trans);
    }
    catch (tf2::TransformException ex) {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(nh->get_logger(), steady_clock, 1,"%s", ex.what());

        return false;
    }

    Eigen::Affine3d trans_ = tf2::transformToEigen(trans);
    transform = trans_.matrix();
    return true;
}

bool rviz_visualization::getLighthouseTransform(const char *from, bool lighthouse, Matrix4d &transform){
    geometry_msgs::msg::TransformStamped trans;
    try {
        trans = tfBuffer->lookupTransform((lighthouse?"lighthouse2":"lighthouse1"), from, tf2::get_now());
    }
    catch (tf2::TransformException ex) {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(nh->get_logger(), steady_clock,1,"%s", ex.what());
        return false;
    }

    Eigen::Affine3d trans_ = tf2::transformToEigen(trans);
    transform = trans_.matrix();
    return true;
}

bool rviz_visualization::getTransform(const char *from, const char *to, tf2::Transform &transform){
    geometry_msgs::msg::TransformStamped trans;
    try {
        trans = tfBuffer->lookupTransform(to, from, tf2::get_now());
    }
    catch (tf2::TransformException ex) {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(nh->get_logger(), steady_clock,1,"%s", ex.what());
        return false;
    }
    // TODO check if this is correct
    fromMsg(&trans, transform);
    //tf2::Transform(trans.transform.rotation, trans.transform.translation);
    //transform = o tf2::Transform(trans);
    return true;
}

bool rviz_visualization::getTransform(string from, string to, Matrix4d &transform){
    geometry_msgs::msg::TransformStamped trans;
    try {
        trans = tfBuffer->lookupTransform(from.c_str(), to.c_str(), tf2::get_now());
    }
    catch (tf2::TransformException ex) {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(nh->get_logger(), steady_clock,10,"%s", ex.what());
        return false;
    }

    Eigen::Affine3d trans_ = tf2::transformToEigen(trans);
    transform = trans_.matrix();
    return true;
}

bool rviz_visualization::publishTransform(string from, string to, geometry_msgs::msg::Pose &transform){
    static geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = nh->now();
    msg.header.frame_id = from;
//    msg.header.seq++;
    msg.child_frame_id = to;
    msg.transform.rotation = transform.orientation;
    msg.transform.translation.x = transform.position.x;
    msg.transform.translation.y = transform.position.y;
    msg.transform.translation.z = transform.position.z;
    broadcaster->sendTransform(msg);
}
