#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tf/tf.h>
#include <interactive_markers/menu_handler.h>
#include <interactive_markers/interactive_marker_server.h>

using namespace Eigen;

struct COLOR{
    COLOR(float r, float g, float b, float a):r(r),g(g),b(b),a(a){};
    float r,g,b,a;
};

using namespace visualization_msgs;

class rviz_visualization{
public:
    rviz_visualization();
    ~rviz_visualization();

    static void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

    void initializeInteractiveMarkerServer();

    Marker makeBox( InteractiveMarker &msg );

    InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg );

    void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position,
                         bool show_6dof, double scale=1, const char *frame="world", const char *name="interactive_marker",
                         const char *description="for interaction and shit" );

    Vector3d convertGeometryToEigen(const geometry_msgs::Vector3 &vector_in);
    geometry_msgs::Vector3 convertEigenToGeometry(const Vector3d &vector_in);

    /**
     * Publishes a mesh visualization marker
     * @param pos at this position
     * @param orientation with this orientation
     * @param modelname name of the mesh.dae
     * @param frame in this frame
     * @param ns namespace
     * @param message_id unique id
     * @param duration in seconds
     */
    void publishMesh(Vector3d &pos, Vector4d& orientation, const char* modelname,
                     const char *frame, const char *ns, int message_id, double duration);

    /**
     * Publishes a sphere visualization marker
     * @param pos at this positon
     * @param frame in this frame
     * @param ns namespace
     * @param message_id a unique id
     * @param rgda rgb color (0-1) plus transparancy
     * @param duration for this duration in seconds (0=forever)
     */
    void publishSphere(Vector3d &pos, const char* frame, const char* ns, int message_id, COLOR color,float radius = 0.01, double duration=0);

    /**
     * Publishes a cube visualization marker
     * @param pos at this positon
     * @param quat with this orientation
     * @param frame in this frame
     * @param ns namespace
     * @param message_id a unique id
     * @param rgda rgb color (0-1) plus transparancy
     * @param duration for this duration in seconds (0=forever)
     */
    void publishCube(Vector3d &pos, Vector4d &quat, const char* frame, const char* ns, int message_id, COLOR color,float radius = 0.01, double duration=0);

    /**
     * Publishes a ray visualization marker
     * @param pos at this positon
     * @param dir direction
     * @param frame in this frame
     * @param message_id a unique id
     * @param ns namespace
     * @param color rgb color (0-1) plus transparancy
     * @param duration for this duration in seconds (0=forever)
     */
    void publishRay(Vector3d &pos, Vector3d &dir, const char* frame, const char* ns, int message_id, COLOR color, double duration=0);
    /**
     * Publishes a text message marker
     * @param pos at this positon
     * @param text with this text
     * @param frame in this frame
     * @param ns namespace
     * @param message_id a unique id
     * @param color rgb color (0-1) plus transparancy
     * @param duration for this duration in seconds (0=forever)
     * @param height height of the text
     */
    void publishText(Vector3d &pos, const char *text, const char *frame, const char *ns, int message_id, COLOR color, double duration, float height);

    void clearAll();
private:
    ros::NodeHandlePtr nh;
    ros::Publisher visualization_pub;
    static boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server;
    static interactive_markers::MenuHandler menu_handler;
    static bool first;
};