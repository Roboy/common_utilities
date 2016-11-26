//
// Created by bruh on 1/8/16.
//

#ifndef ROBOY_CONTROL_COMMONDEFINITIONS_H
#define ROBOY_CONTROL_COMMONDEFINITIONS_H

enum ControlMode {UNDEFINED_CONTROL = 0, POSITION_CONTROL, VELOCITY_CONTROL, FORCE_CONTROL, MUSCLE_ACTIVITY_CONTROL};

static const char * controlModeStrings[] = { "undefined", "position", "velocity", "force", "muscle_activity" };

static const char * getControlModeString(ControlMode mode) {
	return controlModeStrings[mode];
}

enum ControllerState {  UNDEFINED = 0,
                        INITIALIZED,
                        PREPROCESS_TRAJECTORY,
                        TRAJECTORY_READY,
                        TRAJECTORY_FAILED,
                        TRAJECTORY_PLAYING,
                        TRAJECTORY_PAUSED,
                        TRAJECTORY_DONE,
                        INITIALIZE_ERROR,
                        STOPPED};



//static const char * statusStrings[] = { "UNDEFINED", "INITIALIZED", "PREPROCESS_TRAJECTORY",
//                                        "TRAJECTORY_READY", "TRAJECTORY_FAILED", "TRAJECTORY_PLAYING",
//                                        "TRAJECTORY_DONE", "INITIALIZE_ERROR" };
//
//const char * getStatusString(int enumVal) {
//    return statusStrings[enumVal];
//}

enum SteeringCommand {STOP_TRAJECTORY=0,PLAY_TRAJECTORY,PAUSE_TRAJECTORY,REWIND_TRAJECTORY};

enum LEG{
    LEFT = 0,
    RIGHT,
    NONE
};

enum LEG_STATE{
    Stance = 0,
    Lift_off,
    Swing,
    Stance_Preparation
};

enum VISUALIZATION{
    Tendon,
    COM,
    Forces,
    MomentArm,
    Mesh,
    StateMachineParameters,
    ForceTorqueSensors,
		IMUs,
    InteractiveMarkers
};

enum ABORTION{
    COMheight,
    headingDeviation,
    selfCollision
};

enum SIMULATIONCONTROL{
    Play,
    Pause,
    Rewind,
    Slow_Motion,
    UpdateInteractiveMarker
};

enum{
    POSITION = 0,
    VELOCITY
};

enum PLANE{
    TRAVERSAL,
    SAGITTAL,
    CORONAL
};

// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * (float)M_PI / 180.0f)
// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0f / (float)M_PI)

#endif //ROBOY_CONTROL_COMMONDEFINITIONS_H
