#pragma once

#include "lib-rr/nodes/NodeManager.h"

#include "lib-rr/auton/auton_routines/TestPathAuton.h"
#include "lib-rr/auton/auton_routines/TestPoseAuton.h"
#include "lib-rr/auton/auton_routines/TestTurnAuton.h"
#include "lib-rr/auton/auton_routines/TestTankPathAuton.h"
#include "auton/auton_routines/TestBasicGoalAuton.h"
#include "auton/auton_routines/MatchAuton.h"
#include "auton/auton_routines/tuningRoutine.h"
#include "lib-rr/nodes/subsystems/IClawNode.h"
#include "lib-rr/nodes/subsystems/ILiftNode.h"
#include "lib-rr/nodes/odometry_nodes/OdometryNode.h"
#include "auton/auton_routines/odomTest.h"
#include "auton/auton_routines/testAuton.h"
#include "lib-rr/pathing/PathManager.h"
#include "nodes/BackClawNode.h"

class AutonManagerNode : public Node {
private:
    IDriveNode* m_drive_node;
    IClawNode* m_front_claw_node;
    IClawNode* m_wing_arm;
    BackClawNode* m_back_claw_node;
    OdometryNode* m_odom_node;
    InertialSensorNode* m_inertial_sensor_node;
    ILiftNode* m_liftNode;
    IRollerIntakeNode* m_intakeNode;

    Auton* m_matchAuton;
    Auton* m_tuningAuton;
    Auton* m_testAuton;

public:
    AutonManagerNode(NodeManager* node_manager, IDriveNode* drive_node, OdometryNode* odom_node, 
        InertialSensorNode* inertial_sensor_node, IClawNode* front_claw_node, IClawNode* wing_arm, BackClawNode* back_claw_node, ILiftNode* liftNode, IRollerIntakeNode* intakeNode);

    Auton* selected_auton;

    void initialize();

    void autonPeriodic();
};
