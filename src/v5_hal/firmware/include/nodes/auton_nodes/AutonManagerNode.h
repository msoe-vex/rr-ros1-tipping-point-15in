#pragma once

#include "lib-rr/nodes/NodeManager.h"
#include "lib-rr/auton/auton_routines/TestPathAuton.h"
#include "lib-rr/auton/auton_routines/TestPoseAuton.h"
#include "lib-rr/auton/auton_routines/TestTurnAuton.h"
#include "auton/auton_routines/BasicAuton.h"
#include "lib-rr/pathing/PathManager.h"

class AutonManagerNode : public Node {
private:
    IDriveNode* m_drive_node;
    ILiftNode* m_lift_node;
    IClawNode* m_front_claw_node;
    IClawNode* m_front_claw_node;


    Auton* m_test_path_auton;
    Auton* m_prog_skills;
    Auton* m_basic_auton;

public:
    AutonManagerNode(NodeManager* node_manager, IDriveNode* drive_node, ILiftNode* lift_node, IClawNode* front_claw_node, IClawNode* back_claw_node, IntakeNode*);

    Auton* selected_auton;

    void initialize();

    void autonPeriodic();
};
