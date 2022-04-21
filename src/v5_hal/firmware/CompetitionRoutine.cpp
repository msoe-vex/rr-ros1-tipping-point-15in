#include "auton/auton_routines/tuningRoutine.h"

tuningRoutine::tuningRoutine(IDriveNode* drive_node, OdometryNode* odometry_node, IClawNode* claw_node) : 
        Auton("Tune DriveStraight"), 
        m_drive_node(drive_node),
        m_odometry_node(odometry_node),
        m_claw_node(claw_node) {
    
}

void tuningRoutine::AddNodes() {
    m_forward_node = new AutonNode(10, new DriveStraightAction(m_drive_node, m_odometry_node, 35, 40, 20));
    Auton::AddFirstNode(m_forward_node);
    
    m_claw_close_node = new AutonNode(10, new UseClawAction(m_claw_node, false));
    m_forward_node->AddNext(m_claw_close_node);

    //m_backward_node = new AutonNode(10, new DriveStraightAction(m_drive_node, 800, 10000, 10000));
    
    //m_forward_node->AddNext(m_claw_clos);
}