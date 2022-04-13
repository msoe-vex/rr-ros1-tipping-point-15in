#include "auton/auton_routines/tuningRoutine.h"

tuningRoutine::tuningRoutine(IDriveNode* drive_node) : 
        Auton("Tune DriveStraight"), 
        m_drive_node(drive_node) {
    
}

void tuningRoutine::AddNodes() {
    m_forward_node = new AutonNode(10, new DriveStraightAction(m_drive_node, 30, 50, 10));
    Auton::AddFirstNode(m_forward_node);

    //m_backward_node = new AutonNode(10, new DriveStraightAction(m_drive_node, 800, 10000, 10000));
    
    //m_forward_node->AddNext(m_claw_clos);

}