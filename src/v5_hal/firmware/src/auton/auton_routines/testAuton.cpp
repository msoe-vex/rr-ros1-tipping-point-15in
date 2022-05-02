#include "auton/auton_routines/testAuton.h"

testAuton::testAuton(IDriveNode* drive_node, OdometryNode* odom_node) : 
        Auton("Test Auton"), 
        m_drive_node(drive_node),
        m_odom_node(odom_node) {
    
}

void testAuton::AddNodes() {
    AutonNode* test = new AutonNode(100, new DriveStraightAction(m_drive_node, m_odom_node, m_driveParams, 50, 50, 80));
    Auton::AddFirstNode(test);

}