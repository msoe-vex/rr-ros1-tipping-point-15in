#include "nodes/auton_nodes/AutonManagerNode.h"

AutonManagerNode::AutonManagerNode(NodeManager* node_manager, IDriveNode* drive_node, OdometryNode* odom_node, 
    InertialSensorNode* inertial_sensor_node, IClawNode* front_claw_node, IClawNode* wing_arm, BackClawNode* back_claw_node, ILiftNode* liftNode, IRollerIntakeNode* intakeNode) : 
        Node(node_manager, 50),
        m_drive_node(drive_node),
        m_odom_node(odom_node),
        m_inertial_sensor_node(inertial_sensor_node),
        m_front_claw_node(front_claw_node),
        m_wing_arm(wing_arm),
        m_back_claw_node(back_claw_node),
        m_liftNode(liftNode),
        m_intakeNode(intakeNode) {
    m_leftQuickAuton = new LeftQuickAuton(m_drive_node, m_odom_node, m_front_claw_node, m_wing_arm, m_back_claw_node, m_liftNode, m_intakeNode);
    m_tuningAuton = new tuningRoutine(m_drive_node, m_odom_node);
    m_testAuton = new testAuton(m_drive_node, m_odom_node);
    selected_auton = m_testAuton;
}

void AutonManagerNode::initialize() {
    PathManager::GetInstance()->LoadPathsFile("/usd/pathMatchAuton.json");
}

void AutonManagerNode::autonPeriodic() {
    if(!selected_auton->Complete()) {
        selected_auton->AutonPeriodic();
    }
}