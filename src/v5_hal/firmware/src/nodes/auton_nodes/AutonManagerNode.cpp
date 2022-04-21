#include "nodes/auton_nodes/AutonManagerNode.h"

AutonManagerNode::AutonManagerNode(NodeManager* node_manager, IDriveNode* drive_node, OdometryNode* odom_node, 
    InertialSensorNode* inertial_sensor_node, IClawNode* front_claw_node, ILiftNode* lift_node) : 
        Node(node_manager, 50),
        m_drive_node(drive_node),
        m_odom_node(odom_node),
        m_inertial_sensor_node(inertial_sensor_node),
        m_front_claw_node(front_claw_node),
        m_lift_node(lift_node) {
    m_test_auton = new MatchAuton(m_drive_node, m_odom_node, m_front_claw_node, m_lift_node);
    selected_auton = m_test_auton;
}

void AutonManagerNode::initialize() {
    PathManager::GetInstance()->LoadPathsFile("/usd/pathMatchAuton.json");
}

void AutonManagerNode::autonPeriodic() {
    if(!selected_auton->Complete()) {
        selected_auton->AutonPeriodic();
    }
}