#include "nodes/auton_nodes/AutonManagerNode.h"

AutonManagerNode::AutonManagerNode(NodeManager* node_manager, IDriveNode* drive_node, ILiftNode* lift_node, IClawNode* front_claw_node, IClawNode* back_claw_node) : 
        Node(node_manager, 50),
        m_drive_node(drive_node),
        m_lift_node(lift_node),
        m_front_claw_node(front_claw_node),
        m_back_claw_node(back_claw_node) {
    m_basic_auton = new BasicAuton(m_drive_node, m_front_claw_node);
    selected_auton = m_basic_auton;
}

void AutonManagerNode::initialize() {
    //PathManager::GetInstance()->LoadPathsFile("/usd/path.json");
}

void AutonManagerNode::autonPeriodic() {
    if(!selected_auton->Complete()) {
        selected_auton->AutonPeriodic();
    }
}