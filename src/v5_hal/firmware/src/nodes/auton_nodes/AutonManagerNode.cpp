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
    m_matchAuton = new MatchAuton(m_drive_node, m_odom_node, m_front_claw_node, m_wing_arm, m_back_claw_node, m_liftNode, m_intakeNode);
    m_matchAuton2 = new MatchAuton2(m_drive_node, m_odom_node, m_front_claw_node, m_wing_arm, m_back_claw_node, m_liftNode, m_intakeNode);
    m_tuningAuton = new tuningRoutine(m_drive_node, m_odom_node);
    m_testAuton = new testAuton(m_liftNode);

    autons.insert(autons.end(), { m_matchAuton, m_matchAuton2, m_tuningAuton, m_testAuton });
    selected_auton = autons.at(0);
    pathJSON = "/usd/paths/path.json";
}

void AutonManagerNode::initialize() {
    PathManager::GetInstance()->LoadPathsFile(pathJSON);
}

void AutonManagerNode::setPathsFile(std::string filename) {
    pathJSON = "/usd/paths/" + filename;
    PathManager::GetInstance()->LoadPathsFile(pathJSON);
}

void AutonManagerNode::autonPeriodic() {
    if(!selected_auton->Complete()) {
        selected_auton->AutonPeriodic();
    }
}