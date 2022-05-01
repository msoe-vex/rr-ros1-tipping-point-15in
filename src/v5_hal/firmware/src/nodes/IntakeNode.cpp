#include "nodes/IntakeNode.h"

IntakeNode::IntakeNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, 
pros::controller_digital_e_t intake_button, pros::controller_digital_e_t outtake_button, 
        MotorNode* left_intake) : IRollerIntakeNode(node_manager, handle_name), 
        m_controller(controller->getController()),
        m_intake(left_intake),
        m_state(HOLDING),
        m_intakeButton(intake_button),
        m_outtakeButton(outtake_button) {
    m_handle_name = handle_name.insert(0, "robot/");
}

void IntakeNode::setIntakeVoltage(int voltage) {
    m_intake->moveVoltage(voltage);
}

void IntakeNode::setIntakeVelocity(float velocity) {
    m_intake->moveVelocity(velocity);
}

void IntakeNode::initialize() {

}

void IntakeNode::teleopPeriodic() {

    bool IntakeButtonCurrentState = m_controller->get_digital(m_intakeButton);
    bool OuttakeButtonCurrentState = m_controller->get_digital(m_outtakeButton);

    switch (m_state) {
        case HOLDING:
            setIntakeVoltage(0);
            if(IntakeButtonCurrentState == 1 && OuttakeButtonCurrentState ==1) {
                break;
            } else if (IntakeButtonCurrentState  == 1 && OuttakeButtonCurrentState == 0) {
                m_state = INTAKING;
                break;
            } else if (IntakeButtonCurrentState  == 0 && OuttakeButtonCurrentState == 1) {
                m_state = OUTTAKING;
                break;
            }
        case INTAKING:
            setIntakeVoltage(MAX_MOTOR_VOLTAGE);
            if(IntakeButtonCurrentState == 1 && OuttakeButtonCurrentState ==1) {
                break;
            } else if (IntakeButtonCurrentState  == 1 && OuttakeButtonCurrentState == 0) {
                m_state = HOLDING;
                break;
            } else if (IntakeButtonCurrentState  == 0 && OuttakeButtonCurrentState == 1) {
                m_state = OUTTAKING;
                break;
            }
        case OUTTAKING:
            setIntakeVoltage(-1 * MAX_MOTOR_VOLTAGE);
            if(IntakeButtonCurrentState == 1 && OuttakeButtonCurrentState ==1) {
                break;
            } else if (IntakeButtonCurrentState  == 1 && OuttakeButtonCurrentState == 0) {
                m_state = INTAKING;
                break;
            } else if (IntakeButtonCurrentState  == 0 && OuttakeButtonCurrentState == 1) {
                m_state = HOLDING;
                break;
            }
        default:
            break;
    }
}

void IntakeNode::autonPeriodic() {
    
}

IntakeNode::~IntakeNode() {
    delete m_intake;
}