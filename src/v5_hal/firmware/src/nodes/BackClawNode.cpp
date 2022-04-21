#include "nodes/BackClawNode.h"

BackClawNode::BackClawNode(NodeManager* node_manager, std::string handle_name,
        ControllerNode* controller, pros::controller_digital_e_t pivotButton, 
        pros::controller_digital_e_t clawButton, ADIDigitalOutNode* pivot
        ADIDigitalOutNode* claw) : Node(node_manager, 5),
        m_controller(controller->getController()),
        m_pivotButton(pivotButtton),
        m_clawButton(clawButton),
        m_pivot(pivot),
        m_claw(claw) {
    m_handle_name = handle_name.insert(0, "robot/");
}


void BackClawNode::initialize() {
    m_state = PIVOT_BACK;
}

void BackClawNode::setState(BackClawState state) {
    m_state = state;
}

// if the claw is pivotted back, pivot it forward and open the claw
// if not, pivot it back
void BackClawNode::togglePivot() {
    if (m_state == PIVOT_BACK) {
        m_state = PIVOT_DOWN_CLAW_OPEN;
    } else {
        m_state = PIVOT_BACK;
    }
}

// if the claw is pivotted back, pivot it forward and open the claw
// if not, toggle the position of the claw
void BackClawNode::toggleClaw() {
if (m_state == PIVOT_BACK) {
        m_state = PIVOT_DOWN_CLAW_OPEN;
    } else {
        if (m_state == PIVOT_DOWN_CLAW_OPEN) {
            m_state = PIVOT_DOWN_CLAW_CLOSED;
        } else {
            m_state = PIVOT_DOWN_CLAW_OPEN;
        }
    }
}

void BackClawNode::teleopPeriodic() {
    periodic();

    // this logic is the exact same as ClawNode I wonder 
    // how we could combine the two
    bool pivotButtonCurrentState = m_controller->get_digital(m_pivotButton);
    bool clawButtonCurrentState = m_controller->get_digital(m_clawButton);

    // there feels like there is a better way to do the state logic when a button is pressed
	if (pivotButtonCurrentState == 1 && m_pivotButtonPreivousState == 0) {
        // pivot button has been pressed
        // if the claw is pivotted back, pivot it forward and open the claw
        // if not, pivot it back
        togglePivot();
    }

    if (clawButtonCurrentState == 1 && m_clawButtonPreviousState == 0) {
        // claw button has been pressed
        // if the claw is pivotted back, pivot it forward and open the claw
        // if not, toggle the position of the claw
        toggleClaw();
    }

	m_pivotButtonPreivousState = pivotButtonCurrentState;
    m_clawButtonPreviousState = clawButtonCurrentState;
}

void BackClawNode::autonPeriodic() {
    periodic();
}

// these ones and zeros need to be tested
// I am assuming that 1 is piston retracted 
void BackClawNode::periodic() {
    switch (m_state)
    {
    case PIVOT_BACK:
        m_pivot->setValue(1);
        m_claw->setValue(1);
    break;

    case PIVOT_DOWN_CLAW_OPEN:
        m_pivot->setValue(0);
        m_claw->setValue(0);
    break;

    case PIVOT_DOWN_CLAW_CLOSED:
        m_pivot->setValue(0);
        m_claw->setValue(1);
    break;
    
    default:
        break;
    }
}

BackClawNode::~BackClawNode() {

}
