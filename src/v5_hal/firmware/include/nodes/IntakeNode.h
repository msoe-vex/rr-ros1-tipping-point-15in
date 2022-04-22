#pragma once

#include "lib-rr/nodes/NodeManager.h"
#include "api.h"
#include "lib-rr/nodes/actuator_nodes/MotorNode.h"
#include "lib-rr/nodes/sensor_nodes/ControllerNode.h"
#include "lib-rr/nodes/actuator_nodes/ADIDigitalOutNode.h"
#include "lib-rr/nodes/subsystems/IRollerIntakeNode.h"
#include "lib-rr/util/Constants.h"

class IntakeNode : public IRollerIntakeNode {
private:
    pros::Controller* m_controller;
    MotorNode* m_intake;

    std::string m_handle_name;

    bool m_open = false;

public:
    IntakeNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, MotorNode* intake);

    void setIntakeVoltage(int voltage);

    void setIntakeVelocity(float velocity);

    void initialize();

    void teleopPeriodic();

    void autonPeriodic();

    ~IntakeNode();
};
