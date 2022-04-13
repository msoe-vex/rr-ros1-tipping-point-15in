#pragma once

#include "lib-rr/auton/Auton.h"
#include "lib-rr/nodes/subsystems/IDriveNode.h"
#include "lib-rr/nodes/sensor_nodes/InertialSensorNode.h"
#include "lib-rr/auton/auton_actions/DriveStraightAction.h"
#include "lib-rr/util/Constants.h"
#include "lib-rr/eigen/Eigen/Dense"

class tuningRoutine : public Auton {
public:
    tuningRoutine(IDriveNode* drive_node);

    void AddNodes();

private:
    IDriveNode* m_drive_node;

    AutonNode* m_forward_node;
    //AutonNode* m_backward_node;
};