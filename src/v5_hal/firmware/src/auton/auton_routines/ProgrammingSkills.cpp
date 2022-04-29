#include "auton/auton_routines/ProgrammingSkillsAuton.h"

ProgrammingSkillsAuton::ProgrammingSkillsAuton(IDriveNode* driveNode, OdometryNode* odomNode, IClawNode* frontClawNode, BackClawNode* backClawNode, ILiftNode* liftNode, IRollerIntakeNode* intakeNode) : 
        Auton("15in Match Auton"), 
        m_driveNode(driveNode),
        m_odomNode(odomNode),
        m_frontClawNode(frontClawNode),
        m_backClawNode(backClawNode),
        m_liftNode(liftNode),
        m_intakeNode(intakeNode) {
    
}

void ProgrammingSkillsAuton::AddNodes() {
    // Set the starting position, as measured on the field
    Pose startingPose(Vector2d(-44.75, 16.75), Rotation2Dd(1.449));
    m_odomNode->setCurrentPose(startingPose);

    AutonNode* deploy = new AutonNode(0.1, new DeployAction());
    Auton::AddFirstNode(deploy);

}