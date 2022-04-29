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

    AutonNode* backwardsToAllianceGoal = new AutonNode(2, new DriveStraightAction(m_driveNode, m_odomNode, -1, 10, 10));

    deploy ->AddNext(backwardsToAllianceGoal);

    //code from matchAuton for getting goals with the back claw state machine
    AutonNode* backClawOpen = new AutonNode(0.1, new SetBackClawStateAction(m_backClawNode, BackClawNode::PIVOT_DOWN_CLAW_OPEN));

    backwardsToAllianceGoal -> AddNext(backClawOpen);

    AutonNode* backClawClose = new AutonNode(0.1, new SetBackClawStateAction(m_backClawNode, BackClawNode::PIVOT_BACK));
    backClawOpen->AddNext(backClawClose);

    Path path = PathManager::GetInstance()->GetPath("GettingToNeutralGoal");
    AutonNode* GettingToNeutralGoal = new AutonNode(10, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(path), path, false));

    AutonNode* clawClose = new AutonNode(0.5, new UseClawAction(m_frontClawNode, true));
    GettingToNeutralGoal -> AddNext(clawClose);

    Path path = PathManager::GetInstance()->GetPath("GettingToNeutralGoal");
    AutonNode* GettingToNeutralGoal = new AutonNode(10, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(path), path, false));

    Path GettingToRingLinePath = PathManager::GetInstance()->GetPath("GettingToNeutralGoal");
    AutonNode* GettingToRingLine = new AutonNode(10, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(GettingToRingLinePath), path, false));

    GettingToNeutralGoal -> AddNext(GettingToRingLine);

    AutonNode* liftUpForRings = new AutonNode(0.1, new MoveLiftToPositionAction(m_liftNode, 300, 10));

    AutonNode* ringIntake = new AutonNode(0.1, new RollerIntakeAction(m_intakeNode));

    Path PickingUpRingsPath = PathManager::GetInstance()->GetPath("PickingUpRings");
    AutonNode* PickingUpRings = new AutonNode(10, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(PickingUpRingsPath), path, false));

    GettingToRingLine -> AddNext(liftUpForRings);
    GettingToRingLine -> AddNext(ringIntake);
    GettingToRingLine -> AddNext(PickingUpRings);





}