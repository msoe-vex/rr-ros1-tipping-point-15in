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

    AutonNode* backClawOpen = new AutonNode(0.1, new SetBackClawStateAction(m_backClawNode, BackClawNode::PIVOT_DOWN_CLAW_OPEN));

    AutonNode* backwardsToAllianceGoal = new AutonNode(2., new DriveStraightAction(m_driveNode, m_odomNode, 6, 70, 80));

    deploy -> AddNext(backClawOpen);

    deploy ->AddNext(backwardsToAllianceGoal);

    AutonNode* backClawClose = new AutonNode(0.1, new SetBackClawStateAction(m_backClawNode, BackClawNode::PIVOT_BACK));
    backwardsToAllianceGoal->AddNext(backClawClose);

    Path GettingToNeutralGoal1Path = PathManager::GetInstance()->GetPath("GettingToNeutralGoal1");
    AutonNode* GettingToNeutralGoal1 = new AutonNode(10, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(GettingToNeutralGoal1Path), GettingToNeutralGoal1Path, false));

    AutonNode* clawClose = new AutonNode(0.5, new UseClawAction(m_frontClawNode, true));

    Path GettingOnRingLinePath = PathManager::GetInstance()->GetPath("GettingOnRingLine");
    AutonNode* GettingOnRingLine = new AutonNode(10, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(GettingOnRingLinePath), GettingOnRingLinePath, false));

    GettingToNeutralGoal1 -> AddNext(clawClose);
    GettingToNeutralGoal1 -> AddNext(GettingOnRingLine);

    AutonNode* liftUpForRings = new AutonNode(0.1, new MoveLiftToPositionAction(m_liftNode, 300, 10));

    AutonNode* ringIntake = new AutonNode(0.1, new RollerIntakeAction(m_intakeNode));

    Path ThroughRingsPath = PathManager::GetInstance()->GetPath("ThroughRings");
    AutonNode* ThroughRings = new AutonNode(10, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(ThroughRingsPath), ThroughRingsPath, false));

    GettingOnRingLine -> AddNext(liftUpForRings);
    GettingOnRingLine -> AddNext(ringIntake);
    GettingOnRingLine -> AddNext(ThroughRings);





}