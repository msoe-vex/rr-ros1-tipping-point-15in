#include "auton/auton_routines/ProgrammingSkillsAuton.h"

ProgrammingSkillsAuton::ProgrammingSkillsAuton(IDriveNode* driveNode, OdometryNode* odomNode, IClawNode* frontClawNode, BackClawNode* backClawNode, IClawNode* WingArmNode, ILiftNode* liftNode, IRollerIntakeNode* intakeNode) : 
        Auton("Programming Skills"), 
        m_driveNode(driveNode),
        m_odomNode(odomNode),
        m_frontClawNode(frontClawNode),
        m_backClawNode(backClawNode),
        m_WingArmNode(WingArmNode),
        m_liftNode(liftNode),
        m_intakeNode(intakeNode) {
    
}

void ProgrammingSkillsAuton::AddNodes() {
    // Set the starting position, as measured on the field
    //Pose startingPose(Vector2d(-48.0, 14.625), Rotation2Dd(M_PI - toRadians(9)));
    //m_odomNode->setCurrentPose(startingPose);

    AutonNode* deploy = new AutonNode(0.1, new DeployAction());
    Auton::AddFirstNode(deploy);

    AutonNode* backClawOpen = new AutonNode(0.1, new SetBackClawStateAction(m_backClawNode, BackClawNode::PIVOT_DOWN_CLAW_OPEN));

    deploy->AddNext(backClawOpen);

    AutonNode* Wait1 = new AutonNode(0.5, new WaitAction(0.5));

    deploy->AddNext(Wait1); 

    Path BackingUpToAllianceGoal1Path = PathManager::GetInstance()->GetPath("BackingUpToAllianceGoal1");
    AutonNode* BackingUpToAllianceGoal1 = new AutonNode(10, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(BackingUpToAllianceGoal1Path), BackingUpToAllianceGoal1Path, true));

    Wait1->AddNext(BackingUpToAllianceGoal1);

    AutonNode* backClawClose = new AutonNode(0.1, new SetBackClawStateAction(m_backClawNode, BackClawNode::PIVOT_BACK));
    BackingUpToAllianceGoal1->AddNext(backClawClose);

    AutonNode* WingArmDown = new AutonNode(0.1, new UseClawAction(m_WingArmNode, true));

    Path GettingToNeutralGoal1Path = PathManager::GetInstance()->GetPath("GettingToNeutralGoal1");
    AutonNode* GettingToNeutralGoal1 = new AutonNode(10, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(GettingToNeutralGoal1Path), GettingToNeutralGoal1Path, false));

    BackingUpToAllianceGoal1->AddNext(GettingToNeutralGoal1);
    BackingUpToAllianceGoal1->AddNext(WingArmDown);


    AutonNode* clawClose = new AutonNode(0.5, new UseClawAction(m_frontClawNode, true));

    Path GettingOnRingLinePath = PathManager::GetInstance()->GetPath("GettingOnRingLine");
    AutonNode* GettingOnRingLine = new AutonNode(10, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(GettingOnRingLinePath), GettingOnRingLinePath, false));

    AutonNode* Wait2 = new AutonNode(0.2, new WaitAction(0.2));

    GettingToNeutralGoal1->AddNext(clawClose);
    GettingToNeutralGoal1->AddNext(GettingOnRingLine);
    GettingToNeutralGoal1->AddNext(Wait2);

    AutonNode* liftUpForRings1 = new AutonNode(0.5, new MoveLiftToPositionAction(m_liftNode, 600, 10));

    Wait2->AddNext(liftUpForRings1);

    // AutonNode* ringIntake = new AutonNode(0.1, new RollerIntakeAction(m_intakeNode));

    // Path ThroughRingsPath = PathManager::GetInstance()->GetPath("ThroughRings");
    // AutonNode* ThroughRings = new AutonNode(10, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(ThroughRingsPath), ThroughRingsPath, false));

    // GettingOnRingLine->AddNext(liftUpForRings);
    // GettingOnRingLine->AddNext(ringIntake);
    // GettingOnRingLine->AddNext(ThroughRings);

    Path NeutralDrop1Path = PathManager::GetInstance()->GetPath("NeutralDrop1");
    AutonNode* NeutralDrop1 = new AutonNode(10, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(NeutralDrop1Path), NeutralDrop1Path, false));

    



    





}                  