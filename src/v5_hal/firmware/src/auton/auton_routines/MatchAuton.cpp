#include "auton/auton_routines/MatchAuton.h"

MatchAuton::MatchAuton(IDriveNode* driveNode, OdometryNode* odomNode, IClawNode* frontClawNode, BackClawNode* backClawNode, ILiftNode* liftNode) : 
        Auton("15in Match Auton"), 
        m_driveNode(driveNode),
        m_odomNode(odomNode),
        m_frontClawNode(frontClawNode),
        m_backClawNode(backClawNode),
        m_liftNode(liftNode) {
    
}

void MatchAuton::AddNodes() {
    // Set the starting position, as measured on the field
    Pose startingPose(Vector2d(-44.75, 16.75), Rotation2Dd(1.449));
    m_odomNode->setCurrentPose(startingPose);

    AutonNode* deploy = new AutonNode(0.1, new DeployAction());
    Auton::AddFirstNode(deploy);

    AutonNode* forward = new AutonNode(2, new DriveStraightAction(m_driveNode, m_odomNode, 38, 70, 80));
   
    AutonNode* clawCloseDelay = new AutonNode(1.1, new WaitAction(1.1));

    deploy->AddNext(forward);
    deploy->AddNext(clawCloseDelay);

    AutonNode* clawClose = new AutonNode(0.5, new UseClawAction(m_frontClawNode, true));
    clawCloseDelay ->AddNext(clawClose);

    AutonNode* wait = new AutonNode(0.5, new WaitAction(0.5));
    clawClose->AddNext(wait);

    AutonNode* liftUp = new AutonNode(0.5, new MoveLiftToPositionAction(m_liftNode, 100, 10));
    wait->AddNext(liftUp);

    Path path = PathManager::GetInstance()->GetPath("LeftGoalToWall");
    AutonNode* leftGoalToWall = new AutonNode(4.9, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(path), path, false));

    AutonNode* backClawOpenDelay = new AutonNode(2, new WaitAction(2.));

    forward->AddNext(leftGoalToWall);
    forward->AddNext(backClawOpenDelay);

    AutonNode* backClawOpen = new AutonNode(0.1, new SetBackClawStateAction(m_backClawNode, BackClawNode::PIVOT_DOWN_CLAW_OPEN));

    backClawOpenDelay->AddNext(backClawOpen);

    AutonNode* backClawClose = new AutonNode(0.1, new SetBackClawStateAction(m_backClawNode, BackClawNode::PIVOT_BACK));
    leftGoalToWall->AddNext(backClawClose);
}