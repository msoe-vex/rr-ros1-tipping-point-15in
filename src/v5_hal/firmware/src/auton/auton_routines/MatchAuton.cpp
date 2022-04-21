#include "auton/auton_routines/MatchAuton.h"

MatchAuton::MatchAuton(IDriveNode* driveNode, OdometryNode* odomNode, IClawNode* clawNode, ILiftNode* liftNode) : 
        Auton("15in Match Auton"), 
        m_driveNode(driveNode),
        m_odomNode(odomNode),
        m_clawNode(clawNode),
        m_liftNode(liftNode) {
    
}

void MatchAuton::AddNodes() {
    // AutonNode* forwardNode = new AutonNode(10, new DriveStraightAction(m_drive_node, 30, 50, 10));
    // Auton::AddFirstNode(forwardNode);
    AutonNode* clawCloseNode = new AutonNode(0.5, new UseClawAction(m_clawNode, true));
    Auton::AddFirstNode(clawCloseNode);

    AutonNode* waitNode = new AutonNode(0.5, new WaitAction(0.5));
    clawCloseNode->AddNext(waitNode);

    AutonNode* liftUpNode = new AutonNode(0.5, new LiftVelocityAction(m_liftNode, MAX_VELOCITY, 0.25));

    Path path = PathManager::GetInstance()->GetPath("LeftGoalToWall");
    AutonNode* leftGoalToWall = new AutonNode(10, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(path), path, true));
    waitNode->AddNext(liftUpNode);
    waitNode->AddNext(leftGoalToWall);
}