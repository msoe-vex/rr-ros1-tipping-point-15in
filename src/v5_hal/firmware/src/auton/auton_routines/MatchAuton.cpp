#include "auton/auton_routines/MatchAuton.h"

MatchAuton::MatchAuton(IDriveNode* driveNode, OdometryNode* odomNode, IClawNode* frontClawNode, BackClawNode* backClawNode, ILiftNode* liftNode, IRollerIntakeNode* intakeNode) : 
        Auton("15in Match Auton"), 
        m_driveNode(driveNode),
        m_odomNode(odomNode),
        m_frontClawNode(frontClawNode),
        m_backClawNode(backClawNode),
        m_liftNode(liftNode),
        m_intakeNode(intakeNode) {
    
}

void MatchAuton::AddNodes() {
    // Set the starting position, as measured on the field
    Pose startingPose(Vector2d(-44.75, 16.75), Rotation2Dd(1.449));
    m_odomNode->setCurrentPose(startingPose);

    AutonNode* deploy = new AutonNode(0.1, new DeployAction());
    Auton::AddFirstNode(deploy);

    AutonNode* forward = new AutonNode(2, new DriveStraightAction(m_driveNode, m_odomNode, 38, 70, 80));
   
    AutonNode* clawCloseDelay = new AutonNode(1.2, new WaitAction(1.2));

    deploy->AddNext(forward);
    deploy->AddNext(clawCloseDelay);

    AutonNode* clawClose = new AutonNode(0.5, new UseClawAction(m_frontClawNode, true));
    clawCloseDelay ->AddNext(clawClose);

    AutonNode* wait = new AutonNode(0.5, new WaitAction(0.5));
    clawClose->AddNext(wait);

    AutonNode* liftUp = new AutonNode(0.5, new MoveLiftToPositionAction(m_liftNode, 100, 10));
    wait->AddNext(liftUp);

    Path path = PathManager::GetInstance()->GetPath("LeftGoalToWallReverse");
    AutonNode* leftGoalToWall = new AutonNode(10, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(path), path, false));

    AutonNode* backClawOpenDelay = new AutonNode(1.5, new WaitAction(1.5));

    forward->AddNext(leftGoalToWall);
    forward->AddNext(backClawOpenDelay);

    AutonNode* backClawOpen = new AutonNode(0.1, new SetBackClawStateAction(m_backClawNode, BackClawNode::PIVOT_DOWN_CLAW_OPEN));

    backClawOpenDelay->AddNext(backClawOpen);

    AutonNode* backClawClose = new AutonNode(0.1, new SetBackClawStateAction(m_backClawNode, BackClawNode::PIVOT_BACK));
    leftGoalToWall->AddNext(backClawClose);

    Path wallToCornerPath = PathManager::GetInstance()->GetPath("WallToCornerGoalDrop");
    AutonNode* wallToCorner = new AutonNode(4, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(wallToCornerPath), wallToCornerPath, false));

    leftGoalToWall->AddNext(wallToCorner);
    
    AutonNode* frontClawDropNeutral = new AutonNode(0.1, new UseClawAction(m_frontClawNode, false));
    wallToCorner->AddNext(frontClawDropNeutral);

    Path cornerGoalToReversePointPath = PathManager::GetInstance()->GetPath("CornerGoalDropToReversePoint");
    AutonNode* cornerGoalToReversePoint = new AutonNode(4, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(cornerGoalToReversePointPath), cornerGoalToReversePointPath, false));

    AutonNode* delayRingIntake = new AutonNode(1., new WaitAction(1.));

    frontClawDropNeutral->AddNext(cornerGoalToReversePoint);
    frontClawDropNeutral->AddNext(delayRingIntake);

    AutonNode* ringIntake = new AutonNode(0.1, new RollerIntakeAction(m_intakeNode));

    AutonNode* liftUpForRings = new AutonNode(0.1, new MoveLiftToPositionAction(m_liftNode, 300, 10));

    delayRingIntake->AddNext(ringIntake);
    delayRingIntake->AddNext(liftUpForRings);

    Path reversePointToOppositeRingPath = PathManager::GetInstance()->GetPath("ReversePointToOppositeRing");
    AutonNode* reversePointToOppositeRing = new AutonNode(7, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(reversePointToOppositeRingPath), reversePointToOppositeRingPath, false));

    cornerGoalToReversePoint->AddNext(reversePointToOppositeRing);

    Path oppositeRingToReversePointPath = PathManager::GetInstance()->GetPath("OppositeRingToReversePoint");
    AutonNode* oppositeRingToReversePoint = new AutonNode(8, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(oppositeRingToReversePointPath), oppositeRingToReversePointPath, false));
    reversePointToOppositeRing->AddNext(oppositeRingToReversePoint);

    Path reversePointToCornerGoalPath = PathManager::GetInstance()->GetPath("ReversePointToCornerGoalDrop");
    AutonNode* reversePointToCornerGoal = new AutonNode(5, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(reversePointToCornerGoalPath), reversePointToCornerGoalPath, false));

    AutonNode* liftDownForGoal = new AutonNode(0.1, new MoveLiftToPositionAction(m_liftNode, 20, 10));

    oppositeRingToReversePoint->AddNext(reversePointToCornerGoal);
    oppositeRingToReversePoint->AddNext(liftDownForGoal);

    AutonNode* frontClawGrabNeutral = new AutonNode(0.1, new UseClawAction(m_frontClawNode, true));
    reversePointToCornerGoal->AddNext(frontClawGrabNeutral);
}