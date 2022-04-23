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

    AutonNode* forward = new AutonNode(2, new DriveStraightAction(m_driveNode, m_odomNode, 41, 70, 80));

    deploy->AddNext(forward);

    AutonNode* clawClose = new AutonNode(0.5, new UseClawAction(m_frontClawNode, true));
    forward ->AddNext(clawClose);

    AutonNode* wait = new AutonNode(0.5, new WaitAction(0.5));
    clawClose->AddNext(wait);

    AutonNode* liftUp = new AutonNode(0.5, new MoveLiftToPositionAction(m_liftNode, 100, 10));
    wait->AddNext(liftUp);

    Path path = PathManager::GetInstance()->GetPath("LeftGoalToWallReverse");
    AutonNode* leftGoalToWall = new AutonNode(10, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(path), path, false));

    AutonNode* backClawOpenDelay = new AutonNode(1, new WaitAction(1));

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

    AutonNode* delayRingIntake = new AutonNode(1.5, new WaitAction(1.5));

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

    Path reversePointToCornerGoalPath = PathManager::GetInstance()->GetPath("ReversePointToWallRingPickup");
    AutonNode* reversePointToCornerGoal = new AutonNode(5, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(reversePointToCornerGoalPath), reversePointToCornerGoalPath, false));

    // AutonNode* liftDownForGoal = new AutonNode(0.1, new MoveLiftToPositionAction(m_liftNode, 20, 10));

    oppositeRingToReversePoint->AddNext(reversePointToCornerGoal);
    // oppositeRingToReversePoint->AddNext(liftDownForGoal);

    AutonNode* driveForwardForRings = new AutonNode(1.5, new DriveStraightAction(m_driveNode, m_odomNode, 15, 10, 80));
    reversePointToCornerGoal->AddNext(driveForwardForRings);

    AutonNode* driveBackward = new AutonNode(1.5, new DriveStraightAction(m_driveNode, m_odomNode, -15, 10, 80));
    driveForwardForRings->AddNext(driveBackward);

    AutonNode* driveForwardForRings2 = new AutonNode(1.5, new DriveStraightAction(m_driveNode, m_odomNode, 15, 10, 80));
    driveBackward->AddNext(driveForwardForRings2);

    AutonNode* driveBackward2 = new AutonNode(1.5, new DriveStraightAction(m_driveNode, m_odomNode, -15, 10, 80));
    driveForwardForRings2->AddNext(driveBackward2);

    AutonNode* driveForwardForRings3 = new AutonNode(1.5, new DriveStraightAction(m_driveNode, m_odomNode, 15, 10, 80));
    driveBackward2->AddNext(driveForwardForRings3);

    AutonNode* driveBackward3 = new AutonNode(1.5, new DriveStraightAction(m_driveNode, m_odomNode, -15, 10, 80));
    driveForwardForRings3->AddNext(driveBackward3);

    AutonNode* driveForwardForRings4 = new AutonNode(1.5, new DriveStraightAction(m_driveNode, m_odomNode, 15, 10, 80));
    driveBackward3->AddNext(driveForwardForRings4);

    AutonNode* driveBackward4 = new AutonNode(1.5, new DriveStraightAction(m_driveNode, m_odomNode, -15, 10, 80));
    driveForwardForRings4->AddNext(driveBackward4);

    AutonNode* driveForwardForRings5 = new AutonNode(1.5, new DriveStraightAction(m_driveNode, m_odomNode, 15, 10, 80));
    driveBackward4->AddNext(driveForwardForRings5);

    AutonNode* driveBackward5 = new AutonNode(1.5, new DriveStraightAction(m_driveNode, m_odomNode, -15, 10, 80));
    driveForwardForRings5->AddNext(driveBackward5);

    AutonNode* driveForwardForRings6 = new AutonNode(1.5, new DriveStraightAction(m_driveNode, m_odomNode, 15, 10, 80));
    driveBackward5->AddNext(driveForwardForRings6);

    Path wallRingPickupToGoalReversePointPath = PathManager::GetInstance()->GetPath("WallRingPickupToGoalReversePoint");
    AutonNode* wallRingPickupToGoalReversePoint = new AutonNode(5, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(wallRingPickupToGoalReversePointPath), wallRingPickupToGoalReversePointPath, false));

    AutonNode* liftDownForGoalPickup = new AutonNode(0.1, new MoveLiftToPositionAction(m_liftNode, 20, 20));

    driveForwardForRings6->AddNext(wallRingPickupToGoalReversePoint);
    driveForwardForRings6->AddNext(liftDownForGoalPickup);

    Path goalReversePointToCornerGoalPath = PathManager::GetInstance()->GetPath("GoalReversePointToCornerGoal");
    AutonNode* goalReversePointToCornerGoal = new AutonNode(3, new FollowPathAction(m_driveNode, m_odomNode, new TankPathPursuit(goalReversePointToCornerGoalPath), goalReversePointToCornerGoalPath, false));

    wallRingPickupToGoalReversePoint->AddNext(goalReversePointToCornerGoal);

    AutonNode* neutralGoalGrab = new AutonNode(0.5, new UseClawAction(m_frontClawNode, true));

    wallRingPickupToGoalReversePoint->AddNext(neutralGoalGrab);

    // AutonNode* frontClawGrabNeutral = new AutonNode(0.1, new UseClawAction(m_frontClawNode, true));
    // reversePointToCornerGoal->AddNext(frontClawGrabNeutral);
}