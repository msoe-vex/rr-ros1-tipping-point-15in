#include "auton/auton_routines/LeftQuickAuton.h"

LeftQuickAuton::LeftQuickAuton(IDriveNode* driveNode, OdometryNode* odomNode, IClawNode* frontClawNode, IClawNode* wingArm, BackClawNode* backClawNode, ILiftNode* liftNode, IRollerIntakeNode* intakeNode) : 
        Auton("15in Match Auton"), 
        m_driveNode(driveNode),
        m_odomNode(odomNode),
        m_frontClawNode(frontClawNode),
        m_wingArm(wingArm),
        m_backClawNode(backClawNode),
        m_liftNode(liftNode),
        m_intakeNode(intakeNode) {
    
}

void LeftQuickAuton::AddNodes() {
    // Set the starting position, as measured on the field
    Pose startingPose(Vector2d(-45, 17.375), Rotation2Dd(M_PI_2));
    m_odomNode->setCurrentPose(startingPose);

    AutonNode* deploy = new AutonNode(0.1, new DeployAction());
    Auton::AddFirstNode(deploy);

    AutonNode* liftDownForCenterDash = new AutonNode(0.1, new MoveLiftToPositionAction(m_liftNode, 270, 10));

    AutonNode* wingArmDeploy = new AutonNode(0.1, new UseClawAction(m_wingArm, true));

    AutonNode* forward = new AutonNode(2, new DriveStraightAction(m_driveNode, m_odomNode, DRIVE_CONFIG, 24, 70, 80));

    deploy->AddNext(forward);
    deploy->AddNext(liftDownForCenterDash);
    deploy->AddNext(wingArmDeploy);

    AutonNode* backClawOpen = new AutonNode(0.1, new SetBackClawStateAction(m_backClawNode, BackClawNode::PIVOT_DOWN_CLAW_OPEN));

    AutonNode* backward = new AutonNode(1.5, new DriveStraightAction(m_driveNode, m_odomNode, DRIVE_CONFIG, -20, 70, 80));

    AutonNode* wingArmRetractGrab = new AutonNode(0.1, new UseClawAction(m_wingArm, false));

    forward->AddNext(backClawOpen);
    forward->AddNext(backward);
    forward->AddNext(wingArmRetractGrab);

    AutonNode* wingReleaseDelay = new AutonNode(0.7, new WaitAction(0.7));
    // Helpful at dragging goal to side

    AutonNode* wingArmDeployRelease = new AutonNode(0.1, new UseClawAction(m_wingArm, true));

    Path goalDragToColorGoalPath = PathManager::GetInstance()->GetPath("GoalDragToColorGoal");
    AutonNode* goalDragToColorGoal = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(goalDragToColorGoalPath), 
            goalDragToColorGoalPath, 
            false
        )
    );

    backward->AddNext(wingReleaseDelay);
    backward->AddNext(goalDragToColorGoal);

    wingReleaseDelay->AddNext(wingArmDeployRelease);

    AutonNode* backClawCloseColor = new AutonNode(0.1, new SetBackClawStateAction(m_backClawNode, BackClawNode::PIVOT_BACK));

    goalDragToColorGoal->AddNext(backClawCloseColor);

    AutonNode* colorGoalGrabDelay = new AutonNode(0.5, new WaitAction(0.5));

    AutonNode* delayRingIntake = new AutonNode(1., new WaitAction(1.));

    AutonNode* wingArmDeployRetract = new AutonNode(0.1, new UseClawAction(m_wingArm, false));

    Path colorGoalToOppositeRingPath = PathManager::GetInstance()->GetPath("ColorGoalToOppositeRing");
    AutonNode* colorGoalToOppositeRing = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(colorGoalToOppositeRingPath), 
            colorGoalToOppositeRingPath, 
            false
        )
    );

    backClawCloseColor->AddNext(colorGoalGrabDelay);
    backClawCloseColor->AddNext(wingArmDeployRetract);
    
    colorGoalGrabDelay->AddNext(colorGoalToOppositeRing);
    colorGoalGrabDelay->AddNext(delayRingIntake);

    AutonNode* ringIntake = new AutonNode(0.1, new RollerIntakeAction(m_intakeNode));

    AutonNode* liftUpForRings = new AutonNode(0.1, new MoveLiftToPositionAction(m_liftNode, 700, 10));

    delayRingIntake->AddNext(ringIntake);
    delayRingIntake->AddNext(liftUpForRings);

    Path oppositeRingToWallRingPath = PathManager::GetInstance()->GetPath("OppositeRingToWallRing");
    AutonNode* oppositeRingToWallRing = new AutonNode(
        4, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(oppositeRingToWallRingPath), 
            oppositeRingToWallRingPath, 
            false
        )
    );

    colorGoalToOppositeRing->AddNext(oppositeRingToWallRing);

    AutonNode* liftUpForWallRings = new AutonNode(0.1, new MoveLiftToPositionAction(m_liftNode, 1800, 10));

    Path wallRingForwardPath = PathManager::GetInstance()->GetPath("WallRingForward");
    AutonNode* wallRingForward = new AutonNode(
        6, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(wallRingForwardPath), 
            wallRingForwardPath, 
            false
        )
    );

    oppositeRingToWallRing->AddNext(liftUpForWallRings);
    oppositeRingToWallRing->AddNext(wallRingForward);

    AutonNode* liftDownDelay = new AutonNode(0.75, new WaitAction(0.75));

    AutonNode* liftDownForGoal = new AutonNode(0.1, new MoveLiftToPositionAction(m_liftNode, 270, 10));

    AutonNode* conveyorStop = new AutonNode(0.1, new RollerIntakeAction(m_intakeNode, 0));

    Path wallRingReversePath = PathManager::GetInstance()->GetPath("WallRingReverse");
    AutonNode* wallRingReverse = new AutonNode(
        6, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(wallRingReversePath), 
            wallRingReversePath, 
            false
        )
    );

    wallRingForward->AddNext(wallRingReverse);
    wallRingForward->AddNext(liftDownDelay);

    liftDownDelay->AddNext(liftDownForGoal);
    liftDownDelay->AddNext(conveyorStop);

    AutonNode* wingDownForFunnel = new AutonNode(0.1, new UseClawAction(m_wingArm, true));

    Path neutralGoalGrabArcPath = PathManager::GetInstance()->GetPath("NeutralGoalGrabArc");
    AutonNode* neutralGoalGrabArc = new AutonNode(
        6, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(neutralGoalGrabArcPath), 
            neutralGoalGrabArcPath, 
            false
        )
    );

    wallRingReverse->AddNext(neutralGoalGrabArc);
    wallRingReverse->AddNext(wingDownForFunnel);

    AutonNode* neutralGoalGrab = new AutonNode(0.1, new UseClawAction(m_frontClawNode, true));

    neutralGoalGrabArc->AddNext(neutralGoalGrab);

    Path neutralGoalReversePath = PathManager::GetInstance()->GetPath("NeutralGoalReverse");
    AutonNode* neutralGoalReverse = new AutonNode(
        4, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(neutralGoalReversePath), 
            neutralGoalReversePath, 
            false
        )
    );

    neutralGoalReverse->AddAction(new MoveLiftToPositionAction(m_liftNode, 550, 10));

    neutralGoalGrab->AddNext(neutralGoalReverse);

    AutonNode* liftUpToReleaseArm = new AutonNode(4, new MoveLiftToPositionAction(m_liftNode, 1800, 10, true));

    liftUpToReleaseArm->AddAction(new UseClawAction(m_wingArm, false));

    neutralGoalReverse->AddNext(liftUpToReleaseArm);

    AutonNode* liftDownWithGoal = new AutonNode(4, new MoveLiftToPositionAction(m_liftNode, 270, 10, true));

    liftUpToReleaseArm->AddNext(liftDownWithGoal);

    Path neutralGoalCornerPath = PathManager::GetInstance()->GetPath("NeutralGoalCorner");
    AutonNode* neutralGoalCorner = new AutonNode(
        6, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(neutralGoalCornerPath), 
            neutralGoalCornerPath, 
            false
        )
    );

    liftDownWithGoal->AddNext(neutralGoalCorner);

    AutonNode* liftDownForNeutralGoal = new AutonNode(0.1, new MoveLiftToPositionAction(m_liftNode, 270, 10));

    neutralGoalCorner->AddNext(liftDownForNeutralGoal);

    AutonNode* releaseNeutralGoalCorner = new AutonNode(0.1, new UseClawAction(m_frontClawNode, false));

    liftDownForNeutralGoal->AddNext(releaseNeutralGoalCorner);

    Path cornerGoalDropToReversePointPath = PathManager::GetInstance()->GetPath("CornerGoalDropToReversePoint");
    AutonNode* cornerGoalDropToReversePoint = new AutonNode(
        6, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(cornerGoalDropToReversePointPath), 
            cornerGoalDropToReversePointPath, 
            false
        )
    );

    releaseNeutralGoalCorner->AddNext(cornerGoalDropToReversePoint);

    Path reversePointToWallRingPickupPath = PathManager::GetInstance()->GetPath("ReversePointToWallRingPickup");
    AutonNode* reversePointToWallRingPickup = new AutonNode(
        5, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(reversePointToWallRingPickupPath), 
            reversePointToWallRingPickupPath, 
            false
        )
    );

    reversePointToWallRingPickup->AddAction(new RollerIntakeAction(m_intakeNode));

    AutonNode* liftUpForPreloads = new AutonNode(0.1, new MoveLiftToPositionAction(m_liftNode, 1700, 10));

    cornerGoalDropToReversePoint->AddNext(reversePointToWallRingPickup);
    cornerGoalDropToReversePoint->AddNext(liftUpForPreloads);

    AutonNode* preloads1 = getPreloadsSequence(reversePointToWallRingPickup, m_driveNode, m_odomNode); 
    
    AutonNode* preloads2 = getPreloadsSequence(preloads1, m_driveNode, m_odomNode);

    Path wallRingPickupToGoalReversePointPath = PathManager::GetInstance()->GetPath("WallRingPickupToGoalReversePoint");
    AutonNode* wallRingPickupToGoalReversePoint = new AutonNode(
        5, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(wallRingPickupToGoalReversePointPath), 
            wallRingPickupToGoalReversePointPath, 
            false
        )
    );

    wallRingPickupToGoalReversePoint->AddAction(new RollerIntakeAction(m_intakeNode, 0));

    preloads2->AddNext(wallRingPickupToGoalReversePoint);

    AutonNode* liftDownForCornerGoal = new AutonNode(0.1, new MoveLiftToPositionAction(m_liftNode, 270, 10));

    wallRingPickupToGoalReversePoint->AddNext(liftDownForCornerGoal);
    
    Path goalReversePointToCornerGoalPath = PathManager::GetInstance()->GetPath("GoalReversePointToCornerGoal");
    AutonNode* goalReversePointToCornerGoal = new AutonNode(
        5, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(goalReversePointToCornerGoalPath), 
            goalReversePointToCornerGoalPath, 
            false
        )
    );

    liftDownForCornerGoal->AddNext(goalReversePointToCornerGoal);

    AutonNode* neutralGoalGrabCorner = new AutonNode(0.1, new UseClawAction(m_frontClawNode, true));

    goalReversePointToCornerGoal->AddNext(neutralGoalGrabCorner);
}