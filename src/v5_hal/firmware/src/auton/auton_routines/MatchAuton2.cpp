#include "auton/auton_routines/MatchAuton2.h"

MatchAuton2::MatchAuton2(IDriveNode* driveNode, OdometryNode* odomNode, IClawNode* frontClawNode, IClawNode* wingArm, BackClawNode* backClawNode, ILiftNode* liftNode, IRollerIntakeNode* intakeNode) : 
        Auton("15in Match Auton 2"), 
        m_driveNode(driveNode),
        m_odomNode(odomNode),
        m_frontClawNode(frontClawNode),
        m_wingArm(wingArm),
        m_backClawNode(backClawNode),
        m_liftNode(liftNode),
        m_intakeNode(intakeNode) {
    
}

void MatchAuton2::AddNodes() {
    // Set the starting position, as measured on the field
    Pose startingPose(Vector2d(31.917, 14.25), Rotation2Dd(M_PI_2+toRadians(36)));
    m_odomNode->setCurrentPose(startingPose);

    DriveStraightAction::DriveStraightParams driveParams = {
        900.,
        5. / 3.,
        M_PI * 3.25
    };

    AutonNode* deploy = new AutonNode(0.1, new DeployAction());
    Auton::AddFirstNode(deploy);

    AutonNode* liftDownForCenterDash = new AutonNode(0.1, new MoveLiftToPositionAction(m_liftNode, 270, 10));

    AutonNode* wingArmDeploy = new AutonNode(0.1, new UseClawAction(m_wingArm, true));

    AutonNode* forward = new AutonNode(5, new DriveStraightAction(m_driveNode, m_odomNode, driveParams, 41.5, 65, 90));

    deploy->AddNext(forward);
    deploy->AddNext(liftDownForCenterDash);
    deploy->AddNext(wingArmDeploy);

    //AutonNode* backward = new AutonNode(1.5, new DriveStraightAction(m_driveNode, m_odomNode, -20, 70, 80));

    AutonNode* wingArmRetractGrab = new AutonNode(0.1, new UseClawAction(m_wingArm, false));

    //forward->AddNext(backward);
    forward->AddNext(wingArmRetractGrab);

    AutonNode* wingReleaseDelay = new AutonNode(0.7, new WaitAction(0.7));
    // // Helpful at dragging goal to side

    AutonNode* wingArmDeployRelease = new AutonNode(0.1, new UseClawAction(m_wingArm, true));
    AutonNode* backClawOpen = new AutonNode(0.1, new SetBackClawStateAction(m_backClawNode, BackClawNode::PIVOT_DOWN_CLAW_OPEN));

    Path dragMiddleNeutral = PathManager::GetInstance()->GetPath("RipNeutral");
    AutonNode* goalDragToColorGoal = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(dragMiddleNeutral), 
            dragMiddleNeutral, 
            false
        )
    );

    forward->AddNext(goalDragToColorGoal);
    goalDragToColorGoal->AddNext(wingReleaseDelay);
    goalDragToColorGoal->AddNext(backClawOpen);


    wingReleaseDelay->AddNext(wingArmDeployRelease);

    Path toBlueGoal = PathManager::GetInstance()->GetPath("ToBlue");
    AutonNode* ToBlueGoal = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(toBlueGoal), 
            toBlueGoal, 
            false
        )
    );

    wingArmDeployRelease->AddNext(ToBlueGoal);

    AutonNode* backClawCloseColor = new AutonNode(0.1, new SetBackClawStateAction(m_backClawNode, BackClawNode::PIVOT_BACK));

    ToBlueGoal->AddNext(backClawCloseColor);

    AutonNode* clawOpen = new AutonNode(0.5, new UseClawAction(m_frontClawNode, false));
    ToBlueGoal->AddNext(clawOpen);


    Path blueToNeutralLeft = PathManager::GetInstance()->GetPath("ToNeutralLeft");
    AutonNode* toNeutralLeft = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(blueToNeutralLeft), 
            blueToNeutralLeft, 
            false
        )
    );

    ToBlueGoal->AddNext(toNeutralLeft);
    
    AutonNode* clawOpen2 = new AutonNode(0.5, new UseClawAction(m_frontClawNode, false));
    toNeutralLeft->AddNext(clawOpen2);

    //LiftForNeutral

    Path liftForNeutral = PathManager::GetInstance()->GetPath("LiftForNeutral");
    AutonNode* LiftForNeutral = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(liftForNeutral), 
            liftForNeutral, 
            false
        )
    );

    toNeutralLeft->AddNext(LiftForNeutral);

    AutonNode* clawClose = new AutonNode(0.5, new UseClawAction(m_frontClawNode, true));
    LiftForNeutral->AddNext(clawClose);

    AutonNode* liftUp = new AutonNode(0.5, new MoveLiftToPositionAction(m_liftNode, 1800, 10, true));
    clawClose->AddNext(liftUp);

    AutonNode* wingArmRetract = new AutonNode(0.1, new UseClawAction(m_wingArm, false));
    clawClose->AddNext(wingArmRetract);

    AutonNode* wait = new AutonNode(0.5, new WaitAction(0.5));
    wingArmRetract->AddNext(wait);

    AutonNode* liftDown = new AutonNode(0.5, new MoveLiftToPositionAction(m_liftNode, 700, 10));
    wait->AddNext(liftDown);

    AutonNode* ringIntake = new AutonNode(0.1, new RollerIntakeAction(m_intakeNode));

    wait->AddNext(ringIntake);

    Path NeutralToCorner = PathManager::GetInstance()->GetPath("NeutralLeftToCorner");
    AutonNode* toCorner = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(NeutralToCorner), 
            NeutralToCorner, 
            false
        )
    );

    ringIntake->AddNext(toCorner);

    //ToRings

    Path ToRings = PathManager::GetInstance()->GetPath("ToRings");
    AutonNode* toRings = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(ToRings), 
            ToRings, 
            false
        )
    );

    toCorner->AddNext(toRings);

    AutonNode* clawOpen3 = new AutonNode(0.5, new UseClawAction(m_frontClawNode, false));
    toRings->AddNext(clawOpen3);

    AutonNode* quickReverseRingIntake = new AutonNode(0.1, new RollerIntakeAction(m_intakeNode, -12000));

    clawOpen3->AddNext(quickReverseRingIntake);

    AutonNode* wait2 = new AutonNode(0.5, new WaitAction(0.5));
    quickReverseRingIntake->AddNext(wait2);

    AutonNode* ringIntake2 = new AutonNode(0.1, new RollerIntakeAction(m_intakeNode));

    wait2->AddNext(ringIntake2);

    //If No goal, get the other neutral goal

    //ToCollectHandRings


    Path ToHandCollect = PathManager::GetInstance()->GetPath("ToCollectHandRings");
    AutonNode* toHandCollect = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(ToHandCollect), 
            ToHandCollect, 
            false
        )
    );

    clawOpen3->AddNext(toHandCollect);

    AutonNode* preloads1 = getPreloadsSequence(toHandCollect, m_driveNode, m_odomNode);

    AutonNode* preloads2 = getPreloadsSequence(preloads1, m_driveNode, m_odomNode);
    
    AutonNode* preloads3 = getPreloadsSequence(preloads2, m_driveNode, m_odomNode);

    AutonNode* conveyorStop2 = new AutonNode(0.1, new RollerIntakeAction(m_intakeNode, 0));

    preloads3->AddNext(conveyorStop2);

}