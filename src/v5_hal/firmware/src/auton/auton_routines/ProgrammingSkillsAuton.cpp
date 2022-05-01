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

    AutonNode* Wait0 = new AutonNode(0.5, new WaitAction(0.5));


    AutonNode* frontClawClose = new AutonNode(0.5, new UseClawAction(m_frontClawNode, false));

    deploy->AddNext(backClawOpen);

    deploy->AddNext(frontClawClose);

    deploy->AddNext(Wait0); 

    Path GetBlueRampGoalPath = PathManager::GetInstance()->GetPath("GetBlueRampGoal");
    AutonNode*  GetBlueRampGoal = new AutonNode(
        10,
        new FollowPathAction(
            m_driveNode,
            m_odomNode,
            new TankPathPursuit(GetBlueRampGoalPath), 
            GetBlueRampGoalPath,
            true
            )
        );

    Wait0->AddNext(GetBlueRampGoal);

    AutonNode* backClawClose = new AutonNode(0.1, new SetBackClawStateAction(m_backClawNode, BackClawNode::PIVOT_BACK));
    
    AutonNode* Wait1 = new AutonNode(0.5, new WaitAction(0.5));
    
    GetBlueRampGoal->AddNext(backClawClose);
    GetBlueRampGoal->AddNext(Wait1);

    AutonNode* WingArmDown = new AutonNode(0.1, new UseClawAction(m_WingArmNode, true));

    Path ToNeutralGoalLeftPath = PathManager::GetInstance()->GetPath("ToNeutralGoalLeft");
    AutonNode* ToNeutralGoalLeft = new AutonNode(
        10,
        new FollowPathAction(
            m_driveNode,
            m_odomNode, 
            new TankPathPursuit(
                ToNeutralGoalLeftPath), 
                ToNeutralGoalLeftPath,
                false
            )
        );

    Wait1->AddNext(ToNeutralGoalLeft);
    Wait1->AddNext(WingArmDown);

    AutonNode* clawClose = new AutonNode(0.5, new UseClawAction(m_frontClawNode, true));

    ToNeutralGoalLeft->AddNext(clawClose);

    AutonNode* Wait2 = new AutonNode(0.5, new WaitAction(0.5));

    clawClose->AddNext(Wait2);    

    AutonNode* liftUpForRelease = new AutonNode(0.5, new MoveLiftToPositionAction(m_liftNode, 1850, 10, true));

    Wait2->AddNext(liftUpForRelease);


    AutonNode* WingArmUp = new AutonNode(0.1, new UseClawAction(m_WingArmNode, false));
    
    liftUpForRelease->AddNext(WingArmUp);

    AutonNode* Wait3 = new AutonNode(0.5, new WaitAction(0.5));
    
    WingArmUp->AddNext(Wait3);
    
    AutonNode* BringDownForDrive = new AutonNode(1., new MoveLiftToPositionAction(m_liftNode, 700, 10, true));

    Wait3->AddNext(BringDownForDrive);

    Path BackFromNeutralLeftPath = PathManager::GetInstance()->GetPath("BackFromNeutralLeft");
     AutonNode* BackFromNeutralLeft = new AutonNode(
         10, 
         new FollowPathAction(
             m_driveNode, 
             m_odomNode, 
             new TankPathPursuit(
                 BackFromNeutralLeftPath),
                 BackFromNeutralLeftPath,
                 false
            )
        );

    BringDownForDrive->AddNext(BackFromNeutralLeft);

    AutonNode* ringIntake = new AutonNode(0.1, new RollerIntakeAction(m_intakeNode));

    Path CollectRingsLeftPath = PathManager::GetInstance()->GetPath("CollectRingsLeft");
    AutonNode* CollectRingsLeft = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(
                CollectRingsLeftPath), 
                CollectRingsLeftPath, 
                false
            )
        );

    BackFromNeutralLeft->AddNext(ringIntake);
    BackFromNeutralLeft->AddNext(CollectRingsLeft);

    AutonNode* RaiseLeftToScore = new AutonNode(0.5, new MoveLiftToPositionAction(m_liftNode, 1800, 10, true));

    CollectRingsLeft->AddNext(RaiseLeftToScore);


    Path PlatformLeftPath = PathManager::GetInstance()->GetPath("PlatformLeft");
    AutonNode* PlatformLeft = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(
                PlatformLeftPath), 
                PlatformLeftPath, 
                false
            )
        );

    RaiseLeftToScore->AddNext(PlatformLeft);

    AutonNode* UnTip = new AutonNode(0.5, new MoveLiftToPositionAction(m_liftNode, 1400, 10, true));

    PlatformLeft->AddNext(UnTip);

    AutonNode* releaseLeft = new AutonNode(0.5, new UseClawAction(m_frontClawNode, false));

    UnTip->AddNext(releaseLeft);

    AutonNode* ClearRamp = new AutonNode(0.5, new MoveLiftToPositionAction(m_liftNode, 1600, 10, true));

    releaseLeft->AddNext(ClearRamp);

    Path PlatformLeftBackPath = PathManager::GetInstance()->GetPath("PlatformLeftBack");
    AutonNode* PlatformLeftBack = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(
                PlatformLeftBackPath), 
                PlatformLeftBackPath, 
                false
            )
        );

    ClearRamp->AddNext(PlatformLeftBack);

    AutonNode* dropBlueGoal = new AutonNode(0.1, new SetBackClawStateAction(m_backClawNode, BackClawNode::PIVOT_DOWN_CLAW_OPEN));

    PlatformLeftBack->AddNext(dropBlueGoal);


    AutonNode* lowerFront = new AutonNode(0.5, new MoveLiftToPositionAction(m_liftNode, 275, 10));

    AutonNode* ringIntakeOff = new AutonNode(0.1, new RollerIntakeAction(m_intakeNode, 0));

    Path GetRedGoalLeftPath = PathManager::GetInstance()->GetPath("GetRedGoalLeft");
    AutonNode* GetRedGoalLeft = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(
                GetRedGoalLeftPath), 
                GetRedGoalLeftPath, 
                false
            )
        );

    dropBlueGoal->AddNext(ringIntakeOff);
    dropBlueGoal->AddNext(GetRedGoalLeft);
    dropBlueGoal->AddNext(lowerFront);

    Path GetRedGoalLeftBackPath = PathManager::GetInstance()->GetPath("GetRedGoalLeftBack");
    AutonNode* GetRedGoalLeftBack = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(
                GetRedGoalLeftBackPath), 
                GetRedGoalLeftBackPath, 
                false
            )
        );
    
    GetRedGoalLeft->AddNext(GetRedGoalLeftBack);

    AutonNode* pickUpRedGoal = new AutonNode(0.1, new SetBackClawStateAction(m_backClawNode, BackClawNode::PIVOT_BACK));

    GetRedGoalLeftBack->AddNext(pickUpRedGoal);

    AutonNode* Wait5 = new AutonNode(0.5, new WaitAction(0.5));

    pickUpRedGoal->AddNext(Wait5);

    AutonNode* WingArmDown2 = new AutonNode(0.1, new UseClawAction(m_WingArmNode, true));

    Path GetBlueDropPath = PathManager::GetInstance()->GetPath("GetBlueDrop");
    AutonNode* GetBlueDrop = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(
                GetBlueDropPath), 
                GetBlueDropPath, 
                false
            )
        );
    
    Wait5->AddNext(WingArmDown2);
    Wait5->AddNext(GetBlueDrop);

    AutonNode* pickupBlueDrop = new AutonNode(0.5, new UseClawAction(m_frontClawNode, true));

    GetBlueDrop->AddNext(pickupBlueDrop);

    AutonNode* Wait6 = new AutonNode(0.5, new WaitAction(0.5));

    pickupBlueDrop->AddNext(Wait6);

    AutonNode* LiftBlueToScore = new AutonNode(0.5, new MoveLiftToPositionAction(m_liftNode, 1800, 10, true));

    AutonNode* ringIntakeOn2 = new AutonNode(0.1, new RollerIntakeAction(m_intakeNode));

    Path PlaceBlueDropPath = PathManager::GetInstance()->GetPath("PlaceBlueDrop");
    AutonNode* PlaceBlueDrop = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(
                PlaceBlueDropPath), 
                PlaceBlueDropPath, 
                false
            )
        );

    pickupBlueDrop->AddNext(ringIntakeOn2);
    pickupBlueDrop->AddNext(LiftBlueToScore); //TODO Test to see if we need a wait before this
    LiftBlueToScore->AddNext(PlaceBlueDrop);

    AutonNode* dropBlueDrop = new AutonNode(0.5, new UseClawAction(m_frontClawNode, false));

    PlaceBlueDrop->AddNext(dropBlueDrop);

    Path BlueDropBackPath = PathManager::GetInstance()->GetPath("BlueDropBack");
    AutonNode* BlueDropBack = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(
                BlueDropBackPath), 
                BlueDropBackPath, 
                false
            )
        );

    dropBlueDrop->AddNext(BlueDropBack);

    AutonNode* dropFront2 = new AutonNode(0.5, new MoveLiftToPositionAction(m_liftNode, 275, 10));

    Path GetYellowDropPath = PathManager::GetInstance()->GetPath("GetYellowDrop");
    AutonNode* GetYellowDrop = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(
                GetYellowDropPath), 
                GetYellowDropPath, 
                false
            )
        );

    BlueDropBack->AddNext(dropFront2);
    BlueDropBack->AddNext(GetYellowDrop);

    AutonNode* grabYellowDrop = new AutonNode(0.5, new UseClawAction(m_frontClawNode, true));

    GetYellowDrop->AddNext(grabYellowDrop);

    AutonNode* Wait7 = new AutonNode(0.5, new WaitAction(0.5));

    grabYellowDrop->AddNext(Wait7);

    AutonNode* raiseYellowToScore = new AutonNode(0.5, new MoveLiftToPositionAction(m_liftNode, 1800, 10));

    Wait7->AddNext(raiseYellowToScore);

    Path YellowDropBackPath = PathManager::GetInstance()->GetPath("YellowDropBack");
    AutonNode* YellowDropBack = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(
                YellowDropBackPath), 
                YellowDropBackPath, 
                false
            )
        );

    raiseYellowToScore->AddNext(YellowDropBack);

    Path PlaceYellowPath = PathManager::GetInstance()->GetPath("PlaceYellow");
    AutonNode* PlaceYellow = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(
                PlaceYellowPath), 
                PlaceYellowPath, 
                false
            )
        );
        
    YellowDropBack->AddNext(PlaceYellow);

    AutonNode* dropYellowDrop = new AutonNode(0.5, new UseClawAction(m_frontClawNode, false));

    PlaceYellow->AddNext(dropYellowDrop);



    Path PlaceYellowBackPath = PathManager::GetInstance()->GetPath("PlaceYellowBack");
    AutonNode* PlaceYellowBack = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(
                PlaceYellowBackPath), 
                PlaceYellowBackPath, 
                false
            )
        );

    dropYellowDrop->AddNext(PlaceYellowBack);

    AutonNode* WingArmUp2 = new AutonNode(0.1, new UseClawAction(m_WingArmNode, false));

    AutonNode* lowerFront3 = new AutonNode(0.5, new MoveLiftToPositionAction(m_liftNode, 275, 10));

    AutonNode* ringIntakeOff2 = new AutonNode(0.1, new RollerIntakeAction(m_intakeNode, 0));

    Path GoCornerPath = PathManager::GetInstance()->GetPath("GoCorner");
    AutonNode* GoCorner = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(
                GoCornerPath), 
                GoCornerPath, 
                false
            )
        );
    PlaceYellowBack->AddNext(ringIntakeOff2);
    PlaceYellowBack->AddNext(WingArmUp2);
    PlaceYellowBack->AddNext(lowerFront3);
    PlaceYellowBack->AddNext(GoCorner);

//================================================================================================















    


    
    AutonNode* frontClawCloseOnNeutralGoal = new AutonNode(0.1, new UseClawAction(m_frontClawNode, true));
    GoCorner->AddNext(frontClawCloseOnNeutralGoal);

    AutonNode* waitBeforeMove = new AutonNode(0.5, new WaitAction(.5));
    frontClawCloseOnNeutralGoal->AddNext(waitBeforeMove);

    Path BackupFromCornerPath = PathManager::GetInstance()->GetPath("BackupFromCorner");
    AutonNode* BackupFromCorner = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(BackupFromCornerPath), 
            BackupFromCornerPath, 
            true
        )
    );

    AutonNode* liftUpGoal = new AutonNode(0.1, new MoveLiftToPositionAction(m_liftNode, 500, 10));

    waitBeforeMove->AddNext(BackupFromCorner);
    waitBeforeMove->AddNext(liftUpGoal);

    AutonNode* moveLiftToScoringPosition = new AutonNode(0.1, new MoveLiftToPositionAction(m_liftNode, 1800, 10));

    Path PlaceCornerGoalPath = PathManager::GetInstance()->GetPath("PlaceCornerGoal");
    AutonNode* PlaceCornerGoal = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(PlaceCornerGoalPath), 
            PlaceCornerGoalPath, 
            false
        )
    );

    BackupFromCorner->AddNext(moveLiftToScoringPosition);
    BackupFromCorner->AddNext(PlaceCornerGoal);

    AutonNode* frontClawDropGoal = new AutonNode(0.1, new UseClawAction(m_frontClawNode, false));

    
    AutonNode* waitBeforeLeavingPlatform = new AutonNode(0.1, new WaitAction(.5));

    PlaceCornerGoal->AddNext(frontClawDropGoal);
    PlaceCornerGoal->AddNext(waitBeforeLeavingPlatform);

    Path BackupFromPlatformPath = PathManager::GetInstance()->GetPath("BackupFromPlatform");
    AutonNode* BackupFromPlatform = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(BackupFromPlatformPath), 
            BackupFromPlatformPath, 
            false
        )
    );

    AutonNode* liftAboveRings = new AutonNode(0.1, new MoveLiftToPositionAction(m_liftNode, 700, 10));

    waitBeforeLeavingPlatform->AddNext(BackupFromPlatform);
    

    Path CollectRingsOnReturnPath = PathManager::GetInstance()->GetPath("CollectRingsOnReturn");
    AutonNode* CollectRingsOnReturn = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(CollectRingsOnReturnPath), 
            CollectRingsOnReturnPath, 
            false
        )
    );

    AutonNode* startConveyor = new AutonNode(0.1, new RollerIntakeAction(m_intakeNode));

    BackupFromPlatform->AddNext(startConveyor);
    BackupFromPlatform->AddNext(CollectRingsOnReturn);
    BackupFromPlatform->AddNext(liftAboveRings);

    Path BackupFromRingLinePath = PathManager::GetInstance()->GetPath("BackupFromRingLine");
    AutonNode* BackupFromRingLine = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(BackupFromRingLinePath), 
            BackupFromRingLinePath, 
            false
        )
    );
    
    CollectRingsOnReturn->AddNext(BackupFromRingLine);

    Path PreloadPickupPath = PathManager::GetInstance()->GetPath("PreloadPickup");
    AutonNode* PreloadPickup = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(PreloadPickupPath), 
            PreloadPickupPath, 
            false
        )
    );

    BackupFromRingLine->AddNext(PreloadPickup);
    
    AutonNode* liftAboveWall = new AutonNode(0.1, new MoveLiftToPositionAction(m_liftNode, 1800, 10));

    PreloadPickup->AddNext(liftAboveWall);

    AutonNode* waitUntilLift = new AutonNode(0.1, new WaitAction(.5));

    liftAboveWall->AddNext(waitUntilLift);

    Path WallAlignPath = PathManager::GetInstance()->GetPath("WallAlign");
    AutonNode* WallAlign = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(WallAlignPath), 
            WallAlignPath, 
            false
        )
    );

    waitUntilLift->AddNext(WallAlign);


    Path BackupFromPreloadPickupPath = PathManager::GetInstance()->GetPath("BackupFromPreloadPickup");
    AutonNode* BackupFromPreloadPickup = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(BackupFromPreloadPickupPath), 
            BackupFromPreloadPickupPath, 
            true
        )
    );

    WallAlign->AddNext(BackupFromPreloadPickup);

    Path BuddyClimbPath = PathManager::GetInstance()->GetPath("BuddyClimb");
    AutonNode* BuddyClimb = new AutonNode(
        10, 
        new FollowPathAction(
            m_driveNode, 
            m_odomNode, 
            new TankPathPursuit(BuddyClimbPath), 
            BuddyClimbPath, 
            false
        )
    );

    AutonNode* stopConveyor = new AutonNode(0.1, new RollerIntakeAction(m_intakeNode, 0));

    AutonNode* liftDown = new AutonNode(0.1, new MoveLiftToPositionAction(m_liftNode, 270, 10));

    BackupFromPreloadPickup->AddNext(BuddyClimb);
    BackupFromPreloadPickup->AddNext(stopConveyor);
    BackupFromPreloadPickup->AddNext(liftDown);

//=======================================================================================================================================================================


   



    





}                  