package org.firstinspires.ftc.teamcode.vault;
    import com.pedropathing.util.Timer;
    import com.qualcomm.hardware.limelightvision.Limelight3A;
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.bylazar.configurables.annotations.Configurable;
    import com.bylazar.telemetry.TelemetryManager;
    import com.bylazar.telemetry.PanelsTelemetry;
    import com.pedropathing.geometry.BezierCurve;
    import com.pedropathing.geometry.BezierLine;
    import com.pedropathing.follower.Follower;
    import com.pedropathing.paths.PathChain;
    import com.pedropathing.geometry.Pose;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.DistanceSensor;
    import com.qualcomm.robotcore.hardware.Servo;

    import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
    import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "GrantsBlueAutonomous", group = "Autonomous")
    @Configurable // Panels
    public class BlueAutonNEAR extends OpMode {
    private Pose pose;

    private Servo gate, indicatorLight1, indicatorLight2;
    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    private DcMotorEx flywheelLeft, flywheelRight, intakeOuter, intakeInner;

    private DistanceSensor intakeSensor1, intakeSensor2;

    private Servo hood, raxon, laxon;

    private Limelight3A limelight;

    private boolean gateOpen = false;

    private boolean driving;
    private boolean intakeOn = false;
    private boolean flywheelOn = false;
    private boolean aprilTagTracking = false;

    private boolean debounceA, debounceX, debounceRightStick, debounceBack, debounceLEFT_TRIGGER;

    private double FlywheelV = 2000;
    private double hoodPos = .67;
    private double raxonPos = .63;
    private double laxonPos = .63;

    private Timer actionTimer = new Timer();

    private Timer pathTimer = new Timer();
      private TelemetryManager panelsTelemetry; // Panels Telemetry instance
      public Follower follower; // Pedro Pathing follower instance
      private int pathState; // Current autonomous path state (state machine)
      private Paths paths; // Paths defined in the Paths class
      
      @Override
      public void init() {

          gate = hardwareMap.get(Servo.class, "gate");


          indicatorLight1 = hardwareMap.get(Servo.class, "lightOne");
          indicatorLight2 = hardwareMap.get(Servo.class, "lightTwo");

          frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fl");
          frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr");
          backLeftMotor = hardwareMap.get(DcMotorEx.class, "bl");
          backRightMotor = hardwareMap.get(DcMotorEx.class, "br");

          flywheelLeft = hardwareMap.get(DcMotorEx.class, "flyL");
          flywheelRight = hardwareMap.get(DcMotorEx.class, "flyR");
          flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          flywheelLeft.setDirection(DcMotor.Direction.FORWARD);
          flywheelRight.setDirection(DcMotor.Direction.REVERSE);

          intakeOuter = hardwareMap.get(DcMotorEx.class, "intOuter");
          intakeInner = hardwareMap.get(DcMotorEx.class, "intInner");
          intakeOuter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          intakeInner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          intakeOuter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          intakeInner.setDirection(DcMotor.Direction.REVERSE);
          intakeOuter.setDirection(DcMotor.Direction.REVERSE);
          intakeInner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

          intakeSensor1 = hardwareMap.get(DistanceSensor.class, "intakeSensor1");
          intakeSensor2 = hardwareMap.get(DistanceSensor.class, "intakeSensor2");

          hood = hardwareMap.get(Servo.class, "hood");
          raxon = hardwareMap.get(Servo.class, "raxon");
          laxon = hardwareMap.get(Servo.class, "laxon");

          follower = Constants.createFollower(hardwareMap);


          panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(180)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
      }
      
      @Override
      public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        telemetry.addData("Follower Status", follower.isBusy());
        telemetry.addData("State", state);
        telemetry.addData("path following...", follower.getCurrentPath());
        telemetry.update();
        panelsTelemetry.update(telemetry);
      }

      
  public static class Paths {
    public PathChain Start;
public PathChain ScoreHuman;
public PathChain HumanScore;
public PathChain ScoreGate;
public PathChain GateToWall;
public PathChain ToWall;
public PathChain WallScore;
public PathChain ScoreEnd;
    
    public Paths(Follower follower) {
      Start = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(56.000, 8.000),
            
            new Pose(56.000, 8.000)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
              .setTValueConstraint(.95)
        .build();

ScoreHuman = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(56.000, 8.000),
            
            new Pose(8.000, 7.512)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
        .setTValueConstraint(.95)
        .build();

HumanScore = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(8.000, 7.512),
            
            new Pose(56.000, 8.000)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
        .setTValueConstraint(.95)
        .build();

ScoreGate = follower.pathBuilder().addPath(
          new BezierCurve(
            new Pose(56.000, 8.000),
            new Pose(62.721, 74.085),
new Pose(63.360, 71.790),
            new Pose(14.832, 72.520)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
        .setTValueConstraint(.95)
        .build();

GateToWall = follower.pathBuilder().addPath(
          new BezierCurve(
            new Pose(14.832, 72.520),
            new Pose(89.624, 75.048),
new Pose(58.305, 18.984),
            new Pose(23.879, 22.847)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
        .setTValueConstraint(.95)
        .build();

ToWall = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(23.879, 22.847),
            
            new Pose(8.000, 23.000)
          )
        ).setTangentHeadingInterpolation()
        .setTValueConstraint(.95)
        .build();

WallScore = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(8.000, 23.000),
            
            new Pose(56.000, 8.000)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
        .setTValueConstraint(.95)
        .build();

ScoreEnd = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(56.000, 8.000),
            
            new Pose(59.000, 35.000)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
        .setTValueConstraint(.95)
        .build();
    }
  }
  
    enum State {
          START,

          SCORE1,
          SCOREHUMAN,
          HUMANSCORE,

          SCORE2,
          SCOREGATE,
          GATETOWALL,
          TOWALL,
          WALLSCORE,

          SCORE3,
          SCOREEND,

          DUMMY

        }
          State state = State.START;
      public void autonomousPathUpdate() {

              switch (state) {
                  case START:
                      raxon.setPosition(raxonPos);
                      laxon.setPosition(laxonPos);
                      flywheelLeft.setVelocity(FlywheelV);
                      flywheelRight.setVelocity(FlywheelV);
                      hood.setPosition(hoodPos);
                      if (intakeSensor1.getDistance(DistanceUnit.CM) > 15 && intakeSensor2.getDistance(DistanceUnit.CM) > 15) intakeOuter.setPower(-.8);
                      else intakeOuter.setPower(0);
                      actionTimer.resetTimer();
                      setPathState(State.SCORE1);
                      break;



                  case SCORE1:
                      intakeOuter.setPower(-.8);
                      intakeInner.setPower(.3);
                      gate.setPosition(.88);
                      if (actionTimer.getElapsedTimeSeconds() > 3 && !follower.isBusy()){
                        gate.setPosition(.5);
                        follower.followPath(paths.ScoreHuman, true);
                        setPathState(State.SCOREHUMAN);
                      }
                      break;
                  case SCOREHUMAN:
                      if (intakeSensor1.getDistance(DistanceUnit.CM) > 15 && intakeSensor2.getDistance(DistanceUnit.CM) > 15) intakeOuter.setPower(-.8);
                      else intakeOuter.setPower(0);
                      if (!follower.isBusy()){
                          follower.followPath(paths.HumanScore, true);
                          setPathState(State.HUMANSCORE);
                      }
                      break;
                  case HUMANSCORE:
                      if (intakeSensor1.getDistance(DistanceUnit.CM) > 15 && intakeSensor2.getDistance(DistanceUnit.CM) > 15) intakeOuter.setPower(-.8);
                      else intakeOuter.setPower(0);
                      if (!follower.isBusy()){
                          setPathState(State.SCORE2);
                          actionTimer.resetTimer();
                      }
                      break;
                  case SCORE2:

                      if (follower.isBusy()){
                          actionTimer.resetTimer();
                      }else{
                          intakeOuter.setPower(-.8);
                          intakeInner.setPower(.3);
                          gate.setPosition(.88);
                      }
                      if (actionTimer.getElapsedTimeSeconds() > 3) {
                          gate.setPosition(.5);
                          follower.followPath(paths.ScoreGate, true);
                          setPathState(State.SCOREGATE);
                      }
                      break;
                  case SCOREGATE:
                      if (intakeSensor1.getDistance(DistanceUnit.CM) > 15 && intakeSensor2.getDistance(DistanceUnit.CM) > 15) intakeOuter.setPower(-.8);
                      else intakeOuter.setPower(0);
                      if (follower.isBusy()){
                          actionTimer.resetTimer();
                      }else if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 3){
                          setPathState(State.GATETOWALL);
                          follower.followPath(paths.GateToWall, true);
                      }
                      break;
                  case GATETOWALL:
                      if (intakeSensor1.getDistance(DistanceUnit.CM) > 15 && intakeSensor2.getDistance(DistanceUnit.CM) > 15) intakeOuter.setPower(-.8);
                      else intakeOuter.setPower(0);
                      if (!follower.isBusy()){
                          setPathState(State.TOWALL);
                          follower.followPath(paths.ToWall, true);
                          actionTimer.resetTimer();
                      }
                      break;
                  case TOWALL:
                      if (intakeSensor1.getDistance(DistanceUnit.CM) > 15 && intakeSensor2.getDistance(DistanceUnit.CM) > 15) intakeOuter.setPower(-.8);
                      else intakeOuter.setPower(0);
                      if (!follower.isBusy()){
                          setPathState(State.SCORE3);
                          follower.followPath(paths.WallScore, true);
                          actionTimer.resetTimer();
                      }
                      break;
                  case SCORE3:

                      if (follower.isBusy()){
                          actionTimer.resetTimer();
                      }else{
                          intakeOuter.setPower(-.8);
                          intakeInner.setPower(.3);
                          gate.setPosition(.88);
                      }
                      if (actionTimer.getElapsedTimeSeconds() > 3){
                          gate.setPosition(.5);
                          follower.followPath(paths.ScoreEnd, true);
                          setPathState(State.SCOREEND);
                      }
                      break;
                  case SCOREEND:
                     if (!follower.isBusy()) {
                        setPathState(State.DUMMY);
                     }

              }



          // Add your state machine Here
          // Access paths with paths.pathName
          // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
      }

        public void setPathState(BlueAutonNEAR.State stateCooler) {
            state = stateCooler;
            pathTimer.resetTimer();
        }
    }
    