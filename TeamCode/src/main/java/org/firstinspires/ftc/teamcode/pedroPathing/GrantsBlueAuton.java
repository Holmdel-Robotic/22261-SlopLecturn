package org.firstinspires.ftc.teamcode.pedroPathing;
    import com.pedropathing.util.Timer;
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
    
    @Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
    @Configurable // Panels
    public class GrantsBlueAuton extends OpMode {
        private Timer pathTimer = new Timer();
      private TelemetryManager panelsTelemetry; // Panels Telemetry instance
      public Follower follower; // Pedro Pathing follower instance
      private int pathState; // Current autonomous path state (state machine)
      private Paths paths; // Paths defined in the Paths class
      
      @Override
      public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

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
        
        .build();

ScoreHuman = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(56.000, 8.000),
            
            new Pose(8.000, 7.512)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
        
        .build();

HumanScore = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(8.000, 7.512),
            
            new Pose(56.000, 8.000)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
        
        .build();

ScoreGate = follower.pathBuilder().addPath(
          new BezierCurve(
            new Pose(56.000, 8.000),
            new Pose(62.721, 74.085),
new Pose(63.360, 71.790),
            new Pose(14.832, 72.520)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
        
        .build();

GateToWall = follower.pathBuilder().addPath(
          new BezierCurve(
            new Pose(14.832, 72.520),
            new Pose(89.624, 75.048),
new Pose(58.305, 18.984),
            new Pose(23.879, 22.847)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
        
        .build();

ToWall = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(23.879, 22.847),
            
            new Pose(8.000, 23.000)
          )
        ).setTangentHeadingInterpolation()
        
        .build();

WallScore = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(8.000, 23.000),
            
            new Pose(56.000, 8.000)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
        
        .build();

ScoreEnd = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(56.000, 8.000),
            
            new Pose(59.000, 35.000)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
        
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
          SCOREEND

        }
          State state = State.START;
      public void autonomousPathUpdate() {

              switch (state) {

              }


          // Add your state machine Here
          // Access paths with paths.pathName
          // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
      }

        public void setPathState(GrantsBlueAuton.State stateCooler) {
            state = stateCooler;
            pathTimer.resetTimer();
        }
    }
    