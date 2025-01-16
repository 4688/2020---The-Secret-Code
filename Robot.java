/***
 *    ___________.__               _________                            __    _______________ _______________    _________            .___      
 *    \__    ___/|  |__   ____    /   _____/ ____   ___________   _____/  |_  \_____  \   _  \\_____  \   _  \   \_   ___ \  ____   __| _/____  
 *      |    |   |  |  \_/ __ \   \_____  \_/ __ \_/ ___\_  __ \_/ __ \   __\  /  ____/  /_\  \/  ____/  /_\  \  /    \  \/ /  _ \ / __ |/ __ \ 
 *      |    |   |   Y  \  ___/   /        \  ___/\  \___|  | \/\  ___/|  |   /       \  \_/  /       \  \_/   \ \     \___(  <_> ) /_/ \  ___/ 
 *      |____|   |___|  /\___  > /_______  /\___  >\___  >__|    \___  >__|   \_______ \_____ \_______ \_____  /  \______  /\____/\____ |\___  >
 *                    \/     \/          \/     \/     \/            \/               \/     \/         \/     \/          \/            \/    \/ 
 * 
 *                           Created By: Hunter Mailloux | Recovered By: Matthew Dupuis | Commented By: Brandon Mailloux
 */

 //Hello all, this is Brandon, I'm commenting this to make it as easy to read and understand as possible, if you have any questions relating to how the robot worked, 
 //asked the mentors, they were all kids when this robot was made and they always want to reminisce on the glory days.

 //Hunter wrote this code, and he has my favorite coding style / naming conventions so enjoy this master piece.
 // If you need any thing further explained or I messed up somewhere, just reach out and I can fix it :D

 //PACKAGES AND IMPORTS (IF ALL THE ERRORS ARE GONE HERE, YOU HAVE ALL THE VENDOR LIBRARIES INSTALLED)
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//START OF ROBOT CODE (SAME AS ALWAYS)
public class Robot extends TimedRobot {

//DEFINING VARIABLES TO BE REMEMBERED BY THE ROBOT

/* Defining Components */

    //Defining the sparkMaxes to control the drive train - its a standard tank drive
    private CANSparkMax frontLeft;
    private CANSparkMax frontRight;
    private CANSparkMax backLeft;
    private CANSparkMax backRight;
    /*
    Defining the sparkMax to control the climber going up and TalonSRXs to pull the robot up
    A tapemeasure lifts the hooks up, the seatbelts then make the robot do a chin up)
    */
    private CANSparkMax climber;
    private TalonSRX seatbelt_l;
    private TalonSRX seatbelt_r;

    //This robot was designed to use two controllers, 1 xbox controller (Matt), and the DIY button board (Chet)
    private Joystick MattDupuis;
    private Joystick ChetPetro;

    //This robot fired dodge balls by forcing them through two high speed wheels controlled by TalonSRXs
    private TalonSRX shooter_l;
    private TalonSRX shooter_r;

    //This robot had two parts to picking up the dodge balls, the TalonSRX controlling the intake that made the beater bar spin
    //And the conveyer belt (Pulley) system that then brought the balls from the beater bar to the shooter
    private TalonSRX pulley;
    private TalonSRX intake;

    //This robot had a double solenoid (meaning True would make it push out, and False would make it actively suck in) to control pushing the beater bar in and out
    private Solenoid solenoid_Double;

    //This robot had a single solenoid (meaning True would make it push out, and False would just cut pressure and it would slowly deflate) to stick a wheel out the top of the robot to spin The Wheel of Fortune
    private Solenoid solenoid_single;

    //This is the well named servo (A servo is a fancy motor where the motor controller is built in and can set to a specific position easily (but they are slow and weak))
    //This servo would spin the Wheel of Fortune to a specific location
    private Servo servy_boi;

    //This is the unimportant stuff, just the compressor and pressureReader that manage the high pressure bomb in the middle of the robot
    //Compressor fills the bomb with more pressure, the pressureReader tells us the current pressure in the bomb, and we use that to turn off
    //the compressor before the robot explodes.
    private AnalogInput pressureReader;
    private Compressor compressor;

/* Defining Constants (Numbers that don't change unless we change it here) */

    //Handycap limits the robot to 50% speed when the turbo is not pressed. 
    //Gunspeed is the percent increase in speed added every 20ms, we don't just instantly set the motor to 100%, because the wheels are so heavy it could break the motor
    private Double HANDYCAP = 0.5, GUNSPEED = 0.1;

    //Variables for the forward/back and left/right from the joystick for the tank drive
    private Double x;
    private Double y;

    //Variables for value that gets set to the left and right side of the tank drive
    private Double l;
    private Double r;

    //This variable stores the final multiplier to l and r, if its in turbo mode its 1.0, if its not turbo mode, its set to HANDYCAP (calculation done later)
    private Double turbo;

    //This variable stores how fast the shooter can possibly be going without breaking for a hard jolt, to start, the max speed is GUNSPEED (10%)
    private Double accel = GUNSPEED;

    //The variables to store the output power to the shooter wheels (One for some reason could spin a little faster then the other, hence the different strengths)
    double leftP = 0.5;
    double rightP = 0.45;

    //This variable stores wether or not the limelight is looking at the target
    private boolean m_LimelightHasValidTarget = false;

    //These variables track the target angles up and down / left and right the target is from the limelight
    public double degree_y = 0.0;
    public double degree_x = 0.0;

    //These variables store how far the robot is from the target after doing calculations with the variables above
    public double A = 0.0;
    public double B = 0.0;

    //This variable stores the power that should go to the drivetrain to make the robot face the target, we only need one, not two because setting both sides to the same value with make it spin to face the target.
    private double h_degree;

    //This variable sets the target rpm of the wheels to make the ball perfectly hit the target (This was just trial and error)
    double rpm = 1800.0;

    //These variables track the difference in each shooter wheels speed (from encoders) from the target and are used to make one side speed up or slow down
    double rpmDifR;
    double rpmDifL;

    //This right here, this is a mess, a real hot mess, this was a random number that was used in a random calculation that didn't really work :)
    double COIF = 1300000.0;

    //We need the robot to pause for a moment so that the limelight can see the target and calculate accurate distances before trying to shoot,
    //We for funzies didn't use a proper timer and instead knowing periodic functions run every 20ms, delay would get 1 added to it each run
    //until it equalled DELNUM at which point we would reset it.
    double delay = 0.0;
    int DELNUM = 40;
    
    //The pressure reading returned from the pressureReader is a voltage, that we then run a calculation to turn it into a PSI
    public double actualPressureDouble; //This has the specific decimal number for the psi
    public int actualPressure;//This has the much nicer and simpler whole numbered psi
    
    //These variables store the power of the intake beater bar
    double intake_power_in = -0.5;
    double intake_power_out = 0.5;

    //These variables store the power of the conveyer that moves the balls from the beater bar to the shooter
    double pulley_power_in = -1.0;
    double pulley_power_out = 1.0;


/* THE INIT FUNCTION */
    //This is self explanatory, when the robot turns on it sets up the different components
    public void robotInit() {
        //Setting up the sparkMaxes with their IDs
        frontLeft = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontRight = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        backLeft = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
        backRight = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
        climber = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);

        //Doing the good practice of factory resting all the motor controllers when the robot turns on incase something funky happened to their settings
        frontRight.restoreFactoryDefaults();
        frontLeft.restoreFactoryDefaults();
        backLeft.restoreFactoryDefaults();
        backRight.restoreFactoryDefaults();
        climber.restoreFactoryDefaults();

        //Setting up the joysticks
        MattDupuis = new Joystick(0);
        ChetPetro = new Joystick(1);

        //THIS IS A COOL FEATURE, by using the follow command on the back motors to follow the front ones, from now on, we only need to set the power of the front motors and the back will automatically match it
        backLeft.follow(frontLeft);
        backRight.follow(frontRight);

        //Setting up all our TalonSRXs on their proper IDs for the different features of the robot
        pulley = new TalonSRX(5);
        shooter_r = new TalonSRX(7);
        shooter_l = new TalonSRX(8);
        intake = new TalonSRX(11);
        seatbelt_l = new TalonSRX(9);
        seatbelt_r = new TalonSRX(10);

        //Turning on LED on the limelight on. This is done by going on the local network (NetworkTableInstance) which is imported as a library and navigating to Default/limelight/LEDMode and setting it to 1 (On),
        //Setting it to 0 would turn it off.
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);

        //Setting up the more intreating and unique components on the robot
        compressor = new Compressor(9); //ID 9
        compressor.setClosedLoopControl(true); //Not sure was this does, ask Cory or Alex
        pressureReader = new AnalogInput(0); //This isn't plugged into the CANBUS like all the other components, its put on the Analog pins, another place the rio can communicate to components,
        //its set to port 0 there, different from ID 0 on the CANBUS
        servy_boi = new Servo(0); //I'm pretty sure this isn't ID 0 on the CANBUS either, this is port 0 on the DIO port on the rio, yet another place you can plug in components

        solenoid_Double = new Solenoid(9, 6);//These are set up where you specify the ID of the compressor it gets the air from (9) and then its own ID (6)
        solenoid_single = new Solenoid(9, 7);//These are set up where you specify the ID of the compressor it gets the air from (9) and then its own ID (7)
    }

    public void autonomousInit() {}
    public void autonomousPeriodic() {}
    public void teleopInit() {}

/*THE REAL FUN, THE ONLY THING WE GOT DONE! NO AUTO, NO FANCY INITS, JUST GOOD OL FASHIONED TELEOP!!!! */
    public void teleopPeriodic() {

        //TalonSRXs have a feature where you can plug in an encoder (a sensor to track spinning) directly into the top of it, and then you can use commands through the TalonSRX to access it, which is what we did here
        //We are doing shooter_r.getSensorCollection().getQuadratureVelocity() to get the current speed of the motor. There is then a gearbox that reduces our motors speed by a factor in exchange for more power, and
        //we also need to convert it from rps to RPM, this all got simplified into multiplying each value by 12.5 to get the actual RPM of the wheel.
        double enc_r = shooter_r.getSensorCollection().getQuadratureVelocity() * 12.5;
        SmartDashboard.putNumber("Encoder speed for talon (right)", enc_r);

        double enc_l = shooter_l.getSensorCollection().getQuadratureVelocity() * -12.5;//negative because this wheel spins the other way
        SmartDashboard.putNumber("Encoder speed for talon (left)", enc_l);

        //Servy_boi never got finished, not even sure what this meant, I hope it didn't just spin at full speed all the time
        servy_boi.set(1.0);

        //When Chet pressed button 5 on the board, the Wheel would swing out the top to spin the big colour wheel (True), when he pressed 6 it slowly deflate (false)
        if (ChetPetro.getRawButton(5)) {
            solenoid_Double.set(true);
        } else if (ChetPetro.getRawButton(6)) {
            solenoid_Double.set(false);
        }

        //When Matt (It was actually Hunter driving, he just named it after him out of respect)
        //pressed button 7 on the Xbox controller, the beater bar for the intake would shoot out (True), when he pressed 6 it would retract (False)
        if (MattDupuis.getRawButton(7)) {
            solenoid_single.set(true);
        } else if (MattDupuis.getRawButton(8)) {
            solenoid_single.set(false);
        }

        //TV was a value we didn't use but we knew we could grab from the Limelight over the network at limelight/tv. The command getDouble(0.0) meant that if it
        //couldnt find it, it would return 0.0 instead. We knew this value should be more than 1 if it was actually pulling a value from the limelight, so if it was, we sent that info to the driver that
        //the limelight is locked onto a target
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
        m_LimelightHasValidTarget = tv >= 1.0;
        SmartDashboard.putBoolean("Limelight has Target", m_LimelightHasValidTarget);

        //Similar to above we are getting ty from the limelight or 0.0 if it couldn't find it. Ty is the angle between the robot and the target, so if it was 30, that mean the limelight was looking at the target,
        //which is up from the camera by 30 degrees. The camera was on a mount that we knew tilted the camera at 30.67 Degrees up (I think we got this from a gyro in the limelight), so the actual angle between the
        //target and robot was (degree_y + 30.67765415). We then need to convert the angle to radiants, by multiplying it by Math.PI / 180.0 so that we could use it in the Tan function. What this results in is the
        //ratio between the opposite side (the height of the target) and the adjacent side (our distance from the base of the target). From the game manual we know the target is 180.81cm tall, so by dividing
        // 180.81 by our ratio we can get the exact distance the robot is from the base of the target. In modern limelights, you can just get the distance directly from the network without doing all this fun highschool math :D
        degree_y = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
        A = 180.81625 / Math.tan((degree_y + 30.67765415) * Math.PI / 180.0);
        SmartDashboard.putNumber("Distance", A);
        SmartDashboard.putNumber("degree Y", degree_y);

        //In a similar way we also got the angle left and right the target was from the limelight, if the target was in the left of our vision, it would be a negative number like -15 degrees.
        degree_x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
        SmartDashboard.putNumber("degree X", degree_x);

        //I think we were converting from CM to Feet here, either way the numbers slightly wrong :')
        double lime_distance = A / 30.46;

        //This big and scary equation was SOOOOOOO much fun to make. We spent 2 days shooting the robot from different distances away from the target and made a table of how far away we were with a tape measure, and what RPM we
        //had to manually set the shooter to so that it perfectly hit the target. We then got the help of Mr.B (A math teacher) who then entered our table into a graphing calculator and returned us an approximate equation
        //that went through all our data points. This way, we can enter a distance, and this equation figures out approximately how fast to shoot to hit the target. 
        //This is the very basis of AI development and it worked really well! This thing can shoot from anywhere!
        rpm = -0.02965 * Math.pow(lime_distance, 4.0) + 2.08087 * Math.pow(lime_distance, 3.0) - 52.09085 * Math.pow(lime_distance, 2.0) + 555.93407 * lime_distance - 328.88545;

        //LOL haven't seen this in a long time, the sparkmax motor controller get the motor temps from the motors, we took this info and posted it to the dashboard so the driver would know if their robot was running to hot
        //Personally, I've never used this feature, I mean you should, I just think smoke is a better indicator of motor temp.
        SmartDashboard.putNumber("Back Left Motor Temp", backLeft.getMotorTemperature());
        SmartDashboard.putNumber("Back Right Motor Temp", backRight.getMotorTemperature());
        SmartDashboard.putNumber("Front Left Motor Temp", frontLeft.getMotorTemperature());
        SmartDashboard.putNumber("Front Right Motor Temp", frontRight.getMotorTemperature());

        //This code tells us how close the bomb in the robot is to exploding. The pressure sensor returns a voltage, that we then use an equation we found online to convert it into a PSI to display to the driver,
        //Not sure why were rounded the number in a separate step, we could have done actualPressureDouble = (int) 250.0 * pressureReader.getVoltage() / 5.0 - 20.0; but oh well.
        actualPressureDouble = 250.0 * pressureReader.getVoltage() / 5.0 - 20.0;
        actualPressure = (int) actualPressureDouble;
        SmartDashboard.putNumber("Pressure", actualPressure);

        //This right here, this is a mess, don't do this, this was a failed attempt to make the robot start off slow and then speed up as it drives instead of jolting to 100%.
        //Problem is it never resets and is also increasing, even with the robot parked, so it would eventually be a huge number that would make the robot undrivable. better off to just have Jolting.
        //The better solution would have been to use a proper PID to handle revving up.
        accel += 0.01;

        //Getting the X and Y from the Joystick for tank drive, you will notice we do x = x * Math.abs(x); which I never talk about, this was so that low inputs would be really low, 0.1 * 0.1 = 0.01
        // and 0.5 * 0.5 = 0.25 This meant the driver had a ton of control over precise movements with robot being slow most of the time, but 1 * 1 = 1 means it could still go full speed if you pushed the stick all the way,
        //It just meant you could either go slow or fast, and nothing in between as its rarely needed in comp. The abs is so that way we keep the negative sign. 
        //-1 * -1 = 1 which is not what we want so abs strips the negatives and we get -1 * 1 = -1.
        x = MattDupuis.getRawAxis(1);
        x = x * Math.abs(x);

        y = MattDupuis.getRawAxis(4);
        y = y * Math.abs(y) / 2.0;

        //This is us getting the turbo button (button 1 on the joystick). This is a fancy short-hand if statement. I never use these, Hunter and Chet do. Its just a normal if statement but instead of doing if(Boolean){this}else{that} they
        // do boolean ? this : that. By putting the ? after the boolean equation they can then say what the value will be set to if its true, and then after the : say what to set it to if its false. So in this case,
        //If the button if pressed, turbo is set to 100% otherwise, if its false its set to the value of Handycap (0.5)
        turbo = MattDupuis.getRawButton(1) ? 1.0 : HANDYCAP;

        //If our robot is not moving (0.03 for dead bands)
        if (Math.abs(x) < 0.03 && Math.abs(y) < 0.03) {
            //Set the motors power to 0
            l = 0.0;
            r = 0.0;

            //If we are pressing down button 2 when were stopped
            if (MattDupuis.getRawButton(2)) {
                //Set the LED on the limelight to flash (3)
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
                //If we have a target from the check done earlier
                if (m_LimelightHasValidTarget) {
                    //This equation was made with trial and error to convert the degree we were off left and right from facing the target and turn it into the power we set the motors to spin the robot to face the target
                    //The farther away we were, the larger the angle, and the faster we would spin to fix it, eventually going really slow with 1 degree off so that we didnt over shoot it.
                    h_degree = degree_x / -100.0 - 0.15 * degree_x / Math.abs(degree_x);
                    //When we were finally within 1 degree of facing the target, we said that was close enough and it wasn't work fine tuning our position any more
                    //Looking at this, we should have then had an else to move the pully to fire a ball, but I guess Hunter wanted to fire manually after we were lined up.
                    if (Math.abs(degree_x) > 1.0) {
                        //When farther than 1 degree from facing, spin to face it (electrical wired the right side backwards so we actually had to set the right side to negative to get it to spin.)
                        l = h_degree;
                        r = -h_degree;
                    }
                }
            } else {
                //If we were not automatically lining up, IDK what we were doing here with this value, its not good whatever we were doing
                accel = GUNSPEED;
                //Set the limelight back to a solid light (1)
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
            }

        //The rest of this is your standard tank drive code with deadbands.
        } else if (Math.abs(x) < 0.03) {
            l = -y;
            r = y;
        } else if (Math.abs(y) < 0.03) {
            l = x;
            r = x;
        } else {
            l = x - y;
            r = x + y;
        }

        //Multiplying the turbo at the end, which I would not have done like this looking back, because if the robot is auto lining up, then its setting l and r based on that equation.
        //So if you held down the lineup button and the turbo button at the same time, you would have a hyper charged lineup that would likely just whip the robot around. Instead, set the turbo
        //only in the tank drive code at the bottom of each of the three above if statements. 
        l *= turbo;
        r *= turbo;

        //Again, something stupid we were messing with to get the robot to rev up to speed, again this would make the robot way too fast, and again, directly ontop of L and R no matter what, so you could have an EXTRA GIGA 
        //CHARGED auto aline that would just be a bucking bull of a robot.
        if (Math.abs(r) > accel) {
            r = accel * Math.abs(r) / r;
        }
        if (Math.abs(l) > accel) {
            l = accel * Math.abs(l) / l;
        }

        //Good practice of showing the motor speeds to the driver
        SmartDashboard.putNumber("R Speed", r);
        SmartDashboard.putNumber("L Speed", l);

        //Setting the motor speeds to the motors, again, electrical wired one side backwards so we had to stick a negative on there.
        frontLeft.set(-l);
        frontRight.set(r);

        //This whole if statement was stupid highschool students trying to reinvent the PID. The goal was if the shooter was on and reving up, every 40 ticks of the delay, it would check to see if either shooter wheel 
        //was going too fast or too slow, we wait 40 ticks between because when its every 20 ms the robot does not have time to rev up in between to know if the change in power fixed it.
        //It would then set the desired shooter speeds to the shooter or turn it off if the shooter trigger was not held down.
        if (MattDupuis.getRawAxis(3) >= 0.9) {
            if (delay >= DELNUM) {
                rpmDifR = rpm - enc_r;
                rpmDifL = rpm - enc_l;
                leftP += rpmDifL * Math.abs(rpmDifL) / COIF;
                rightP += rpmDifR * Math.abs(rpmDifR) / COIF;
                delay = 0.0;
            } else {
                delay++;
            }

            shooter_l.set(ControlMode.PercentOutput, leftP);
            shooter_r.set(ControlMode.PercentOutput, rightP);
        } else {
            shooter_l.set(ControlMode.PercentOutput, 0.0);
            shooter_r.set(ControlMode.PercentOutput, 0.0);
            delay = 0.0;
        }

        //If button 5 was pressed, then move the pulleys on the conveyer to feed the balls into the shooter to fire.
        if (MattDupuis.getRawButton(5)) {
            pulley.set(ControlMode.PercentOutput, pulley_power_out);
        } else {
            pulley.set(ControlMode.PercentOutput, 0.0);
        }

        //If 6 was pressed, back up the conveyor away from the shooter, this should have been an elif with the if statement above.
        if (MattDupuis.getRawButton(6)) {
            pulley.set(ControlMode.PercentOutput, pulley_power_in * 0.8);
        }

        //Because the climb hooks could hook unevenly or even 1 failing to hook on, we had two buttons to climb each side individually, ideally you press them both at the same time.
        //This was also done wrong, this means you can't climb with both at the same time because if 9 is pressed, it sets the left belt and never checks the right belt, they should be separate if statements.
        if (MattDupuis.getRawButton(9)) {
            seatbelt_l.set(ControlMode.PercentOutput, 1.0);
        } else if (MattDupuis.getRawButton(10)) {
            seatbelt_r.set(ControlMode.PercentOutput, 1.0);
        } else {
            seatbelt_l.set(ControlMode.PercentOutput, 0.0);
            seatbelt_r.set(ControlMode.PercentOutput, 0.0);
        }

        //Finally this sets the speed of the intake beater bar to half the value of the xbox trigger. We didn't see a need at that point to have the beater bar spin in reverse. 
        intake_power_out = MattDupuis.getRawAxis(2) * -0.5;
        intake.set(ControlMode.PercentOutput, intake_power_out);
    }

    public void testInit() {}

    public void testPeriodic() {}
}
