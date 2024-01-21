#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/logger/stdout.hpp"
#include "pros/misc.h"





//Variables

//Motor Definitions

pros::Motor left_top_mtr(11, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor left_bottom_mtr(12, pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor left_front_mtr(13, pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor right_top_mtr(20, pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_bottom_mtr(19, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_front_mtr(18, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor intake1(5, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor intake2(6, pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor cata(16, pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor_Group left({11,12,13});
pros::Motor_Group right({20,19,18});




//solenoid ports


#define DIGITAL_SENSOR_PORT 'G'
#define DIGITAL_SENSOR_PORT_OTHER_WING 'H'
pros::ADIDigitalOut wingsSolanoidLeft (DIGITAL_SENSOR_PORT, false);
pros::ADIDigitalOut wingsSolanoidRight (DIGITAL_SENSOR_PORT_OTHER_WING, false);

#define HANGER_SENSOR_PORT 'F'// change port

pros::ADIDigitalOut hangerSolanoid(HANGER_SENSOR_PORT, false);







//Controller Definition
pros::Controller cntrlr(pros::E_CONTROLLER_MASTER);

pros::Rotation cataRotation (1);


///state machin and cata PID
int cataError;
int cataTargetPos;
int cataPrevError;
int deltaError;
//bool resetCataSensors = true;
float cataPosition;
int cataPower;
float cP = 50;
float cD = 10;

int motor_min_velocity = 0;
int motor_max_velocity = 12000;

void cataPD() {

    
    //cataRotation.reset_position();
    
    cataPosition = cataRotation.get_angle()/100;
    cataError = cataTargetPos - cataPosition;
    deltaError = cataPrevError -cataError;
    
    cataPower = (cataError * cP + deltaError * cD);

    cataPrevError = cataError;

    


    cata.move_voltage(cataPower);

    // resetCataSensors = false;

    

}



enum CATA_STATE {
  STOPPED,
  FIRING,
  RESETTING
};

const CATA_STATE CATA_START_STATE = CATA_STATE::STOPPED;


class CataStateMachine {
  private:
    CATA_STATE state = CATA_START_STATE;

    pros::Task* updateTask;

    bool isDoneFiring() {
      // return whether the cata has finished firing
      cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    }

    bool isDoneResetting() {
      // return whether the cata has finished firing
      return cataRotation.get_angle()/100 >= 355;
    }

    void update() {
      switch (this->state) {
        case CATA_STATE::STOPPED:
          cata.brake();
          // or
          // cataMotors.move_voltage(0);
          break;
        case CATA_STATE::FIRING:
          // do whatever makes the cata fire
          cata.move(127);
          if (this->isDoneFiring()) this->reset();
          break;
        case CATA_STATE::RESETTING:
          // do whatever makes the cata reset (in your case this should be your
          // pid)
          cataTargetPos = 360;
          cataPD();
          if (this->isDoneResetting()) this->stop();
          break;
      }
    }
  public:
    void initialize() {
      updateTask = new pros::Task {[=]() {
        while (1) {
          this->update();
          pros::delay(10);
        }
      }};
    }

    void fire() { this->state = CATA_STATE::FIRING; }

    void stop() { this->state = CATA_STATE::STOPPED; }

    void reset() { this->state = CATA_STATE::RESETTING; }
};











//Sensors and Tracking Wheel Definitions - REMEMBER TO CHANGE THE PORTS 
//and REMEASURE EVERYTHING








pros::Imu imuSensor(3); 

// horizontal tracking wheel. 2.75" diameter, 3.7" offset, back of the robot


// drivetrain settings
lemlib::Drivetrain drivetrain(&left, // left motor group
                              &right, // right motor group
                              11.5, // 15 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              360, // drivetrain rpm is 360
                              10 // chase power is 2. If we had traction wheels, it would have been 8
);


// lateral motion controller
lemlib::ControllerSettings linearController(50, // proportional gain (kP)
                                            00,
                                            100, // derivative gain (kD)
                                            0,
                                            0.1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            0.5, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            17 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController( 2.5, // proportional gain (kP)
                                              0,
                                              14,
                                              0, 
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              2, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              17 // maximum acceleration (slew)
);



// sensors for odometry
// note that in this example we use internal motor encoders, so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to nullptr as we don't have one
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have one
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imuSensor // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

void openWings() {
    wingsSolanoidLeft.set_value(true);
    wingsSolanoidRight.set_value(true);

}
void closeWings() {
    wingsSolanoidLeft.set_value(false);
    wingsSolanoidRight.set_value(false);
}
void intake() {
  intake1.move(127);
  intake2.move(127);
}
void outtake() {
  intake1.move(-127);
  intake2.move(-127);
}
void stopIntake(){
  intake1.move(0);
  intake2.move(0);
}

void checkMotorsAndReturnTemperature() {
    std::vector<pros::Motor> motors = {
        left_bottom_mtr, left_top_mtr, left_front_mtr, right_front_mtr, right_bottom_mtr, right_top_mtr, cata, intake1, intake2
    };

    while (true) {
        double totalTemp = 0.0;
        int count = 0;

        for (auto& motor : motors) {
            double temp = motor.get_temperature();
            if (temp == PROS_ERR_F) { // PROS_ERR_F is returned when the motor is unplugged
                cntrlr.set_text(0, 0, "Motor " + std::to_string(motor.get_port()) + " unplugged.");
                pros::delay(250);
                cntrlr.rumble("---");
            }

            if (count < 6) {
                totalTemp += temp;
            }
            ++count;
        }

        if (count == 0) cntrlr.set_text(0, 0, "No motors found.");

        double averageTempCelsius = totalTemp / count;
        double averageTempFahrenheit = averageTempCelsius * 9.0 / 5.0 + 32.0;
        cntrlr.set_text(0, 0, "Avg Temp: " + std::to_string(averageTempFahrenheit));

        pros::delay(250);
    }
}




void initialize() {
	
  // task to make sure all motors are plugged in and check the temperature of the drivetrain
  pros::Task motorCheck(checkMotorsAndReturnTemperature);
  // selector::init(); 

  pros::lcd::initialize();
    
    
    
	chassis.calibrate();

    

	// thread to for brain screen and position logging
  pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
  });
  
    
	
    
	
}



void disruptionShort() {
  chassis.setPose(-44.019, -56.649, 20 );

  //go for middle triball
  chassis.moveToPose(-25.37, -8.352, 20, 4000);
  chassis.waitUntilDone();
  openWings();
  intake();
  pros::delay(300);
  stopIntake();
  chassis.turnTo(-9.338, -8.352, 2000);
  //score the preload
  chassis.moveToPose(-41.722, -8.352, 90, 3000, {.forwards = false});
  //push over middle bar triball
  chassis.moveToPose(-7.254, -9.474, 90, 400);
  chassis.waitUntilDone();
  closeWings();
  //push balls over to other side and set to bowl
  chassis.moveToPose(-44.019, -56.649, 200, 2000);
  chassis.moveToPose(-6.452, -58.852, 90, 4000);
  chassis.moveToPose(-41.402, -60.776, 90, 2000);

}

void normalShort() {
  chassis.setPose(-48.765, -57.876, 135);

  chassis.moveToPose(-55.512, -51.261, 135, 2000, {.forwards = false});
  chassis.waitUntilDone();
  openWings();
  chassis.moveToPose(-48.765, -57.876, 135, 4000);
  chassis.waitUntil(5);
  closeWings();
  chassis.moveToPose(-60.633, -28.046, 180, 4000, {.forwards = false});
  chassis.moveToPose(-44.247, -56.833, 90, 3000);
  chassis.moveToPose(-5.688, -58.64, 90,3000);
  outtake();

}




void farAutoDisruptionPrev() {
  chassis.setPose(44.699, -57.283, 325);

  // wingsSolanoidRight.set_value(true);
  // pros::delay(200);
  // wingsSolanoidRight.set_value(false);                                                                        
  //move to middle triballs
  chassis.moveToPose(9.252, -5.216, 313, 2000);
  // chassis.waitUntilDone();
  intake();
  pros::delay(1900);
  stopIntake();
  pros::delay(200);
  
  //score two middle triballs
  //chassis.turnTo(40.676, -7.672, 4000);

  chassis.moveToPose(41.408, 1.672, 90, 1550, {.chasePower = 50});
  chassis.waitUntil(5);
  openWings();
  
  chassis.waitUntilDone();
  closeWings();
  
  // // chassis.moveToPose(38.408, 0.672, 90, 2000, {.forwards = false});
  // // chassis.moveToPose(42.408, 0.672, 90, 2000, {.chasePower = 50});
  // // //go for other long barrier
  chassis.moveToPose(25.694, 1.672, 90, 1000, {.forwards = false, .chasePower = 3});
  chassis.turnTo(12.699, -17.609, 870);
  chassis.moveToPose(11.717, -12.699, 235, 1450);
  // chassis.waitUntilDone();
  intake();
  pros::delay(900);
  stopIntake();
  
  // // //get matchload
  chassis.turnTo(44.372, -50.019, 790);
  chassis.moveToPose(44.372,-46.019, 145, 2700);
  // //chassis.turnTo(60.314, -29.101, 2000);
  
  chassis.moveToPose(67.314,-23.377, 0, 2000, {.chasePower = 50000});
  chassis.waitUntil(2.5);
  wingsSolanoidRight.set_value(true);
  pros::delay(600); 
  wingsSolanoidRight.set_value(false);
  chassis.moveToPose(69.314,-40.413, 0, 1000, {.forwards = false });
  chassis.turnTo(69.017, -53.088, 800);
  chassis.moveToPose(73.314,-27.377, 180, 2000, {.forwards = false, .chasePower = 50000});
  chassis.moveToPose(73.314,-40.413, 180, 1000, {.forwards = false});


}



void skillsAuto(){
  chassis.setPose(-46.115, -57.157, 235);

  cata.move(127);
  pros::delay(27000);
  cata.move(0);
  chassis.moveToPose(-32.896, -59.03, 90, 4000);
  chassis.moveToPose(35.064, -59.705, 90, 4000);
  chassis.waitUntilDone();
  openWings();
  
  chassis.moveToPose(59.718, -30.998,0, 4000);
  chassis.waitUntil(10);
  closeWings();
  chassis.moveToPose(59.718, -40.689,180, 4000,{.forwards = false});
  chassis.moveToPose(59.718, -30.998,180, 4000, {.chasePower = 50});
  chassis.moveToPose(59.718, -40.689,180, 4000,{.forwards = false});
  chassis.moveToPose(59.718, -30.998,180, 4000, {.chasePower = 50});
  chassis.moveToPose(59.718, -40.689,180, 4000,{.forwards = false});
  chassis.moveToPose(8.722, -26.27, 290, 4000);
  chassis.moveToPose(42.832, -10.735, 90, 4000);
  chassis.waitUntil(10);
  openWings();
  pros::delay(2000);
  closeWings();
  chassis.moveToPose(5.682, -10.735, 90, 4000, {.forwards = false});
  chassis.turnTo(6.02, 37.897, 4000);
  chassis.moveToPose(6.02, 37.897, 0, 4000);
  chassis.moveToPose(39.117, 12.23, 90, 4000);
  chassis.moveToPose(18.1, 32.479, 135,4000, {.forwards = false});
  chassis.moveToPose(43.39, 53.349, 47.5, 4000);
  chassis.moveToPose(58.367, 23.64, 180, 4000);




   
}

void tune(){
  chassis.setPose(34.551, -61.805, 0);

  chassis.moveToPose(34.551, -61.805, 180, 4000);
  chassis.moveToPose(34.551, -61.805, 360, 4000);

}






void autonomous() {
  normalShort();
  

}


 

void screen() {


}





void competition_initialize() {
	


}






void arcadeDrive() {
  int forward = cntrlr.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int turn = cntrlr.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

  // Calculate the left and right motor velocities
  int left_velocity = forward + turn;
  int right_velocity = forward - turn;

  

  
    // Set the left and right motor velocities
    left.move(left_velocity);
    right.move(right_velocity);
}



void tankDrive() {
    // Read joystick values
    int leftY = cntrlr.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = cntrlr.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);


    // Set motor voltages
    left.move(leftY);
    right.move(rightY);

}











void opcontrol() {

    // autonomous();
    // pros::delay(150000); 

    bool wingsOut = true;
    bool hangerUp = true;
    


    // CataStateMachine cata {};
    // cata.initialize();

    
    
   
	


	while (true) {
		
		
		
		arcadeDrive(); 
		

    if(cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      intake();

    }
    else if(cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      outtake();
    }
    else{
      stopIntake();
    }

    if(cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      cata.move(127);
    }
    if(cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      cata.move(0);
    }
     
		if(cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_A) && wingsOut) {
			openWings();
      pros::delay(500);
      wingsOut = false;
    }
    else if(cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_A) && !wingsOut){
      closeWings();
      pros::delay(500);
      wingsOut = true;
    }
    if(cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
			hangerSolanoid.set_value(true);
      
    }
    else if(cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
      hangerSolanoid.set_value(false);
      
    }

        
        
        




       

		

		
	}
}