const double PI=3.1415;
const int DISTANCE_FOR_AUTOMODE_DRIVE_FORWARD = 100;
const char* CAMERA_NAME = "cam1"; //this is the name of the camera on the roborio

// buttons
const int TILT_FORWARD_BUTTON=3;
const int CLOSE_HOLDER_BUTTON=4;
const int OPEN_HOLDER_BUTTON=5;
const int LIFT_ARMS_BUTTON=6;
const int LOWER_ARMS_BUTTON=7;
const int OPEN_ARMS_BUTTON=8;
const int CLOSE_ARMS_BUTTON=9;
const int DRIVE_ARM_TOGGLE=1;
const int READY_CONTAINER_PICKUP_BUTTON=10;
const int LIFT_CONTAINER_BUTTON=11;


const int OPSTICK_ARM_TOGGLE=1;
const int OP_STACK_UP=6;
const int OP_STACK_DOWN=7;

// PWMs
#ifdef FRC2014
const int WINCH_PWM=3;
const int DRIVE_LEFT_FRONT_PWM=4;
const int DRIVE_LEFT_BACK_PWM=5;
const int DRIVE_RIGHT_FRONT_PWM=6;
const int DRIVE_RIGHT_BACK_PWM=7;
const int STACKER_PWM=9;
#else
const int WINCH_PWM=5;
const int DRIVE_LEFT_FRONT_PWM=1;
const int DRIVE_LEFT_BACK_PWM=2;
const int DRIVE_RIGHT_FRONT_PWM=3;
const int DRIVE_RIGHT_BACK_PWM=4;
#endif

// relay
const int COMPRESSOR_RELAY = 1;

// digital IO
#ifdef FRC2014
const int RIGHT_ENCODER_A_DIO = 9;
const int RIGHT_ENCODER_B_DIO = 14;
const int PRESSURE_INPUT_DIO = 5;
const int BOTTOM_LIMIT_DIO = 11;
const int DROPOFF_LIMIT_DIO = 12;
const int PICK_UP_LIMIT_DIO = 2;
#else
const int RIGHT_ENCODER_A_DIO = 6;
const int RIGHT_ENCODER_B_DIO = 7;
const int LEFT_ENCODER_A_DIO = 8;
const int LEFT_ENCODER_B_DIO = 9;
const int PRESSURE_INPUT_DIO = 5;
const int BOTTOM_LIMIT_DIO = 13;
const int DROPOFF_LIMIT_DIO = 14;
const int PICK_UP_LIMIT_DIO = 2;
#endif

// joystick ports
#ifdef FRC2014
const int DRIVER_JOYSTICK_PORT=1;
const int OPERATOR_JOYSTICK_PORT=2;
#else
const int DRIVER_JOYSTICK_PORT=0;
const int OPERATOR_JOYSTICK_PORT=1;
#endif

//analog
#ifdef FRC2014
const int GYRO_ANALOG_CHANNEL=1;
#else
const int GYRO_ANALOG_CHANNEL=0;
#endif

// solenoids
#ifdef FRC2014
const int TILT_BACK_SOLENOID=1;
const int TILT_FORWARD_SOLENOID=2;
const int OPEN_ARMS_SOLENOID=3;
const int CLOSE_ARMS_SOLENOID=4;
const int OPEN_HOLDER_SOLENOID=5;
const int CLOSE_HOLDER_SOLENOID=6;
#else
const int OPEN_HOLDER_SOLENOID=5;
const int CLOSE_HOLDER_SOLENOID=0;
const int OPEN_ARMS_SOLENOID=2;
const int CLOSE_ARMS_SOLENOID=3;
const int TILT_BACK_SOLENOID=1;
const int TILT_FORWARD_SOLENOID=4;
#endif

//automodes
const int DRIVE_FORWARD=0;
