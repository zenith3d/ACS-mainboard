/*
 * mainboard.ino
 * 
 * Nicolas BROSSEAU
 * 27/11/2019
 */
 
#include <math.h>
#include <SPI.h>
#include "mcp_can.h"


/***   CAN IDENTIFIERS   ***/
#define SPI_CS_PIN          9       // SPI CS pin attached to the CAN shield
#define CAN_INT_PIN         2       // CAN interrupt pin
#define CAN_ID_ACCEL_X      0x10    // Motorboard (X axis) acceleration data
#define CAN_ID_ACCEL_Y      0x11    // Motorboard (Y axis) acceleration data
#define CAN_ID_ACCEL_Z      0x12    // Motorboard (Z axis) acceleration data
#define CAN_ID_IMU          0x20    // IMU data
#define CAN_ID_SPEED_X      0x30    // Motorboard (X axis) speed data
#define CAN_ID_SPEED_Y      0x31    // Motorboard (Y axis) speed data
#define CAN_ID_SPEED_Z      0x32    // Motorboard (Z axis) speed data

/***   CONTROLLER   ***/
#define GAIN_P              4.5     // [s-2]  Proportional gain
#define GAIN_D              3.0     // [s-1]  Derivative gain  
#define INERTIA_RATIO_XX    60.16   // []     Approximation of {Inertia system}/{Inertia rotor}
#define INERTIA_RATIO_YY    55.38   // []     Approximation of {Inertia system}/{Inertia rotor}
#define INERTIA_RATIO_ZZ    49.64   // []     Approximation of {Inertia system}/{Inertia rotor}

/*** PATH TO FOLLOW ***/
#define PATH_INTERVAL       5000    // [ms]   The time before switching to a new attitude

/***   IMU FILTERING   ***/
#define GYRO_MIN            0.05    // [rad.s-1]  The minimum value tolerated to avoid bias

/***   STRUCTURES   ***/
typedef struct {
    float x;
    float y;
    float z;
} Vector;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

/***   CAN VARIABLES   ***/
MCP_CAN CAN(SPI_CS_PIN);            // Set the SPI CS pin
uint8_t can_id;
uint8_t can_dlc;
uint8_t can_buf[8];
uint8_t can_send_buf[8];
uint8_t can_rx_flag = 0;

uint8_t in_use = 0;

/***   GLOBAL VARIABLES   ***/
Vector wheels_speed = {0.0, 0.0, 0.0};      // [rad.s-1]    The 3 rotors rate of turn in the system frame
Vector rateofturn = {0.0, 0.0, 0.0};        // [rad.s-1]    The rate of turn in the system frame
Quaternion attitude = {1.0, 0.0, 0.0, 0.0}; // []           The attitude quaternion
Quaternion desired = {1.0, 0.0, 0.0, 0.0};  // []           The desired attitude
Vector desired_accel = {0.0, 0.0, 0.0};     // [rad.s-2]    The required wheels acceleration
Vector vector_error = {0.0, 0.0, 0.0};      // []           The error vector corresponding to the quaternion error

/***  PATH SETUP  ***/
Quaternion path[4];


/***   SETUP   ***/
void setup() {
    cli();

    // Initialize the timer used for attitude control at given frequency
    TCCR1A = 0x00;
    TCCR1B = 0x0C;  // CTC Mode | 256 prescaler (62,5kHz)
    TIMSK1 = 0x02;  // Output Compare A Match Interrupt Enable
    OCR1A = 62;     // 62500/62 = 1008Hz
    
    // Initialize the serial communication
    Serial.begin(500000);
    
    // Initilize the can bus at a 500kbps baudrate
    while (CAN.begin(CAN_500KBPS) != CAN_OK) delay(1000);

    // Attach interrupt to the CAN flag on CAN_INT_PIN
    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), MCP2515_ISR, FALLING);

    // Initialize motors accel to 0 rad/s²
    set_wheel_accel_X();
    set_wheel_accel_Y();
    set_wheel_accel_Z();

    // Initialiaze the path to follow
    path[0] = (Quaternion) {1.0, 0.0, 0.0, 0.0};
    path[1] = (Quaternion) {0.8944, -0.4472, 0.0, 0.0};
    path[2] = (Quaternion) {0.8944, 0.0, 0.4472, 0.0};

    sei();
    delay(10000);    
}

/***   LOOP   ***/
void loop() {

    in_use = 1;

    int k = (millis()/PATH_INTERVAL) % 3;
    //desired = {path[k].w, path[k].x, path[k].y, path[k].z};
    desired = {0.707, 0.0, 0.707, 0.0};
    
    //filter_IMU_data();
    compute_desired_accel();
    set_wheel_accel_X();
    set_wheel_accel_Y();
    set_wheel_accel_Z();

    Serial.print(k);
    Serial.print(" (");
    Serial.print(vector_error.x); Serial.print(",");
    Serial.print(vector_error.y); Serial.print(",");
    Serial.print(vector_error.z);
    Serial.print("),(");
    Serial.print(rateofturn.x); Serial.print(",");
    Serial.print(rateofturn.y); Serial.print(",");
    Serial.print(rateofturn.z);
    Serial.print("),(");
    Serial.print(wheels_speed.x); Serial.print(",");
    Serial.print(wheels_speed.y); Serial.print(",");
    Serial.print(wheels_speed.z);
    Serial.print(") -> (");
    Serial.print(desired_accel.x); Serial.print(",");
    Serial.print(desired_accel.y); Serial.print(",");
    Serial.print(desired_accel.z);
    Serial.println(")");
    
    in_use = 0;
    delay(50);
}


/**********************************/
/***     ATTITUDE CONTROLLER    ***/
/**********************************/
void compute_desired_accel() {  
    // Compute the error quaternion/vector
    vector_error.x = attitude.w * desired.x - attitude.x * desired.w - attitude.y * desired.z + attitude.z * desired.y;
    vector_error.y = attitude.w * desired.y + attitude.x * desired.z - attitude.y * desired.w - attitude.z * desired.x;
    vector_error.z = attitude.w * desired.z - attitude.x * desired.y + attitude.y * desired.x - attitude.z * desired.w;

    // Compute the required wheels acceleration
    desired_accel.x = +(-INERTIA_RATIO_XX * (GAIN_P * vector_error.x - GAIN_D * rateofturn.x) - (rateofturn.y * (-wheels_speed.z) - rateofturn.z * (-wheels_speed.y)));
    desired_accel.y = -(-INERTIA_RATIO_YY * (GAIN_P * vector_error.y - GAIN_D * rateofturn.y) - (rateofturn.z * (+wheels_speed.x) - rateofturn.x * (-wheels_speed.z)));
    desired_accel.z = -(-INERTIA_RATIO_ZZ * (GAIN_P * vector_error.z - GAIN_D * rateofturn.z) - (rateofturn.x * (-wheels_speed.y) - rateofturn.y * (+wheels_speed.x)));
}

void filter_IMU_data() {
    // Avoid gyro bias
    if (rateofturn.x > -GYRO_MIN && rateofturn.x < GYRO_MIN) rateofturn.x = 0.0;
    if (rateofturn.y > -GYRO_MIN && rateofturn.y < GYRO_MIN) rateofturn.y = 0.0;
    if (rateofturn.z > -GYRO_MIN && rateofturn.z < GYRO_MIN) rateofturn.z = 0.0;
}

/**********************************/
/***    CAN UPDATE FUNCTIONS    ***/
/**********************************/
void update_attitude() {
    /**
     * Update attitude and rate of turn values function corresponding to the can_buf[0] index
    **/
    switch (can_buf[0]){    
        case 0: memcpy(&attitude.w, &can_buf[1], sizeof(float)); break;
        case 1: memcpy(&attitude.x, &can_buf[1], sizeof(float)); break;
        case 2: memcpy(&attitude.y, &can_buf[1], sizeof(float)); break;
        case 3: memcpy(&attitude.z, &can_buf[1], sizeof(float)); break;
        case 4: memcpy(&rateofturn.x, &can_buf[1], sizeof(float)); break;
        case 5: memcpy(&rateofturn.y, &can_buf[1], sizeof(float)); break;
        case 6: memcpy(&rateofturn.z, &can_buf[1], sizeof(float)); break;
        default: break;
    }
}
void update_wheel_speed_X() {
    /**
     * Update wheel speed function when the corresponding  CAN message is received
    **/
    memcpy(&wheels_speed.x, &can_buf[0], sizeof(float));
}
void update_wheel_speed_Y() {
    /**
     * Update wheel speed function when the corresponding  CAN message is received
    **/
    memcpy(&wheels_speed.y, &can_buf[0], sizeof(float));
}
void update_wheel_speed_Z() {
    /**
     * Update wheel speed function when the corresponding  CAN message is received
    **/
    memcpy(&wheels_speed.z, &can_buf[0], sizeof(float));
}


/**********************************/
/***      CAN SEND FUNCTIONS    ***/
/**********************************/
void set_wheel_accel_X() {
    /**
     * Send the wheel acceleration on the X axis
     */
    memcpy(can_send_buf, &(desired_accel.x), sizeof(float));
    CAN.sendMsgBuf(CAN_ID_ACCEL_X, 0, 4, can_send_buf);
}
void set_wheel_accel_Y() {
    /**
     * Send the wheel acceleration on the Y axis
     */
    memcpy(can_send_buf, &(desired_accel.y), sizeof(float));
    CAN.sendMsgBuf(CAN_ID_ACCEL_Y, 0, 4, can_send_buf);
}
void set_wheel_accel_Z() {
    /**
     * Send the wheel acceleration on the X axis
     */
    memcpy(can_send_buf, &(desired_accel.z), sizeof(float));
    CAN.sendMsgBuf(CAN_ID_ACCEL_Z, 0, 4, can_send_buf);
}

/**********************************/
/***      CAN RECV FUNCTIONS    ***/
/**********************************/
void read_received_data() {
    /**
     * Read the received data
     */
    //cli();  // To prevent bugs ! (Aliasing on variables?)

    if(can_rx_flag) {
        can_rx_flag = 0;
        while (CAN_MSGAVAIL == CAN.checkReceive()) {
            CAN.readMsgBuf(&can_dlc, can_buf);      // Read the CAN msg in the buffer
            can_id = CAN.getCanId();                // Retrieve the msg ID
            switch(can_id) {
                case CAN_ID_SPEED_X: update_wheel_speed_X(); break;
                case CAN_ID_SPEED_Y: update_wheel_speed_Y(); break;
                case CAN_ID_SPEED_Z: update_wheel_speed_Z(); break;
                case CAN_ID_IMU: update_attitude(); break;
                default: break;
            }
        }
    }

    //sei();
}

ISR(TIMER1_COMPA_vect) {
    // Check for new data in the MCP2515 can buffer
    if (!in_use) read_received_data();
}

/**********************************/
/***        CAN INTERRUPT       ***/
/**********************************/
void MCP2515_ISR() {
    /**
     * CAN new message flag interruption
     */
    can_rx_flag = 1;
}
