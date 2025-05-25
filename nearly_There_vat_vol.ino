
										  //////////////////////////////////////////////////////////////////////////////////////////////////
										  //              Program Name =    AGRINET_VAT_TEMP_1_5_7    version date = 09/09/18            //
										  //                                copyright Russ Mcmillan 2018                                //
										  ///////////////////////////////////////////////////////////////////////////////////////////////
										  //                                     VIRTUAL PINS                                         //
										  /////////////////////////////////////////////////////////////////////////////////////////////
										  // V0  | V1  |     |     |     |     | AM and PM timer for cooling alarm                  //
										  // V3  |     |     |     |     |     | Milk vat temp                                     //   |  | V24 | |  |  
										  // v4  |     |     |     |     |     | LOCK BUTTON                                      //
										  // V5  |     |     |     |     |       Temperature correction (+ | -)                  //
										  // V6  |     |     |     |     |        cone shape switch                             //
										  // V10 |vat demen  | V13  V14 Temp core| Text from Widget for instruction             //
										  // V16 |     |     |     |     |     | Carrier telco avaiable  
										  // V17                                 signal strength 
										  // V19                                 total vat volume 
										  // V21 |     |                         Firmware version                             //
										  // V20 |     |     |     |     |     | LED alarm                                   //
										  // V9  |     |     |     |     |     | cooling profile "ON/OFF" text              //
										  // V2  |     |     |     |     |     | vat type
										  // V56                                 H2O temp of cip 
										  //  V55                                plant wash temp
										  // V23                                 Max Plant wash temp   
										  // V22                                  time stamp of max plant 
										  // V59                                 vat measurements activeate
										  //V25                                 menu of what to change 
										  // V28                                  button to activate OTA
										  // V26                                   OTA saftey 
										  // V11                                  vat demsinput
										  //
										  //////////////////////////////////////////////////////////////////////////////////
										  //*************************************   TO DO LIST  ****************************************************
										  //         
										  //	     
										  //
										  //
										  //
										  //
										  //
										  //
										  //
										  //
										  //
										  //
										 //
										 //////////////////////////////////////////////////////////////////////////////////

										  
										  /////////////////////     test address  ////////////////////////
										/*
																		#define BLYNK_TEMPLATE_ID "TMPLAMVMsierra02"
															#define BLYNK_TEMPLATE_NAME "AgriNet Milk Vat Monitor WIFI OTA"
															#define BLYNK_AUTH_TOKEN "iX66Y7txTBsp69Fa7SoWPGn_rnLZMkht"   ////// marks unit
										*/				   
									
										  #define BLYNK_TEMPLATE_ID "TMPLAMVMsierra02"
										  #define BLYNK_TEMPLATE_NAME "AgriNet Milk Vat Monitor WIFI OTA"
										  #define BLYNK_AUTH_TOKEN "j8c_rMSe7nG9KesikYq2v9gw1U6kvc-l"
									
										  #define BLYNK_PRINT Serial
										  #define LED_pin 4  // LED pins for wifi and power
										  #define power_pin 0
							#define _USE_MATH_DEFINES

										  #include "SPI.h"
										  #include <Arduino.h>
										  #include <WiFiManager.h>
										  #include <BlynkSimpleEsp32.h>
										  #include <HTTPClient.h>
										  //#include <Update.h>       //*********   library might need updateing or removing from sketch **************//
										  #include <EEPROM.h>
										  #include "TimeLib.h"
										  #include "DS18B20.h"
										  
										  #include <WiFi.h>  // For Wi-Fi functionality
										  
										  #include <ModbusMaster.h>
										  #include <math.h>
									
									

										  ModbusMaster node;

										  /////////////////////////////////////////////////////////////////////////////////////////////////
										  //            CLIENT ATHORITY CODE FOR BOTH SIERRA AND MKR 100
										  ///////////////////////////////////////////////////////////////////////////////////////////////////  ***************

										  char AUTH[] = BLYNK_AUTH_TOKEN;  // ADD CLIENT CODE HERE FROM BLY 2.00      ***********************************************************************

										  //////////////////////////////////////////////////////////////////////////////////////////////////    ****************
										  //----------------------------------------------------------------------------------------------------------------------
										  // location of firmware file on external web server
										  // // change to your actual .bin location
										  // //*********************************************************************************************************//
										  #define FWURL "https://agrinetcloud.com/OTA/TPG/MILKTEMPLEVEL/AGRI_TEMP_LEVEL_OTA_TPG_Ver__"  // removeextra  - \
																																						  // //software will add the version and .bin at the end, add to host papa fw versnumber plus .bin loose .ino \
																																						  // //*********************************************************************************************************//




										  #define EEPROM_SIZE 3
										  #define TRIGGER_PIN 32  //wifi reset address
#define NUM_CONE_SLICES 100


									// --- Modbus (Pressure) Sensor Setup ---
									#define PRESSURE_REG     0x0004    // Modbus register for pressure reading
									


									#define RX_PIN           26        // GPIO26 for Modbus RX
									#define TX_PIN           27        // GPIO27 for Modbus TX
									HardwareSerial modbusSerial(2);




										  ///////////////////////////////////////////////////////////////
										  //                     min--sec--millisec
										  //////////////////////////////////////////////////////////////
										  #define CoolingPhase1 20 * (60 * 1000)   // 20 MINS
										  #define CoolingPhase2 40 * (60 * 1000)   // 40 MINS
										  #define CoolingPhase3 60 * (60 * 1000)   // 60 MINS
										  #define CoolingPhase4 80 * (60 * 1000)   // 80 MINS
										  #define CoolingPhase5 120 * (60 * 1000)  // 120 MINS
										  #define CoolingPhase6 150 * (60 * 1000)  // 150 MINS  =2.5 HRS
										  #define CoolingPhase7 155 * (60 * 1000)  //  final check to ensure temp is at 5 degrees or less
																				  /*
																							  #define CoolingPhase1  2 * (60 * 1000)        // 20 MINS
																							  #define CoolingPhase2  4 * (60 *1000)        // 40 MINS
																							  #define CoolingPhase3  6 * (60 * 1000)      // 60 MINS 
																							  #define CoolingPhase4  8 *(60 * 1000)      // 80 MINS
																							  #define CoolingPhase5  12 *(60 * 1000)    // 120 MINS  
																							  #define CoolingPhase6  15 *(60 * 1000)   // 150 MINS  =2.5 HRS
																							  #define CoolingPhase7  15.5 *(60 * 1000)   // 
																							  */
										  ///////////////////////////////////////////////////////////
										  //Data wire for temp probe is plugged into pin #
										  //////////////////////////////////////////////////////////
										  //*************************************
										  //                                   //  CHECK PORT CONNECTION
										  //                                 //*********************************
										  #define ONE_WIRE_BUS 25  //  needs to be on esp32 temp1 (33)and temp2 (25)


										  ///////////////////////////////////////////////////////////////////////////////////////////////////
										  // Create a variable to hold timer id
										  ////////////////////////////////////////////////////////////////////////////////////////////////
										  BlynkTimer timer;

										  WiFiManager wm;

										  HTTPClient client;

										  int timeout = 240;  // seconds to run the portal for

										  int timerId;

										  ///////////////////////////////////////////////////////////////////////////////////////////////////
										  // Flag to indicate which cooling phase is running to update LEDs
										  //////////////////////////////////////////////////////////////////////////////////////////////////

										  byte fLEDs = 0;

										  ///////////////////////////////////////////////////////////////////////////////////////////////////
										  // Setup a oneWire instance to communicate with any OneWire devices
										  ////////////////////////////////////////////////////////////////////////////////////////////////

										  OneWire oneWire(ONE_WIRE_BUS);

										  ///////////////////////////////////////////////////////////////////////////////////////////////////
										  // Pass our oneWire reference
										  ////////////////////////////////////////////////////////////////////////////////////////////////

										  DS18B20 sensor(&oneWire);

										  ///////////////////////////////////////////////////////////////////////////////////////////////////
										  // Fuction to set our colors for app to change too
										  ////////////////////////////////////////////////////////////////////////////////////////////////

										  #define BLYNK_BLACK "#000000"
										  #define BLYNK_BLUE "#0525a8"    //was dark blue
										  #define BLYNK_RED "#EF0053"     //                            blynk colors    red ="#D3435C"
										  #define BLYNK_GREEN "#008000"   // COLORS TO CHANGE TO                         green = "#23C48E"
										  #define BLYNK_YELLOW "#f9ac1b"  // PERSONAL CHOICE TO                           Yellow blynk "#ED9D00"
										  //#define BLYNK_OTHERBLUE          "#0089B7"         //   TRY AND MATCH OUR LOGO                      blue = "#0081FF"   2583EB
										  #define BLYNK_TELSTRA "#000099"        //  try to match telstra color bar
										  #define BLYNK_TELSTRAYELLOW "#ff751a"  //    AND NAME
										  #define BLYNK_BLUEOPTUS "#FDCC08"      //    try to match optus color BAR
										  #define BLYNK_OPTUS "#006181"          //   AND NAME
										  #define BLYNK_SPECIALGREEN "#00b300"







										  //////////////////////////////////////////////////////////////////////////////////////////////////////
										  //        Global variables TEMP
										  ////////////////////////////////////////////////////////////////////////////////////////////////////
										  float max_temp;
										  float PLANT_WASH;            ///  equals temperatureInside
										  float HOT_WATER;             //   equals temperatureOutside
										  float TempCorrection = 0;       //  Used to adjust readings, if the sensor needs calibration = 0	  
										 // bool ButtonUnlock = false;
										  
										  
										  
										  float MilkVatTemp  ;           //  Milk temperature after correction
float milkNow =6;    // to be removed once temp robe attached 
										  int notified = 0;      //  Notification flag
										  int Notif = 0;         //  Notification flag  for final step
										  String Response = "";  //  Text output to SETTINGS value widget
										  int ButtonUnlock;      //  TO ACTIVATE TEMP CHANGE
										  int statusButtonAccess;
										  String receivedLabel = "";
										  String sendslave = "";
										  String wifiNum = "";
										  
										  /*********************** Reconction Vaiables*****************************/
										  int ReCnctCount = 0;  // Reconnection counter
										  int ReCnctFlag;

										  /*************** Keep this flag not to re-sync on every reconnection****/

										  bool isFirstConnect = true;
										  boolean reconnectionScheduledFlag = false;  // Reconnection scheduled Flag
										  int reconnectionCount = 0;                  // Reconnection counter
										  int i;
										  bool wifiBegun(false);


										  boolean flagEmergencyButtonUnlocked = false;




										  
										  long t;
										  String CurrentTime;
										  String OTA_UpDate = "23:30";     // update firmware at this time everyday in 24 hour format time
										  String CLear_AM_high = "14:30";  // Reset high temp at 2:30 PM
										  String CLear_PM_high = "04:30";  // Reset high temp at 4:30 AM


										  bool res;
										  bool safetyEnabled = false;         // Variable to track if the safety button is enabled
										  unsigned long safetyHoldStartTime;  // Variable to track when the hold starts

										  const unsigned long holdDuration = 15000;  // Hold time in milliseconds (15 seconds) OTA saftey


										  //int FWversion = 0;
										  int FWversion;          //  int FWversion;   set as 0 for first download  int FWversion = 0;
										  int totalLength;        //total size of firmware
										  int currentLength = 0;  //current size of written firmware
										  int fwNewVersion;

											 //***************presure sensor level variables********************************* 
											 ////////////////////////////////////////////////////////////////////////////////
											 
											 
																	// globals
									bool VATButtonUnlock = false;
									int  orientation     = 0;  // 0=vertical,1=horizontal
									int  coneTYPE        = 0;  // 0=none,1=elliptical,2=conical,3=spherical
									int  optionSelectedMenu = 0;
									
									
											 // Tank geometry (millimeters)
									 float R             = 0;  // Radius
									 float L             = 0;  // Cylinder body length
									 float CAP           = 0;   // End-cap length for conical/elliptical/spherical
									 float H_MAX         = 0;  // Max fluid height
									float REF_HEIGHT_MM = H_MAX * 0.5; // Reference calibration height

									// Sensor calibration: empty offset (raw count at 0 mm)
									const int RAW_EMPTY = 10;  //9


									// --- Fluid Properties ---
									const float REF_TEMP       = 10;    // °C at which DENSITY_REF applies 10
									const float DENSITY_REF    =  1033.0;  // kg/m³ @ 10 °C    1033.0;  = milk
									const float DENS_COEFF     = 0.32;    // kg/m³ per °C
									const float PRESSURE_SCALE = 100.0;   // Pa per raw count
									const float G              = 9.80665; // m/s²

									// Derived constants
									float rawRefEstimated; // Estimated raw count at REF_HEIGHT_MM
									float heightScale;     // mm per raw count
									
                   float tankVolumeTotal = 0.0f;
							float readTemperature();
							float computeDensity(float tempC);
							float readHeight(float density);


									/* Function prototypes
									static float horizontalCylinderVolume(float r, float length, float h);
									// static float coneSegmentVolume(float r, float H, float h); // Old prototype
									static float horizontalConicalCapVolume(float R_base, float H_cap, float y_fill_tank, int num_slices); // New prototype
									// static float sphericalSegmentVolume(float r, float h); // Old prototype
									static float sphericalSegmentVolume(float R_tank, float r_cap_depth, float h_fill_in_cap); // New prototype
								// static float tankCalcHorizontalEllipticalCapVolume(float R_tank_base, float cap_depth_D, float y_fill_tank, int num_slices); // New prototype


									static float computeTotalVolume(); */


// --- UTILITY ---
float minf(float a, float b) { return (a < b) ? a : b; }
float maxf(float a, float b) { return (a > b) ? a : b; }

// --- PATCH: Convert user-entered horizontal cap depth (yellow) to vertical dome height ---
/* Now unused
float horizontalToVerticalCapDepth(float radius, float horizontalCap) {
    // For a true 2:1 elliptical end: b = a/2, horizontal depth d = a - sqrt(a^2 - b^2)
    // Solving for b: b = sqrt(a^2 - (a - d)^2)  [a=radius, d=horizontal cap]
    if (horizontalCap <= 0) return 0;
    if (horizontalCap >= radius) return radius / 2.0f;
    float b = sqrtf(radius * radius - (radius - horizontalCap) * (radius - horizontalCap));
    return b;
}
*/

// --- VOLUME GEOMETRY ---
// float verticalTankVolume(float r, float h) { // Now unused
//     return PI * r * r * h / 1e6f;
// }
float horizontalCylinderVolume(float r, float length, float h) {
    h = constrain(h, 0.0f, 2.0f*r);
    if (h <= 0) return 0;
    if (h >= 2.0f*r) return PI * r * r * length / 1e6f;
    float theta = acosf((r - h) / r);
    float area  = r * r * (theta - sinf(2.0f * theta) / 2.0f);
    return area * length / 1e6f;
}

// Calculates volume of one horizontal conical end cap using numerical integration of circular segments.
// R_base: Radius of the cone at its base (where it meets cylinder) (mm)
// H_cap: Length/height of the conical cap (mm)
// y_fill_tank: Overall fill height in the tank, measured from the bottom of the tank (mm)
// num_slices: Number of slices for numerical integration
// Returns volume in Liters
float horizontalConicalCapVolume(float R_base, float H_cap, float y_fill_tank, int num_slices) {
    if (y_fill_tank <= 0 || H_cap <= 0 || R_base <= 0) return 0.0f;

    float total_volume_mm3 = 0.0f;
    float dx = H_cap / (float)num_slices; // Thickness of each slice

    for (int i = 0; i < num_slices; i++) {
        // x is distance from the cylinder-cone interface into the cone (from base towards apex)
        float x = (i + 0.5f) * dx; 
        
        // Radius of the cone slice at this x
        float r_slice = R_base * (1.0f - x / H_cap);
        if (r_slice <= 0) continue; // Should not happen if x < H_cap

        // Determine the fill height within this specific circular slice.
        // y_fill_tank is measured from the bottom of the main tank cylinder.
        // The bottom of this slice is at (R_base - r_slice) above the tank centerline,
        // or effectively, its lowest point relative to y_fill_tank's reference can be complex.
        // Let's simplify: The circular slice is vertical. Its center is at height R_base from tank bottom.
        // The liquid level y_fill_tank cuts this slice.
        // h_in_slice is the height of liquid in this current vertical circular slice.
        
        float h_in_slice = constrain(y_fill_tank, 0.0f, 2.0f * r_slice);
        
        if (h_in_slice <= 1e-3) { // Effectively empty or too small
            continue;
        }
        if (fabs(h_in_slice - 2.0f * r_slice) < 1e-3) { // Effectively full circle
            total_volume_mm3 += PI * r_slice * r_slice * dx;
            continue;
        }

        float acos_arg = (r_slice - h_in_slice) / r_slice;
        // Clamp argument to acos to prevent domain errors due to floating point inaccuracies
        if (acos_arg > 1.0f) acos_arg = 1.0f;
        if (acos_arg < -1.0f) acos_arg = -1.0f;

        float theta = acosf(acos_arg); // This is half the angle of the segment
        
        // Area of circular segment: r_slice^2 * (theta - sin(theta)*cos(theta))
        // Or R^2 * arccos((R-h)/R) - (R-h) * sqrt(2Rh - h^2)
        // Using the second form:
        float term_R_minus_h = r_slice - h_in_slice;
        float sqrt_arg = 2.0f * r_slice * h_in_slice - h_in_slice * h_in_slice;
        if (sqrt_arg < 0) sqrt_arg = 0; // Clamp due to potential float inaccuracies

        float area_segment = r_slice * r_slice * acosf(term_R_minus_h / r_slice) - term_R_minus_h * sqrtf(sqrt_arg);
        
        total_volume_mm3 += area_segment * dx;
    }
    
    return total_volume_mm3 / 1e6f; // Convert mm^3 to Liters
}

// Calculates volume of one horizontal spherical end cap using TankCalc's deduced method.
// R_tank: Radius of the cylindrical part of the tank (mm)
// r_cap_depth: Depth of the spherical cap (Arduino's CAP variable) (mm)
// h_fill_in_cap: Fill height within this specific cap (i.e., min(overall_fill_height, r_cap_depth)) (mm)
// Returns volume in Liters
float sphericalSegmentVolume(float R_tank, float r_cap_depth, float h_fill_in_cap) {
    if (h_fill_in_cap <= 0 || r_cap_depth <= 0) return 0.0f;

    // Ensure h_fill_in_cap does not exceed r_cap_depth
    h_fill_in_cap = constrain(h_fill_in_cap, 0.0f, r_cap_depth);

    // Calculate the radius of the sphere (mr) from which the cap is made.
    // mr = (cap_depth^2 + R_tank^2) / (2 * cap_depth)
    float mr = (r_cap_depth * r_cap_depth + R_tank * R_tank) / (2.0f * r_cap_depth);

    // Volume of a spherical cap: V = (1/3) * PI * h^2 * (3*mr - h)
    // where h is h_fill_in_cap and mr is the sphere's radius.
    float volume_mm3 = (1.0f / 3.0f) * PI * h_fill_in_cap * h_fill_in_cap * (3.0f * mr - h_fill_in_cap);
    
    return volume_mm3 / 1e6f; // Convert mm^3 to Liters
}
// --- TRUE ELLIPSOIDAL (TankCalc-style) END CAP --- (Now unused for horizontal elliptical, replaced by tankCalcHorizontalEllipticalCapVolume)
/*
float ellipsoidalCapVolume(float a, float b, float h) {
    // a = major (radius), b = minor (vertical dome, not horizontal!), h = fill height (≤ b)
    if (h <= 0) return 0;
    if (h >= b) return (2.0f / 3.0f) * PI * a * b * b / 1e6f; // full cap
    float x = h / b;
    return PI * a * b * b / 3.0f * (2 - x) * x * x / 1e6f; // mm³ to L
}
*/

// Calculates volume of one horizontal elliptical end cap using numerical integration of circular segments.
// R_tank_base: Radius of the cylinder at the point where the cap attaches (mm).
// cap_depth_D: The depth of the elliptical cap (length along the tank's main axis from base to tip of cap) (mm).
// y_fill_tank: Overall fill height in the tank, measured from the absolute bottom of the tank (mm).
// num_slices: Number of slices for numerical integration.
// Returns volume in Liters.
float tankCalcHorizontalEllipticalCapVolume(float R_tank_base, float cap_depth_D, float y_fill_tank, int num_slices) {
    if (y_fill_tank <= 1e-3f || cap_depth_D <= 1e-3f || R_tank_base <= 1e-3f) {
        return 0.0f;
    }

    float total_volume_mm3 = 0.0f;
    float dx = cap_depth_D / (float)num_slices; // Thickness of each slice

    for (int i = 0; i < num_slices; i++) {
        // x_pos_in_cap is distance from the cylinder-cap interface into the cap (from base towards apex)
        float x_pos_in_cap = (i + 0.5f) * dx;

        // Calculate the radius of the vertical circular slice at this x_pos_in_cap
        // Ellipse formula: (x/D)^2 + (r_slice/R_base)^2 = 1
        // Our x_pos_in_cap is from base (0) to depth D (tip).
        // So, r_slice = R_tank_base * sqrt(1 - (x_pos_in_cap / cap_depth_D)^2)
        float r_slice_sq_term = x_pos_in_cap / cap_depth_D;
        float r_slice = R_tank_base * sqrtf(1.0f - r_slice_sq_term * r_slice_sq_term);

        if (r_slice <= 1e-3f) { // Effectively zero radius or invalid
            continue;
        }

        // Determine the fill height within this specific vertical circular slice.
        float h_in_slice = constrain(y_fill_tank, 0.0f, 2.0f * r_slice);

        if (h_in_slice <= 1e-3f) { // Effectively empty or too small to calculate segment
            continue;
        }
        
        float area_segment;
        if (fabsf(h_in_slice - 2.0f * r_slice) < 1e-3f) { // Effectively full circle for this slice
            area_segment = PI * r_slice * r_slice;
        } else {
            // Area of circular segment: r_slice^2 * acos((r_slice-h_in_slice)/r_slice) - (r_slice-h_in_slice) * sqrt(2*r_slice*h_in_slice - h_in_slice^2)
            float term_R_minus_h = r_slice - h_in_slice;
            
            float acos_arg = term_R_minus_h / r_slice;
            acos_arg = constrain(acos_arg, -1.0f, 1.0f); // Clamp argument to acos
            
            float sqrt_arg = 2.0f * r_slice * h_in_slice - h_in_slice * h_in_slice;
            if (sqrt_arg < 0) sqrt_arg = 0; // Clamp due to potential float inaccuracies

            area_segment = r_slice * r_slice * acosf(acos_arg) - term_R_minus_h * sqrtf(sqrt_arg);
        }
        
        total_volume_mm3 += area_segment * dx;
    }

    return total_volume_mm3 / 1e6f; // Convert mm^3 to Liters
}

// --- Sloped Floor Volume (vertical only) ---
float computeVolumeVertical(float measuredH) {
    // CAP for vertical tanks is slope in degrees.
    // Check if the floor is flat (slope angle CAP is zero or negligible)
    if (fabsf(CAP) < 1e-3f) { // Assuming CAP holds the slope angle for vertical tanks
        // Flat bottom: Use direct formula PI * R^2 * H
        return PI * R * R * measuredH; // Result in mm^3
    } else {
        // Sloped bottom: Use existing numerical integration
        const int N = 25;
        float dy = (2 * R) / N;
        float theta = CAP * PI / 180.0f; // Degrees to radians
        float hDrop = 2 * R * tanf(theta);
        float vol = 0;
        for (int i = 0; i < N; i++) {
            float y = -R + (i + 0.5f) * dy;
            float floorOffset = ((y + R) / (2 * R)) * hDrop;
            float localH = measuredH - floorOffset;
            if (localH < 0) localH = 0;
            // The following constraint might need review based on H_MAX definition,
            // but for now, retain original logic for sloped part.
            if (localH > H_MAX + hDrop) localH = H_MAX + hDrop; 
            
            float sliceWidth = 2 * sqrtf(R * R - y * y);
            vol += sliceWidth * localH * dy;
        }
        return vol; // Result in mm^3
    }
}

// --- TANK CAPACITY (MAX) ---
float computeTankCapacity() {
    float vol = 0;
    if (orientation == 0) {
        vol = computeVolumeVertical(H_MAX) / 1e6f; // Modified for consistency
    } else {
        // float b = horizontalToVerticalCapDepth(R, CAP); // Keep for now
        vol = horizontalCylinderVolume(R, L, 2.0f * R);
        switch (coneTYPE) {
            case 1: vol += 2.0f * tankCalcHorizontalEllipticalCapVolume(R, CAP, 2.0f * R, NUM_CONE_SLICES); break; // Elliptical, updated in prior step
            case 2: vol += 2.0f * horizontalConicalCapVolume(R, CAP, 2.0f * R, NUM_CONE_SLICES); break; // Conical
            case 3: // Spherical
                  // This is the line to change:
                  // vol += 2.0f * sphericalSegmentVolume(R, CAP, CAP); // Old call (CAP for full fill in cap)
                  // Change to:
                  vol += 2.0f * tankCalcHorizontalEllipticalCapVolume(R, R, 2.0f * R, NUM_CONE_SLICES); // New call for spherical, using R for cap depth, 2.0f*R for full height
                  break;
        }
    }
    return vol;
}

// --- SENSOR HELPERS ---
float readTemperature() {
    sensor.requestTemperatures();
    return sensor.getTempC() + TempCorrection;
}
float computeDensity(float tC) {
    return DENSITY_REF + DENS_COEFF * (REF_TEMP - tC);
}
float readHeight(float density) {
    node.readHoldingRegisters(PRESSURE_REG, 1);
    uint16_t raw = node.getResponseBuffer(0);
    int16_t corr = maxf((int)raw - RAW_EMPTY, 0);
    float pPa    = corr * PRESSURE_SCALE;
    return constrain((pPa / (density * G)) * 1000.0f, 0.0f, H_MAX);
}

// --- VOLUME BY SENSOR FILL ---
float computeTotalVolume() {
    float dens = computeDensity(readTemperature());
    float measuredH = readHeight(dens);
    float vol = 0.0f;
    if (orientation == 0) {
        float vol_mm3 = computeVolumeVertical(measuredH);
        vol = vol_mm3 * 1e-6f;
    } else {
        // float b = horizontalToVerticalCapDepth(R, CAP); // Keep for now
        float vol_cyl = horizontalCylinderVolume(R, L, measuredH);
        float vol_caps = 0;
        switch (coneTYPE) {
            case 1: vol_caps = 2 * tankCalcHorizontalEllipticalCapVolume(R, CAP, measuredH); break;
            case 2: vol_caps = 2 * horizontalConicalCapVolume(R, CAP, measuredH, NUM_CONE_SLICES); break; // Updated call
            case 3: vol_caps = 2 * sphericalSegmentVolume(R, CAP, minf(measuredH, CAP)); break; // Updated call
        }
        vol = vol_cyl + vol_caps;
    }
    return vol;
}



										  void setup() {

											Serial.begin(115200);
											// Initialize Modbus UART
									  modbusSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
									  node.begin(1, modbusSerial);

									  Serial.println("Waiting for user to configure tank dimensions...");

											
											EEPROM.begin(EEPROM_SIZE);
											sensor.begin();
											sensor.setResolution(12);
											// EEPROM.write(0, FWversion); // comment out after first upload  ****************************************
											//EEPROM.commit();         //  comment out after first upload  ****************************************
											pinMode(LED_pin, OUTPUT);
											pinMode(power_pin, OUTPUT);
											pinMode(TRIGGER_PIN, INPUT_PULLUP);

											WiFi.mode(WIFI_STA);
											// invert theme, dark
											wm.setDarkMode(true);


											// Add custom header
											const char *customHeader = "<h1>AGRINET SOLUTIONS</h1>";
											wm.setCustomHeadElement(customHeader);


											//add custom parameters
											WiFiManagerParameter custom_text("<p>Select your Wi-Fi address from those visible and enter your password then select save. Your unit should connect automatically.</p>");
											wm.addParameter(&custom_text);
											res = wm.autoConnect("  AGRINET WIFI ");
											//automatically connect using saved credentials if they exist
											if (!res) {
											  Serial.println("Starting configuration portal.");
											} else {

											  Serial.println("Connected to WiFi successfully.");
											  Serial.print("STA IP Address: ");
											  Serial.println(WiFi.localIP());
											}



                       tankVolumeTotal = computeTankCapacity();
											//LockReset();  //   Sending Lock to V4 , String to V10 and V13
											// Seed V5 (orientation) and V3 (cap type) labels:
									  Blynk.setProperty(V5, "labels", "Vertical Vat","Horizontal Vat");
									  // Start in Vertical mode:
									  Blynk.virtualWrite(V5, orientation);

									  // For V6, only “None” when vertical, but we’ll call updateCapMenu()
									  updateMenus();

											//////////////////////////////////////////////////////////////////////////////////////////////////////
											// Timers for  functions to be called
											/////////////////////////////////////////////////////////////////////////////////////////////////////

											timer.setInterval(15005L, sendUptime);

											//timer.setInterval(3610L, dataLedWidget);
											//timer.setInterval(3303L, PowerOutLED);  // sierra
											timer.setInterval(140170L, top_Temp);


											timer.setInterval(33310L, CheckWifiBlynkConnection);
											timer.setInterval(3370L, fire_wifistartup);
											//timer.setInterval(85800005L, checkupdate);

											timer.setInterval(37000L, checkupdate);  // use for testing of OTA use other for field
											timer.setInterval(21013L, time_to_get_newfirmware);
											timer.setInterval(24011L, time_Clear_HiTemp);  // time to reset hotwater max temps


											timer.setInterval(11011L, requestTime);
											//timer.setInterval(130030L, processStoredData);



											Serial.println("\nSETUP() FINISHED");
										  #if defined DEBUG
											Serial.println("\nWORKING WITH NORMAL DEBUG...");
										  #endif
										  #if defined DEBUG_LD
											Serial.println("\nWORKING WITH LIMITED DEBUG...");
										  #endif
										  #if not defined DEBUG and not defined DEBUG_LD
											Serial.println("\nWORKING WITHOUT DEBUG...");
										  #endif
										  }


										  //------------------------------------------------------------------------------
										  BLYNK_CONNECTED()
										  //------------------------------------------------------------------------------
										  {
											Blynk.sendInternal("rtc", "sync");
											//reconnectionCount = 0;
											ReCnctFlag = 1;
											// Blynk.syncVirtual(V42);
										  #if defined DEBUG
											Serial.print("\n---------- Messages BLYNK_CONNECTED()");
										  #endif


										  #if defined DEBUG
											Serial.print("---------- End Messages BLYNK_CONNECTED()");
										  #endif
										  }





										  //---------------------------------------------------------------------------------
										  //    Main Loop
										  //---------------------------------------------------------------------------------
										  void loop() {
											timer.run();
											Blynk.run();
											wm.process();
										  }


										  BLYNK_WRITE(V55) {
											PLANT_WASH = param.asFloat();  // Receive data sent from Device B
											Serial.print("Received PLANT_WASH: ");
											Serial.println(PLANT_WASH);
											Blynk.virtualWrite(V15, PLANT_WASH);  // Inside temperature
										  }

										  BLYNK_WRITE(V56) {
											HOT_WATER = param.asFloat();
											Serial.print("Received HOT_WATER: ");
											Serial.println(HOT_WATER);
											Blynk.virtualWrite(V18, HOT_WATER);  // Outside temperature
										  }

										  BLYNK_WRITE(V57) {
											receivedLabel = param.asStr();
											Serial.print("Received Label: ");
											Serial.println(receivedLabel);
											Blynk.virtualWrite(V30, receivedLabel);  // Slave label
										  }
										  BLYNK_WRITE(V58) {
										   wifiNum  = param.asStr();
											Serial.print("Received Label: ");
											Serial.println(wifiNum );
											//Blynk.virtualWrite(V30, receivedLabel);  // Slave channel
										  }
										  BLYNK_WRITE(V59) {
										   sendslave = param.asStr();
											Serial.print("Received Label: ");
											Serial.println(sendslave);
											//Blynk.virtualWrite(V30, receivedLabel);  // Slave channel
										  }

										  


										  //***************************************************
										  // Clear high temperature at specific times (AM/PM)
										  //***************************************************
										  void time_Clear_HiTemp() {
											requestTime();  // Sync and get the current time

											// Reset the high temperature at specified times
											if (CurrentTime == CLear_AM_high) {
											  Reset_hot_Temps();
											} else if (CurrentTime == CLear_PM_high) {
											  Reset_hot_Temps();
											}
										  }

										  //--------------------------------------------------------
										  // Function to reset both milking plant and max temperatures
										  //--------------------------------------------------------
										  void Reset_hot_Temps() {
											PLANT_WASH = 0;  // Reset current temperature
											max_temp = 0;    // Reset maximum recorded temperature
											Serial.println("Temperatures reset to 0.");
											Blynk.virtualWrite(V23, max_temp);    // Update display for the reset
											Blynk.virtualWrite(V15, PLANT_WASH);  // Inside temperature
										  }

										  /***************************************************
															// Function to track and update the highest temperature
															//***************************************************/
										  void top_Temp() {
											float milking_Plant = PLANT_WASH;  // Assuming PLANT_WASH holds the temperature value

											// Print the current milking_Plant and max_temp for debugging
											Serial.print("Current PLANT_WASH temp: ");
											Serial.println(milking_Plant);
											Serial.print("Current max_temp: ");
											Serial.println(max_temp);

											if (milking_Plant > max_temp) {
											  max_temp = milking_Plant;              // Update max_temp with current reading
											  Blynk.virtualWrite(V23, max_temp);     // Update Numeric Display with max temp
											  Blynk.virtualWrite(V22, CurrentTime);  // Log the current time when max temp is recorded

											  // Output to serial the new max temp and time
											  Serial.print("New high temp: ");
											  Serial.println(max_temp);

											  // Print the recorded time in a more reliable way
											  Serial.print("Recorded at: ");
											  Serial.println(CurrentTime);  // Make sure CurrentTime is up to date and valid
											}
										  }





										  // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -
										  //          TIME FUNCTIONS TO SET TIME STAMP ON MOTOR START
										  //--------------------------------------------------------------------------

										  void requestTime() {

											Blynk.sendInternal("rtc", "sync");
											//
											CurrentTime = twoDigits(hour()) + ":" + twoDigits(minute());
											//                CurrentTime = String(hour()) + ":" + minute() + ":" + second();
											Serial.print("current time = ");
											Serial.print(CurrentTime);
											Serial.println();
										  }

										  BLYNK_WRITE(InternalPinRTC) {
											t = param.asLong();
											setTime(t);
											// CurrentTime = t;
											Serial.print(t);
											Serial.println();
										  }

										  //-------------------- utility function for digital clock display: prints leading 0

										  String twoDigits(int digits) {
											if (digits < 10) {
											  String i = '0' + String(digits);
											  return i;
											} else {
											  return String(digits);
											}
										  }





										  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
										  /////////    WIFI connection and reconnect function runs every 15 second
										  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

										  // Retry logic for Wi-Fi and Blynk connection
										  // Constants for Wi-Fi and Blynk connection management
										  const int maxWiFiRetries = 4;             // Max retries for Wi-Fi reconnection attempts
										  const int maxBlynkRetries = 4;            // Max retries for Blynk reconnection attempts
										  const int maxFailureCount = 5;            // Max number of connection check failures before restart
										  unsigned long previousAttemptMillis = 0;  // Store the time of the last Wi-Fi retry
										  const long retryInterval = 4000;          // Interval between Wi-Fi retries (2 seconds)
										  int failureCount = 0;                     // Track the number of connection check failures
										  bool isConnectionCheckRunning = false;    // Flag to prevent overlapping calls



										  void CheckWifiBlynkConnection() {
											if (isConnectionCheckRunning) {
											  return;  // Exit if a previous call is still running
											}
											isConnectionCheckRunning = true;
											Serial.println("---------- Checking connection");

											// Check Blynk connection status
											if (Blynk.connected()) {
											  Serial.println("Connected to Blynk server!");
											  blinkLed(9, 110);            // Visual indicator for success
											  int32_t rssi = WiFi.RSSI();  // Get WiFi signal strength in dBm
											  int strengthPercent = map(rssi, -100, -38, 0, 100);
											  Blynk.virtualWrite(V16, " WIFI ");
											  Blynk.setProperty(V16, "color", BLYNK_SPECIALGREEN);
											  Blynk.virtualWrite(V17, strengthPercent);
											  Blynk.setProperty(V17, "color", BLYNK_SPECIALGREEN);

											  Serial.print("WiFi Signal Strength: ");
											  Serial.print(strengthPercent);
											  Serial.println("%");
											  failureCount = 0;  // Reset failure count
											  isConnectionCheckRunning = false;
											  return;  // Exit if already connected to Blynk
											}

											// Check Wi-Fi connection status
											if (WiFi.status() != WL_CONNECTED) {
											  Serial.println("Not connected to Wi-Fi!");
											  int wifiRetries = 0;
											  while (WiFi.status() != WL_CONNECTED && wifiRetries < maxWiFiRetries) {
												unsigned long currentMillis = millis();
												if (currentMillis - previousAttemptMillis >= retryInterval) {
												  previousAttemptMillis = currentMillis;  // Save the last retry time
												  Serial.println("Attempting to reconnect to Wi-Fi...");
												  WiFi.begin();  // Reconnect using saved credentials in NVS
												}
												if (WiFi.status() == WL_CONNECTED) {
												  Serial.println("Connected to Wi-Fi!");
												  break;
												} else {
												  wifiRetries++;
												  Serial.print("Wi-Fi reconnection attempt ");
												  Serial.println(wifiRetries);
												  power_led_on(3, 250);
												}
											  }
											  // If Wi-Fi reconnection fails after max retries
											  if (WiFi.status() != WL_CONNECTED) {
												Serial.println("Failed to connect to Wi-Fi after maximum retries.");
												IncrementFailureCount();
												isConnectionCheckRunning = false;
												return;  // Exit if Wi-Fi connection failed
											  }
											}

											// Attempt to reconnect to Blynk if needed
											int blynkRetries = 0;
											while (!Blynk.connected() && blynkRetries < maxBlynkRetries) {
											  Serial.println("Not connected to Blynk server! Attempting to reconnect...");
											  Blynk.config(AUTH, "agrinet.io", 8080);  // Configure Blynk connection
											  Blynk.connect(5000);                     // Force Blynk to reconnect with a 5-second timeout

											  if (Blynk.connected()) {
												Serial.println("Connected to Blynk server!");
												String receivedMac = WiFi.macAddress();  // Get MAC address
												//String wifi_Num  = WiFi.channel();
												Blynk.syncVirtual(V42);                  // Sync virtual pin after connection
												Blynk.virtualWrite(V27, receivedMac);    // Send MAC address to Blynk
												 //Blynk.virtualWrite(V25, receivedMac); 
												failureCount = 0;  // Reset failure count
												isConnectionCheckRunning = false;
												return;  // Exit once connected to Blynk
											  } else {
												blynkRetries++;
												Serial.print("Blynk reconnection attempt ");
												Serial.println(blynkRetries);
											  }
											}

											// If Blynk reconnection fails after max retries
											if (!Blynk.connected()) {
											  Serial.println("Failed to connect to Blynk after maximum retries.");
											  IncrementFailureCount();
											  power_led_on(6, 250);
											}

											Serial.println("---------- End connection check");
											isConnectionCheckRunning = false;
										  }

										  // Function to increment the failure count and restart the ESP if the limit is reached
										  void IncrementFailureCount() {
											failureCount++;
											Serial.print("Failure count: ");
											Serial.println(failureCount);

											if (failureCount >= maxFailureCount) {
											  Serial.println("Max failure count reached, restarting ESP...");
											  ESP.restart();  // Restart the ESP32
											}
										  }
										  //----------------------------------------------------
										  void resetWifi() {
											//----------------------------------------------------
											WiFi.disconnect();
											wm.resetSettings();
											Serial.println("Resetting WiFi credentials.");

											wm.setConfigPortalTimeout(timeout);

											if (!wm.startConfigPortal("AGRINET WIFI CONNECT")) {
											  Serial.println("Failed to connect and hit timeout, restarting.");
											  delay(2000);
											  ESP.restart();
											  delay(1000);
											}

											Serial.println("Connected to WiFi successfully.");
											Serial.println("connected...yeey :)");
										  }
										  //***************************************************
										  void fire_wifistartup()
										  //***************************************************
										  {
											if (digitalRead(TRIGGER_PIN) == LOW) {
											  resetWifi();
											}
											int pinState = digitalRead(TRIGGER_PIN);  // Read the state of digital pin 2
											Serial.print("The state of TRIGGER_PIN is: ");
											Serial.println(pinState);  // Print the state of the pin
											Serial.println();
										  }
										  void power_led_on(int repeats, int time)

										  {
											for (int i = 0; i < repeats; i++) {
											  digitalWrite(power_pin, HIGH);
											  delay(time);
											  digitalWrite(power_pin, LOW);
											  delay(time);
											}
										  }
										  //***********************************************
										  void blinkLed(int repeats, int time)
										  //***********************************************
										  {
											for (int i = 0; i < repeats; i++) {
											  digitalWrite(power_pin, LOW);
											  delay(150);
											  digitalWrite(LED_pin, HIGH);
											  delay(time);
											  digitalWrite(LED_pin, LOW);
											  delay(time);
											  delay(150);
											  digitalWrite(power_pin, HIGH);
											}
										  }
										  //***************************************************
										  void time_to_get_newfirmware()
										  //***************************************************
										  {
											// requestTime(); // Update CurrentTime before checking for update time
											if (CurrentTime == OTA_UpDate) {
											  checkupdate();
											}
										  }
										  // Function to handle virtual button press for OTA
										  BLYNK_WRITE(V28) {
											int buttonState = param.asInt();          // Get the state of the OTA button
											if (buttonState == 1 && safetyEnabled) {  // If OTA button is pressed and safety is enabled
											  Serial.println("Safety is enabled. Starting OTA...");
											  checkupdate();          // Process OTA update
																	  // Reset safety after OTA process is initiated (or you can reset it after OTA completes)
											  safetyEnabled = false;  // Reset safety after OTA
											} else if (buttonState == 1) {
											  Serial.println("Safety not enabled. Cannot start OTA.");
											}
										  }
										  BLYNK_WRITE(V26) {
											int buttonSaftey = param.asInt();  // Get the state of the virtual safety button
											if (buttonSaftey == 1) {           // Button pressed
											  safetyHoldStartTime = millis();  // Start the timer when the button is pressed
											  Serial.println("Safety button is being held...");
											} else if (buttonSaftey == 0) {                             // Button released
											  unsigned long holdTime = millis() - safetyHoldStartTime;  // Calculate hold time

											  if (holdTime >= holdDuration) {
												safetyEnabled = true;  // Enable safety if held long enough
												Serial.println("Safety enabled after holding for 3 seconds.");
											  } else {
												Serial.println("Safety not enabled. Button was not held long enough.");
											  }
											}
										  }


										  //***************************************************
										  void checkupdate()
										  //****************************************************
										  {
											// Connect to external web server
											Serial.println("Checking if new firmware is available.");

											FWversion = EEPROM.read(0);
											Serial.print("Current firmware: v");
											Serial.println(FWversion);
											Serial.println("HOPE THIS WORKS AS IAM OFF TO BED IF IT  DOES ");
											Serial.print("Firmware: V");
											Serial.println(FWversion);
											Blynk.virtualWrite(V21, "Version:  " + String(FWversion));
											Blynk.setProperty(V21, "color", BLYNK_BLUE);

											fwNewVersion = FWversion + 1;  //cerca la versione successiva del firmware.
											String fwVersionURL = FWURL + String(fwNewVersion) + ".bin";
											Serial.println(fwVersionURL);
											Serial.print("new firmware is ");
											Serial.println(fwNewVersion);
											client.begin(fwVersionURL);
											// Get file, just to check if each reachable
											int resp = client.GET();
											Serial.print("Response: ");
											Serial.println(resp);
											// If file is reachable, start downloading
											if (resp == 200) {
											  FWversion = FWversion + 1;
											  Serial.print("updated firmware is ");
											  Serial.println(FWversion);
											  EEPROM.write(0, FWversion);
											  EEPROM.commit();
											  //Blynk.virtualWrite(V1, FWversion);power_pinV10
											  //Blynk.setProperty(V1, "color", BLYNK_RED);
											  Blynk.virtualWrite(V21, "Firmware Updated at: " + CurrentTime);
											  // get length of document (is -1 when Server sends no Content-Length header)
											  totalLength = client.getSize();
											  // transfer to local variable
											  int len = totalLength;
											  // this is required to start firmware update process
											  Update.begin(UPDATE_SIZE_UNKNOWN);
											  Serial.printf("FW Size: %u", totalLength);
											  // create buffer for read
											  uint8_t buff[128] = { 0 };
											  // get tcp stream
											  WiFiClient *stream = client.getStreamPtr();
											  // read all data from server
											  Serial.println("Updating firmware...");
											  //Blynk.virtualWrite(V1, "Firmware Updated :" +CurrentTime );
											  while (client.connected() && (len > 0 || len == -1)) {
												// get available data size
												size_t size = stream->available();
												if (size) {
												  // read up to 128 byte
												  int c = stream->readBytes(buff, ((size > sizeof(buff)) ? sizeof(buff) : size));
												  // pass to function
												  updateFirmware(buff, c);
												  if (len > 0) {
													len -= c;
												  }
												}
												delay(1);
											  }
											} else {
											  Serial.println("Cannot download firmware file.");
											}


											client.end();
										  }
										  //********************************************************************
										  // Function to update firmware incrementally
										  // Buffer is declared to be 128 so chunks of 128 bytes
										  // from firmware is written to device until server closes

										  void updateFirmware(uint8_t *data, size_t len)

										  {
											Update.write(data, len);
											currentLength += len;
											// Print dots while waiting for update to finish
											Serial.print('.');
											// if current length of written firmware is not equal to total firmware size, repeat
											if (currentLength != totalLength) return;
											Update.end(true);
											// Serial.printf("Update Success, Total Size: %uRebooting...", currentLength);
											// Restart ESP32 to see changes
											Serial.println("update worked ok ");
											ESP.restart();
										  }






										  ///////////////////////////////////////////////////////////////////////////////////////////////////
										  //     Function to Update LEDS if app was turned off
										  //////////////////////////////////////////////////////////////////////////////////////////////////
										  void updateLEDWidgets() {
										   
											switch (fLEDs) {
											  case 0:  //All off
											   
												Blynk.virtualWrite(V9, String(" "));
												break;
											  case 1:  //Cooling Phase 1
											   
												Blynk.virtualWrite(V9, String(" Cooling check is #1 'ACTIVATED' "));
												Blynk.setProperty(V9, "color", BLYNK_BLUE);
												break;
											  case 2:  //Cooling Phase 2
												
												Blynk.virtualWrite(V9, String(" Cooling check is #2 'ACTIVATED' "));
												Blynk.setProperty(V9, "color", BLYNK_BLUE);
												break;
											  case 3:  //Cooling Phase 3
											   
												Blynk.virtualWrite(V9, String(" Cooling check is  #3 'ACTIVATED' "));
												Blynk.setProperty(V9, "color", BLYNK_BLUE);
												break;
											  case 4:  //Cooling Phase 4
											   
												Blynk.virtualWrite(V9, String(" Cooling check is #4 'ACTIVATED' "));
												Blynk.setProperty(V9, "color", BLYNK_BLUE);
												break;
											  case 5:  //Cooling Phase 5
											   
												Blynk.virtualWrite(V9, String(" Cooling check is #5 'ACTIVATED' "));
												Blynk.setProperty(V9, "color", BLYNK_BLUE);
												break;
											  case 6:  //Cooling Phase 6
											   
												Blynk.virtualWrite(V9, String(" Cooling check is #6 'ACTIVATED' "));
												Blynk.setProperty(V9, "color", BLYNK_BLUE);
												break;
											  default:  //All off
											   
												Blynk.virtualWrite(V9, String(" "));
											}
										  }

										  /////////////////////////////////////////////////
										  //   temp read and report function
										  /////////////////////////////////////////////////
										  void firstRead() {
											if (MilkVatTemp >= 25 && notified == 0) {
											  notified = 1;  //Set flag

											  Blynk.logEvent("milk_temperature_");
											} else if (MilkVatTemp < 22) {
											  // set lower than 25... to stop the 24.95-25.0 bouncing.
											  notified = 0;
											}
										  }
										  void secondRead() {
											if (MilkVatTemp >= 21 && notified <= 1) {
											  // check to see if flag is also set
											  notified = 2;

											  Blynk.logEvent("milk_temperature_");

											} else if (MilkVatTemp < 19) {
											  // set lower than 21... to stop the 20.95-21.0 bouncing.
											  notified = 0;
											}
										  }
										  void thirdRead() {
											if (MilkVatTemp >= 18 && notified <= 2) {
											  // check to see if flag is also set
											  notified = 3;

											  Blynk.logEvent("milk_temperature_");
											} else if (MilkVatTemp < 16) {
											  // set lower than 18... to stop the 17.95-18.0 bouncing.
											  notified = 0;
											}
										  }
										  void fourthRead() {
											if (MilkVatTemp >= 15 && notified <= 3) {
											  // check to see if flag is also set
											  notified = 4;

											  Blynk.logEvent("milk_temperature_");
											} else if (MilkVatTemp < 12) {
											  // set lower than 15... to stop the 14.95-15.0 bouncing.
											  notified = 0;
											}
										  }
										  void fifthRead() {
											if (MilkVatTemp >= 11 && notified <= 4) {
											  // check to see if flag is also set
											  notified = 5;

											  Blynk.logEvent("milk_temperature_");
											} else if (MilkVatTemp < 9) {
											  // set lower than 11... to stop the 10.95-11.0 bouncing.
											  notified = 0;
											}
										  }
										  void sixthRead() {
											if (MilkVatTemp >= 8 && notified <= 5) {
											  // check to see if flag is also set
											  notified = 6;

											  Blynk.logEvent("milk_temperature_");

											} else if (MilkVatTemp < 5) {
											  // set lower than 8... to stop the 7.95-8.0 bouncing.
											  notified = 0;
											}
										  }

										  void Final_Look() {
											if ((MilkVatTemp <= 7.9) && (MilkVatTemp >= 5.1) && (Notif == 0)) {
											  Notif = 1;
											  Blynk.logEvent("final_temp_check");
											  //                   Serial.println (" the final check has been done and found to be hot ");
											} else if (MilkVatTemp < 5) {
											  Notif = 0;
											  //                    Serial.println (" the final check was done and is ok  ");
											}
										  }


										  //////////////////////////////////////////////////////////////////////////////////////////////
										  //   Cooling profile flags
										  //////////////////////////////////////////////////////////////////////////////////////////////
										  void Profile1_ON() {
											fLEDs = 1;  // Update flag
											firstRead();  //  Check temp and report back via notification
										  }
										  void Profile2_ON() {
											fLEDs = 2;  // Update flag
											secondRead();  //check temp and report back via notification
										  }
										  void Profile3_ON() {
											fLEDs = 3;  // Update flag
											thirdRead();  //check temp and report back via notification
										  }
										  void Profile4_ON() {
											fLEDs = 4;  // Update flag
											fourthRead();  //check temp and report back via notification
										  }
										  void Profile5_ON() {
											fLEDs = 5;  // Update flag
											fifthRead();  //check temp and report back via notification
										  }
										  void Profile6_ON() {
											fLEDs = 6;  // Update flag
											sixthRead();  //check temp and report back via notification
										  }
										  void Profile6_OFF() {
											fLEDs = 0;  // Update flag       
										  }
										  void Profile_final_Check() {
											Final_Look();
										  }

										  ///////////////////////////////////////////////////////////////////////////////////////////////////
										  // Cooling Profile Timers Function
										  ///////////////////////////////////////////////////////////////////////////////////////////////////
										  void actionCooling() {
											timer.setTimeout(CoolingPhase1, Profile2_ON);   // delay 20 min then turn on profile2
											timer.setTimeout(CoolingPhase2, Profile3_ON);   // start  P3 20 min after P2
											timer.setTimeout(CoolingPhase3, Profile4_ON);   // start  P4 20 min after P3
											timer.setTimeout(CoolingPhase4, Profile5_ON);   // start  P5 20 min after P4
											timer.setTimeout(CoolingPhase5, Profile6_ON);   // turn on P6 after 20 min
											timer.setTimeout(CoolingPhase6, Profile6_OFF);  // turn off P6 after 20
											timer.setTimeout(CoolingPhase7, Profile_final_Check);
											Profile1_ON();
										  }
										  //////////////////////////////////////////////////////////////////////
										  // Function to fetch milk temperature periodically (every 3000 ms)
										  /////////////////////////////////////////////////////////////////////////
									   
									// --- ADDED FUNCTION ---

							static float estimateRawAtHeight(float height_mm) {
							  float density = DENSITY_REF + DENS_COEFF * (REF_TEMP - TempCorrection);
							  float pressure_pa = (density * G * height_mm) / 1000.0;
							  return pressure_pa / PRESSURE_SCALE + RAW_EMPTY;
							}

							// --- UPDATED sendUptime FUNCTION LOGIC ---
							// --- UPDATED sendUptime FUNCTION LOGIC ---
						// === MAIN SENDUPTIME LOGIC ===
//float MilkVatTemp = readTemperature(); // uncomment
// === MAIN SENDUPTIME LOGIC ===
// --- sendUptime LOGIC (patch: uses vertical dome cap depth for ellipsoidal) ---

void sendUptime() {
	float tempoffset = 6; // used as no temp prode as it is -127
    //float MilkVatTemp = readTemperature();
		float MilkVatTemp = tempoffset + TempCorrection;
    Serial.printf("orientation=%d (0=Vertical, 1=Horizontal)\n", orientation);
    Serial.printf("Temp: %.1f °C  ", MilkVatTemp);
    float density = computeDensity(MilkVatTemp);
    Serial.printf("Density: %.1f kg/m³  ", density);
    if (R <= 0 || H_MAX <= 0) {
        Serial.println("Tank dimensions not configured. Skipping volume computation.");
        return;
    }
    if (node.readHoldingRegisters(PRESSURE_REG, 1) == node.ku8MBSuccess) {
        uint16_t raw = node.getResponseBuffer(0);
        int16_t corrected = (int16_t)raw - RAW_EMPTY;
        corrected = maxf(corrected, 0);
        float pressure_pa = corrected * PRESSURE_SCALE;
        Serial.printf("Pressure: %.1f Pa  ", pressure_pa);
        float height_mm = constrain((pressure_pa / (density * G)) * 1000.0f, 0.0f, H_MAX);
        Serial.printf("Height: %.1f mm  ", height_mm);
        float vol = 0.0f;
        if (orientation == 0) {
            float vol_mm3 = computeVolumeVertical(height_mm); // mm³
            vol = vol_mm3 * 1e-6f; // litres
            // Serial.printf("verticalTankVolume(R=%.1f, h=%.1f)\n", R, height_mm); // Call removed
        } else {
            // float b = horizontalToVerticalCapDepth(R, CAP); // Keep for now, might be used by computeTotalVolume or other logic
            vol = horizontalCylinderVolume(R, L, height_mm);
            Serial.printf("horizontalCylinderVolume(R=%.1f, L=%.1f, h=%.1f)\n", R, L, height_mm);
            switch (coneTYPE) {
                case 1:
                    // Use the new function for horizontal elliptical caps
                    vol += 2.0f * tankCalcHorizontalEllipticalCapVolume(R, CAP, height_mm, NUM_CONE_SLICES);
                    break;
                case 2:
                    vol += 2.0f * horizontalConicalCapVolume(R, CAP, height_mm, NUM_CONE_SLICES); // Updated call
                    break;
                case 3: // Spherical
                    // This is the line to change:
                    // vol += 2.0f * sphericalSegmentVolume(R, CAP, minf(height_mm, CAP)); // Old call
                    // Change to:
                    vol += 2.0f * tankCalcHorizontalEllipticalCapVolume(R, R, height_mm, NUM_CONE_SLICES); // New call for spherical
                    break;
            }
        }
        float pct = (tankVolumeTotal > 0.1f) ? (vol / tankVolumeTotal) * 100.0f : 0.0f;
        Serial.printf("Volume: %.1f L  (%.1f%% full)\n", vol, pct);
        Blynk.virtualWrite(V19, vol);
        Blynk.virtualWrite(V12, pct);
    } else {
        Serial.println("Modbus read error");
    }
    Serial.print("Temp: ");
    Serial.println(MilkVatTemp);
    Serial.print("Temperature Correction: ");
    Serial.println(TempCorrection);
    Blynk.virtualWrite(V3, MilkVatTemp);
   colorUptime();
}



										  

										  //////////////////////////////////////////////////////////////////////
										  // Function for color change  for visual apprasial
										  /////////////////////////////////////////////////////////////////////

										  void colorUptime() {
											if (MilkVatTemp < 5) {
											  Blynk.setProperty(V3, "color", BLYNK_GREEN);   // green gauge
											  Blynk.setProperty(V19, "color", BLYNK_GREEN);  // green gauge
												 Blynk.setProperty(V12, "color", BLYNK_GREEN);  // green gauge
											} else if (MilkVatTemp > 10) {
											  Blynk.setProperty(V3, "color", BLYNK_RED);   // red gauge
											  Blynk.setProperty(V19, "color", BLYNK_RED);  // red gauge
                        Blynk.setProperty(V12, "color", BLYNK_RED);  // red gauge
											} else {
											  Blynk.setProperty(V3, "color", BLYNK_YELLOW);   // yellow gauge
											  Blynk.setProperty(V19, "color", BLYNK_YELLOW);  // yellow gauge
                        Blynk.setProperty(V12, "color", BLYNK_YELLOW);  // yellow gauge
											}
											Blynk.virtualWrite(V42, TempCorrection, PLANT_WASH, HOT_WATER, max_temp, H_MAX, L, CAP, R);  //   add other bits to this 
											Serial.print("---------- Print to V42  ");
										  }



										  ///////////////////////////////////////////////////////////////////////////////////
										  //   Timer function to start cooling profiles
										  /////////////////////////////////////////////////////////////////////////////////
										  // AM Timer
										  //////////////////////////////////////////////////////////////////////////////////
										  BLYNK_WRITE(V0) {
											const int TimerAM = param.asInt();
											if (TimerAM == 1) {
											  Blynk.virtualWrite(V9, String(" Initiating Cooling Check "));
											  Blynk.setProperty(V9, "color", BLYNK_BLUE);
											  actionCooling();
											} else {
											  Blynk.virtualWrite(V9, String(" "));
											  updateLEDWidgets();
											}
										  }
										  ///////////////////////////////////////////////////////////////////////////////////////////////
										  // PM Timer
										  //////////////////////////////////////////////////////////////////////////////////////////////
										  BLYNK_WRITE(V1) {
											int TimerPM = param.asInt();
											if (TimerPM == 1) {
											  Blynk.virtualWrite(V9, String(" Initiating Cooling Check "));
											  Blynk.setProperty(V9, "color", BLYNK_BLUE);
											  actionCooling();
											} else {
											  Blynk.virtualWrite(V9, String(" "));
											  updateLEDWidgets();
											}
										  }
										  

										  ///////////////////////////////////////////////////////////////////////////////////////////////////
										  // Fuction to modifying code to get + or - step (step is either +1 or -1)
										  ////////////////////////////////////////////////////////////////////////////////////////////////
										  // Track lock state and current correction


									BLYNK_WRITE(V4) {
									  ButtonUnlock = param.asInt();

									  if (ButtonUnlock) {
										// UNLOCKED: show instructions and enable the ± widget
										Blynk.setProperty(V14, "color", BLYNK_BLUE);
										Blynk.virtualWrite(V14, "PRESS + or - TO CHANGE TEMP");
										Blynk.virtualWrite(V5, TempCorrection);
									  }
									  else {
										// LOCKED: show the live correction value
										Blynk.setProperty(V14, "color", BLYNK_BLACK);
										Blynk.virtualWrite(V14, 
										  String("VAT TEMP CORRECTION: ") + TempCorrection + "°"
										);
									  }
									}

									BLYNK_WRITE(V5) {
									  float newVal = param.asFloat();
									  if (ButtonUnlock) {
										TempCorrection = newVal;
										// immediately reflect the change in V14
										Blynk.virtualWrite(V14, 
										  String("VAT TEMP CORRECTION: ") + TempCorrection + "°"
										);
										Serial.print("TempCorrection = ");
										Serial.println(TempCorrection);
									  }
									  // if locked, ignore any presses
									}
float diameter;

										  BLYNK_WRITE(V42)
{
    TempCorrection = param[0].asFloat();
    PLANT_WASH     = param[1].asFloat();
    HOT_WATER      = param[2].asFloat();
    max_temp       = param[3].asFloat();
    H_MAX          = param[4].asFloat();

    L = param[5].asFloat();
    CAP              = param[6].asFloat();
     diameter   = param[7].asFloat();


    R = diameter / 2.0;                // Convert diameter to radius
    //L = totalLength - 2.0 * CAP;       // Subtract end caps to get cylinder length

    // Recalculate calibration and tank volume now that user inputs are known
    rawRefEstimated = estimateRawAtHeight(REF_HEIGHT_MM);
    heightScale     = REF_HEIGHT_MM / (rawRefEstimated - RAW_EMPTY);
    tankVolumeTotal = computeTankCapacity();

    Serial.printf("Calibrated: refRaw=%.1f, scale=%.3f mm/count, totalVol=%.1f L\n",
                  rawRefEstimated, heightScale, tankVolumeTotal);

    Serial.print("Temp Correction  syncronized (V42): ");
    Serial.println(TempCorrection);
    Serial.print("PLANT_WASH: "); Serial.println(PLANT_WASH);
    Serial.print("HOT_WATER: ");  Serial.println(HOT_WATER);
    Serial.print("max_temp: ");   Serial.println(max_temp);
    Serial.println(" degree");
    Serial.print(H_MAX  ); Serial.println(" max height mm");
    Serial.print(L  ); Serial.println(" length mm (cylinder only)");
    Serial.print(CAP  ); Serial.println(" cap mm");
    Serial.print(R ); Serial.println(" radius mm");
    Serial.println("---------- End Messages BLYNK_WRITE( V42)");
}



									//////////////////////////////////////////new menus and switches to alter V pins///////////////////////////////

								// --- Calibration Safety Lock (V50) ---
							BLYNK_WRITE(V50) {
								VATButtonUnlock = param.asInt();
								if (VATButtonUnlock) {
									Blynk.virtualWrite(V10, String("CALIBRATION UNLOCKED\nSelect VAT type"));
	                Blynk.setProperty(V10, "color", BLYNK_GREEN);
									Blynk.setProperty(V2, "color", BLYNK_GREEN);
									Blynk.setProperty(V6, "color", BLYNK_GREEN);
									Blynk.setProperty(V7, "color", BLYNK_BLACK);
										Blynk.virtualWrite(V7, String("SELECT THE MENU OPTION "));
								} else {
									Blynk.virtualWrite(V10, String("CALIBRATION LOCKED"));
                  	Blynk.setProperty(V10, "color", BLYNK_BLACK); 
									Blynk.setProperty(V2, "color", BLYNK_BLACK);
									Blynk.setProperty(V6, "color", BLYNK_BLACK);
                  Blynk.virtualWrite(V7, String(" "));

								}
							}

							BLYNK_WRITE(V2) {
    if (!VATButtonUnlock) return;
    orientation = param.asInt();  // 0 = vertical, 1 = horizontal
    updateMenus();

    // Correct tank volume update:
    if (orientation == 0) {
        tankVolumeTotal = computeVolumeVertical(H_MAX) / 1e6f; // Modified for consistency
    } else {
        // float b = horizontalToVerticalCapDepth(R, CAP); // Keep for now
        tankVolumeTotal = horizontalCylinderVolume(R, L, 2 * R);
        switch (coneTYPE) {
            case 1: tankVolumeTotal += 2.0f * tankCalcHorizontalEllipticalCapVolume(R, CAP, 2.0f * R, NUM_CONE_SLICES); break; // Elliptical, updated in prior step
            case 2: tankVolumeTotal += 2.0f * horizontalConicalCapVolume(R, CAP, 2.0f * R, NUM_CONE_SLICES); break; // Conical
            case 3: // Spherical
                  // tankVolumeTotal += 2.0f * sphericalSegmentVolume(R, CAP, CAP); // Old call (CAP for full fill in cap)
                  tankVolumeTotal += 2.0f * tankCalcHorizontalEllipticalCapVolume(R, R, 2.0f * R, NUM_CONE_SLICES); // New call for spherical, using R for cap depth, 2.0f*R for full height
                  break;
        }
    }
    Serial.printf("Orientation: %s\n", orientation == 0 ? "Vertical" : "Horizontal");
}

						BLYNK_WRITE(V6) {
    if (!VATButtonUnlock || orientation == 0) return;
    int idx = param.asInt();
    coneTYPE = idx + 1;
    tankVolumeTotal = computeTankCapacity();
}


BLYNK_WRITE(V25) {
    if (!VATButtonUnlock) return;
    optionSelectedMenu = param.asInt();
    Serial.printf("[V25] Menu Option Selected: %d (0=Diam,1=H/L,2=Cap)\n", optionSelectedMenu);
    Serial.printf("Current R=%.1f L=%.1f H_MAX=%.1f CAP=%.1f\n", R, L, H_MAX, CAP);
    switch (optionSelectedMenu) {
        case 0:
            Blynk.virtualWrite(V11, R * 2.0f);
            Blynk.virtualWrite(V7, String("VAT DIAMETER: ") + (R * 2.0f) + "MM");
						
            break;
        case 1:
            if (orientation == 0) {
                Blynk.virtualWrite(V11, H_MAX);
                Blynk.virtualWrite(V7, String("VAT HEIGHT: ") + H_MAX + "MM");
            } else {
                Blynk.virtualWrite(V11, L);
                Blynk.virtualWrite(V7, String("VAT LENGTH: ") + L + "MM");
            }
            break;
        case 2:
            Blynk.virtualWrite(V11, CAP);
            if (orientation == 0) {
                Blynk.virtualWrite(V7, String("SLOPE ANGLE: ") + CAP + "°");
            } else {
                Blynk.virtualWrite(V7, String("CONE DEPTH: ") + CAP + "MM");
            }
            break;
    }
	
    
}



	BLYNK_WRITE(V11) {
    if (!VATButtonUnlock) return;
    float v = param.asFloat();
    switch(optionSelectedMenu) {
        case 0: R = v / 2.0f; break; // Diameter entry, convert to radius
        case 1:
            if (orientation == 0) H_MAX = v;
            else L = v;
            break;
        case 2: CAP = v; break;
    }
    tankVolumeTotal = computeTankCapacity();
    Serial.printf("[V11] Updated tankVolumeTotal: %.1f L\n", tankVolumeTotal);
}


							// --- Update Both Menus Based on Orientation ---
							void updateMenus() {
								if (orientation == 0) {
									// Vertical vat: Diameter, Height, Slope Angle
									Blynk.setProperty(V25, "labels", "Diameter of Vat","Height of Vat", "Slope of Floor");
									//Blynk.setProperty(V25, "values", "0,1,2");
									optionSelectedMenu = 0;
									Blynk.virtualWrite(V25, optionSelectedMenu);
									// Cone menu disabled
									Blynk.setProperty(V6, "labels", "None");
									coneTYPE = 0;
									Blynk.virtualWrite(V6, 0);
								} else {
									// Horizontal vat: Diameter, Length, Cone Depth
									Blynk.setProperty(V25, "labels", "Diameter of Vat", "Length of Vat", "End Cap Depth (horizontal, mm)");
									//Blynk.setProperty(V25, "values", "0,1,2");
									optionSelectedMenu = 0;
									Blynk.virtualWrite(V25, optionSelectedMenu);
									// Enable cone type selection
									Blynk.setProperty(V6, "labels", " Elliptical", "Conical", "Spherical");
									if (coneTYPE == 0) coneTYPE = 1;
									Blynk.virtualWrite(V6, coneTYPE - 1);
								}
							}

									////////////////////////////////////////by google 

									
									
									

									
									

								
							
							


						


							

							
								

									

