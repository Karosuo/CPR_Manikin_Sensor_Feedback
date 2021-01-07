#include <FastLED.h>
#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

//#define DEBUG_STATES 1
//#define OTHER_SENSOR 1
//#define DEBUG_IDLE_TIMES 1
//#define DEBUG_PUSH_TIMES 1
//#define DEBUG_FINAL_VALS 1
//#define DEBUG_CALIBRATE_DISTANCE 1
//#define DEBUG_LED_STRIP_REF_LED 1
//#define DEBUG_LED_IDX_SELECTOR 1

//LED constants
const uint8_t num_of_leds = 60;
CRGB led_matrix[num_of_leds];
const uint8_t led_pin = 4;
const uint8_t led_strip_pwr_ctrl_pin = 6;
uint8_t base_color = 64; //indicates the color of the led strip, green or yellow
uint8_t c_led_top_offset = 0; //indicates the last led to be painted from center to sides

//Problem domain default consts
const uint8_t ok_led = 8; //The 'OK' green led
const uint8_t yellow_led = 7; //The led that indicates press started but not deep enough
const uint8_t bad_led = 9; //Indicates that pushed too far
const uint16_t unactive_dist = 150; //In mm, fixed distance when manikin is unactive
const uint32_t unactive_max_time = 4000000; //In micro secs, if unactive time reaches this, uC restart hash flag
const uint32_t unactive_max_time_range = 500000;
const uint16_t threshold_range = 5; // +- Xmm to hit thresholds
const uint16_t threshold_simple_push = 15; //In mm, minimum diminish of the distance to consider it a push
const uint16_t threshold_good_push = 65; //In mm, pushed distance from untouched_distance to consider it a correctly done push
const uint8_t frequency = 2; //In Hz, default 120 per minute
const uint32_t freq_range_time = 100000; //In micro secs, range to determine if time_delta is close to expected frequency
const uint32_t freq_period_time = 500000; //In micro secs, 1 / frequency (T), period time width

//counters and variables
uint16_t push_counter = 0;
uint64_t st_time = 0; //In micros, time delta to calculate the diff of micros between pushes (st_time is short for start_time)
uint64_t st_time_idle = 0; //In micros, same as above but for idle time
uint64_t time_bt_pushs = 0; //in micros, time between each push to calculate the frequency between them
int64_t excercise_hash = 0; //will save the random() value, to identify the current excercise measures
bool good_current_push = false; //Indicates if current press is valid 5cm deep (true) or not (false)
uint64_t sample_tstamp = 0; //same value that st_time, it will be sent as time ref of when was taken that measure
uint16_t sensor_distance = 0; //Wrapper for seansor measure, so maybe another sensor could be used
uint32_t idle_diff_time = 0; //Counts the diff between s0 and s0 measures in micro secs
bool target_within_range = false; //Assumes that initial sensor read is not within range
uint16_t last_dist = unactive_dist; //Previous sensor measure to know if current is deeper or not, if deeper it will be saved as current deepest



const uint8_t s0 = 0; //state 0
const uint8_t s1 = 1; //state 1
const uint8_t s2 = 2; //state 2
const uint8_t s3 = 3; //state 3

//States are as follows
// each state is represented by an int number from 0 to 3
// ex. S0 is numer 0, S1 is number 1 and so forth
// S0 -> Unactive, should be not pressed
//      when condition to change match, should get the diff of time and reset the start time reference
// S1 -> First press down of the cycle, do not do anything, it's just for reference for next push
// S2 -> First edge from down to up, do not need to do anything, it's just for reference for next push
// S3 -> Second press down of the cycle
//      when condition to change match, should get the new diff of time, send data over Bluetooth and reset the start time reference

uint8_t current_state = s0;

///Function prototypes
static bool is_pressed_down(uint16_t measure);
static bool is_press_ok(uint16_t measure);
static bool is_press_bad(uint16_t measure);
static bool is_freq_ok(uint64_t time_delta);
static void send_json_values();

void setup() {
  //init LED in FastLED
  FastLED.addLeds<WS2812B, led_pin, GRB>(led_matrix, num_of_leds);

  //Setup the led strips power control
  pinMode(led_strip_pwr_ctrl_pin, OUTPUT);

  //Always allow power to led strips
  digitalWrite(led_strip_pwr_ctrl_pin, HIGH);
  
  //Configure the green 'OK' led
  pinMode(ok_led, OUTPUT);

  
  Serial.begin(9600);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 

  //Analong pin 0 NEED TO BE DISCONNECTED
  // so the noise makes a good different seed for random
  // reference taken from https://www.arduino.cc/reference/en/language/functions/random-numbers/random/
  randomSeed(analogRead(0));

  //Create first exercise id
  excercise_hash = random(100); //Create new hash
}


void loop() {  
  VL53L0X_RangingMeasurementData_t measure;  
  //Read measurement and put it into the object vars
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!   

  #ifdef OTHER_SENSOR
    target_within_range = true; //Always enable the range
    sensor_distance = 0; //Read from the other sensor
  #else
    //If not != 4, means target is out of range
    if(measure.RangeStatus != 4)
    {
      target_within_range = true;  
    }
    else
    {
      target_within_range = false;  
    }

    sensor_distance = measure.RangeMilliMeter;
  #endif   
  
  if(target_within_range)
  {

    #ifdef DEBUG_LED_STRIP_REF_LED
      //green led, 12th LED of the strip
      led_matrix[11] = CHSV(96, 255, 120);  
    #endif
  
    /**LED strip reset and catch current distance**/
    //clear all leds from 4 to 8, (all left to right)
    //all the way down to black at first loop
    fadeToBlackBy(led_matrix, num_of_leds, 255);
    //map the current measure to the led index
    c_led_top_offset = map(sensor_distance, unactive_dist, (unactive_dist-threshold_good_push), 0, 7);
    //paint all the required LEDs
    for(uint8_t led_idx=0; led_idx < c_led_top_offset; led_idx++)
    {
      led_matrix[11+led_idx] = CHSV(base_color, 255, 120);
      led_matrix[11-led_idx] = CHSV(base_color, 255, 120);
      #ifdef DEBUG_LED_IDX_SELECTOR
        Serial.print("c_led_top_offset: ");Serial.println(c_led_top_offset);
        Serial.print("led_idx: ");Serial.println(led_idx);
        Serial.print("offset+: ");Serial.println(11+led_idx);
        Serial.print("offset-: ");Serial.println(11-led_idx);
        Serial.println("-------------");
      #endif
    }
    //send data to led strip
    FastLED.show();
    /***/

    //Show current distance, to calibrate states
    #ifdef DEBUG_CALIBRATE_DISTANCE
      Serial.print("mm: ");Serial.println(sensor_distance);
    #endif
    
    //Check if press is ok and save its status
    if(is_press_ok(sensor_distance))
    {      
      good_current_push = true;  
    }
    else
    {      
      good_current_push = false;  
    }     


    switch(current_state)
    {
      case s0:      
        if(is_pressed_down(sensor_distance))
        {
          current_state = s1;
          
          #ifdef DEBUG_STATES
            Serial.println("Pressed simple...");
          #endif          

          //Count pushes
          push_counter++;

          //Get the time passed between current pushes
          time_bt_pushs = micros() - st_time;           

                    //Show the push diffs
          #ifdef DEBUG_PUSH_TIMES
            Serial.print("micros: ");Serial.println(micros());
            Serial.print("st_time: ");Serial.println((long)st_time);
            Serial.print("time_bt_pushs: ");Serial.println((long) time_bt_pushs);
          #endif
          
          st_time = micros(); //Get time reference for first push
          sample_tstamp = st_time; //Get start time as timestamp for this measure     
          
          //Reset idle time          
          idle_diff_time = 0;

          //Set the current press deeper distance so far
          last_dist = sensor_distance;

          //Enable leds
          digitalWrite(yellow_led, HIGH);
          digitalWrite(ok_led, LOW);
          digitalWrite(bad_led, LOW);

          //Paint yellow the leds
          // yellow in CHSV
          base_color = 64;
        }        
        else
        { //If not pushed, then measure time idle to know if a new exercise is needed                    
          uint32_t lower_limit = unactive_max_time - unactive_max_time_range;
          uint32_t higher_limit = unactive_max_time + unactive_max_time_range;
          idle_diff_time += (micros() - st_time_idle); //Check the time passed          
          st_time_idle = micros(); //Restart the loop time counter
          
          #ifdef DEBUG_IDLE_TIMES
            Serial.print("lower_limit ");Serial.println(lower_limit);
            Serial.print("higher_limit ");Serial.println(higher_limit);
            Serial.print("Idle... ");Serial.println(idle_diff_time);
          #endif
          
          if((idle_diff_time >= lower_limit) && (idle_diff_time <= higher_limit))
          {                        
            #ifdef DEBUG_FINAL_VALS
              Serial.println("RESET HASH!!!");Serial.println();
            #endif            
            excercise_hash = random(100); //If idle for more than unactive_max_time, means it needs a new exercise            
            push_counter = 0; //Reset push counter for new exercise                        
            idle_diff_time = 0; //reset idle time adder
          }                    
        }      
      break;
      
      case s1:
        if(!is_pressed_down(sensor_distance)) //Means pulls up
        {
          current_state = s0;                   
          
          #ifdef DEBUG_STATES
            Serial.print("State 1, measure: ");Serial.println(sensor_distance);
          #endif         

          #ifdef DEBUG_FINAL_VALS
            //Send data as single press ended
            Serial.print("TIMESTAMP: "); Serial.println((long) sample_tstamp);
            Serial.print("HASH: "); Serial.println((long) excercise_hash);
            Serial.print("DIST: "); Serial.println(last_dist);
            Serial.print("CURR_FREQ: "); Serial.println(frequency);
            Serial.print("PRESS_COUNTER: "); Serial.println(push_counter);
            Serial.print("RESULT_DIST: "); Serial.println(is_press_ok(last_dist)?"OK":"NG");
            Serial.print("RESULT_FREQ: "); Serial.println(is_freq_ok(time_bt_pushs)?"OK":"NG");
            Serial.print("time_bt_pushs: "); Serial.println((long) time_bt_pushs); 
          #else
            send_json_values();
          #endif
          
          

          //Disable leds
          digitalWrite(yellow_led, LOW);
          digitalWrite(ok_led, LOW);
          digitalWrite(bad_led, LOW);
        }

        //Since the less distance on the sensor means deeper push
        if(sensor_distance < last_dist)
        {
          //have a new deepest press
          last_dist = sensor_distance;
        }        

        //Reset idle timer since we're pressing in s1
        st_time_idle = micros();

        //Enable led layers
        if(is_press_ok(last_dist))
        {
          digitalWrite(yellow_led, LOW);
          digitalWrite(ok_led, HIGH);
          digitalWrite(bad_led, LOW);  
        }
        else if(is_press_bad(last_dist))
        {
          digitalWrite(yellow_led, LOW);
          digitalWrite(ok_led, LOW);
          digitalWrite(bad_led, HIGH);  
        }
      break;           
      
      default:
        current_state = s0; //In any unstated case, it should go back to State 0        
      break;
    }              
  }

  //If timing and distance are ok, turn LEDS green
  if(is_freq_ok(time_bt_pushs))
  {
    //Paint green the leds
    // green in CHSV
    base_color = 96;
  }  
}//end loop

static bool is_pressed_down(uint16_t measure)
{/**
  Checks that the measure is bellow the distance that is marked as pressed, including the threshold range
  measure <- the distance measure from the sensor in mm

  Return true, if it is is_pressed_down
  Return false, if it's not pressed down so it's up in unactive state
  */
  uint16_t press_threshold = unactive_dist - threshold_simple_push - threshold_range;  
  bool press_state = false;
  if(measure <= press_threshold)
  {
    press_state = true;
  }

  return press_state;
}

static bool is_press_ok(uint16_t measure)
{
  /**
  Compares the distances with the threshold stated to be the right press deep
  measure <- the distance measure from the sensor in mm

  return true, if the measure is approximatelly in the expected deep
  return false, if measure is beyond or behind of the expected range
  */  
  uint16_t lower_limit = unactive_dist - threshold_good_push - threshold_range;
  uint16_t higher_limit = unactive_dist - threshold_good_push + threshold_range; 
  bool press_ok = false;
  if((measure >= lower_limit) && (measure <= higher_limit))
  {
    press_ok = true;
  }

  return press_ok;
}


static bool is_press_bad(uint16_t measure)
{
  /*
  Compaers the distance with the threshold stated to be right press
  if the measure passed to this function goes deeper than the lower limit for good press then means
  it's a bad press (too much distance pressed)

  measure <- the distance measure from the sensor in mm
  return true, if measure is deeper than lower limit of good press (measure < lower limit)
  return false, otherwise
  */
  uint16_t lower_limit = unactive_dist - threshold_good_push - threshold_range;
  bool press_bad = false;
  if(measure < lower_limit)
  {
    press_bad = true;  
  }

  return press_bad;  
}

static bool is_freq_ok(uint64_t time_delta)
{/**
  Uses the preset frequency to know if the time_delta passed as parameter
  corresponds to that freq (or closes to it) or not

  return true if close to expected freq, using the freq threshold
  return false if not close
  */  
  uint32_t lower_limit = freq_period_time - freq_range_time;
  uint32_t higher_limit = freq_period_time + freq_range_time;
  bool freq_ok = false;  

  #ifdef DEBUG_PUSH_TIMES
    Serial.println("Within is_freq_ok");
    Serial.print("lower_lim: ");Serial.println((long)lower_limit);
    Serial.print("higher_lim: ");Serial.println((long) higher_limit);
    Serial.print("time_delta: ");Serial.println((long) time_delta);
  #endif
  if((time_delta >= lower_limit) && (time_delta <= higher_limit))
  {
    freq_ok = true;  
  } 

  return freq_ok;
}

static void send_json_values()
{  
  //Send data as single press ended
  Serial.print("{\"TIMESTAMP\": \""); Serial.print((long) sample_tstamp);Serial.print("\",");
  Serial.print("\"HASH_ID\": \""); Serial.print((long) excercise_hash);Serial.print("\",");
  Serial.print("\"DIST\": \""); Serial.print(last_dist);Serial.print("\",");
  Serial.print("\"CURR_FREQ\": \""); Serial.print(frequency);Serial.print("\",");
  Serial.print("\"PRESS_COUNTER\": \""); Serial.print(push_counter);Serial.print("\",");
  Serial.print("\"RESULT_DIST\": \""); Serial.print(is_press_ok(last_dist)?"OK":"NG");Serial.print("\",");
  Serial.print("\"RESULT_FREQ\": \""); Serial.print(is_freq_ok(time_bt_pushs)?"OK":"NG");Serial.print("\",");
  Serial.print("\"time_bt_pushs\": \""); Serial.print((long) time_bt_pushs);Serial.print("\"}\r\n");      
}
