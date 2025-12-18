#include <Wire.h>
#include <stdbool.h>
#include <string.h>

#define LIGHT_MIN 0
#define LIGHT_MAX 255

#define SPEED_THRESHOLD 5.6

#define NUM_NEIGHBORS 8

#define CELL_LENGTH 40
#define GRID_SIZE 8
#define TOTAL_CELLS (GRID_SIZE * GRID_SIZE)

#define THRESHOLD 800

// events
#define TIME 0
#define MOVEMENT 1
#define DIRECTED_MOVEMENT 2
#define LAMP_FAILURE 254
#define LAMP_OK 255

const unsigned long debounceDelay = 50;
bool clock_synced = false;

struct Frame {
  uint8_t destination;
  uint8_t source;
  uint8_t event;
  unsigned long timestamp;
};

const int BUTTON_PINS[2] = {13, 12};
int lastButtonStates[2] = {LOW, LOW};
int buttonStates[2] = {LOW, LOW};
unsigned long lastButtonPressTimes[2] = {0, 0};
bool isMovement[2] = {false, false};

int x, y;
int device_address;

const int red_led = 10;
const int green_led = 9;
unsigned long red_timer = 0;
unsigned long green_timer = 0;
long time_ref[2] = {0, 0};

const int x_1 = 2;
const int x_2 = 3;
const int x_3 = 4;
const int y_1 = 6;
const int y_2 = 7;
const int y_3 = 8;

unsigned long sync_cell_0[NUM_NEIGHBORS] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned long sync_cell_1[NUM_NEIGHBORS] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned long sync_neighbors_red[NUM_NEIGHBORS] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned long sync_neighbors_green[NUM_NEIGHBORS] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned long time_neighbors_red[NUM_NEIGHBORS] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned long time_neighbors_green[NUM_NEIGHBORS] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned long time_neighbors_red_directed[NUM_NEIGHBORS] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned long time_neighbors_green_directed[NUM_NEIGHBORS] = {0, 0, 0, 0, 0, 0, 0, 0};

enum LED_STATE {
  OFF = LIGHT_MIN,
  SAFETY = (LIGHT_MAX - LIGHT_MIN) / 4,
  COMFORT = (LIGHT_MAX - LIGHT_MIN) / 2,
  ON = LIGHT_MAX
};

enum LED_STATE red_state = OFF;
enum LED_STATE green_state = OFF;
unsigned long last_led_state_change[2] = {0.0, 0.0};

int yellow_led = 11;
bool lamp_working[TOTAL_CELLS] = {true};
bool red_failure = false;
bool green_failure = false;
int red_voltage_pin = A1;
int green_voltage_pin = A2;

const int light_sensor = A0;
int light_status = 0;
int lightMax = 0;
int lightMin = 1023;
bool lightCalibrated = false;

/*
* function for reading out coordinates from input pins
* and setting I2C address of this controller
*/
void get_coordinates() {
  int x1 = digitalRead(x_1);
  int x2 = digitalRead(x_2);
  int x3 = digitalRead(x_3);
  int y1 = digitalRead(y_1);
  int y2 = digitalRead(y_2);
  int y3 = digitalRead(y_3);

  x = x1 | (x2 << 1) | (x3 << 2);
  y = y1 | (y2 << 1) | (y3 << 2);

  // I2C address
  device_address = ((x / 2) << 4) | y;
}


/*
* helper function for getting coordinates of a neighbor
* from the neighbor list
* 
* index: the index of the neighbor list
* cell_number: which cell to find neighbors for (0 for red, 1 for green)
* 
* returns: the coordinates in 8 bits (0xxx 0yyy)
*/
uint8_t get_neighbor_coordinates(int index, int cell_number) {
  int dy;
  int dx;

  switch (index) {
  case 0:
    dx = -1;
    dy = 1;
    break;
  case 1:
    dx = 0;
    dy = 1;
    break;
  case 2:
    dx = 1;
    dy = 1;
    break;
  case 3:
    dx = -1;
    dy = 0;
    break;
  case 4:
    dx = 1;
    dy = 0;
    break;
  case 5:
    dx = -1;
    dy = -1;
    break;
  case 6:
    dx = 0;
    dy = -1;
    break;
  case 7:
    dx = 1;
    dy = -1;
    break;
  }

  int neighbor_x;
  if (cell_number == 0) {
    neighbor_x = x + dx;
  } else {
    neighbor_x = x + 1 + dx;
  }
  int neighbor_y = y + dy;

  return ((neighbor_x & 0x0F) << 4) | (neighbor_y & 0x0F);
}


/*
* helper function for getting index in list of size [NUM_NEIGHBORS]
* 
* _x, _y: the x and y coordinates for current cell
* destination: the cell to find index for
* 
* returns: index in list
*/
int get_index(int _x, int _y, uint8_t destination) {
  int neighbor_x = (destination >> 4) & 0x0F;
  int neighbor_y = destination & 0x0F;
  int dx = neighbor_x - _x; // -1, 0, or 1
  int dy = neighbor_y - _y; // -1, 0, or 1

  // 0 1 2
  // 3 * 4
  // 5 6 7

  // Convert dx,dy to index based on position
  if (dy == -1) {       // Top row
    return dx + 1;      // Maps to 0,1,2
  } else if (dy == 0) { // Middle row
    if (dx == -1) {
      return 3;
    }
    else {
      return 4;
    }
  } else {              // Bottom row (dy == 1)
    return dx + 7;      // Maps to 5,6,7
  }
}


/*
* Processes a received movement message, updates states, and logs neighbor times.
* 
* destination: the target coordinates in 8 bits (0xxx 0yyy).
* source: the source coordinates in 8 bits (0xxx 0yyy).
* state: the current LED state (e.g., COMFORT or other states).
* 
* Updates red_state, green_state, and their respective timers based on 
* the received message. Logs the time for neighbors in either red or green states.
*/
void received_movement(uint8_t destination, uint8_t source, unsigned long timestamp,
                       enum LED_STATE state) {
  if ((destination >> 4) == x) {
    red_state = state;
    int i = get_index(x, y, source);
    if (state == COMFORT) {
      time_neighbors_red[i] = timestamp;
    }
    else {
      time_neighbors_red_directed[i] = timestamp;
    }
  } else {
    green_state = state;
    int i = get_index(x + 1, y, source);
    if (state == COMFORT) {
      time_neighbors_green[i] = timestamp;
    }
    else {
      time_neighbors_green_directed[i] = timestamp;
    }
  }
}


/*
* Checks the ambient light level and sets the light_status accordingly.
* 
* Reads a value from the light sensor, maps it to a calibrated range,
* and determines the light status based on thresholds.
* 
* No parameters.
* 
* Modifies:
*   light_status: a global variable indicating the light state.
*/
void check_light() {
  int val = analogRead(light_sensor);
  val = map(val, lightMin, lightMax, LIGHT_MIN, LIGHT_MAX);

  int range = LIGHT_MAX - LIGHT_MIN;
  if (val > range / 4) {
    light_status = 0;
    return;
  }

  if (val < range / 8) {
    light_status = range / 4;
    return;
  }

  light_status = LIGHT_MAX / 4 - val;
}


/*
* Controls the brightness and state of a specified LED based on movement, 
* ambient light, and neighbor activity.
* 
* led: the LED to control (e.g., red_led, green_led, yellow_led).
* state: the desired LED state (e.g., ON, OFF, COMFORT).
* is_movement: a boolean indicating if the control is triggered by movement.
* 
* The function adjusts lighting behavior based on:
* - Bright daylight
* - Movement detection in the current cell
* - Directed movement or comfort lighting triggered by neighbors
* - Ambient light levels
*/
void control_light(int led, enum LED_STATE state, bool is_movement) {
  if (led == yellow_led) {
    analogWrite(yellow_led, state);
    return;
  }

  // bright day. TODO: make this a range
  if (light_status == 0) {
    analogWrite(led, LIGHT_MIN);
    return;
  }

  int i;
  if (led == red_led) {
    i = 0;
  } else {
    i = 1;
  }

  // keep light on for 10s when movement detected
  if (static_cast<unsigned long>(static_cast<signed long>(millis()) - time_ref[i]) - lastButtonPressTimes[i] < 10000) {
    analogWrite(led, ON);
    return;
  }

  // directed movement
  unsigned long neighbor_directed_movement_time = 0;
  unsigned long temp = 0;
  if (led == red_led) {
    for (int j = 0; j < NUM_NEIGHBORS; j++) {
      temp = time_neighbors_red_directed[j];
      if (temp > neighbor_directed_movement_time) {
        neighbor_directed_movement_time = temp;
      }
    }
  } else {
    for (int j = 0; j < NUM_NEIGHBORS; j++) {
      temp = time_neighbors_green_directed[j];
      if (temp > neighbor_directed_movement_time) {
        neighbor_directed_movement_time = temp;
      }
    }
  }

  if (static_cast<unsigned long>(static_cast<signed long>(millis()) - time_ref[i]) - neighbor_directed_movement_time < 10000) {
    analogWrite(led, ON);
    return;
  }

  // comfort lighting
  unsigned long neighbor_movement_time = 0;
  temp = 0;
  if (led == red_led) {
    for (int j = 0; j < NUM_NEIGHBORS; j++) {
      temp = time_neighbors_red[j];
      if (temp > neighbor_movement_time) {
        neighbor_movement_time = temp;
      }
    }
  } else {
    for (int j = 0; j < NUM_NEIGHBORS; j++) {
      temp = time_neighbors_green[j];
      if (temp > neighbor_movement_time) {
        neighbor_movement_time = temp;
      }
    }
  }

  if (static_cast<unsigned long>(static_cast<signed long>(millis()) - time_ref[i]) - neighbor_movement_time < 10000) {
    analogWrite(led, COMFORT);
    return;
  }

  if (is_movement) {
    analogWrite(led, state);
    return;
  }

  analogWrite(led, light_status);
}


/*
* Checks the operational status of red and green LEDs and reports failures.
* 
* Reads the voltage levels of the red and green LEDs to detect disconnection 
* or short circuits. Sends notifications if a lamp failure or recovery is detected.
*
* Sends message to master controller to notify about lamp failures or recoveries.
* 
* No parameters.
* 
* Preconditions:
* - light_status is not zero.
* - The clock is synchronized (clock_synced).
* - Light calibration has been completed (lightCalibrated).
* 
* Modifies:
* - red_failure: flag indicating red LED failure.
* - green_failure: flag indicating green LED failure.
*/
void lamp_failure() {
  if (light_status == 0 || !clock_synced || !lightCalibrated)
    return;

  bool red_status = false;
  bool green_status = false;
  int red_led_status;
  int green_led_status;
  unsigned long time = millis();
  // at least two periods of low volage in PWM
  while (millis() - time < 4) {
    red_led_status = analogRead(red_voltage_pin);
    green_led_status = analogRead(green_voltage_pin);
    if (red_led_status > THRESHOLD)
      red_status = true;
    if (green_led_status > THRESHOLD)
      green_status = true;
  }

  if (red_status) {
    red_failure = true;
    send_data(x, y, 0, 0, LAMP_FAILURE, static_cast<unsigned long>(static_cast<signed long>(millis()) - time_ref[0]));  
  } else {
    if (red_failure) {
      red_failure = false;
      send_data(x, y, 0, 0, LAMP_OK, static_cast<unsigned long>(static_cast<signed long>(millis()) - time_ref[0]));
    }
  }

  if (green_status) {
    green_failure = true;
    send_data(x + 1, y, 0, 0, LAMP_FAILURE, static_cast<unsigned long>(static_cast<signed long>(millis()) - time_ref[1]));
  } else {
    if (green_failure) {
      green_failure = false;
      send_data(x + 1, y, 0, 0, LAMP_OK, static_cast<unsigned long>(static_cast<signed long>(millis()) - time_ref[1]));
    }
  }
}


/*
* Updates the controller's view of the grid's operational status.
* 
* source: the 8-bit address of the reporting node (0xxx 0yyy format).
* status_working: a boolean indicating whether the source node is operational.
* 
* If the controller is located at grid position (0, 0), it updates the 
* lamp_working array with the status of the source node.
* 
* Modifies:
* - lamp_working: an array tracking the operational status of all nodes in the grid.
*/
void monitor_controller(uint8_t source, bool status_working) {
  if (x == 0 && y == 0) {
    int source_x = (source >> 4) & 0x0F;
    int source_y = source & 0x0F;
    lamp_working[source_y * GRID_SIZE + source_x] = status_working;
  }
}


/*
* Handles an incoming frame and processes it based on the event type.
* 
* frame: a pointer to the Frame structure containing the event, source, 
*        destination, and timestamp data.
* 
* This function performs different actions based on the frame's event field:
*   - 0: Synchronizes the local cell and neighbor times.
*   - 1: Processes a movement event with COMFORT state.
*   - 2: Processes a movement event with ON state.
*   - 254: Disables monitoring for the specified controller.
*   - 255: Enables monitoring for the specified controller.
*   - Default: Logs an unrecognized event.
*/
void handle_frame(struct Frame *frame) {
  switch (frame->event) {
  case 0:
    if ((frame->destination >> 4) == x) {
      int i = get_index(x, y, frame->source);
      sync_cell_0[i] = millis();
      sync_neighbors_red[i] = frame->timestamp * 10;
    } else {
      int i = get_index(x + 1, y, frame->source);
      sync_cell_1[i] = millis();
      sync_neighbors_green[i] = frame->timestamp * 10;
    }
    return;
  case 1:
    received_movement(frame->destination, frame->source, frame->timestamp * 10, COMFORT);
    return;
  case 2:
    received_movement(frame->destination, frame->source, frame->timestamp * 10, ON);
    return;
  case 254:
    monitor_controller(frame->source, false);
    return;
  case 255:
    monitor_controller(frame->source, true);
    return;
  default:
    Serial.print("[WARNING]: Unable to recognize event from ");
    Serial.print(frame->source);
    Serial.print(". Event received was: ");
    Serial.println(frame->event);
  }
}


/*
* Sends data from one cell to another, performing checks for valid addresses
* and formatting the transmission packet. Handles errors and logs the result.
*
* Handles intra-board transmissions directly and uses I2C for inter-board communication.
* 
* x_source: x-coordinate of the source cell (0-7).
* y_source: y-coordinate of the source cell (0-7).
* x_dest: x-coordinate of the destination cell (0-7).
* y_dest: y-coordinate of the destination cell (0-7).
* event: the event type to transmit.
* timestamp: the time of the event, adjusted to a 10ms granularity.
*/
void send_data(int x_source, int y_source, int x_dest, int y_dest,
               uint8_t event, unsigned long timestamp) {
  // should be 0xxx 0yyy
  if (y_source > 7 || y_source < 0) {
    Serial.print("[ERROR]: Invalid source address in send_data, y-value ");
    Serial.print(y_source, HEX);
    Serial.println(" invalid.");
    return;
  }
  if (x_source > 7 || x_source < 0) {
    Serial.print("[ERROR]: Invalid source address in send_data, x-value ");
    Serial.print(x_source, HEX);
    Serial.println(" invalid.");
    return;
  }

  // should be 0xxx 0yyy
  if (y_dest > 7 || y_dest < 0) {
    Serial.print("[ERROR]: Invalid destination address in send_data, y-value ");
    Serial.print(y_dest, HEX);
    Serial.println(" invalid.");
    return;
  }
  if (x_dest > 7 || x_dest < 0) {
    Serial.print("[ERROR]: Invalid destination address in send_data, x-value ");
    Serial.print(x_dest, HEX);
    Serial.println(" invalid.");
    return;
  }

  // only send to itself if master and lamp status
  if ((x_dest == x_source) && (y_dest == y_source) && (event != 254) && (event != 255)) {
    Serial.print("[Warning]: (");
    Serial.print(x_source);
    Serial.print(", ");
    Serial.print(y_source);
    Serial.println(" trying to send to itself.");
    return;
  }

  timestamp = (timestamp / 10); // should be a factor of 10ms
  uint8_t millis_4 = (timestamp >> 24) & 0xFF;
  uint8_t millis_3 = (timestamp >> 16) & 0xFF;
  uint8_t millis_2 = (timestamp >> 8) & 0xFF;
  uint8_t millis_1 = timestamp & 0xFF;

  uint8_t dest_addr = ((x_dest / 2) << 4) | y_dest;

  uint8_t source = (x_source << 4) | y_source;
  uint8_t destination = (x_dest << 4) | y_dest;

  // send to other cell on this board
  if (dest_addr == device_address) {
    struct Frame frame = {destination, source, event, timestamp};
    handle_frame(&frame);
    return;
  }

  Wire.beginTransmission(dest_addr);

  Wire.write(destination);
  // data block
  Wire.write(source);
  Wire.write(event);
  Wire.write(millis_4);
  Wire.write(millis_3);
  Wire.write(millis_2);
  Wire.write(millis_1);

  int ret = Wire.endTransmission();
  if (ret != 0) {
    Serial.print("[ERROR]: Transmission failed from (");
    Serial.print(x_source);
    Serial.print(", ");
    Serial.print(y_source);
    Serial.print(") to (");
    Serial.print(x_dest);
    Serial.print(", ");
    Serial.print(y_dest);
    Serial.print("). Returned with exit code ");
    Serial.println(ret);
  } else {
    Serial.print("[INFO]: Successful transmission from (");
    Serial.print(x_source);
    Serial.print(", ");
    Serial.print(y_source);
    Serial.print(") to (");
    Serial.print(x_dest);
    Serial.print(", ");
    Serial.print(y_dest);
    Serial.println(").");
  }
}


/*
* Determines if movement is correlated and calculates the forecasted position.
* 
* source_x: the x-coordinate of the source cell.
* source_y: the y-coordinate of the source cell.
* cell_number: the cell to analyze (0 for red, 1 for green).
* t1: the timestamp when the movement was first detected.
* t2: the timestamp when the movement was last detected.
* 
* If the calculated speed exceeds the SPEED_THRESHOLD, this function predicts
* the next position based on the movement vector and sends the data as a 
* directed movement message.
*/
void determine_movement(int source_x, int source_y, int cell_number,
                        unsigned long t1, unsigned long t2) {
  int _x;
  if (cell_number == 0) {
    _x = x;
  } else {
    _x = x + 1;
  }

  int dx = _x - source_x;
  int dy = y - source_y;

  float distance = sqrt(dx * CELL_LENGTH * dx * CELL_LENGTH +
                        dy * CELL_LENGTH * dy * CELL_LENGTH);

  // Calculate time difference and speed
  unsigned long delta_t = t2 - t1;
  float speed = distance / (delta_t / 1000.0);

  if (speed >= SPEED_THRESHOLD) {
    int forecast_x = _x + dx;
    int forecast_y = y + dy;

    send_data(_x, y, forecast_x, forecast_y, DIRECTED_MOVEMENT,
              static_cast<unsigned long>(static_cast<signed long>(millis()) - time_ref[cell_number]));
  }
}


/*
* Detects movement in a specific cell by monitoring button presses
* and performs associated actions like controlling lights, sending data, 
* and determining movement based on neighboring cells.
* 
* cell_number: the cell to check for movement (0 for red, 1 for green).
*/
void detect_movement(int cell_number) {
  int reading = digitalRead(BUTTON_PINS[cell_number]);

  // If the button state changed, reset the debounce timer
  if (reading != lastButtonStates[cell_number]) {
    lastButtonPressTimes[cell_number] = static_cast<unsigned long>(static_cast<signed long>(millis()) - time_ref[cell_number]);
  }

  // Check if debounce time has passed
  if ((static_cast<unsigned long>(static_cast<signed long>(millis()) - time_ref[cell_number]) - lastButtonPressTimes[cell_number]) >
      debounceDelay) {
    if (reading != buttonStates[cell_number]) {
      buttonStates[cell_number] = reading;

      // Update movement status
      isMovement[cell_number] = (buttonStates[cell_number] == LOW);
      unsigned long t2 = static_cast<unsigned long>(static_cast<signed long>(millis()) - time_ref[cell_number]);

      int _x;
      if (cell_number == 0) {
        _x = x;
        control_light(red_led, ON, true);
      } else {
        _x = x + 1;
        control_light(green_led, ON, true);
      }

      // notify if movement
      for (int i = 0; i < NUM_NEIGHBORS; i++) {
        uint8_t dest = get_neighbor_coordinates(i, cell_number);

        int source_x = dest >> 4;
        int source_y = dest & 0x0F;

        if (source_x < 0 || source_y < 0) {
          continue;
        }

        send_data(_x, y, source_x, source_y, MOVEMENT, t2);
      }

      // check if directed movement
      for (int i = 0; i < NUM_NEIGHBORS; i++) {
        uint8_t dest = get_neighbor_coordinates(i, cell_number);

        int source_x = dest >> 4;
        int source_y = dest & 0x0F;

        if (source_x < 0 || source_y < 0) {
          continue;
        }

        unsigned long t1;
        if (cell_number == 0) {
          t1 = time_neighbors_red[i];
        } else {
          t1 = time_neighbors_green[i];
        }

        determine_movement(source_x, source_y, cell_number, t1, t2);
      }
    }
  }
  // Update last button state
  lastButtonStates[cell_number] = reading;
}


/*
* Synchronizes the clock by sending time data to neighbors and calculating offsets.
* 
* Sends the current time (via millis()) to neighboring cells for both red and green cells.
* Then, computes the time synchronization offsets based on received neighbor data.
* 
* No parameters.
* 
* Modifies:
*   - sync_neighbors_red and sync_neighbors_green: arrays holding neighbor sync times.
*   - time_ref[0] (red) and time_ref[1] (green): updated with calculated deltas.
*/
void sync_clock() {
  while (millis() < 5000) {
    // send to neighbors for red
    send_data(x, y, x, y + 1, TIME, millis());
    send_data(x, y, x + 1, y, TIME, millis());
    send_data(x, y, x, y - 1, TIME, millis());
    send_data(x, y, x - 1, y, TIME, millis());

    // Send to neighbors for green
    send_data(x + 1, y, x + 1, y + 1, TIME, millis());
    send_data(x + 1, y, x + 1 + 1, y, TIME, millis());
    send_data(x + 1, y, x + 1, y - 1, TIME, millis());
    send_data(x + 1, y, x + 1 - 1, y, TIME, millis());
  }

  int count_red = 0;
  int count_green = 0;
  for (int i = 0; i < NUM_NEIGHBORS; i++) {
    if (sync_neighbors_red[i] != 0)
      count_red++;
    if (sync_neighbors_green[i] != 0)
      count_green++;
  }


  long di = 0.0;
  int i;
  for (i = 0; i < NUM_NEIGHBORS; i++) {
    if (sync_neighbors_red[i] != 0) {
      di += (sync_cell_0[i] - sync_neighbors_red[i]);
    }
  }

  if (count_red > 0) {
    long delta = di / count_red;
    time_ref[0] = delta;
  }

  di = 0.0;
  for (i = 0; i < NUM_NEIGHBORS; i++) {
    if (sync_neighbors_green[i] != 0) {
      di += (sync_cell_1[i] - sync_neighbors_green[i]);
    }
  }

  if (count_green > 0) {
    long delta = di / count_green;
    time_ref[1] = delta;
  }
}


/*
* Handles incoming data packets from the I2C interface.
* 
* numBytes: the number of bytes received in the current packet.
* 
* If the packet is not exactly 7 bytes, it is discarded with a warning.
* Otherwise, the function parses the packet, extracting the destination,
* source, event, and timestamp. It then constructs a Frame structure
* and passes it to handle_frame.
* 
* Expected Packet Structure:
*   Byte 0: Destination (uint8_t)
*   Byte 1: Source (uint8_t)
*   Byte 2: Event (uint8_t)
*   Bytes 3-6: Timestamp (unsigned long, little-endian)
*/
void receive_data(int numBytes) {
  if (numBytes != 7) {
    return;
  }

  uint8_t destination = Wire.read();
  uint8_t source = Wire.read();
  uint8_t event = Wire.read();

  // TODO: is this correct slicing? Little vs big endian
  unsigned long timestamp = 0;
  timestamp = ((unsigned long)Wire.read() << 24);
  timestamp |= ((unsigned long)Wire.read() << 16);
  timestamp |= ((unsigned long)Wire.read() << 8);
  timestamp |= (unsigned long)Wire.read();

  struct Frame frame = {destination, source, event, timestamp};
  handle_frame(&frame);
}


/*
* Checks the status of the master lamp and controls the yellow LED
* based on whether any lamp is not working.
* 
* No parameters.
* 
* If the current cell is the master cell (x == 0 && y == 0), the function
* iterates through the lamp_working array to check if any lamp is not
* functioning. If any lamp is not working, the yellow LED is turned on;
* otherwise, it is turned off.
*/
void check_master_lamp() {
  if (x == 0 && y == 0) {
    bool light_lamp = false;
    for (int i = 0; i < TOTAL_CELLS; i++) {
      if (!lamp_working[i]) {
        light_lamp = true;
        break;
      }
    }
    if (light_lamp) {
      control_light(yellow_led, ON, false);
    }
    else {
      control_light(yellow_led, OFF, false);
    }
  }
}


/*
* Calibrates the light sensor by finding the minimum and maximum sensor values.
* 
* No parameters.
* 
* The function reads the light sensor value repeatedly for 5 seconds
* and updates lightMax and lightMin to reflect the highest and
* lowest values recorded during that period. This calibration helps
* to set appropriate thresholds for light intensity measurements.
*/
void calibrateLight() {
  while (millis() < 10000) {
    int sensorValue = analogRead(light_sensor);

    if (sensorValue > lightMax) {
      lightMax = sensorValue;
    }

    if (sensorValue < lightMin) {
      lightMin = sensorValue;
    }
  }
}

void setup() {
  Serial.begin(9600);

  get_coordinates();

  Wire.begin(device_address);
  Wire.onReceive(receive_data);

  pinMode(yellow_led, OUTPUT);
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);

  pinMode(x_1, INPUT);
  pinMode(x_2, INPUT);
  pinMode(x_3, INPUT);
  pinMode(y_1, INPUT);
  pinMode(y_2, INPUT);
  pinMode(y_3, INPUT);

  pinMode(BUTTON_PINS[0], INPUT_PULLUP);
  pinMode(BUTTON_PINS[1], INPUT_PULLUP);

  pinMode(red_voltage_pin, INPUT);
  pinMode(green_voltage_pin, INPUT);

  memset(lamp_working, 1, TOTAL_CELLS * sizeof(bool));
}

void loop() {
  if (!clock_synced) {
    sync_clock();
    clock_synced = true;
  }

  if (!lightCalibrated) {
    calibrateLight();
    lightCalibrated = true;
  }

  if (millis() % 500 == 0) {
    lamp_failure();
    check_master_lamp();
  }

  if (millis() % 100 == 0) {
    check_light();

    detect_movement(0);
    detect_movement(1);

    control_light(red_led, red_state, false);
    control_light(green_led, green_state, false);
  }
}
