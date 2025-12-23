// Motor driver setup
const int IN1 = 16; // RM Dir 1
const int IN2 = 17; // RM Dir 2
const int IN3 = 18; // LM Dir 1
const int IN4 = 19; // LM Dir 2
const int ENA = 4;  // PWM for RM
const int ENB = 5;  // PWM for LM

// Learning and Optimized 
bool learning = true;
// const int BUTTON_PIN = 22;
// bool button_pressed_before = false;
int optimized_index;

// Turns data
char turns[200];
int turns_length = 0;


// Sensor setup
const int sensor_count = 8; 
const int ir_sensor_pins[8] = {14, 27, 26, 25, 33, 32, 35, 34};
int sensor_values[8];

float position;
const int LED_PIN = 23;

// PID Values
float kp = 200;
float ki = 0;
float kd = 600;

float error = 0, last_error = 0;
float integral = 0;

int base_speed = 180;
int max_speed = 255;
int threshold = 500;

// To avoid skibiddi
bool just_turned = false;

// All white conformation
bool junction_checked = false;

void setup()
{

  for (int i = 0; i < sensor_count; i++)
    pinMode(ir_sensor_pins[i], INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  // pinMode(BUTTON_PIN, INPUT_PULLUP);
}


void loop()
{


  if (learning)
  {
    int white_count = 0;
    int black_count = 0;


    for (int i = 0; i < sensor_count; i++)
    {
      sensor_values[i] = analogRead(ir_sensor_pins[i]);
      if (sensor_values[i] < threshold) white_count++;
      else black_count++;
    }



    if (black_count == sensor_count)
    {
      if (!just_turned)
      {
        move_motors(150, 150);
        delay(300);
        turns[turns_length] = 'B';
        turns_length++;
        just_turned = true;
        position = read_line_position();
        while (position >= 0)
        {
        turn_left_reverse();
        delay(5);
        for (int i = 0; i < sensor_count; i++)
        {
          sensor_values[i] = analogRead(ir_sensor_pins[i]);
        }
        position = read_line_position();
        }
        return;
      }
      else 
      {
        position = last_error;
        error = position;
      }
    }

    else 
    {
      position = read_line_position();
      error = position;
    }



    integral += error;
    float derivative = error - last_error;
    float correction = kp * error + kd * derivative + ki * integral;



    // Turns
    if ((white_count == sensor_count) && !just_turned) // Junction with right and left both available/ right, left and straight all three available
    {

      move_motors(180, 180);
      delay(200);
      int all_white_check = 0;
      for (int i = 0; i < sensor_count; i++)
      {
        sensor_values[i] = analogRead(ir_sensor_pins[i]);
        if (sensor_values[i] < threshold) all_white_check++;
      }
      if (all_white_check == sensor_count) 
      {
        move_motors(150, 150);
        delay(200);
        digitalWrite(LED_PIN, HIGH);
        stop_motors();
        delay(20000);
        digitalWrite(LED_PIN, LOW);
        optimize();
        learning = false;
        optimized_index = 0;
        return;

      }
      turns[turns_length] = 'L';
      turns_length++;
      turn_left();
      delay(200);
      just_turned = true;

      for (int i = 0; i < sensor_count; i++)
      {
        sensor_values[i] = analogRead(ir_sensor_pins[i]);
      }
      position = read_line_position();

      while (position >= 0) // Check again
      {
        turn_left();
        delay(5);
        for (int i = 0; i < sensor_count; i++)
        {
          sensor_values[i] = analogRead(ir_sensor_pins[i]);
        }

        position = read_line_position();
      }
      junction_checked = false;
      return;
    }




    else if ((position > 1) && !just_turned) // Right Turn 
    {
      if (!junction_checked)
      {
        move_motors(150, 150);
        delay(40);
        junction_checked = true;
        return;
      }
      just_turned = true;
      move_motors(150, 150);
      delay(150);

      black_count = 0;
      for (int i = 0; i < sensor_count; i++)
      {
        sensor_values[i] = analogRead(ir_sensor_pins[i]);
        if (sensor_values[i] > threshold) black_count++;
      }

      if (black_count == sensor_count) // If straight is not available turn right until you find line
      {
        turn_right();
        delay(200);
        for (int i = 0; i < sensor_count; i++)
        {
          sensor_values[i] = analogRead(ir_sensor_pins[i]);
        }
        position = read_line_position();

        while (position <= 0)
        {
          turn_right();
          delay(5);
          for (int i = 0; i < sensor_count; i++)
          {
            sensor_values[i] = analogRead(ir_sensor_pins[i]);
          }
    
          position = read_line_position();
        }
      }

      else 
      {
        for (int i = 0; i < sensor_count; i++)
        {
          sensor_values[i] = analogRead(ir_sensor_pins[i]);
        }
        position = read_line_position();

        if (position > 1)
        {
          turn_right();
          delay(100);
          int all_white_check = 0;
          for (int i = 0; i < sensor_count; i++)
          {
            sensor_values[i] = analogRead(ir_sensor_pins[i]);
            if (sensor_values[i] < threshold) all_white_check++;
          }
          if (all_white_check == sensor_count) 
          {
            digitalWrite(LED_PIN, HIGH);
            stop_motors();
            delay(20000);
            digitalWrite(LED_PIN, LOW);
            optimize();
            learning = false;
            optimized_index = 0;
            return;
          }
        }

        else // If straight is available follow pid
        {
          turns[turns_length] = 'S';
          turns_length++;
          junction_checked = false;
          return;
        }
      }
    }
  
      


     
    
    



    else if ((position < -1) && !just_turned) // Left Turn
    {
      if (!junction_checked)
      {
        move_motors(150, 150);
        delay(40);
        junction_checked = true;
        return;
      }

      move_motors(150, 150);
      delay(150);
      just_turned = true;

      black_count = 0;
      for (int i = 0; i < sensor_count; i++)
      {
        sensor_values[i] = analogRead(ir_sensor_pins[i]);
        if (sensor_values[i] > threshold) black_count++;
      }

      if (black_count == sensor_count) // If no line ahead it isn't a junction
      {
        turn_left();
        delay(100);
        for (int i = 0; i < sensor_count; i++)
        {
          sensor_values[i] = analogRead(ir_sensor_pins[i]);
        }
        position = read_line_position();

        while (position >= 0)
        {
          turn_left();
          delay(5);
          for (int i = 0; i < sensor_count; i++)
          {
            sensor_values[i] = analogRead(ir_sensor_pins[i]);
          }
    
          position = read_line_position();
        }
      }

      else // Straight line ahead so its a junction
      {
        for (int i = 0; i < sensor_count; i++)
        {
          sensor_values[i] = analogRead(ir_sensor_pins[i]);
        }
        position = read_line_position();

        if (position < -1)
        {
          turn_left();
          delay(200);
          int all_white_check = 0;
          for (int i = 0; i < sensor_count; i++)
          {
            sensor_values[i] = analogRead(ir_sensor_pins[i]);
            if (sensor_values[i] < threshold) all_white_check++;
          }
          if (all_white_check == sensor_count) 
          {
            digitalWrite(LED_PIN, HIGH);
            stop_motors();
            delay(20000);
            digitalWrite(LED_PIN, LOW);
            optimize();
            learning = false;
            optimized_index = 0;
            return;
          }
        }

        else 
        {
          turns[turns_length] = 'L';
          turns_length++;   
          turn_left();
          delay(200);
          for (int i = 0; i < sensor_count; i++)
          {
            sensor_values[i] = analogRead(ir_sensor_pins[i]);
          }
          position = read_line_position();

          while (position >= 0)
          {
            turn_left();
            delay(5);
            for (int i = 0; i < sensor_count; i++)
            {
              sensor_values[i] = analogRead(ir_sensor_pins[i]);
            }
    
            position = read_line_position();
            }  
            junction_checked = false;
            return;
          }
        }
      }
    
  


  
    int left_speed = base_speed + correction;
    int right_speed = base_speed - correction;

    left_speed = constrain(left_speed, 0, max_speed);
    right_speed = constrain(right_speed, 0, max_speed);

    move_motors(left_speed, right_speed);
    last_error = error;
    // Serial.print("Correction: "); Serial.print(correction);
    // Serial.print(" | L: "); Serial.print(left_speed);
    // Serial.print(" R: "); Serial.println(right_speed);

    if (position > -1.0 && position < 1.0)
    {
      just_turned = false;
    }

    delay(5);    
  }

  else if (!learning)
  {
    int white_count = 0;
    int black_count = 0;
    for (int i = 0; i < sensor_count; i++)
    { 
      sensor_values[i] = analogRead(ir_sensor_pins[i]);
      if (sensor_values[i] < threshold) white_count++;
      else black_count++;
    }
    position   = read_line_position();
    
    if (black_count == sensor_count)
    {
      position = last_error;
      error = position;
    }
    else 
    {
      position = read_line_position();
      error = position;
    }



    integral += error;
    float derivative = error - last_error;
    float correction = kp * error + kd * derivative + ki * integral;



    if (white_count == sensor_count)
    {
      move_motors(180, 180);
      delay(200);
      int all_white_check = 0;
      for (int i = 0; i < sensor_count; i++)
      {
        sensor_values[i] = analogRead(ir_sensor_pins[i]);
        if (sensor_values[i] < threshold) all_white_check++;
      }
      if (all_white_check == sensor_count) 
      {
        move_motors(150, 150);
        delay(200);
        digitalWrite(LED_PIN, HIGH);
        stop_motors();
        delay(100000);
        return;
      }
      follow_path();
      return;
    }
    else if (position < -1 && !just_turned)
    {
      if (!junction_checked)
      {
        move_motors(150, 150);
        delay(40);
        junction_checked = true;
        return;
      }

      just_turned = true;
      move_motors(150, 150);
      delay(150);
      black_count = 0;
      for (int i = 0; i < sensor_count; i++)
      {
        sensor_values[i] = analogRead(ir_sensor_pins[i]);
        if (sensor_values[i] > threshold) black_count++;
      }

      if (black_count == sensor_count)
      {
        turn_left();
        delay(200);
        for (int i = 0; i < sensor_count; i++)
        {
          sensor_values[i] = analogRead(ir_sensor_pins[i]);
        } 
        position = read_line_position();

        while (position >= 0)
        {
          turn_left();
          delay(5);
          for (int i = 0; i < sensor_count; i++)
          {
            sensor_values[i] = analogRead(ir_sensor_pins[i]);
          }
          position = read_line_position();
        }
      }

      else 
      {
        follow_path(); 
      }
      junction_checked = false;
      return;
    }


      else if (position > 1 && !just_turned)
      {
        if (!junction_checked)
        {
          move_motors(150, 150);
          delay(40);
          junction_checked = true;
          return;
        }
        move_motors(150, 150);
        delay(150);
        black_count = 0;
        for (int i = 0; i < sensor_count; i++)
        {
          sensor_values[i] = analogRead(ir_sensor_pins[i]);
          if (sensor_values[i] > threshold) black_count++;
        }

        if (black_count == sensor_count)
        {
          turn_right();
          delay(200);
          for (int i = 0; i < sensor_count; i++)
          {
            sensor_values[i] = analogRead(ir_sensor_pins[i]);
          }
          position = read_line_position();

          while (position >= 0)
          {
          turn_right();
          delay(5);
          for (int i = 0; i < sensor_count; i++)
          {
            sensor_values[i] = analogRead(ir_sensor_pins[i]);
          }
          position = read_line_position();
        }
      }

      else 
      {
        follow_path(); 
      }
      junction_checked = false; 
      return;
    }

    int left_speed = base_speed + correction;
    int right_speed = base_speed - correction;

    left_speed = constrain(left_speed, 0, max_speed);
    right_speed = constrain(right_speed, 0, max_speed);

    move_motors(left_speed, right_speed);
    last_error = error;

    if (position > -1.0 && position < 1.0)
    {
      just_turned = false;
    }
    delay(5);

  }    
}

void follow_path()
{
  if (turns[optimized_index] == 'S')
  {
    move_motors(150, 150);
    delay(200);

  }

  else if (turns[optimized_index] == 'R')
  {
    just_turned = true;
    move_motors(150, 150);
    delay(150);

    turn_right();
    delay(200);
    for (int i = 0; i < sensor_count; i++)
    {
      sensor_values[i] = analogRead(ir_sensor_pins[i]);
    }
    position = read_line_position();

    while (position <= 0)
    {
      turn_right();
      delay(5);
      for (int i = 0; i < sensor_count; i++)
      {
        sensor_values[i] = analogRead(ir_sensor_pins[i]);
      }
      position = read_line_position();
    }
  }

  else if (turns[optimized_index] == 'L')
  {
    just_turned = true;
    move_motors(150, 150);
    delay(150);

    turn_left();
    delay(200);
    for (int i = 0; i < sensor_count; i++)
    {
      sensor_values[i] = analogRead(ir_sensor_pins[i]);
    }
    position = read_line_position();

    while (position >= 0)
    {
      turn_left();
      delay(5);
      for (int i = 0; i < sensor_count; i++)
      {
        sensor_values[i] = analogRead(ir_sensor_pins[i]);
      }
    
      position = read_line_position();
    }
  }
  optimized_index++;
  return;
}

void turn_right()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 200);
  analogWrite(ENB, 255);
}

void turn_left()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, 255);
  analogWrite(ENB, 200);
}

void turn_left_reverse()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, 250);
  analogWrite(ENB, 250);
}

float read_line_position()
{
  float weighted_sum = 0;
  long sum = 0;
  float weights[8] = {-5.5 , -3, -1, 0, 0, 1, 3, 5.5};

  for (int i = 0; i < sensor_count; i++)
  {
    int val = (sensor_values[i] < threshold) ? 1000 : 0;
    weighted_sum += val * weights[i];
    sum += val;
  }

  if (sum == 0) return 0;
  return (float)weighted_sum / sum;
}


void move_motors(int left_speed, int right_speed)
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, right_speed);
  analogWrite(ENB, left_speed);
}

void stop_motors()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void optimize()
{
  bool change = true;
  while (change)
  {
    change = false;
    for (int i = 0; i < turns_length - 2; i++)
    {
      if (turns[i] == 'L' && turns[i+1] == 'B' && turns[i+2] == 'L')
      {
        turns[i] = 'S';  
        for (int j = i+1; j < turns_length - 2; j++)
          turns[j] = turns[j+2];
        turns_length -= 2;
        change = true;
        break;
      }
      
      else if (turns[i] == 'L' && turns[i+1] == 'B' && turns[i+2] == 'S')
      {
        turns[i] = 'R';  
        for (int j = i+1; j < turns_length - 2; j++)
          turns[j] = turns[j+2];
        turns_length -= 2;
        change = true;
        break;
      }
      
      else if (turns[i] == 'L' && turns[i+1] == 'B' && turns[i+2] == 'R')
      {
        turns[i] = 'B';  
        for (int j = i+1; j < turns_length - 2; j++)
          turns[j] = turns[j+2];
        turns_length -= 2;
        change = true;
        break;
      }
      
      else if (turns[i] == 'S' && turns[i+1] == 'B' && turns[i+2] == 'L')
      {
        turns[i] = 'R';  
        for (int j = i+1; j < turns_length - 2; j++)
          turns[j] = turns[j+2];
        turns_length -= 2;
        change = true;
        break;
      }
      
      else if (turns[i] == 'S' && turns[i+1] == 'B' && turns[i+2] == 'S')
      {
        turns[i] = 'B';  
        for (int j = i+1; j < turns_length - 2; j++)
          turns[j] = turns[j+2];
        turns_length -= 2;
        change = true;
        break;
      }
      
      else if (turns[i] == 'S' && turns[i+1] == 'B' && turns[i+2] == 'R')
      {
        turns[i] = 'L';  
        for (int j = i+1; j < turns_length - 2; j++)
          turns[j] = turns[j+2];
        turns_length -= 2;
        change = true;
        break;
      }
      
      else if (turns[i] == 'R' && turns[i+1] == 'B' && turns[i+2] == 'L')
      {
        turns[i] = 'B'; 
        for (int j = i+1; j < turns_length - 2; j++)
          turns[j] = turns[j+2];
        turns_length -= 2;
        change = true;
        break;
      }
      
      else if (turns[i] == 'R' && turns[i+1] == 'B' && turns[i+2] == 'S')
      {
        turns[i] = 'L';  
        for (int j = i+1; j < turns_length - 2; j++)
          turns[j] = turns[j+2];
        turns_length -= 2;
        change = true;
        break;
      }
      
      else if (turns[i] == 'R' && turns[i+1] == 'B' && turns[i+2] == 'R')
      {
        turns[i] = 'S';  
        for (int j = i+1; j < turns_length - 2; j++)
          turns[j] = turns[j+2];
        turns_length -= 2;
        change = true;
        break;
      }
      
      else if (turns[i] == 'B' && turns[i+1] == 'B' && turns[i+2] == 'L')
      {
        turns[i] = 'R';  
        for (int j = i+1; j < turns_length - 2; j++)
          turns[j] = turns[j+2];
        turns_length -= 2;
        change = true;
        break;
      }
      
      else if (turns[i] == 'B' && turns[i+1] == 'B' && turns[i+2] == 'S')
      {
        turns[i] = 'S';  
        for (int j = i+1; j < turns_length - 2; j++)
          turns[j] = turns[j+2];
        turns_length -= 2;
        change = true;
        break;
      }
      
      else if (turns[i] == 'B' && turns[i+1] == 'B' && turns[i+2] == 'R')
      {
        turns[i] = 'L';  
        for (int j = i+1; j < turns_length - 2; j++)
          turns[j] = turns[j+2];
        turns_length -= 2;
        change = true;
        break;
      }
    }
  }  
}
