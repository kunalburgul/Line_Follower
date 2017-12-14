  //PID for black path on white background
  #define l1 8
  #define l2 9
  #define r1 10
  #define r2 11
  #define en1 5
  #define en2 6
  #define Kp 0.1
  #define Ki 0.0001
  #define Kd 1.0
  #define max_motor_speed  100

  float sensor_average = 0, sensor_sum = 0;
  int last_proportional=0, integral=0, found_binary = 0;
  int a[7] = {0,0,0,0,0,0,0};
  char binary_code;
  
  void read_sensor_values();
  int calculate_error();
  void calculate_PID();
  void motor_control();
  void set_motor_speed(int, int);
  char select_turn(unsigned char found_left,unsigned char found_right,unsigned char found_st);
  int mod(int v);
  void turn(char dir);
  void read_binary();
  
  void setup()
  {  
    delay(1000);
    pinMode(l1, OUTPUT);       //Left Motor Pin 1
    pinMode(l2, OUTPUT);       //Left Motor Pin 2
    pinMode(r1, OUTPUT);       //Right Motor Pin 1
    pinMode(r2, OUTPUT);       //Right Motor Pin 2
    pinMode(en2, OUTPUT);      //EN1 Pin 1
    pinMode(en1, OUTPUT);      //EN2 Pin 2
    Serial.begin(9600);        //Enable Serial Communications
  }
  
  void loop() {
    calculate_PID();    //Control invoked

    if(found_binary==1)
      read_binary();
    unsigned char found_left=0;
    unsigned char found_right=0;   // turn paratmeter.
    unsigned char found_st=0;
        
    read_sensor_values();
      if(a[0]==LOW)
        found_left=1;
        
      if(a[6]==LOW)
        found_right=1;
     
      if(a[2]==LOW || a[3]==LOW||a[4]==LOW)
        found_st=1;

      if(a[0]==HIGH&&a[1]==HIGH&&a[2]==HIGH&&a[3]==HIGH&&a[4]==HIGH&&a[5]==HIGH&&a[6]==LOW)
        {
          set_motor_speed(120, 80);
          delay(100);
        }

      if(a[0]==LOW&&a[1]==HIGH&&a[2]==HIGH&&a[3]==HIGH&&a[4]==HIGH&&a[5]==HIGH&&a[6]==HIGH)
        {
          set_motor_speed(80, 120);
          delay(100);
        }
    unsigned char dir;
  
    dir = select_turn(found_left, found_right, found_st);
    turn(dir);
  }

  void read_sensor_values()
  {
    a[0] = digitalRead(12);
    a[1] = digitalRead(A0);
    a[2] = digitalRead(A1);
    a[3] = digitalRead(A2);
    a[4] = digitalRead(A3);
    a[5] = digitalRead(A4);
    a[6] = digitalRead(13);
  }


  int calculate_error()
  {
    int error = 0;
    sensor_average = ( 1000*a[0] + 2000*a[1] + 3000*a[2] + 4000*a[3] + 5000*a[4] + 6000*a[5] + 7000*a[6] );
    sensor_sum = ( a[0] + a[1] + a[2] + a[3] + a[4] + a[5] + a[6] );
    sensor_average /= sensor_sum;
    error = sensor_average - 4000;
    //Serial.println(error);
    return error;
  }

  void calculate_PID()
  { 
    int i;       // Control function
    int PID_value = 0;
    unsigned int position;
    int derivative,proportional;
    while(1)
    { 
      read_sensor_values();
      
      proportional = calculate_error();
      integral = integral + proportional;
      derivative = proportional - last_proportional;
      last_proportional = proportional;
        
      PID_value = proportional*Kp + integral*Ki + derivative*Kd;
      //PID_value = PID_value/4;
      //Serial.println(PID_value);
      if( PID_value > max_motor_speed )
       PID_value = max_motor_speed;
      if( PID_value < -max_motor_speed )
       PID_value = ( -1 * max_motor_speed );

      if( PID_value < 0 )
      {
       set_motor_speed( max_motor_speed - PID_value, max_motor_speed + (PID_value/2));  //right
      }
      else
      {
       set_motor_speed( max_motor_speed - (PID_value/2), max_motor_speed + PID_value );  //left
      }
      Serial.println("-----------");
      //delay(2000);
      read_sensor_values();
      if(a[0]==HIGH&&a[1]==HIGH&&a[2]==HIGH&&a[3]==HIGH&&a[4]==HIGH&&a[5]==HIGH&&a[6]==HIGH)
        {
          found_binary = 1;
          return;
        }
      
      else if(a[0]==LOW || a[6]==LOW)
        return;
      }
  }
  
  void set_motor_speed(int l, int r) // Motor setup
  {
      //Serial.println(l);
      //Serial.println(r);

    if(l>0&&r>0)
    {
      analogWrite(en1,l);
      analogWrite(en2,r);
        
      digitalWrite(l1,HIGH);
      digitalWrite(l2,LOW);
      
      digitalWrite(r1,HIGH);
      digitalWrite(r2,LOW);

      Serial.println("Positive");
    }
    
    else if(l<0&&r>0)
    {
      analogWrite(en1,mod(l));
      analogWrite(en2,mod(r));
    
      digitalWrite(l1,LOW);
      digitalWrite(l2,HIGH);
      
      digitalWrite(r1,HIGH);
      digitalWrite(r2,LOW);

      Serial.println("Left");
    }
    
    else if(l>0&&r<0)
    {
      analogWrite(en1,mod(l));
      analogWrite(en2,mod(r));
    
      digitalWrite(l1,HIGH);
      digitalWrite(l2,LOW);
      
      digitalWrite(r1,LOW);
      digitalWrite(r2,HIGH);

      Serial.println("Right");
    }
    
    else if(l==0&&r==0)
    {
      analogWrite(en1,0);
      analogWrite(en2,0);
    
      digitalWrite(l1,LOW);
      digitalWrite(l2,LOW);
      
      digitalWrite(r1,LOW);
      digitalWrite(r2,LOW);

      Serial.println("Nan");
    }
  }

  void turn(char dir)   //Turning setup
  {
    switch(dir)
    {
      case 'L':
        set_motor_speed(-150, 150);
        delay(130);
        break;
      case 'R':
        set_motor_speed(150, -150);
        delay(130);
        break;
//      case 'B':
//        set_motor_speed(200,-200);
//        delay(450);
//        break;
      case 'S':
        set_motor_speed(100, 100);
        break;
    }
  }

  char select_turn(unsigned char found_left,unsigned char found_right,unsigned char found_st) // AI Function
  {
   if(found_left==1)
    return 'L';
   else if(found_right==1)
    return 'R';
   else if(found_st==1)
    return 'S';
   else
    return 'S';
   
  }

  int mod(int v)
  {
    if(v<0)
    return -1*v;
    else if(v>0)
    return v;
  }
  void stop()
  {
    analogWrite(en1,0);
    analogWrite(en2,0);
  
    digitalWrite(l1,LOW);
    digitalWrite(l2,LOW);
    
    digitalWrite(r1,LOW);
    digitalWrite(r2,LOW);
  }

  void read_binary()
  {
    while( a[0]==HIGH && a[1]==HIGH && a[2]==HIGH && a[3]==HIGH && a[4]==HIGH && a[5]==HIGH && a[6]==HIGH )
    {
      set_motor_speed(40, 40);
      read_sensor_values();
    }
    int x = calculate_error();
    if(x<0)
      {
        binary_code = 'L';
        //apply_binary(binary_code);
        delay(500);
      }

    else if (x>0)
      {
        binary_code = 'R';
        //apply_binary(binary_code);
        delay(500);
      }
      
    set_motor_speed(60, 60);
    delay(200);

      while(1)
      {
        calculate_PID();
        read_sensor_values();
        if(( a[0]==LOW && a[1]==LOW && a[2]==LOW && a[3]==LOW ) || (a[3]==LOW && a[4]==LOW && a[5]==LOW && a[6]==LOW ))
        {
          apply_binary(binary_code);
          break;
        }

        if(a[0]==HIGH&&a[1]==HIGH&&a[2]==HIGH&&a[3]==HIGH&&a[4]==HIGH&&a[5]==HIGH&&a[6]==LOW)
        {
          set_motor_speed(120, 80);
          delay(100);
        }

      if(a[0]==LOW&&a[1]==HIGH&&a[2]==HIGH&&a[3]==HIGH&&a[4]==HIGH&&a[5]==HIGH&&a[6]==HIGH)
        {
          set_motor_speed(80, 120);
          delay(100);
        }
      }
  }

  void apply_binary(char d)
  {
    switch(d)
    {
      case 'L':
        set_motor_speed(-150, 150);
        delay(180);
        found_binary = 0;
        break;
      case 'R':
        set_motor_speed(150, -150);
        delay(180);
        found_binary = 0;
        break;
      case 'S':
        set_motor_speed(200, 200);
        delay(180);
        found_binary = 0;
        break;
    }
  }

