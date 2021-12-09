#include <DynamixelWorkbench.h>
#include <math.h>



#define DEVICE_NAME "3"

#define BAUDRATE  1000000

#define joints 5


DynamixelWorkbench dxl;     

float pi = 3.14159265;      // Число пи

float unit = 0.111;         // Переменная, отвечающая за перевод радиан в условные единицы положения привода двигателя 

float l1 = 0.07;            // Длины звеньев манипулятора
float l2 = 0.08;
float l3 = 0.12;
float l4 = 0.06;
float l5 = 0.06;
float l45 = l4 + l5;

float max_velocity = 50;   // Максимальная скорость сервоприводов (измените, если хотите уменьшить или увеличить скорость перемещения манипулятора)

float CONSTRAINTS[joints][2] = {{-2.62, 2.62},     // Двумерный массив, содержащий предельные значения для каждой степени подвижности
                           {-2.5, 0.5},
                           {-2.3, 2.6},
                           {-1.57, 1.85},
                           {-2.62, 2.62}};

float HOME_POSITION[joints] = {0, -pi/2, 0, 0, 0};  // Массив, задающий начальное положение манипулятора

uint8_t dxl_id[joints] = {1, 2, 3, 4, 5};           // Массив ID приводов

float Q_0[joints];   // Текущие обобщенные координаты сочленений, рад 

float Q_d[joints];  // Вычисленные обобщенные координаты сочленений, рад

float X, Y, Z, pitch;   // Текущие координаты схвата, см 

float X_d, Y_d, Z_d, pitch_d, roll_d;  // Заданные координаты и ориентация схвата, см





void setup() {
  

  Serial.begin(57600);   // Инициализация Serial-порта
 
  while(!Serial);   // Ждем открытия Serial-порта

  // Инициализируем параметры регуляторов
  
  uint8_t CW_slope[5] = {32, 32, 32, 32, 32};
  uint8_t CW_margin[5] = {1, 1, 1, 1, 255};
  //uint8_t Punch[5] = {2, 2, 2, 2, 64};
  
  // Инициализация сервоприводов в цикле for
  
  uint16_t model_number = 0;
  const char *log;
  bool result;
  
  for (int i = 0; i < joints; i++) {
  
  result = dxl.init(DEVICE_NAME, BAUDRATE, &log);
  if (result) {
    Serial.print("Succeeded to init : ");
    Serial.println(BAUDRATE);
  }
  
  
  result = dxl.ping(dxl_id[i], &model_number, &log);
  if(result) {
    Serial.print("Succeeded to ping, id: ");
    Serial.println(dxl_id[i]);
   }
  
  // Устанавливаем для сервопривода режим шарнира
  
   result = dxl.jointMode(dxl_id[i], 0, 0, &log);
   if (result) {
    Serial.print("Succeeded to joint mode, id: ");
    Serial.println(dxl_id[i]);
   }
  
  // Записываем в Control Table выбранные параметры регуляторов
  
  
  dxl.itemWrite(dxl_id[i], "CW_Compliance_Margin", CW_margin[i], &log);
  dxl.itemWrite(dxl_id[i], "CCW_Compliance_Margin", CW_margin[i], &log);
  
  dxl.itemWrite(dxl_id[i], "CW_Compliance_Slope", CW_slope[i], &log);
  dxl.itemWrite(dxl_id[i], "CCW_Compliance_Slope", CW_slope[i], &log);
  
  }
  
  go_home();       // Перемещение манипулятора в начальное положение
  
  print_menu();      // Вывод меню взаимодействия с Serial-портом

}






void loop() {

  // Ожидаем ввода данных, согласно меню, в Serial-порт
//  get_pos(); 
//  print_cartesian_pos(Q_0); 
//  delay(1000);
  if(Serial.available() > 0) {
    int mode = Serial.parseInt();   
    switch(mode) {
      
      // Перемещение манипулятора в начальное положение
      case 1:
        go_home();
        break;
        
      // Режим управления манипулятором в декартовых координатах   
      case 2:
        cartesian_mode();
        break;
      
      // Режим управления манипулятором по степеням подвижности  
      case 3: 
        joint_mode();
        break;
    }
    print_menu();
  }
}


void moving(float* q) {
  
  Serial.println("Robot move...");
  float q_dot_max = max_velocity * unit * pi / 30;    // Пересчет значения максимальной скорости из условных единиц в рад/с
  float T_max = 0;    // Инициализируем переменную времени перемещения самого медленного "привода"    
  float dq[joints];   // Инициализируем массив разностей между заданным и текущим положением сервоприводов

  get_pos();  // Считываем текущие положения сервоприводов Q_0
  
  float* q0 = Q_0;  // Инициализируем вспомогательный указатель  

  // В цикле вычисляем T_max
  for (int i = 0; i < joints; i++) {
    
    dq[i] = q[i] - q0[i];
    float temp = abs((3 * dq[i])/(2 * q_dot_max));
    if (temp > T_max) T_max = temp;
    
  }

  
  // Вычисляем требуемые скорости приводов и перемещаем их в требуемое положение
  for (int i = 0; i < joints; i++) {
    
  float q_dot = abs((dq[i] * 3)/(2 * T_max));
  
  dxl.goalVelocity(dxl_id[i], q_dot);
  dxl.goalPosition(dxl_id[i], q[i]);
  
  }
  
  check_moving();   // Ожидаем, пока манипулятор окончит перемещение
  
  get_pos();                   // Считываем текущее положение сервоприводов
  print_cartesian_pos(Q_0);    // и выводим в Serial-порт текущие координаты схвата
}



void go_home() {
  copy_arr(Q_d, HOME_POSITION, joints);
  moving(Q_d);
}



void cartesian_mode() {
  Serial.println("***************Cartesian mode***************");
  while(1) {
    if (Serial.available() > 0) {         // Ожидаем ввода координат или команды exit в Serial-порт
      String str = Serial.readString();
      if (str.equals("exit")) break;      // При вводе в Serial-порт команды "exit" выходим из режима управления в декартовых координатах
      
      parsingGlobal(str);                 // Парсим строку с введенными координатами
      if(inverse_problem()) {             // Решаем обратную задачу кинематики для введенных координат,
        moving(Q_d);                      // если получено корректное решение передаем его в функцию moving
      }
      
      else Serial.println("Collision!");
    }
  }
}



void joint_mode() {
  Serial.println("***************Joint mode***************");
  while(1) {      // Инициализируем бесконечый цикл
    if (Serial.available() > 0) {
      String str = Serial.readString();
      if (str.equals("exit")) break;     // При вводе команды exit выходим из цикла while и, соответственно, из режима joint mode
      parsingGlobal(str);               // Парсим строку с введенными обобщенными координатами 

      // Проверяем введенную конфигурацию на ограничения, используя check_constrain, в случае успеха перемещаем манипулятор
      if (check_constrain(Q_d)) 
        moving(Q_d);    
      }
   }
}

void forward_kinematics(float *q) {
  
  float q1 = q[0];
  float q2 = q[1];
  float q3 = q[2];
  float q4 = q[3] - pi/2;
  float q5 = q[4];

  // Присваиваем глобальным переменным X, Y, Z, pitch рассчитанные координаты схвата
      
  X = l2 * cos(q1) * cos(q2) + l3 * cos(q1) * cos(q2+q3) - l45 * cos(q1) * sin(q2+q3+q4);
 
  Y = l2 * sin(q1) * cos(q2) + l3 * sin(q1) * cos(q2+q3) - l45 * sin(q1) * sin(q4+q2+q3);
 
  Z = l1 - l3 * sin(q3+q2) - l2 * sin(q2) - l45 * cos(q2 + q3 + q4);

  float zz = sin(q2+q3)*sin(q4) - cos(q2+q3)*cos(q4);
  pitch = pi/2 - acos(zz);
}



bool  inverse_problem() {
  float x = X_d;
  float y = Y_d;
  float z = Z_d;
  float pitch = pitch_d;

  float z0 = z - l1;
  float w = sqrt(x*x + y*y);
  
  float q1 = atan2(y, x);
  float q5 = roll_d;
  

  float z1 = z0 - l45 * sin(pitch);
  float w1 = w - l45 * cos(pitch);

  float a, b, c, c1;

  c1 = (l3*l3 - w1*w1 - z1*z1 - l2*l2)/(-2);
  
  a = z1*z1 + w1*w1;
  b =  -2*z1*c1;
  c = c1*c1 - l2*l2*w1*w1;

  float D = b*b - 4*a*c;

  
  if (D > 0) {
    
    float z21, z22, w21, w22;
    z21 = (-b + sqrt(D)) / (2*a);

    z22 = (-b - sqrt(D)) / (2*a);


    w21 = (l2*l2 - l3*l3 + w1*w1 + z1*z1 - 2*z21*z1)/(2*w1);
    w22 = (l2*l2 - l3*l3 + w1*w1 + z1*z1 - 2*z22*z1)/(2*w1);

    float q21, q31, q41, q22, q32, q42;
    
    q21 = atan2(z21, w21);
    q22 = atan2(z22, w22);

    float alpha, beta;
    
    alpha = atan2(z1-z21, w1-w21);
    q31 =  q21-alpha;
    
    beta = atan2(z1-z22, w1-w22); 
    q32 = q22 - beta;
    
    q41 =  alpha -pitch;
    q42 =  beta - pitch; 

    // Пересчет вычисленных угловых координат в координаты двигателей

    float Q1[5] = {q1 , -q21, q31, q41, q5};

    float Q2[5] = {q1, -q22, q32, q42, q5};

    // С помощью функции check_constrain проверяем полученные решения исходя из ограничений на положение сервоприводов 
    
    if (check_constrain(Q1)) {
      
      copy_arr(Q_d, Q1, joints);    // Копируем полученное решение в массив Q_d 
      return true;
    }

    if (check_constrain(Q2)) {

      copy_arr(Q_d, Q2, joints);
      return true;
    }


  }

  // Возвращаем false, если решения ОЗК для заданной конфигурации схвата не существует (D<0)
  // или ни одной из полученных решений не удовлетворяет ограничениям
    
  return false;      
  
}





void get_pos() {
  for (int i = 0; i < joints; i++) {
    
    if (!dxl.getRadian(dxl_id[i], &(Q_0[i])))
    {
      Serial.print("Error get position id : ");
      Serial.println(dxl_id[i]);
    }
  }
}


void check_moving() {  
  int32_t data;
  for (int i = 0; i < 5; i++) {
    do dxl.itemRead(dxl_id[i], "Moving", &data);  // Считываем регистр "Moving" 
    while(data != 0);                             // пока значение в нем не станем равным 0          
    }
}


bool check_constrain(float *q) {
  bool result;
  for(int i = 0; i < joints; i++) {
    result = q[i] > CONSTRAINTS[i][0] && q[i] < CONSTRAINTS[i][1];
    if(!result) return false; 
  }
  return true;
}


void parsingGlobal(String tempString) {
  
  byte dividerIndex = tempString.indexOf(' ');  // Поиск символа-разделителя (в данном случае пробела)
  String buf1 = tempString.substring(0, dividerIndex);   // Записываем в переменную buf1 часть строки до разделителя
  tempString = tempString.substring(dividerIndex + 1);   // Записываем в переменную tempString оставшуюся часть строки после разделителя

  parsingLocal(buf1);   // Вызываем функцию локального парсинга строки до разделителя buf1
  
  if (dividerIndex != 255) parsingGlobal(tempString);  // Если не доситгнут конец строки, рекурсивно вызываем функцию parsingGlobal для парсинга оставшейся части строки tempString
}

void parsingLocal(String tempString) {
  
  byte dividerIndex = tempString.indexOf('=');  //  Поиск символа разделителя "="
  String buf1 = tempString.substring(0, dividerIndex);   // Записываем в переменную buf1 символ до разделителя, обозначающий декартову или обобщенную координату
  String buf2 = tempString.substring(dividerIndex + 1);  // Записываем в переменную buf2 численное значение декартовой или обобщенной в виде строки 

  // Распознаем символ до разделителя и присваиваем соответствующим переменным численное значение
  if (buf1.equals("x"))
    X_d = buf2.toFloat() / 100;

  if (buf1.equals("y"))
    Y_d = buf2.toFloat() / 100;
      
  if (buf1.equals("z"))
    Z_d = buf2.toFloat() / 100;

  if (buf1.equals("p")) 
    pitch_d = buf2.toFloat() * pi / 180;

  if (buf1.equals("q1")) 
    Q_d[0] = buf2.toFloat() * pi / 180;
  
  if (buf1.equals("q2"))
    Q_d[1] = buf2.toFloat() * pi / 180;

  if (buf1.equals("q3")) 
    Q_d[2] = buf2.toFloat() * pi / 180;

  if (buf1.equals("q4")) 
    Q_d[3] = buf2.toFloat() * pi / 180;

  if (buf1.equals("q5")) 
    Q_d[4] = buf2.toFloat() * pi / 180;
    
}





void copy_arr(float* in, float* out, int n) {
  for(int i = 0; i < n; i++) {
    in[i] = out[i];
  }
}


void print_array(float *q) {
      
      for (int i = 0; i < joints; i++) {
          Serial.print(q[i]);
          Serial.print(" ");
         }
      Serial.println();
}

void print_cartesian_pos(float* q) {
  forward_kinematics(q);
  Serial.print("Current position: ");
  Serial.print(X*100);
  Serial.print(' ');
  Serial.print(Y*100);
  Serial.print(' ');
  Serial.print(Z*100);
  Serial.print(' ');
  Serial.print("pitch = ");
  Serial.println(pitch*180/pi);
  Serial.println("\n");
}


void print_menu() {
  Serial.println();
  Serial.println("1 - home position");
  Serial.println("2 - cartesian mode");
  Serial.println("3 - joint mode");
}
