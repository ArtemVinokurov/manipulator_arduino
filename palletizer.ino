#include <DynamixelWorkbench.h>
#include <math.h>

#define DEVICE_NAME "3"

#define BAUDRATE  1000000

#define joints 4

DynamixelWorkbench dxl;


float pi = 3.14159265;      // Число пи

float unit = 0.111;         // Переменная, отвечающая за перевод радиан в условные единицы положения привода двигателя 


float l1 = 0.07;            // Длины звеньев манипулятора
float l2 = 0.14;
float l3 = 0.14;
float l4 = 0.013;
float l5 = 0.052;

float max_velocity = 70;    // Максимальная скорость сервоприводов (измените, если хотите уменьшить или увеличить скорость перемещения манипулятора)

float CONSTRAINTS[joints][2] = {{-2.62, 2.62},    // Двумерный массив, содержащий предельные значения для каждой степени подвижности
                                {-0.9, 1.25},
                                {-0.35, 1.21},
                                {-2.62, 2.62}};

float HOME_POSITION[joints] = {0, 0, 0, 0};     // Массив, задающий начальное положение манипулятора

uint8_t dxl_id[joints] = {1, 2, 3, 4};        // Массив ID приводов

float Q_d[joints];    // Текущие обобщенные координаты сочленений, рад 

float Q_0[joints];    // Вычисленные обобщенные координаты сочленений, рад

float X_d, Z_d, phi_d;    // Заданные координаты и ориентация схвата, см

float X, Z, phi;      // Теукщие координаты схвата

float roll_d = 0;     // Угол поворота схвата 



void setup() {
  
  Serial.begin(57600);   // Инициализация Serial-порта
 
  while(!Serial);   // Ждем открытия Serial-порта

  // Инициализируем параметры регуляторов (выбираются экспериментально)
  
  uint8_t CW_slope[joints] = {64, 64, 64, 32};
  uint8_t CCW_slope[joints] = {64, 64, 64, 32};
  uint8_t CW_margin[joints] = {1, 1, 1, 1};
  
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
  dxl.itemWrite(dxl_id[i], "CCW_Compliance_Slope", CCW_slope[i], &log);
  
  }
  
  go_home();       // Перемещение манипулятора в начальное положение
  
  print_menu();      // Вывод меню взаимодействия с Serial-портом

}




void loop() {

  // Ожидаем ввода данных, согласно меню, в Serial-порт

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

  get_pos();  // Считываем текущие положения сервоприводов в массив Q_0
  
  float* q0 = Q_0;  // Инициализируем вспомогательный указатель  

  // В цикле вычисляем T_max
  for (int i = 0; i < joints; i++) {
    
    dq[i] = q[i] - q0[i];
    float temp = abs((3 * dq[i])/(2 * q_dot_max));
    if (temp > T_max) T_max = temp;
    
  }

  // Вычисляем требуемые скорости приводов и перемещаем их в заданное положение
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
      if(inverse_problem(Q_d)) {             // Решаем обратную задачу кинематики для введенных координат,
        moving(Q_d);                      // если получено корректное решение передаем его в функцию moving
        print_cartesian_pos(Q_d);
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


void forward_kinematics(float* q){
  float q1 = q[0];
  float q2 = pi/2 - q[1];
  float q3 = q[2];
  float x = l2 * cos(q2) + l3 * cos(q3);
  float z = l2 * sin(q2) - l3*sin(q3);
  X = x + l5;
  Z = z + l1 - l4;
  phi = q1;
  
}



bool inverse_problem(float* q_d) {
  float x = X_d - l5;
  float z = Z_d - l1 + l4;
  float q1, q2, q3, q3_, q4;
   
  q1 = phi_d;
    
  float d = sqrt(x*x + z*z);
  
  float gamma;
  
  // Проверяем не является ли заданное положение вырожденным
  
  float arg1 = (l2*l2 + d*d - l3*l3)/(2*l2*d);
  if (arg1 >= -1 && arg1 <= 1) {
    gamma = acos(arg1);
  }
  else {
    Serial.println("Uncorrect coordinates!");
    return false;
  }
  

  float beta = atan2(z, x);
  q2 = pi/2 - (gamma + beta);
  
  float alpha;
  float arg2 = (l2*l2 + l3*l3 - d*d)/(2*l2*l3);

  // Проверяем не является ли заданное положение вырожденным
  if (arg2 >= -1 && arg2 <= 1) {
    alpha = acos(arg2);
  }
  else {
    Serial.println("Uncorrect coordinates!");
    return false;
  }
  q3_ = pi - alpha;
  q3 = q3_ - (pi/2 - q2);
  q4 = roll_d * pi / 180;

  float q[joints] = {q1, q2, q3, q4};

  // Проверка полученного решения на ограничения сервоприводов
  
  if (check_constrain(q)) {
    copy_arr(q_d, q, joints);
    return true;
  }
  else {
    Serial.println("Constrain!");
    return false;
  }
}



void get_pos() {
  for (int i = 0; i < joints; i++) {
    
    if (!dxl.getRadian(dxl_id[i], &(Q_0[i])))
    {
      Serial.print("Error get position id : ");
      Serial.println(dxl_id[i]);
    }
    //else Serial.println(Q_0[i]);
  }
}




void check_moving() {  
  int32_t data;
  for (int i = 0; i < joints; i++) {
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
  
  byte dividerIndex = tempString.indexOf(' ');  
  String buf1 = tempString.substring(0, dividerIndex);
  tempString = tempString.substring(dividerIndex + 1);

  parsingLocal(buf1);
  
  if (dividerIndex != 255) parsingGlobal(tempString);
}


void parsingLocal(String tempString) {
  
  byte dividerIndex = tempString.indexOf('=');  //  Поиск символа разделителя "="
  String buf1 = tempString.substring(0, dividerIndex);   // Записываем в переменную buf1 символ до разделителя, обозначающий декартову или обобщенную координату
  String buf2 = tempString.substring(dividerIndex + 1);  // Записываем в переменную buf2 численное значение декартовой или обобщенной в виде строки 

  // Распознаем символ до разделителя и присваиваем соответствующим переменным численное значение
  if (buf1.equals("x"))
    X_d = buf2.toFloat() / 100;

  if (buf1.equals("z"))
    Z_d = buf2.toFloat() / 100;
      
  if (buf1.equals("p"))
    phi_d = buf2.toFloat() * pi / 180;

  if (buf1.equals("q1")) 
    Q_d[0] = buf2.toFloat() * pi / 180;
  
  if (buf1.equals("q2"))
    Q_d[1] = buf2.toFloat() * pi / 180;

  if (buf1.equals("q3")) 
    Q_d[2] = buf2.toFloat() * pi / 180;

  if (buf1.equals("q4")) 
    Q_d[3] = buf2.toFloat() * pi / 180;
   
}



void copy_arr(float* in, float* out, int n) {
  for(int i = 0; i < n; i++) {
    in[i] = out[i];
  }
}



void print_cartesian_pos(float* q) {
  forward_kinematics(q);
  Serial.print("Current position: ");
  Serial.print(X*100);
  Serial.print(' ');
  Serial.print(Z*100);
  Serial.print(' ');
  Serial.print("phi = ");
  Serial.println(phi*180/pi);
  Serial.println("\n");
}


void print_menu() {
  Serial.println();
  Serial.println("1 - home position");
  Serial.println("2 - cartesian mode");
  Serial.println("3 - joint mode");
}
