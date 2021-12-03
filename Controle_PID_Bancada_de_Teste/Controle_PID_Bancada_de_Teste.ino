///////////////////////////////////////////////////////////////////////////////
//                             Equipe Hawkings                               //
///////////////////////////////////////////////////////////////////////////////
// Desenvolvido por: Enrique, Mauro.                                         //
// Última att: 03/12/2021.                                                   //
///////////////////////////////////////////////////////////////////////////////
// Descrição:                                                                //
//                                                                           //
//  Esse código realiza um controle PID em uma bancada de testes com um      //
//  eixo de liberdade. A leitura do ângulo da bancada é feita utilizando     //
//  um MPU-6050                                                              //
///////////////////////////////////////////////////////////////////////////////
//  Conexões:                                                                //
//                                                                           //
//  MPU-6050 - Arduino                                                       //
//  VCC  -  5V                                                               //
//  GND  -  GND                                                              //
//  SDA  -  A4                                                               //
//  SCL  -  A5                                                               //
//                                                                           //
//  Transistor (motor) - Arduino                                             //
//  Base  -  Pino 3                                                          //
///////////////////////////////////////////////////////////////////////////////

#include <Wire.h>

//Declaração das variaveis globais
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;                                 //Define se a primeira leitura do MPU já foi realizada
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                     Controle PID                                                     //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float erro=0,kp=1.4,ki=0.01,kd=15,p=0,i=0,d=0,erro_anterior=0;
int setpoint=0,pid=0,atraso=10;
long tempo=0;

//kp= 1.4
//ki= 0.03
//kd= 15



void setup() {
  Wire.begin();                                                        //Inicia o I2C como master
  Serial.begin(57600);                                                 //Inicia a comunicação serial
  pinMode(13, OUTPUT);                                                 //Configura a porta 13 (led) como saida
  
  setup_mpu_6050_registers();                                          //Configura os resgistros do MPU-6050 (500dfs / +/-8g) e inicia o giroscópio 

  digitalWrite(13, HIGH);                                              //Acende o led para indicar o inicio da calibração do MPU

                                     
                                                    
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Execulta o código 2000 vezes para calibrar o MPU
    if(cal_int % 125 == 0)Serial.print(".");                           //Imprime um ponto "." no monitor serial a cada 125 leituras
    read_mpu_6050_data();                                              //Lê os dados brutos do acelerometro e do giroscopio do MPU-6050
    gyro_x_cal += gyro_x;                                              //Adiciona o deslocamento do eixo X do giroscopio à variavel gyro_x_cal
    gyro_y_cal += gyro_y;                                              //Adiciona o deslocamento do eixo Y do giroscopio à variavel gyro_y_cal
    gyro_z_cal += gyro_z;                                              //Adiciona o deslocamento do eixo Z do giroscopio à variavel gyro_z_cal
    delay(3);                                                          //Delay de 3us para simular um loop a 250Hz
  }
  gyro_x_cal /= 2000;                                                  //Divide a variavel gyro_x_cal por 2000 para obter o deslocamento médio
  gyro_y_cal /= 2000;                                                  //Divide a variavel gyro_y_cal por 2000 para obter o deslocamento médio
  gyro_z_cal /= 2000;                                                  //Divide a variavel gyro_z_cal por 2000 para obter o deslocamento médio

  
 digitalWrite(13, LOW);                                               //Tudo pronto, desliga o LED
  
  loop_timer = micros();                                               //Reinicializa o timer do loop

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                     Controle PID                                                     //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pinMode(3,OUTPUT);

  
}

void setup() {
  Wire.begin();                                                        //Inicia o I2C como master
  Serial.begin(57600);                                                 //Inicia a comunicação serial
  pinMode(13, OUTPUT);                                                 //Configura a porta 13 (led) como saida
  
  setup_mpu_6050_registers();                                          //Configura os resgistros do MPU-6050 (500dfs / +/-8g) e inicia o giroscópio 

  digitalWrite(13, HIGH);                                              //Acende o led para indicar o inicio da calibração do MPU

                                     
                                                    
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Execulta o código 2000 vezes para calibrar o MPU
    if(cal_int % 125 == 0)Serial.print(".");                           //Imprime um ponto "." no monitor serial a cada 125 leituras
    read_mpu_6050_data();                                              //Lê os dados brutos do acelerometro e do giroscopio do MPU-6050
    gyro_x_cal += gyro_x;                                              //Adiciona o deslocamento do eixo X do giroscopio à variavel gyro_x_cal
    gyro_y_cal += gyro_y;                                              //Adiciona o deslocamento do eixo Y do giroscopio à variavel gyro_y_cal
    gyro_z_cal += gyro_z;                                              //Adiciona o deslocamento do eixo Z do giroscopio à variavel gyro_z_cal
    delay(3);                                                          //Delay de 3us para simular um loop a 250Hz
  }
  gyro_x_cal /= 2000;                                                  //Divide a variavel gyro_x_cal por 2000 para obter o deslocamento médio
  gyro_y_cal /= 2000;                                                  //Divide a variavel gyro_y_cal por 2000 para obter o deslocamento médio
  gyro_z_cal /= 2000;                                                  //Divide a variavel gyro_z_cal por 2000 para obter o deslocamento médio

  
 digitalWrite(13, LOW);                                               //Tudo pronto, desliga o LED
  
  loop_timer = micros();                                               //Reinicializa o timer do loop
}

void loop(){

  read_mpu_6050_data();                                                //Lê os dados brutos de acelerômetro e do giroscópio do MPU-6050

  gyro_x -= gyro_x_cal;                                                //Subtrai o valor do deslocamento do valor bruto da variavel gyro_x
  gyro_y -= gyro_y_cal;                                                //Subtrai o valor do deslocamento do valor bruto da variavel gyro_y
  gyro_z -= gyro_z_cal;                                                //Subtrai o valor do deslocamento do valor bruto da variavel gyro_z
  
  //Cálculos do ângulo do giroscópio
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calcula o ângulo pitch percorrido e adiciona a variável angle_pitch
  angle_roll += gyro_y * 0.0000611;                                    //Calcula o ângulo roll percorrido e adiciona a variável angle_roll
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) A função sin(seno) do arduino esta em radianos
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //Se o MPU guinou(fez uma mudança brusca de sentido), transfira o ângulo de roll para o ângulo de pitch
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //Se o MPU guinou(fez uma mudança brusca de sentido), transfira o ângulo de pitch para o ângulo de roll
  
  //Cálculos do ângulo do acelerômetro
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calcula o vetor total do acelerômetro
  //57.296 = 1 / (3.142 / 180) A função asin(cosseno) do arduino esta em radianos
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calcula o ângulo do pitch
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calcula o ângulo do roll
  
  //Caso haja um desvio no valor de leitura do MPU para o ângulo de inclinação real, calibrar através dessas linhas, adiocionando ou subtraindo no valor do ângulo do MPU
  angle_pitch_acc -= -1.95;                                              //Calibração dos valores do acelerômetro para o pitch
  angle_roll_acc -= 0.0;                                               //Calibração dos valores do acelerômetro para o roll

  if(set_gyro_angles){                                                 //Compara se é a primeira leitura do MPU
                                                                       /*                                                          
                                                                        No MPU temos um grande problema que é o drift. O drift é como um delay na leitura dos valores de angulação                                                           
                                                                        isso nos gera um atraso entre o ângulo real do MPU e a leitura que ele nos informa. Isso ocorre pois apesar 
                                                                        de ser muito preciso em ler valores quando o MPU esta parado o acelerômetro é pessimo em leituras como MPU em movimento.
                                                                        Para corrigirmos isso usamos o giroscópio que não é tão preciso mas é otimo com leituras quando o MPU esta em movimento.
                                                                        O que fazemos é utilizar o valor do acelerômetro para a primeira leitura quando o MPU esta parado, para que tenhamos uma base precisa.
                                                                        E nas demais leituras utilizamos uma parcela do valor do acelerômetro (menor parcela) e uma parcela do valor do giroscópio (maior parcela).
                                                                        Dessa forma conseguimos um valor mais preciso para nossas leituras. Caso haja algum problema com drift ou com a precisão das nossas leituras
                                                                        Podemos resolver facilmente alterando as proporções do acelerometro e do giroscopio utilizadas.
                                                                         
                                                                       */    
    angle_pitch = angle_pitch * 0.9900 + angle_pitch_acc * 0.0100;     //Corrige o drift do giroscópio no pitch com o ângulo do acelerômetro no pitch
    angle_roll = angle_roll * 0.9900 + angle_roll_acc * 0.0100;        //Corrige o drift do giroscópio no roll com o ângulo do acelerômetro no roll
  }
  else{                                                                //Se for a primeira leitura
    angle_pitch = angle_pitch_acc;                                     //Define o ângulo do pitch do giroscópio igual ao ângulo do pitch do acelerômetro
    angle_roll = angle_roll_acc;                                       //Define o ângulo do roll do giroscópio igual ao ângulo do roll do acelerômetro
    set_gyro_angles = true;                                            //Define que a primeira leitura ja foi realizada
  }
  
  //Filtro complementar, usado para amortecer os ângulos pitch e roll
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Pega 90% do valor do pitch de saída e adicione 10% do valor bruto do pitch
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Pega 90% do valor do roll de saída e adicione 10% do valor bruto do roll
  
  write_serial();                                                      //Chama a função write_serial que escreve os valores de pitch e roll no monitor serial

  while(micros() - loop_timer < 4000);                                 //Espera até que o loop_timer atinja 4000us (250 Hz) antes de iniciar o próximo loop. Isso nos garanteque nosso programarode a 250Hz
  loop_timer = micros();                                               //Reseta o loop_timer
  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                     Controle PID                                                     //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(millis()>=tempo+atraso){ 
    tempo=millis();
    erro_anterior=erro;
   }
  erro=setpoint-angle_pitch_output;
  p=erro*kp;
  i+=erro*ki;
  d=(erro-erro_anterior)*kd;
  if(i>=255){
    i=255;
  }
  
  if(i<=0){
    i=0;
  }
  
  pid=p+i+d;
  if(pid>=255){
    pid=255;
  }
  if(pid<=0){
    pid=0;
  }
  
  analogWrite(3,pid);

}


void read_mpu_6050_data(){                                             //Função que le os dados brutos do giroscópio e do acelerômetro
  Wire.beginTransmission(0x68);                                        //Inicializa a comunicação com o MPU-6050
  Wire.write(0x3B);                                                    //Informa qual o registro solicitado
  Wire.endTransmission();                                              //Finaliza a transmição
  Wire.requestFrom(0x68,14);                                           //Solicite 14 bytes do MPU-6050
  while(Wire.available() < 14);                                        //Aguarda até que todos os bytes solicitados sejam recebidos
  acc_x = Wire.read()<<8|Wire.read();                                  //Adicione o byte inferior e superior à variável acc_x
  acc_y = Wire.read()<<8|Wire.read();                                  //Adicione o byte inferior e superior à variável acc_y
  acc_z = Wire.read()<<8|Wire.read();                                  //Adicione o byte inferior e superior à variável acc_z
  temperature = Wire.read()<<8|Wire.read();                            //Adicione o byte inferior e superior à variável temperature
  gyro_x = Wire.read()<<8|Wire.read();                                 //Adicione o byte inferior e superior à variável gyro_x
  gyro_y = Wire.read()<<8|Wire.read();                                 //Adicione o byte inferior e superior à variável gyro_y
  gyro_z = Wire.read()<<8|Wire.read();                                 //Adicione o byte inferior e superior à variável gyro_z

}

void write_serial(){                                                   //Função que escreve os valores de pitch e roll no monitor serial
  //Para obter um loop de programa de 250Hz (4us), só é possível escrever um caractere por loop
  erro=setpoint-angle_pitch_output;
  Serial.print(" Pitch "); 
  Serial.print(angle_pitch_output);
 // Serial.print(" Roll "); 
 // Serial.print(angle_roll_output);
  Serial.print(" Erro ");
  Serial.print(erro);
  Serial.print(" PWM ");
  Serial.print(pid);
  Serial.println("");
}

void setup_mpu_6050_registers(){
  //Ativaçãodo MPU-6050
  Wire.beginTransmission(0x68);                                        //Inicializa a comunicação com MPU-6050
  Wire.write(0x6B);                                                    //Informa qual o registro solicitado
  Wire.write(0x00);                                                    //Configura o registro inicial solicitado
  Wire.endTransmission();                                              //Finaliza a transmição
  //Configura o acelerômetro (+/- 8g)
  Wire.beginTransmission(0x68);                                        //Inicializa a comunicação com MPU-6050
  Wire.write(0x1C);                                                    //Informa qual o registro solicitado
  Wire.write(0x10);                                                    //Configura o registro inicial solicitado
  Wire.endTransmission();                                              //Finaliza a transmição
  //Configura o giroscópio (escala completa de 500dps)
  Wire.beginTransmission(0x68);                                        //Inicializa a comunicação com MPU-6050
  Wire.write(0x1B);                                                    //Informa qual o registro solicitado
  Wire.write(0x08);                                                    //Configura o registro inicial solicitado
  Wire.endTransmission();                                              //Finaliza a transmição
}
