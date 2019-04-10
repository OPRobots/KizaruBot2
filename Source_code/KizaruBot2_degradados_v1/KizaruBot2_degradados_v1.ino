#include <PIDfromBT.h>

//============================
//Pines de los motores
//============================
int MotorIalante = 4;
int MotorIatras = 5;
int MotorDalante = 7;
int MotorDatras = 8;
int PWM_MotorD = 9;
int PWM_MotorI = 3;

int estanbay = 6;

//============================
//Pines de los Sensores lellendolos de izquierda a derecha viendo el robot de la bateria a los sensores
//============================
int sensor[] = {A7, A6, A3, A2, A1};
int Sensor1 = A7;
int Sensor2 = A6;
int Sensor3 = A3;
int Sensor4 = A2;
int Sensor5 = A1;

//============================
//Pines de los interruptores para diferentes modos de velocidad
//============================
int boton = 2;
int interruptor1 = 10;
int interruptor2 = 11;
int interruptor3 = 12;

//============================
//Arrays de valor de sensores en negro en blanco (utilizados en la calibracion)//
//============================
int valores[] = {0, 0, 0, 0, 0, 0, 0};
int valores_min[] = {1023, 1023, 1023, 1023, 1023, 1023, 1023};
int valores_max[] = {0, 0, 0, 0, 0, 0, 0};
float media = 0;
float media_min = 1000;
float media_max = 0;


bool calibracion1_ok = false;
bool calibracion2_ok = false;

//============================
//Variables PID//
//============================    SIN MILLIS
float kp = 0;
float kd = 0;
float ki = 0;
float ideal = 0;
int posicion = 0;
int posicion_anterior = 0;
int proporcional = 0;
int derivada = 0;
int integral = 0;
int correccion = 0;

//============================
//Variables varias//
//============================
bool calibracion = true;
bool arranque = true;
int i = 0;
long millisAnterior = 0;
long millisInicio = 0;

int posicionMax = 500; //posible calcular comprovar valores
int posicionMin = -500; //     ""


int velD = 0;
int velI = 0;
int vel = 50;    //150

PIDfromBT pid_calibrate(&kp, &ki, &kd, &vel, &ideal, -250, 250, DEBUG);

//☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆
void setup() {
  //☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆

  pinMode(MotorIalante, OUTPUT);
  pinMode(MotorIatras, OUTPUT);
  pinMode(MotorDalante, OUTPUT);
  pinMode(MotorDatras, OUTPUT);


  pinMode(Sensor1, INPUT);
  pinMode(Sensor2, INPUT);
  pinMode(Sensor3, INPUT);
  pinMode(Sensor4, INPUT);
  pinMode(Sensor5, INPUT);


  pinMode(boton, INPUT_PULLUP);
  pinMode(interruptor1, INPUT_PULLUP);
  pinMode(interruptor2, INPUT_PULLUP);
  pinMode(interruptor3, INPUT_PULLUP);

  pinMode(13, OUTPUT);


  digitalWrite(MotorIalante, LOW);
  digitalWrite(MotorIatras, LOW);
  digitalWrite(MotorDalante, LOW);
  digitalWrite(MotorDatras, LOW);
  digitalWrite(estanbay, HIGH);


  Serial.begin(9600);

  //============================
  //realizamos la calibracion inicial obteniendo los minimos y maximos leidos por cada sensor durante 5 seg//
  //============================

  //Doble calibracion, los primeros 5 seg calcula max y min de cada sensor para leerlos haciendole un escalado a cada uno intentando leer sus valores uniforemente
  //Una vez ya podemos escalar el valor de los sensores calibramos max y min de las medias del circuito para saber entre que valores medios oscila para que no se salga
  //el led trece se apaga cuando acaba la primera calibracion y se vuelve a encender cuando comienza la segunda

  if (calibracion1_ok == false) {

    digitalWrite(13, HIGH);
    millisAnterior = millis();
    do {


      // registramos los valores Minimos y maximos en negros y blancos
      for (i = 0; i < 5; i++) {
        valores[i] = analogRead(sensor[i]);

        if (valores[i] < valores_min[i]) { //blanco
          valores_min[i] = valores[i];
        }
        if (valores[i] > valores_max[i]) { //negro
          valores_max[i] = valores [i];
        }
      }

    } while ((millis() - millisAnterior) <= 5000);

    for (i = 0; i < 5; i++) {
      Serial.print(valores_min[i]);
      Serial.print("   ");
    }
    for (i = 0; i < 5; i++) {
      Serial.print(valores_max[i]);
      Serial.print("   ");
    }
    Serial.println("   ");
    calibracion1_ok = true;
    digitalWrite(13, LOW);
    delay(700);
  }

  if (calibracion1_ok == true && calibracion2_ok == false) {

    digitalWrite(13, HIGH);
    millisAnterior = millis();
    do {
      // Calculamos los umbrales para cada sensor
      for (i = 0; i < 5; i++) {
        media = media + map(analogRead(sensor[i]), valores_min[i], valores_max[i], 0, 250);
      }
      media = media / 5;

      if (media < media_min) {
        media_min = media;
      } else if (media > media_max) {
        media_max = media;
      }

    } while ((millis() - millisAnterior) <= 5000);

    Serial.print(media_min);
    Serial.print("   ");
    Serial.print(media_max);
    Serial.println("   ");

    calibracion2_ok = true;
    digitalWrite(13, LOW);
  }

} //fin del setup

//☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆
void loop() {
  //☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆

  //============================
  //Comunicacion del bluethooth para calibracion del PID//
  //============================

  pid_calibrate.update();




  //============================
  //Secuencia de arranque, junto con las configuraciones de velocidad del switch
  //============================


  if (digitalRead(boton) == false && arranque == true) {
    arranque = false;
    millisInicio = millis();
  } else if ((millis() >= (millisInicio + 5000)) && (arranque == false) && (millisInicio > 0)) {

    if (digitalRead(10) == false) {
      vel = 10;
      kp = 0.5;   //1.65
      kd = 20;    //90
      ki = 0;

    } if (digitalRead(11) == false) {
      vel = 10;
      kp = 0.5;   //1.65
      kd = 20;    //90
      ki = 0;

    } if (digitalRead(12) == false) {
      vel = 10;
      kp = 0.5;   //1.65
      kd = 20;    //90
      ki = 0;
    }



    //============================
    //Calculo del analogico de desplazamiento del array de sensores
    //============================


    for (i = 0; i < 5; i++) {
      valores[i] = map(analogRead(sensor[i]), valores_min[i], valores_max[i], 0, 250);
    }
    media = 0;
    for (i = 0; i < 5; i++) {
      media += valores[i];
    }

    posicion = map((media / 5), media_min, media_max, 0, 500);
    
    if (((valores[4] + valores[3]) > (valores[0] + valores[1]))) { //||posicion_anterior < -5 PEDAZO ÑAPA DE LA HOSTIA QUE REVIENTA EL CODIGO AL INTENTAR MOVER EL IDEAL, CON IDEAL 0 DE PUTA MADRE
      posicion = posicion * -1;
    }

    posicion = posicion + ideal;  
    
    //============================
    //Calculo del PID
    //============================


    proporcional = posicion;
    derivada = posicion - posicion_anterior;
    integral = integral + (posicion / 10);
    //integral= constrain(integral,-500,500);

    correccion = ((kp * proporcional) + (kd * derivada) + (ki * integral));

    //  Serial.print(correccion);

    //este map sirve para ajustar la sensivilidad de las constantes del PID. bajando los valores iniciales del map se hace mas sensible.
    correccion = map(correccion, -2000, 2000, -255, 255);
    correccion = constrain(correccion, -255, 255);
    posicion_anterior = posicion;

    // }


    //    Serial.print(correccion);
    //    if(velocidad>0){
    //    Serial.print("   ");
    //    Serial.println(velocidad);
    //    }
    //    Serial.println("   ");
    //    Serial.println(posicion);
    //    Serial.print("   ");
    //   Serial.println(correccion);
    //    }




    //============================
    //Asignacion de velocidades a los motores//
    //============================

    //aqui aplicamos la correccion del pid a las velocidades de los motores
    //de tal modo que si la correccion hace que una rueda se ponga a 255 a la otra se le aplique el doble de correccion

    //  if(correccion>(255-vel)){
    //    velD = vel - (correccion*2);
    //    velI = 255;
    //    Serial.println("TopeI");
    //    }else if(correccion <((255-vel)*(-1))){
    //    velD = 255;
    //    velI = vel + (correccion*2);
    //    Serial.println("TopeD");
    //   }else{
    //    velD = vel - correccion;
    //   velI = vel + correccion;
    // }



    velD = vel - correccion;
    velI = vel + correccion;

    //limitamos desbordamientos
    velD = constrain(velD, -255, 255);
    velI = constrain(velI, -255, 255);

    //asiganmos valores a la rueda derecha teniendo en cuenta de que si el valor es negativo va hacia atras
    if (velD >= 0) {
      digitalWrite(MotorDalante, HIGH);
      digitalWrite(MotorDatras, LOW);
      analogWrite(PWM_MotorD, velD);
    } else {
      digitalWrite(MotorDalante, LOW);
      digitalWrite(MotorDatras, HIGH);
      analogWrite(PWM_MotorD, abs(velD));
    }

    //asiganmos valores a la rueda izquierda teniendo en cuenta de que si el valor es negativo va hacia atras
    if (velI >= 0) {
      digitalWrite(MotorIalante, HIGH);
      digitalWrite(MotorIatras, LOW);
      analogWrite(PWM_MotorI, velI);
    } else {
      digitalWrite(MotorIalante, LOW);
      digitalWrite(MotorIatras, HIGH);
      analogWrite(PWM_MotorI, abs(velI));
    }
    //============================
    //COMPROBACIÓN DE SENSORES
    //============================

    /*
      Serial.print(analogRead(Sensor1));
      Serial.print("   ");
      Serial.print(analogRead(Sensor2));
      Serial.print("   ");
      Serial.print(analogRead(Sensor3));
      Serial.print("   ");
      Serial.print(analogRead(Sensor4));
      Serial.print("   ");
      Serial.print(analogRead(Sensor5));
      Serial.print("   ");
      Serial.print(digitalRead(boton));
      Serial.print("   ");
      Serial.print(digitalRead(interruptor1));
      Serial.print("   ");
      Serial.print(digitalRead(interruptor2));
      Serial.print("   ");
      Serial.println(digitalRead(interruptor3));

    */

  } //fin de la secuencia de arranque
} //fin del loop


/*
  y esto es to...
  esto es to...
  esto es todo amigos
*/

