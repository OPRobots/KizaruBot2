//#include <PIDfromBT.h>

//Pines de los motores
int MotorIalante = 4;
int MotorIatras = 5;
int MotorDalante = 7;
int MotorDatras = 8;
int PWM_MotorD = 9;
int PWM_MotorI = 3;

int estanbay = 6;

//Pines de los Sensores lellendolos de izquierda a derecha viendo el robot de la bateria a los sensores
int sensores[] = {A7, A6, A3, A2, A1};

//Pines de los interruptores para diferentes modos de velocidad
int boton = 2;
int interruptor[] = {10, 11, 12};

//Arrays de valor de sensores en negro en blanco y el umbral (utilizados en la calibracion)
int negros[] = {0, 0, 0, 0, 0};
int blancos[] = {1023, 1023, 1023, 1023, 1023};
int umbrales[5];
int umbrales_map[5];

// este array es un array intermediario para hacer operaciones
long lectura[5];

//Variables PID
float kp = 0;
float kd = 0;
float ki = 0;
int posicion = 0;
int posicion_anterior = 0;
int proporcional = 0;
int derivada = 0;
int integral = 0;
int correccion = 0;

//Variables varias

int pin[] = {0, 0, 0};
int binario = 1;
int estrategia = 0;

bool calibracion = true;
int umbral_blanco = 0; //mantener a 0 para interpretar los valores en "digital"
int umbral_negro = 0; //se puede abrir el abanico de deteccion aumentando estos numeros siempre en positivo

bool arranque = false;
int i = 0;
long millisAnterior = 0;
long millisInicio = 0;
long millisAnteriorPID = 0;

long cont_parada = 0;

unsigned long sumaSensoresPonderados = 0;
unsigned long sumaSensores = 0;

int lineSensors;
int posicionMax = 500; //posible calcular comprovar valores
int posicionMin = -500; //     ""


int velD = 0;
int velI = 0;
int vel = 0;    //Escala 0 - 100.0%

//PIDfromBT pid_calibrate(&kp, &ki, &kd, &vel, DEBUG);







void setup() {
  pinMode(MotorIalante, OUTPUT);
  pinMode(MotorIatras, OUTPUT);
  pinMode(MotorDalante, OUTPUT);
  pinMode(MotorDatras, OUTPUT);

  for (i = 0; i < 5 ; i++) {
    pinMode(sensores[i], INPUT);
  }

  for (i = 0; i < 3 ; i++) {
    pinMode(interruptor[i], INPUT_PULLUP);
  }

  pinMode(boton, INPUT_PULLUP);

  pinMode(13, OUTPUT);


  digitalWrite(MotorIalante, LOW);
  digitalWrite(MotorIatras, LOW);
  digitalWrite(MotorDalante, LOW);
  digitalWrite(MotorDatras, LOW);
  digitalWrite(estanbay, HIGH);


  Serial.begin(9600);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////   realizamos la calibracion inicial obteniendo los minimos y maximos leidos por cada sensor durante 5 seg y calculamos los umbrales    ////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  digitalWrite(13, HIGH);
  millisAnterior = millis();
  do {

    // lectura de los sensores
    for (i = 0; i < 5; i++) {
    }

    // registramos los valores Minimos y maximos en negros y blancos
    for (i = 0; i < 5; i++) {
      lectura[i] = analogRead(sensores[i]);
      if (lectura[i] > negros[i]) {
        negros[i] = lectura[i];
      }
      if (lectura[i] < blancos[i]) {
        blancos[i] = lectura[i];
      }
    }

  } while ((millis() - millisAnterior) <= 5000);

  // Calculamos los umbrales para cada sensor
  for (i = 0; i < 5; i++) {
    umbrales[i] = ((negros[i] + blancos[i]) / 2) + 100;
    umbrales_map[i] = map (umbrales[i], blancos[i], negros[i], 0, 255);
  }

  for (i = 0; i < 5; i++) {
    Serial.print(umbrales_map[i]);
    Serial.print("   ");
  }
  Serial.println("   ");
  calibracion = false;
  digitalWrite(13, LOW);

} //fin del setup








void loop() {
  // /*

  ///////////////////////////////////////////////////////////////////
  ////   Comunicacion del bluethooth para calibracion del PID    ////
  ///////////////////////////////////////////////////////////////////

  //pid_calibrate.update();


  //////////////////////////////////////////////////////////////////////////////////////////
  ////  Secuencia de arranque, junto con las configuraciones de velocidad del switch    ////
  //////////////////////////////////////////////////////////////////////////////////////////

  if (digitalRead(boton) == false) {
    digitalWrite(13, HIGH);
    arranque = true;
    millisInicio = millis();
  } else if ((millis() >= (millisInicio + 5000)) && (arranque == true) && (millisInicio > 0)) {

    binario = 1;
    estrategia = 0;
    for (i = 2; i >= 0; i--) {
      pin[i] = !digitalRead(interruptor[i]);
      estrategia = estrategia + (pin[i] * binario);
      binario = binario * 2;
    }

    switch (estrategia) {

      case 0:
        vel = 300;   //Escala 0 - 100.0%  --  PWM 51
        kp = 6;
        kd = 80;
        break;

      case 1:
        vel = 350;   //Escala 0 - 100.0%  --  PWM 82.85
        kp = 6;
        kd = 80;
        break;

      case 2:
        vel = 400;   //Escala 0 - 100.0%  --  PWM 89.25
        kp = 6;
        kd = 80;
        break;

      case 3:
        vel = 450;   //Escala 0 - 100.0%  --  PWM 95.6
        kp = 6;
        kd = 80;
        break;

      case 4:
        vel = 500;   //Escala 0 - 100.0%  --  PWM 102
        kp = 6;
        kd = 80;
        break;

      case 5:
        vel = 550;   //Escala 0 - 100.0%  --  PWM 108
        kp = 6;
        kd = 80;
        break;

      case 6:
        vel = 600;   //Escala 0 - 100.0%  --  PWM 114.75
        kp = 6;
        kd = 80;
        break;

      case 7:
        vel = 650;   //Escala 0 - 100.0%  --  PWM 121
        kp = 6;
        kd = 80;
        break;

      case 8:
        vel = 700;   //Escala 0 - 100.0%  --  PWM 127.5
        kp = 6;
        kd = 80;
        break;
    }

    //////////////////////////////////////////////////////////////////////////
    ////  Calculo del analogico de desplazamiento del array de sensores   ////
    //////////////////////////////////////////////////////////////////////////

    //cada 1 milisegundos lee los sensores y llama a la funcion que ejecuta el calculo para hacerlo de manera periodica

    if (millis() >= millisAnteriorPID + 1) {

      for (i = 0; i < 5; i++) {
        lectura[i] = map ((analogRead(sensores[i])), blancos[i], negros[i], 0, 255);
        if (lectura[i] < umbrales_map[i] - umbral_blanco) {
          lectura[i] = 0;
        } else if (sensores[i] > umbrales_map[i] + umbral_negro) {
          lectura[i] = 255;
        }
      }

      sumaSensoresPonderados = 0;
      sumaSensores = 0;
      for (i = 0; i < 5; i++) {
        sumaSensoresPonderados = sumaSensoresPonderados + ((i + 1) * lectura[i] * 1000L);
        sumaSensores = sumaSensores + lectura[i];
        if (lectura[i] > umbrales_map[i]) {
          lineSensors = lineSensors + 1;
        }
      }

      if (lineSensors > 0) {
        if (lineSensors >= 4) {
          cont_parada++;
        }else{
        cont_parada = 0;
        }
        posicion = ((sumaSensoresPonderados / sumaSensores) - 3000);   //3000 = (5 + 1) * (float)(1000 / 2))
      } else if (posicion > 0) {
        posicion = 2300;
        cont_parada = cont_parada + 1;
      } else {
        posicion = -2300;
        cont_parada = cont_parada + 1;
      }

      posicion = map(posicion, -2500, 2500, posicionMin, posicionMax);
      lineSensors = 0;

      //  Serial.println(posicion);

      //////////////////////////////
      ////    Calculo del PID   ////
      //////////////////////////////

      proporcional = posicion;
      derivada = posicion - posicion_anterior;
      //integral = integral + (posicion / 10);
      //integral= constrain(integral,-500,500);

      correccion = ((kp * proporcional) + (kd * derivada));      // + (ki * integral));

      posicion_anterior = posicion;

      //este map sirve para ajustar la sensivilidad de las constantes del PID. bajando los valores iniciales del map se hace mas sensible.
      correccion = map(correccion, -10000, 10000, -1000, 1000);

      //  Serial.println(correccion);


      /////////////////////////////////////////////////////
      ////   Asignacion de velocidades a los motores   ////
      /////////////////////////////////////////////////////


      //aqui aplicamos la correccion del pid a las velocidades de los motores

      velD = vel - correccion;
      velI = vel + correccion;

      velD = map(velD, -1000, 1000, -255, 255);
      velI = map(velI, -1000, 1000, -255, 255);

      //de tal modo que si la correccion hace que una rueda se ponga a mas de 255, se limita a 255 y a la otra se le aplique la correccion restante
      if (velD > 255) {
        velI = velI - (velD - 255);
        velD = 255;
      }
      if (velI > 255) {
        velD = velD - (velI - 255);
        velI = 255;
      }

      // si pasa mucho tiempo sin leer linea parar
      if (cont_parada > 150) {
        velD = 0;
        velI = 0;
        correccion = 0;
      }

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
      millisAnteriorPID = millis();
    } // fin de la ejecuccion del codigo cada 1ms


  } else {   //fin de la secuencia de arranque
    digitalWrite(13, LOW);
  }


  ////////////////////////////////////////
  ////    COMPROBACIÓN DE SENSORES    ////
  ////////////////////////////////////////
  // */
  /*
    for (int sensor = 0; sensor < 5; sensor++) {
    Serial.print(analogRead(sensores[sensor]));
    Serial.print("   ");
    }
    Serial.print(digitalRead(boton));
    Serial.print("   ");
    Serial.print(digitalRead(interruptor[0]));
    Serial.print("   ");
    Serial.print(digitalRead(interruptor[1]));
    Serial.print("   ");
    Serial.println(digitalRead(interruptor[2]));

  */


} //fin del loop



/*
  y esto es to...
  esto es to...
  esto es todo amigos
*/
