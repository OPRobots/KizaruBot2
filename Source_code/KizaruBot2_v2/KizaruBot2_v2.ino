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
int Sensor1 = A7;
int Sensor2 = A6;
int Sensor3 = A3;
int Sensor4 = A2;
int Sensor5 = A1;

//Pines de los interruptores para diferentes modos de velocidad
int boton = 2;
int interruptor1 = 10;
int interruptor2 = 11;
int interruptor3 = 12;

//Arrays de valor de sensores en negro en blanco y el umbral (utilizados en la calibracion)
int negros[] = {0, 0, 0, 0, 0};
int blancos[] = {1023, 1023, 1023, 1023, 1023};
int umbrales[5];
int umbrales_map[5];

int pesos[] = { -2, -1, 0, 1, 2};
// este array es un array intermediario para hacer operaciones no tiene relevancia en el programa
int lectura[5];

//Variables PID    SIN MILLIS
float kp = 0.9;
float kd = 27;
float ki = 0;
int posicion = 0;
int posicion_anterior = 0;
int proporcional = 0;
int derivada = 0;
int integral = 0;
int correccion = 0;

//Variables barias
bool calibracion = true;
bool arranque = false;
int i = 0;
int recivido = 0;
long millisAnterior = 0;
long millisInicio = 0;
long millisAnteriorPID = 0;
int umbral_blanco = 0; //mantener a 0 para interpretar los valores en "digital"
int umbral_negro = 0; //se puede abrir el abanico de deteccion aumentando estos numeros siempre en positivo

int sumVals;
int sumValsPesos;
int lineSensors;
int posicionMax = 500; //posible calcular comprovar valores
int posicionMin = -500; //     ""


int velD = 0;
int velI = 0;
int vel = 50;    //150

//PIDfromBT pid_calibrate(&kp, &ki, &kd, &vel, DEBUG);

void setup() {
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

  ///////////////////////////////////////////////////
  /*realizamos la calibracion inicial obteniendo los minimos y maximos leidos por cada sensor durante 5 seg y calculamos los umbrales*/
  ///////////////////////////////////////////////////


  digitalWrite(13, HIGH);
  millisAnterior = millis();
  do {

    // lectura de los sensores
    lectura[0] = analogRead(Sensor1);
    lectura[1] = analogRead(Sensor2);
    lectura[2] = analogRead(Sensor3);
    lectura[3] = analogRead(Sensor4);
    lectura[4] = analogRead(Sensor5);

    // registramos los valores Minimos y maximos en negros y blancos
    for (i = 0; i < 5; i++) {
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


  ///////////////////////////////////////////////////
  //Comunicacion del bluethooth para calibracion del PID
  ///////////////////////////////////////////////////

  //pid_calibrate.update();




  ///////////////////////////////////////////////////
  //Secuencia de arranque, junto con las configuraciones de velocidad del switch
  ///////////////////////////////////////////////////


  if (digitalRead(boton) == false) {
    arranque = true;
    millisInicio = millis();
  } else if ((millis() >= (millisInicio + 5000)) && (arranque == true) && (millisInicio > 0)) {

    if (digitalRead(10) == false) {
      /*vel = 80;
      kp = 1;   //1.65
      kd = 32;    //90
      ki = 0;*/

      vel = 100;
      kp = 1.17;   //1.65
      kd = 38;    //90
      ki = 0;

    } if (digitalRead(11) == false) {
      /*vel = 100;
      kp = 1.17;   //1.65
      kd = 38;    //90
      ki = 0;*/
      vel = 108;
      kp = 1.20;  //1.65
      kd = 44;   //90
      ki = 0;

    } if (digitalRead(12) == false) {
      vel = 115;
      kp = 1.20;  //1.65
      kd = 44;    //90
      ki = 0;
    }



    ///////////////////////////////////////////////////
    //Calculo del analogico de desplazamiento del array de sensores
    ///////////////////////////////////////////////////

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
      
  unsigned long sumaSensoresPonderados = 0;
  unsigned long sumaSensores = 0;
      for (i = 0; i < 5; i++) {
        //sumVals += (lectura[i] * pesos[i]);
    sumaSensoresPonderados += (i + 1) * lectura[i] * 1000L;
    sumaSensores += (long)lectura[i];
        if (lectura[i] > umbrales_map[i]) {
          lineSensors += 1;
        }
      }

      if (lineSensors > 0) {
        //posicion = (double)sumVals / (double)lineSensors;
        posicion = ((sumaSensoresPonderados / sumaSensores) - (5 + 1) * (float)(1000 / 2));
      } else if (posicion > 0) {
        posicion = 2300;
      } else {
        posicion = -2300;
      }
      posicion = map(posicion, -2500,2500,posicionMin, posicionMax);
      //  Serial.println(posicion);
      lineSensors = 0;
      sumVals = 0;
      sumValsPesos = 0;
      ///////////////////////////////////////////////////
      //Calculo del PID
      ///////////////////////////////////////////////////

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


      ///////////////////////////////////////////////////
      //Asignacion de velocidades a los motores
      ///////////////////////////////////////////////////



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
      millisAnteriorPID = millis();
    } // fin de la ejecuccion del codigo cada 1ms

    //Serial.print(velD);
    //  Serial.print("   ");
    //  Serial.println(velI);

    ///////////////////////////////////////
    //COMPROBACIÓN DE SENSORES
    ////////////////////////////////////////

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
