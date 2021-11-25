#include "mbed.h"
#define INTERVAL 50
#define DSTINTERVAL 150
#define IRINTERVAL 80
#define PULSE 10
#define HIGH 1
#define STOP 0
#define PULSEMIN 500
#define PULSEMAX 2350 
#define FOWARD 1
#define BACKWARD 2
#define MOTORPULSE 500
#define DISTANCIA 348 //6cm
#define DISTANCIARANGO 250//4.3 cm 
#define DISTANCIAMAX 1800//20cm aprox
#define ANGULOMAX 90
#define ANGULOMIN -90
#define TURNLEFT 45//pulsos de horuqilla para -90
#define TURNRIGHT 45 //pulsos de horquilla para 90
typedef union{
            unsigned char modeOn: 1;
            unsigned char servoMoved: 1;
            unsigned char idleFlag: 1;
            unsigned char GetLine: 1;
            unsigned char readyPosition: 1;
            unsigned char b5: 1;
            unsigned char b6: 1;
            unsigned char b7: 1;
    }_uflag;
_uflag myFlags;
typedef union {
    int32_t i32;
    uint32_t ui32;
    uint16_t ui16[2];
    uint8_t ui8[4];
}_udat;

_udat myWord;
typedef union {
    int32_t i32;
    int16_t i16[2];
    int8_t i8[4];
}_udatint;

_udatint myNewWord;

typedef enum{
    IDLE,
    FOLLOW,
    LINE,
    ESCAPE,
}_emode;
_emode mode;
typedef enum{
    SEARCHING,
    AVOIDING,
}_emodeState;

_emodeState modeState;

typedef enum{
    START,
    HEADER_1,
    HEADER_2,
    HEADER_3,
    NBYTES,
    TOKEN,
    PAYLOAD
}_eProtocolo;

_eProtocolo estadoProtocolo;
 typedef struct{
    uint8_t timeOut;         //!< TiemOut para reiniciar la máquina si se interrumpe la comunicación
    uint8_t cheksumRx;       //!< Cheksumm RX
    uint8_t cheksumtx;       //!< Cheksumm Tx
    uint8_t indexWriteRx;    //!< Indice de escritura del buffer circular de recepción
    uint8_t indexReadRx;     //!< Indice de lectura del buffer circular de recepción
    uint8_t indexWriteTx;    //!< Indice de escritura del buffer circular de transmisión
    uint8_t indexReadTx;     //!< Indice de lectura del buffer circular de transmisión
    uint8_t bufferRx[256];   //!< Buffer circular de recepción
    uint8_t bufferTx[256];   //!< Buffer circular de transmisión
    uint8_t payload[32];     //!< Buffer para el Payload de datos recibidos
}_sDato ;

volatile _sDato datosComProtocol;

typedef enum{
        ACK=0x0D,
        ALIVE=0xF0,
        GET_IR=0xA0,
        MOTOR_T=0xA1,
        SERVO_T=0xA2,
        GET_DISTANCE=0xA3,
        GET_SPEED=0xA4,
    }_eID;

typedef enum{
    BUTTON_DOWN,    //0
    BUTTON_UP,      //1
    BUTTON_FALLING, //2
    BUTTON_RISING   //3
}_eButtonState;

_eButtonState myButton;

typedef struct{
    uint8_t estado;
    int32_t timeDown;
    int32_t timeDiff;
}_sTeclas;
_sTeclas ourButton;

void miTrigger(); 
void StartEcho();
void EndEcho();
void servoMove(int8_t angulo);
void actuallizaMef();
void startMef();
void heartBeatTask();
void myMotor1(uint8_t direction);
void myMotor2(uint8_t direction); 
void onDataRx();
void decodeProtocol();
void decodeData();
void sendData();
void readSpeedHl();
void readSpeedHr();
void readSensors();
void readIr();

DigitalOut LED(PC_13);
DigitalOut LEDAUX(PB_9);

PwmOut SERVO(PB_10);
PwmOut ENA(PA_8);//pin para controlar la velocidad del motor 
PwmOut ENB(PB_5);//pin para controlar la velocidad del motor 

Serial pcCom(PA_9,PA_10,115200);//para regular el puerto serie 

DigitalOut IN2(PB_14); 
DigitalOut IN1(PB_15);
DigitalOut IN3(PB_6);
DigitalOut IN4(PB_7);

//b12 ech84o b13 trigger
InterruptIn ECHO(PB_12);
DigitalOut TRIGGER(PB_13);

DigitalIn BUTTON(PB_4);

AnalogIn IR0(PA_0);
AnalogIn IR1(PA_1);

InterruptIn HL(PB_9);
InterruptIn HR(PB_8);//horquillas

Timer miTimer; 
Timer miTimer1;
Ticker readSensor;
Timeout timeout;
uint8_t factor,counterM1=0,counterM2=0,counterRight=0,counterLeft=0,rotationLeft=0,rotationRight=0;
uint32_t timeHb=0,timeSaved=0,timeToChk=0,timeServo=0,timeIr=0,timePos=0,speedM1=0,speedM2=0;
uint16_t ANCHO=1000,valueIr0,valueIr1;
int32_t testM1,testM2;
int16_t difValue;
int8_t angulo=0;
 
int main(){
    
    ENA.period_us(1000);
    ENB.period_us(1000);
    ENA.pulsewidth_us(MOTORPULSE);
    ENB.pulsewidth_us(MOTORPULSE);
    factor=(PULSEMAX-PULSEMIN)/180;
    myMotor1(FOWARD);
    myMotor2(FOWARD);
    wait_ms(500);
    
    myMotor1(BACKWARD);
    myMotor2(BACKWARD);
    wait_ms(500);
    servoMove(ANGULOMAX);
    myMotor1(STOP);    
    myMotor2(STOP);

    //SERVO.pulsewidth_us(PULSEMIN);
    
    wait_ms(400);
    servoMove(ANGULOMIN);
    wait_ms(400);
    servoMove(STOP);
    //SERVO.pulsewidth_us((PULSEMIN+PULSEMAX)/2);
    
    startMef();
    miTimer.start();
    miTimer1.start();
    pcCom.attach(&onDataRx,Serial::RxIrq);//
    
    HL.rise(&readSpeedHl);
    HR.rise(&readSpeedHr);
    readSensor.attach_us(&readSensors,PULSEMIN*1000);

    ECHO.rise(&StartEcho);
    ECHO.fall(&EndEcho);
    myFlags.modeOn=STOP;
    myFlags.GetLine=STOP; 
    myFlags.readyPosition=STOP;
    mode=IDLE;
    while(1){
        if(miTimer.read_ms()-timeToChk > DSTINTERVAL){
                TRIGGER.write(HIGH);
                timeout.attach_us(&miTrigger,PULSE);
                timeToChk=miTimer.read_ms();
                LED=!LED;
            }
        if(miTimer.read_ms()-timeHb > INTERVAL){
            actuallizaMef();
            timeHb=miTimer.read_ms();
        }
        if(miTimer.read_ms()-timeIr > IRINTERVAL){
            readIr();
            timeIr=miTimer.read_ms();
        }
        
        switch (mode)
        {
        case IDLE:
                if(myFlags.idleFlag==STOP){
                    myMotor1(STOP);
                    myMotor2(STOP);
                    servoMove(STOP);
                    myFlags.idleFlag=HIGH;
                }
            break;
        case FOLLOW:
            //if(myFlags.modeOn==HIGH){
                ENA.pulsewidth_us(MOTORPULSE);
                ENB.pulsewidth_us(MOTORPULSE);
                if(timeSaved > DISTANCIAMAX){ //si la distancia es menor a 20cm aprox
                    if(miTimer.read_ms()-timeServo > INTERVAL){ //se mueve el servo
                        if(myFlags.servoMoved==HIGH){   //Si es la parte positiva
                            angulo+=3;    
                            if(angulo>=ANGULOMAX)
                                myFlags.servoMoved=STOP;
                                //cambio el valor de la flag    
                            
                        }else{ //si ya llego a 90º cambio el valor de la flag 
                                angulo-=3;    
                            if(angulo<=ANGULOMIN)
                                myFlags.servoMoved=HIGH;
                                //cambio el valor de la flag           
                        }     
                        servoMove(angulo); 
                        if(angulo > STOP)
                            rotationRight=(angulo*TURNRIGHT) /90;
                        if(angulo < STOP)
                            rotationLeft=(angulo*TURNLEFT) /-90;
                        timeServo=miTimer.read_ms();
                    } 
                    counterM1=0;
                    counterM2=0;
                    myMotor1(STOP);    
                    myMotor2(STOP);
                }else{ //si la distancia esta dentro de la maxima 
                    if(angulo==STOP){    
                        counterM2=0;
                        counterM1=0;
                        if(timeSaved<=DISTANCIA+20 && timeSaved >=DISTANCIARANGO){
                            myMotor1(STOP);
                            myMotor2(STOP);
                        }else{
                        
                            if(timeSaved < DISTANCIA){
                                    myMotor1(BACKWARD);
                                    myMotor2(BACKWARD);
                            }else{         
                                    myMotor1(FOWARD);
                                    myMotor2(FOWARD);
                            }                    
                        }
                    }else{
                        if(angulo < STOP){
                            if(counterM2 <= rotationLeft && counterM1 <= rotationLeft){
                                myMotor1(BACKWARD);
                                myMotor2(FOWARD);    
                            }else{
                                counterM1=0;
                                counterM2=0;
                                myMotor1(STOP);
                                myMotor2(STOP);
                                angulo=STOP;
                                servoMove(angulo);
                            }
                        }
                        if(angulo > STOP){
                            if(counterM2 <= rotationRight && counterM1 <= rotationRight){
                                myMotor2(BACKWARD);
                                myMotor1(FOWARD);
                            }else{
                                counterM1=0;
                                counterM2=0;
                                myMotor1(STOP);
                                myMotor2(STOP);
                                angulo=STOP;
                                servoMove(angulo);
                            }
                        }

                    }
                }
            // }else{ //si el modo esta inactivo
            //     myMotor1(STOP);  
            //     myMotor2(STOP);
            //     //servoMove(STOP);
            // }
            break;
        case LINE:                    
           // if(myFlags.modeOn==HIGH){
                // if(valueIr1>valueIr0){           
                //     ENA.pulsewidth_us(MOTORPULSE-100);
                //     ENB.pulsewidth_us(MOTORPULSE);
                //     myMotor1(FOWARD);
                //     myMotor2(FOWARD);
                //     myFlags.GetLine=STOP;
                // }    
                // if(valueIr0 > valueIr1){       
                //     ENB.pulsewidth_us(MOTORPULSE-100);
                //     ENA.pulsewidth_us(MOTORPULSE);
                //     myMotor1(STOP);
                //     myMotor2(FOWARD);
                //     myFlags.GetLine=HIGH;
                // }
                // if(valueIr1 > 20000 && valueIr0 > 20000 ){
                //     myMotor1(FOWARD);
                //     myMotor2(FOWARD);
                // }
                // if(valueIr1 <1000 && valueIr0 < 1000){ //si se perdio la linea 
                //     if(myFlags.GetLine==HIGH){ //si hizo motor 1 STP y motor 2 FWD por ultimo 
                //         myMotor1(FOWARD);
                //         myMotor2(STOP);
                //     }else{ //si por utimo hizo motor2STP y motor 1 FWD
                //         myMotor1(STOP);
                //         myMotor2(FOWARD);
                //     }
                // }
                if(timeSaved < DISTANCIARANGO ){ //frena si detecta obstaculo
                    myMotor1(STOP);    
                    myMotor2(STOP);
                }else{
                    if(valueIr1 > 20000 && valueIr0 < 10000){
                        ENA.pulsewidth_us(MOTORPULSE-270);
                        ENB.pulsewidth_us(MOTORPULSE-105);
                        myMotor1(FOWARD);
                        myMotor2(FOWARD); 
                        myFlags.GetLine=HIGH;
                    }
                    if(valueIr0 > 20000 && valueIr1 < 10000){
                        ENA.pulsewidth_us(MOTORPULSE-105);
                        ENB.pulsewidth_us(MOTORPULSE-270);
                        myMotor1(FOWARD);
                        myMotor2(FOWARD); 
                        myFlags.GetLine=STOP;
                    }
                    if(valueIr1 < 2000 && valueIr0 < 2000){
                        if(myFlags.GetLine==HIGH){//si corrigio con --ENA
                            ENA.pulsewidth_us(MOTORPULSE+15); //-75
                            ENB.pulsewidth_us(MOTORPULSE+15); //-40
                            myMotor1(FOWARD);
                            myMotor2(BACKWARD);
                        }
                        else{//si corrigio con --ENB
                            ENA.pulsewidth_us(MOTORPULSE+15); //-75
                            ENB.pulsewidth_us(MOTORPULSE+15); //-40
                            myMotor1(BACKWARD);
                            myMotor2(FOWARD);
                        }
                    }
                
                
                }
            // }else{
            //     myMotor1(STOP);  
            //     myMotor2(STOP); 
            //     //servoMove(STOP);
            // }
            break;
        case ESCAPE:
                ENA.pulsewidth_us(MOTORPULSE-100);
                ENB.pulsewidth_us(MOTORPULSE-100);
                myMotor2(FOWARD);
                myMotor1(FOWARD);
            break;
        default:
            mode=IDLE;
            break;
        }
        if(datosComProtocol.indexReadRx!=datosComProtocol.indexWriteRx) 
            decodeProtocol();

        if(datosComProtocol.indexReadTx!=datosComProtocol.indexWriteTx) 
            sendData();
    
    } 
    return 0;
}
void startMef(){
   ourButton.estado=BUTTON_UP;
}
void actuallizaMef(){

    switch (ourButton.estado)
    {
    case BUTTON_DOWN:
        if(BUTTON.read())
           ourButton.estado=BUTTON_RISING;
    
    break;
    case BUTTON_UP:
        if(!BUTTON.read())
            ourButton.estado=BUTTON_FALLING;
    
    break;
    case BUTTON_FALLING:
        if(!(BUTTON.read()))
        {
            ourButton.timeDown=miTimer.read_ms();
            ourButton.estado=BUTTON_DOWN;
            //Flanco de bajada
        }
        else
            ourButton.estado=BUTTON_UP;    

    break;
    case BUTTON_RISING:
        if(BUTTON.read()){

            ourButton.estado=BUTTON_UP;
            //Flanco de Subida
            ourButton.timeDiff=miTimer.read_ms()-ourButton.timeDown;
            if(ourButton.timeDiff > 100 && ourButton.timeDiff < 1000){
                if(mode == IDLE){
                    myFlags.modeOn=STOP;//apaga la flag del inicio del modo 
                    mode=FOLLOW;  
                }else{
                    if(mode==FOLLOW){
                        myFlags.modeOn=STOP;
                        servoMove(STOP);
                        mode=LINE;
                    }else{
                        if(mode==LINE){
                            myFlags.modeOn=STOP;
                            mode=ESCAPE;
                        }else{
                            myFlags.idleFlag=STOP;
                            mode=IDLE;
                        }
                    }
                }            
            }
            if(ourButton.timeDiff>=1000 && ourButton.timeDiff<=2000){
                if(myFlags.modeOn==STOP)//si el modo esta off
                    myFlags.modeOn=HIGH;//lo inicio 
            }
            if(ourButton.timeDiff >=3000 && mode!=IDLE){
                myFlags.modeOn=STOP;
                myFlags.idleFlag=STOP;
                mode=IDLE;
            }

        }   
        else
            ourButton.estado=BUTTON_DOWN;
    
    break;
    
    default:
        startMef();
        break;
    
    }
}  
void heartBeatTask(){
    // if(mode==IDLE){
    //     if(miTimer.read_ms()-timeHb > DSTINTERVAL){
    //         LED!=LED;
    //         timeHb=miTimer.read_ms();
    //     }
    // }
    // if(mode==FOLLOW){
    //         if(myFlags.modeOn==STOP){
    //             if(miTimer.read_ms()-timeHb > IRINTERVAL){
    //                 LED!=LED;
    //                 timeHb=miTimer.read_ms();
    //             }else{
    //                 LED=STOP;
    //             }        
    //         }
    //}
    // if(mode==LINE){
    //         if(myFlags.modeOn=STOP){
                   
    //         }else{
                
    //         }
    // }
    // if(mode==ESCAPE){
    //         if(myFlags.modeOn=STOP){

    //         }else{
                
    //         }
    
    // }
}  
void miTrigger(){
    TRIGGER.write(0);
}
void StartEcho(){ //si se termino de mandar el trigger, el ECHO se pone en 1 (arranco un timer)
    miTimer1.reset();
}
void EndEcho(){ //si el ECHO se puso en 0 (paro el timer)
    timeSaved = miTimer1.read_us();
    datosComProtocol.payload[1]=GET_DISTANCE;
    decodeData();
} 
void servoMove(int8_t angulo){
    if(angulo ==ANGULOMIN)
        ANCHO=PULSEMIN;
    if(angulo > ANGULOMIN && angulo < STOP){//si el angulo es positivo
        ANCHO= PULSEMIN +(factor*(angulo+90));
    }
    if(angulo==STOP)
        ANCHO=(PULSEMAX+PULSEMIN)/2;
    if(angulo > STOP && angulo < ANGULOMAX){
        ANCHO=(angulo * factor) + ((PULSEMAX+PULSEMIN)/2);
    }
    if(angulo==ANGULOMAX)
        ANCHO=PULSEMAX;
    //  if(myFlags.servoMoved){   
    //     ANCHO+=45;    
    //      if(ANCHO>=PULSEMAX)
    //          myFlags.servoMoved=STOP;
    //          //cambio el valor de la flag    
         
    //  }else{
    //         ANCHO-=45;    
    //      if(ANCHO<=PULSEMIN)
    //          myFlags.servoMoved=HIGH;
    //          //cambio el valor de la flag           
    //  }    
     SERVO.pulsewidth_us(ANCHO);
       
} 
void myMotor1(uint8_t direction){
    if(direction==FOWARD){
        IN3=HIGH;
        IN4=STOP;
    }
    if(direction==STOP){
        IN3=STOP;
        IN4=STOP;
    }
    if(direction==BACKWARD){
        IN3=STOP;
        IN4=HIGH; 

    }
}
void myMotor2(uint8_t direction){
    if(direction==FOWARD){
        IN1=HIGH;
        IN2=STOP;
    }
    if(direction==STOP){
        IN1=STOP;
        IN2=STOP;
    }
    if(direction==BACKWARD){
        IN1=STOP;
        IN2=HIGH;
    }
}
void onDataRx(void){
    while (pcCom.readable())
    {
        datosComProtocol.bufferRx[datosComProtocol.indexWriteRx++]=pcCom.getc();
    }
}
void sendData(void){
    if(pcCom.writable())
        pcCom.putc(datosComProtocol.bufferTx[datosComProtocol.indexReadTx++]);

}
void decodeProtocol(void)
{
    static int8_t nBytes=0, indice=0;
    while (datosComProtocol.indexReadRx!=datosComProtocol.indexWriteRx)
    {
        switch (estadoProtocolo) {
            case START:
                if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]=='U'){
                    estadoProtocolo=HEADER_1;
                    datosComProtocol.cheksumRx=0;
                }
                break;
            case HEADER_1:
                if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]=='N')
                   estadoProtocolo=HEADER_2;
                else{
                    datosComProtocol.indexReadRx--;
                    estadoProtocolo=START;
                }
                break;
            case HEADER_2:
                if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]=='E')
                    estadoProtocolo=HEADER_3;
                else{
                    datosComProtocol.indexReadRx--;
                   estadoProtocolo=START;
                }
                break;
        case HEADER_3:
            if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]=='R')
                estadoProtocolo=NBYTES;
            else{
                datosComProtocol.indexReadRx--;
               estadoProtocolo=START;
            }
            break;
            case NBYTES:
                nBytes=datosComProtocol.bufferRx[datosComProtocol.indexReadRx++];
               estadoProtocolo=TOKEN;
                break;
            case TOKEN:
                if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]==':'){
                   estadoProtocolo=PAYLOAD;
                    datosComProtocol.cheksumRx ='U'^'N'^'E'^'R'^ nBytes^':';
                    datosComProtocol.payload[0]=nBytes;
                    indice=1;
                }
                else{
                    datosComProtocol.indexReadRx--;
                    estadoProtocolo=START;
                }
                break;
            case PAYLOAD:
                if (nBytes>1){
                    datosComProtocol.payload[indice++]=datosComProtocol.bufferRx[datosComProtocol.indexReadRx];
                    datosComProtocol.cheksumRx ^= datosComProtocol.bufferRx[datosComProtocol.indexReadRx++];
                }
                
                nBytes--;
                
                if(nBytes<=0){
                    estadoProtocolo=START;
                    if(datosComProtocol.cheksumRx == datosComProtocol.bufferRx[datosComProtocol.indexReadRx]){
                        decodeData();
                    }
                }
                break;
            default:
                estadoProtocolo=START;
                break;
        }
    }
    

}
void readIr(){
    valueIr0=IR0.read_u16();    
    valueIr1=IR1.read_u16();
    datosComProtocol.payload[1]=GET_IR;
    decodeData();
}
/*****************************************************************************************************/
/************  Función para procesar el comando recibido ***********************/
void decodeData(void)
{
    uint8_t auxBuffTx[50], indiceAux=0, cheksum;
    auxBuffTx[indiceAux++]='U';
    auxBuffTx[indiceAux++]='N';
    auxBuffTx[indiceAux++]='E';
    auxBuffTx[indiceAux++]='R';
    auxBuffTx[indiceAux++]=0;
    auxBuffTx[indiceAux++]=':';

    switch (datosComProtocol.payload[1]) {
        case ALIVE:
            auxBuffTx[indiceAux++]=ALIVE;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x03;
            break;
        case GET_IR:
            auxBuffTx[indiceAux++]=GET_IR;
            
            myWord.ui16[0]=IR0.read_u16();//lecturaSensorIRLeft
            myWord.ui16[1]=IR1.read_u16();//lecturaSensorIRright
            
            auxBuffTx[indiceAux++]=myWord.ui8[0];
            auxBuffTx[indiceAux++]=myWord.ui8[1];
            auxBuffTx[indiceAux++]=myWord.ui8[2];
            auxBuffTx[indiceAux++]=myWord.ui8[3];
            auxBuffTx[NBYTES]=0x06;
        break;
        case GET_DISTANCE:
            auxBuffTx[indiceAux++]=GET_DISTANCE;
            myWord.ui32=timeSaved;
            auxBuffTx[indiceAux++]=myWord.ui8[0];
            auxBuffTx[indiceAux++]=myWord.ui8[1];
            auxBuffTx[indiceAux++]=myWord.ui8[2];
            auxBuffTx[indiceAux++]=myWord.ui8[3];
            auxBuffTx[NBYTES]=0x06;
        break;
        case GET_SPEED:
            auxBuffTx[indiceAux++]=GET_SPEED;
            myWord.ui32=speedM1; //Horquilla L 
            auxBuffTx[indiceAux++]=myWord.ui8[0];
            auxBuffTx[indiceAux++]=myWord.ui8[1];
            auxBuffTx[indiceAux++]=myWord.ui8[2];
            auxBuffTx[indiceAux++]=myWord.ui8[3];
            
            myWord.ui32=speedM2; //horquilla R
            auxBuffTx[indiceAux++]=myWord.ui8[0];
            auxBuffTx[indiceAux++]=myWord.ui8[1];
            auxBuffTx[indiceAux++]=myWord.ui8[2];
            auxBuffTx[indiceAux++]=myWord.ui8[3];
            auxBuffTx[NBYTES]=0x0A;
        break;
        case MOTOR_T:
                myNewWord.i8[0]=datosComProtocol.payload[2];
                myNewWord.i8[1]=datosComProtocol.payload[3];
                myNewWord.i8[2]=datosComProtocol.payload[4];
                myNewWord.i8[3]=datosComProtocol.payload[5];
                testM1=myNewWord.i32;

                myNewWord.i8[0]=datosComProtocol.payload[6];
                myNewWord.i8[1]=datosComProtocol.payload[7];
                myNewWord.i8[2]=datosComProtocol.payload[8];
                myNewWord.i8[3]=datosComProtocol.payload[9];
                testM2=myNewWord.i32;
                if(mode==IDLE){    
                    if(testM1 < 0){
                        testM1=testM1*-1;
                        ENA.pulsewidth_us(testM1);
                        myMotor1(BACKWARD);
                    }else{
                        ENA.pulsewidth_us(testM1); 
                        myMotor1(FOWARD); 
                    }    
                    
                    if(testM2 < 0){
                        testM2=testM2*-1;
                        ENB.pulsewidth_us(testM2);
                        myMotor2(BACKWARD);
                    }else{
                        ENB.pulsewidth_us(testM2); 
                        myMotor2(FOWARD); 
                    }        
                }
                auxBuffTx[indiceAux++]=MOTOR_T;
                auxBuffTx[indiceAux++]=0x0D;//manda ack de motor
                auxBuffTx[NBYTES]=0x03;
        break;
        case SERVO_T:
            
            if(mode==IDLE){
                angulo=datosComProtocol.payload[2];  
                servoMove(angulo);
            }
            auxBuffTx[indiceAux++]=SERVO_T;       
            auxBuffTx[indiceAux++]=0x0D;              
            auxBuffTx[NBYTES]=0x03;
        break;
        default:
            auxBuffTx[indiceAux++]=0xDD;
            auxBuffTx[NBYTES]=0x02;
            break;
    }
   cheksum=0;
   for(uint8_t a=0 ;a < indiceAux ;a++)
   {
       cheksum ^= auxBuffTx[a];
       datosComProtocol.bufferTx[datosComProtocol.indexWriteTx++]=auxBuffTx[a];
   }
    datosComProtocol.bufferTx[datosComProtocol.indexWriteTx++]=cheksum;

}
void readSpeedHl(){
    counterLeft++;
    counterM2++;
}
void readSpeedHr(){
    counterRight++;
    counterM1++;
}
void readSensors(){
    speedM1=counterRight;
    speedM2=counterLeft;
    datosComProtocol.payload[1]=GET_SPEED;
    decodeData();
    counterRight=0;
    counterLeft=0;
}