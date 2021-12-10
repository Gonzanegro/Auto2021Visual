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
#define PAUSE 3
#define MOTORPULSE 500
#define DISTANCIA 348 //6cm
#define DISTANCIARANGO 250//4.3 cm 
#define DISTANCIAMAX 1700//29cm aprox
#define DISTANCIAEND 2900 //50cm 
#define ANGULOMAX 90
#define ANGULOMIN -90
#define TURNLEFT 45//pulsos de horuqilla para -90
#define TURNRIGHT 45 //pulsos de horquilla para 90
#define PULSETURN 35
typedef union{
                unsigned char modeOn: 1;
                unsigned char servoMoved: 1;
                unsigned char idleFlag: 1;
                unsigned char GetLine: 1;
                unsigned char readyPosition: 1;
                unsigned char turn: 1;
                unsigned char choose: 1;
                unsigned char bifurcacion: 1;
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
    MOVERIGHT,
    MOVELEFT,
    AVOIDING,
    STOPPED,
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
void followLine();
void lookAround(int direccion);

DigitalOut LED(PC_13);
DigitalOut LEDAUX(PB_11);

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
bool wasHere=false,choice=false,bifurcacion=false;
uint8_t factor,direccion=0,rotationLeft=0,rotationRight=0,counterLine=0;
uint32_t timeHb=0,timeSaved=0,timeToChk=0,timeServo=0,timeIr=0,timePos=0,speedM1=0,speedM2=0,timeTurn=0,timeContinue=0;
uint16_t ANCHO=1000,valueIr0,valueIr1,distanceFoward,distanceLeft,distanceRight,counterRight=0,counterLeft=0;
volatile uint16_t counterM1=0,counterM2=0;
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
    LEDAUX=HIGH;
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
    LEDAUX=STOP;
    srand(miTimer.read_ms());
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
                switch(modeState){
                    case SEARCHING:
                        if(timeSaved < DISTANCIARANGO + DISTANCIA+58 ){ //frena si detecta obstaculo
                            myMotor1(STOP);    
                            myMotor2(STOP);
                            servoMove(ANGULOMAX);
                            ENA.pulsewidth_us(MOTORPULSE+100);
                            ENB.pulsewidth_us(MOTORPULSE+100);
                            counterM1=0;
                            counterM2=0;
                            counterLine=0;
                            myFlags.turn=HIGH;
                            modeState=MOVERIGHT;
                        }else{
                            followLine();
                        }
                    break;
                    case MOVERIGHT:
                        if(counterM1 <= 55 && counterM2 <= 55){
                            myMotor1(BACKWARD);
                            myMotor2(FOWARD);
                        }else{
                            ENA.pulsewidth_us(MOTORPULSE);
                            ENB.pulsewidth_us(MOTORPULSE);
                            myMotor2(STOP);
                            myMotor1(STOP);
                            if(myFlags.turn==HIGH){
                                modeState=AVOIDING;
                            }else{
                                modeState=SEARCHING;
                            }
                            counterM2=0;
                            counterM1=0;
                        }
                    break;    
                    case MOVELEFT:                        
                        if(timeSaved > DISTANCIAMAX){
                            ENA.pulsewidth_us(MOTORPULSE-300);
                            ENB.pulsewidth_us(MOTORPULSE);
                            myMotor1(FOWARD);
                            myMotor2(FOWARD);
                        }else{
                            myMotor2(STOP);
                            myMotor1(STOP);
                            ENA.pulsewidth_us(MOTORPULSE);
                            ENB.pulsewidth_us(MOTORPULSE);                            
                            modeState=AVOIDING;
                            //counterM2=0;
                            //counterM1=0;
                        }
                    break;
                    case AVOIDING:
                        if(valueIr1 > 10000 || valueIr0 > 10000){
                            myMotor1(STOP);   
                            myMotor2(STOP);
                            servoMove(STOP);
                            modeState=STOPPED;
                            //modeState=MOVERIGHT;
                        }
                        if(myFlags.turn==HIGH){
                            if(timeSaved >=580 && timeSaved <=870){
                                timeTurn=miTimer.read_ms();
                                
                                ENA.pulsewidth_us(MOTORPULSE-50);
                                ENB.pulsewidth_us(MOTORPULSE-50);
                                myMotor1(FOWARD);
                                myMotor2(FOWARD);
                            
                            }
                            if(timeSaved < 580){
                                    timeTurn=miTimer.read_ms();
                                    ENA.pulsewidth_us(MOTORPULSE-80);
                                    ENB.pulsewidth_us(MOTORPULSE-220);
                                    myMotor1(FOWARD);
                                    myMotor2(FOWARD);  
                            }  
                            if(timeSaved >870 && timeSaved < DISTANCIAMAX ){
                                    
                                    timeTurn=miTimer.read_ms();
                                    
                                    ENA.pulsewidth_us(MOTORPULSE-220);
                                    ENB.pulsewidth_us(MOTORPULSE-80);
                                    myMotor1(FOWARD);
                                    myMotor2(FOWARD); 
                            }                 
                            if(timeSaved > DISTANCIAMAX){
                                myMotor2(STOP);
                                myMotor1(STOP);
                                if(timeTurn - miTimer.read_ms() > 200){
                                    counterM2=0;
                                    counterM1=0;
                                    modeState=MOVELEFT;
                                    timeTurn=miTimer.read_ms();
                                }
                            }
                        }
                    break;
                    case STOPPED:
                        myMotor1(STOP);
                        myMotor2(STOP);
                        if(counterLine == 3){
                            myFlags.turn=STOP;
                            modeState=MOVERIGHT;
                        }else{
                            modeState=AVOIDING;
                            counterLine++;
                            myFlags.turn=HIGH;
                        }
                    break;
                }
            break;
        case ESCAPE:
                switch(modeState){
                        case SEARCHING:
                                    if(timeSaved < DISTANCIAMAX || (valueIr0 >20000 && valueIr1 >20000)){  //si encuentra objeto o bifurcacion 
                                        myMotor2(STOP);
                                        myMotor1(STOP);
                                        direccion=STOP;
                                        modeState=STOPPED;
                                    }else{
                                        followLine();
                                    }
                        break;
                        case MOVELEFT:
                            if(counterM1 <= PULSETURN-5  && counterM2 <=PULSETURN-5 ){ //pulsos fijos para doblar un angulo
                                ENA.pulsewidth_us(MOTORPULSE-10);
                                ENB.pulsewidth_us(MOTORPULSE-10);
                                myMotor1(FOWARD);
                                myMotor2(BACKWARD);
                            }else{ //ya puede buscar linea 
                                if(valueIr0 > 5000 || valueIr1 > 5000){ //si encuentra la linea
                                    ENA.pulsewidth_us(MOTORPULSE);
                                    ENB.pulsewidth_us(MOTORPULSE);
                                    if(wasHere==false){
                                        myMotor2(PAUSE);
                                        myMotor1(PAUSE);
                                        counterM1=PULSETURN+1;
                                        counterM2=PULSETURN+1;
                                        timeContinue=miTimer.read_ms();
                                        wasHere=true;
                                    }else{
                                        if(bifurcacion==false){
                                            if((miTimer.read_ms()-timeContinue) > 1200){
                                                LEDAUX=STOP;
                                                myMotor1(PAUSE);  
                                                myMotor2(PAUSE);
                                                modeState=SEARCHING;
                                             }else{
                                                 followLine();   
                                             }        
                                        }else{
                                            if((miTimer.read_ms()-timeContinue) > 1800){
                                                LEDAUX=STOP;
                                                myMotor1(PAUSE);  
                                                myMotor2(PAUSE);
                                                modeState=SEARCHING;
                                             }else{
                                                 LEDAUX=HIGH;
                                                 followLine();   
                                             }         
                                                                                  
                                        }
                                    
                                    }
                                }else{
                                    if(wasHere==false){
                                        ENA.pulsewidth_us(MOTORPULSE-200);
                                        ENB.pulsewidth_us(MOTORPULSE-200);
                                        myMotor1(FOWARD);
                                        myMotor2(FOWARD);
                                    }else{
                                        followLine();
                                    }
                                }
                            }                        
                        break;
                        case MOVERIGHT: //estado para doblar a la derecha 
                            if(counterM1 <= PULSETURN && counterM2 <=PULSETURN ){
                                ENA.pulsewidth_us(MOTORPULSE-10);
                                ENB.pulsewidth_us(MOTORPULSE-10);
                                myMotor1(BACKWARD);
                                myMotor2(FOWARD);
                                
                            }else{  
                                if(valueIr0 > 5000 || valueIr1 > 5000){ //si encuentra linea 
                                    if(wasHere==false){
                                        ENA.pulsewidth_us(MOTORPULSE);
                                        ENB.pulsewidth_us(MOTORPULSE);
                                        myMotor2(PAUSE);
                                        myMotor1(PAUSE);
                                        counterM1=PULSETURN+1;
                                        counterM2=PULSETURN+1;
                                        timeContinue=miTimer.read_ms();
                                        wasHere=true;
                                    }else{
                                        if(bifurcacion==false){  
                                             if((miTimer.read_ms()-timeContinue) > 1500){
                                                LEDAUX=STOP;
                                                myMotor1(PAUSE);  
                                                myMotor2(PAUSE);
                                                modeState=SEARCHING;
                                             }else{
                                                 followLine();   
                                             }
                                        }else{ //si hay bifucarcion
                                            if((miTimer.read_ms()-timeContinue) > 3000){
                                                LEDAUX=STOP;
                                                myMotor1(PAUSE);  
                                                myMotor2(PAUSE);
                                                modeState=SEARCHING;
                                            }else{
                                                LEDAUX=HIGH;
                                                followLine();   
                                            }
                                        
                                        }
                                    }
                                }else{ //buscando linea 
                                        ENA.pulsewidth_us(MOTORPULSE-200);
                                        ENB.pulsewidth_us(MOTORPULSE-200);
                                        myMotor1(FOWARD);
                                        myMotor2(FOWARD);
                                        if(wasHere==true){
                                            followLine();
                                        }else{
                                            ENA.pulsewidth_us(MOTORPULSE-200);
                                            ENB.pulsewidth_us(MOTORPULSE-200);
                                            myMotor1(FOWARD);
                                            myMotor2(FOWARD);  
                                        }
                                }
                            }
                        break;
                        case STOPPED:                      
                            if(direccion < 3){ //analiza el entorno 
                                if((miTimer.read_ms()-timePos) > 1000){
                                    lookAround(direccion);
                                    direccion++;
                                    timePos=miTimer.read_ms();
                                    }
                            }else{ //si ya analizo el entorno 
                               if(distanceFoward < DISTANCIAMAX ){ //Si hay obstaculo delante 
                                   if(distanceLeft >DISTANCIAMAX && distanceRight > DISTANCIAMAX){ //si tiene que elegir un lado para doblar 
                                        bifurcacion=true;
                                        choice=!choice;
                                        if(choice==true){
                                            counterM1=0;
                                            counterM2=0;        
                                            wasHere=false;
                                            modeState=MOVELEFT; //dobla a la izquierda
                                        }else{
                                            counterM1=0;
                                            counterM2=0;        
                                            wasHere=false;
                                            modeState=MOVERIGHT; //dobla a la derecha 
                                        } 
                                   }else{   //si no hay que elegir y debe doblar si o si a algun lado   
                                        bifurcacion=false;
                                        if(distanceLeft > distanceRight){ //revisa a que lado doblar 
                                            counterM1=0;
                                            counterM2=0;
                                            wasHere=false;
                                            modeState=MOVELEFT;
                                        }else{
                                            counterM1=0;
                                            counterM2=0;
                                            wasHere=false;
                                            modeState=MOVERIGHT;
                                        }
                                   }
                               }else{ //si delante no hay obstaculo (bifurcacion)
                                    if(distanceLeft > DISTANCIAEND && distanceRight > DISTANCIAEND && distanceFoward > DISTANCIAEND){ //si gano 
                                        
                                    }else{ //si no gano 
                                        //myFlags.choose=(rand()%2)+1;    
                                        choice=!choice;// sentido aleatorio 
                                        bifurcacion=true;
                                        if(choice==true){ // si decide doblar mira hacia que lado  
                                            counterM1=0;
                                            counterM2=0;        
                                            wasHere=false;
                                            if(distanceLeft > DISTANCIAMAX){
                                                modeState=MOVELEFT;
                                            }else{
                                                modeState=MOVERIGHT;
                                            }
                                        }else{ //si decide seguir 
                                            counterM1=0;
                                            counterM2=0;        
                                            modeState=AVOIDING;
                                        } 
                                    } 
                               }     
                            }
                        break;
                        case AVOIDING: //si decide seguir adelante en la bifurcacion
                                if(counterM1 <= 45 && counterM2 <= 45){ //saltea la primer bifucacion
                                    ENA.pulsewidth_us(MOTORPULSE-80);
                                    ENB.pulsewidth_us(MOTORPULSE-80);
                                    myMotor1(FOWARD);
                                    myMotor2(FOWARD);
                                }
                                else{
                                    if(counterM1 <= 110 && counterM2 <= 110){ //saltea la segunda bifurcacion
                                        followLine();
                                    }else{
                                        myMotor1(PAUSE);
                                        myMotor2(PAUSE);
                                        if(timeContinue-miTimer.read_ms() > DSTINTERVAL){
                                        modeState=SEARCHING;
                                        timeContinue=miTimer.read_ms();
                                    }    
                                    }                                  
                                }
                        break;
                }
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
                        modeState=SEARCHING;
                        counterM1=0;
                        counterM2=0;
                        mode=LINE;
                    }else{
                        if(mode==LINE){
                            myFlags.modeOn=STOP;
                            modeState=SEARCHING;
                            servoMove(STOP);
                            mode=ESCAPE;
                        }else{
                            myFlags.idleFlag=STOP;
                            LEDAUX=STOP;
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
    if(direction == PAUSE){
        IN3=HIGH;
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
    if(direction == PAUSE){
        IN1=HIGH;
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
void followLine(){
    if(valueIr1 > 20000 && valueIr0 < 10000){
        ENA.pulsewidth_us(MOTORPULSE-260);
        ENB.pulsewidth_us(MOTORPULSE-105);
        myMotor1(FOWARD);
        myMotor2(FOWARD); 
        myFlags.GetLine=HIGH;
    }
    if(valueIr0 > 20000 && valueIr1 < 10000){
        ENA.pulsewidth_us(MOTORPULSE-105);
        ENB.pulsewidth_us(MOTORPULSE-260);
        myMotor1(FOWARD);
        myMotor2(FOWARD); 
        myFlags.GetLine=STOP;
    }
    if(valueIr1 < 2000 && valueIr0 < 2000){
        if(myFlags.GetLine==HIGH){//si corrigio con --ENA
            ENA.pulsewidth_us(MOTORPULSE-95); //-75
            ENB.pulsewidth_us(MOTORPULSE-95); //-40
            myMotor1(FOWARD);
            myMotor2(BACKWARD);
        }
        else{//si corrigio con --ENB
            ENA.pulsewidth_us(MOTORPULSE-80); //-75
            ENB.pulsewidth_us(MOTORPULSE-80); //-40
            myMotor1(BACKWARD);
            myMotor2(FOWARD);
        }
    }
}
void lookAround(int direccion){
    switch (direccion)
    {
    case 0:
        distanceFoward=timeSaved;
        servoMove(ANGULOMIN);
        break;
    case 1:
        distanceRight=timeSaved;
        servoMove(ANGULOMAX);
    break;
    case 2:
        distanceLeft=timeSaved;
        servoMove(STOP);
    break;
    default:
        break;
    }

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