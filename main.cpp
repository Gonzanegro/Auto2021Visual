#include "mbed.h"
#define INTERVAL 50
#define CHKINTERVAL 150
#define PULSE 10
#define HIGH 1
#define STOP 0
#define PULSEMIN 500
#define PULSEMAX 2300 
#define FOWARD 1
#define BACKWARD 2
#define MOTORPULSE 5
#define DISTANCIA 348 //6cm
#define DISTANCIAMAX 1800//20cm aprox
typedef union{
            unsigned char servoMoved: 1;
            unsigned char b1: 1;
            unsigned char b2: 1;
            unsigned char b3: 1;
            unsigned char b4: 1;
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
typedef enum{
    IDLE,
    FOLLOW,
    LINE,
    ESCAPE,
}_emode;
_emode mode;
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
void servoMove();
void actuallizaMef();
void startMef();
void myMotors(uint8_t right,uint8_t left);
void onDataRx();
void decodeProtocol();
void decodeData();
void sendData();

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

Timer miTimer; 
Timer miTimer1;
Timeout timeout;
uint32_t timeHb=0,timeSaved=0,timeToChk=0,timeServo=0;
uint32_t ANCHO=1000;
uint8_t myFlag=0;

int main(){
    
    ENA.pulsewidth_ms(MOTORPULSE);
    ENB.pulsewidth_ms(MOTORPULSE);
    myMotors(FOWARD,FOWARD);
    
    wait_ms(500);
    
    myMotors(BACKWARD,BACKWARD);
    
    wait_ms(500);
   
    myMotors(STOP,STOP);    
    SERVO.pulsewidth_us(PULSEMIN);
    wait_ms(400);
    SERVO.pulsewidth_us(PULSEMAX);
    wait_ms(400);
    SERVO.pulsewidth_us((PULSEMIN+PULSEMAX)/2);
    ANCHO=(PULSEMIN+PULSEMAX)/2;
    startMef();
    miTimer.start();
    miTimer1.start();
    pcCom.attach(&onDataRx,Serial::RxIrq);//
    ECHO.rise(&StartEcho);
    ECHO.fall(&EndEcho);
    mode=IDLE;
    while(1){
        if(miTimer.read_ms()-timeToChk > CHKINTERVAL){
                TRIGGER.write(HIGH);
                timeout.attach_us(&miTrigger,PULSE);
                LED=!LED;
                timeToChk=miTimer.read_ms();
            }
        if(miTimer.read_ms()-timeHb > INTERVAL){
            actuallizaMef();
            timeHb=miTimer.read_ms();
        }
        switch (mode)
        {
        case IDLE:
            
            break;
        case FOLLOW:
            if(timeSaved > DISTANCIAMAX){
                if(miTimer.read_ms()-timeServo > INTERVAL){
                    servoMove();
                    timeServo=miTimer.read_ms();
                } 
                myMotors(STOP,STOP);
            }else{ //si la distancia esta dentro de la maxima 
                if(timeSaved < DISTANCIA){
                        myMotors(BACKWARD,BACKWARD);
                }else{         
                        myMotors(FOWARD,FOWARD);  
                }                    
            }
            break;
        case LINE:
                 myMotors(STOP,STOP); 
            break;
        case ESCAPE:
                myMotors(STOP,STOP); 
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
            if(mode == IDLE){
                mode=FOLLOW;  
            }else{
                if(mode==FOLLOW){
                    mode=LINE;
                }else{
                    if(mode==LINE){
                        mode=ESCAPE;
                    }else{
                        mode=IDLE;
                    }
                }
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
void miTrigger(){
    TRIGGER.write(0);
}
void StartEcho(){ //si se termino de mandar el trigger, el ECHO se pone en 1 (arranco un timer)
    miTimer1.reset();
}
void EndEcho(){ //si el ECHO se puso en 0 (paro el timer)
    timeSaved = miTimer1.read_us();
} 
void servoMove(){
     if(myFlags.servoMoved){   
        ANCHO+=45;    
         if(ANCHO>=PULSEMAX)
             myFlags.servoMoved=STOP;
             //cambio el valor de la flag    
         
     }else{
            ANCHO-=45;    
         if(ANCHO<=PULSEMIN)
             myFlags.servoMoved=HIGH;
             //cambio el valor de la flag           
     }    
    SERVO.pulsewidth_us(ANCHO);
       
} 
void myMotors(uint8_t right,uint8_t left){
    if(right==FOWARD){
        IN3=HIGH;
        IN4=STOP;
    }
    if(right==STOP){
        IN3=STOP;
        IN4=STOP;
    }
    if(right==BACKWARD){
        IN3=STOP;
        IN4=HIGH; 

    }
    if(left==FOWARD){
        IN1=HIGH;
        IN2=STOP;
    }
    if(left==STOP){
        IN1=STOP;
        IN2=STOP;
    }
    if(left==BACKWARD){
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
        case GET_DISTANCE:
            auxBuffTx[indiceAux++]=GET_DISTANCE;
            myWord.ui32=timeSaved;
            auxBuffTx[indiceAux++]=myWord.ui8[0];
            auxBuffTx[indiceAux++]=myWord.ui8[1];
            auxBuffTx[indiceAux++]=myWord.ui8[2];
            auxBuffTx[indiceAux++]=myWord.ui8[3];
            auxBuffTx[NBYTES]=0x06;
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

