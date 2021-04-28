#include <QCoreApplication>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>
#include <qmath.h>

#include <unistd.h>

using namespace std;

//Ações atômicas

float getPlayerRotateAngleTo(float px, float py, SSL_DetectionRobot robot) {
    float rx = robot.x();
    float ry = robot.y();
    float componentX = (px - rx);
    float componentY = (py - ry);
    float distToTarget = sqrt(pow(componentX, 2) + pow(componentY, 2));

    componentX = componentX / distToTarget;

    // Check possible divisions for 0
    if(isnanf(componentX)) {
        return 0.0f;
    }

    float angleOriginToTarget; // Angle from field origin to targetPosition
    float angleRobotToTarget;  // Angle from robot to targetPosition

    if(componentY < 0.0f) {
        angleOriginToTarget = 2*M_PI - acos(componentX); // Angle that the target make with x-axis to robot
    } else {
        angleOriginToTarget = acos(componentX); // Angle that the target make with x-axis to robot
    }

    angleRobotToTarget = angleOriginToTarget - robot.orientation();

    // Adjusting to rotate the minimum possible
    if(angleRobotToTarget > M_PI) angleRobotToTarget -= 2.0 * M_PI;
    if(angleRobotToTarget < -M_PI) angleRobotToTarget += 2.0 * M_PI;

    return angleRobotToTarget;
}

void rotateTo(Vision *vision, Actuator *actuator, bool isYellow, int chosenID, bool pointIsBall, float px, float py, bool spinner) {
    vision->processNetworkDatagrams();

    if (pointIsBall){
        SSL_DetectionBall ball = vision->getLastBallDetection();
        px = ball.x();
        py = ball.y();
    }

    SSL_DetectionRobot robot = vision->getLastRobotDetection(isYellow, chosenID);

    float angleRobotToObjective = getPlayerRotateAngleTo(px, py, robot);
    float ori = robot.orientation();

    if(ori > M_PI) ori -= 2.0 * M_PI;
    if(ori < -M_PI) ori += 2.0 * M_PI;

    float angleRobotToTarget = ori + angleRobotToObjective;

    float vw = 1;

    float angleError = 0.05; //Aproximadamente 3 graus;

    actuator->sendCommand(isYellow, chosenID, 0, 0, vw, spinner, 0, false);

    while(ori != angleRobotToTarget){
        vision->processNetworkDatagrams();
        robot = vision->getLastRobotDetection(isYellow, chosenID);
        ori = robot.orientation();
    }

    actuator->sendCommand(isYellow, chosenID, 0, 0, 0, spinner, 0, false);
}

void goTo (Vision *vision, Actuator *actuator, bool isYellow, int chosenID, bool pointIsBall, float px, float py, bool spinner) {
    vision->processNetworkDatagrams();
    SSL_DetectionRobot robot = vision->getLastRobotDetection(isYellow, chosenID);

    if (pointIsBall){
        SSL_DetectionBall ball = vision->getLastBallDetection();
        px = ball.x();
        py = ball.y();
    }

    float dx = (px - robot.x());
    float dy = (py - robot.y());

    float orientation = robot.orientation();
    float vx = (dx * cos(orientation) + dy * sin(orientation));
    float vy = (dy * cos(orientation) - dx * sin(orientation));

    actuator->sendCommand(isYellow, chosenID, vx*100, vy*100, 0, spinner, 0, false);

    std::cout << "dx e dy: " << dx << " | " << dy << std::endl;

    while (fabs(dx) > 50 || fabs(dy) > 50){
        vision->processNetworkDatagrams();
        robot = vision->getLastRobotDetection(isYellow, chosenID);

        dx = (px - robot.x());
        dy = (py - robot.y());

    }

    actuator->sendCommand(isYellow, chosenID, 0, 0, 0, spinner, 0, false);
    actuator->sendCommand(isYellow, chosenID, 0, 0, 0, spinner, 0, false);
    actuator->sendCommand(isYellow, chosenID, 0, 0, 0, spinner, 0, false);
}

void facePoint (Vision *vision, Actuator *actuator, bool isYellow, int chosenID, bool pointIsBall, float px, float py, bool spinner){
    vision->processNetworkDatagrams();
    //pegando instância atual do robô selecionado
    SSL_DetectionRobot robot = vision->getLastRobotDetection(isYellow, chosenID);
    //pegando instância atual da posição da bola
    SSL_DetectionBall ball = vision->getLastBallDetection();

    //Se true, ele vira para bola. Falso, ele vira para o gol.
    if (pointIsBall){
        px = ball.x();
        py = ball.y();
    }

    //Angulo necessário para o robô apontar
    float angle = getPlayerRotateAngleTo(px, py, robot);

    //O robô parace sempre "fugir" da ponto, então temos que adcionar 180º ao angulo desejado
    //No caso de radianos, 180º == pi
    angle = angle + M_PI;

    //a orientation varia entre -pi e +pi
    if (angle > (M_PI)){
        float resto = angle - M_PI;
        angle = -M_PI + resto;
    }

    //Orientação atual do robô
    float orientation = robot.orientation();

    float vw = 0.5; //velocidade angular

    if (angle - orientation < 0){
        vw = vw * -1;
    }

    //Faz o robô girar
    if (orientation > angle){
        actuator->sendCommand(isYellow, chosenID, 0, 0, -vw, spinner, 0, false);
    } else{
        actuator->sendCommand(isYellow, chosenID, 0, 0, vw, spinner, 0, false);
    }

    float angleError = 0.05; //Aproximadamente 3 graus;
    float faixaInf = angle-angleError;
    float faixaSup = angle+angleError;


    //Robô roda enquanto a orientação dele não estiver perto o bastante da variavel angle
    while((orientation < faixaInf) || (orientation > faixaSup)){
        vision->processNetworkDatagrams();
        robot = vision->getLastRobotDetection(isYellow, chosenID);
        orientation = robot.orientation();
        std::cout << "Angulo necessário: " << angle << std::endl;
        std::cout << "Orientação do robô: " << orientation << std::endl;
    }

    //para de rodar
    actuator->sendCommand(isYellow, chosenID, 0, 0, 0, true, 0, false);
}

void pickBall (Vision *vision, Actuator *actuator, bool isYellow, int chosenID){
    float d = 50000;
    do {
        vision->processNetworkDatagrams();
        //pegando instância atual do robô selecionado
        SSL_DetectionRobot robot = vision->getLastRobotDetection(isYellow, chosenID);
        //pegando instância atual da posição da bola
        SSL_DetectionBall ball = vision->getLastBallDetection();

        float bx = ball.x();
        float by = ball.y();
        float rx = robot.x();
        float ry = robot.y();

         //Calcula distância d entre a bola e o robô
         float c1 = rx - bx;
         float c2 = ry - by;
         c1 = qPow(c1, 2);
         c2 = qPow(c2, 2);
         d = qSqrt(c1 + c2);
         std::cout << "Distância para a bola: " << d << std::endl;

         int divisor;

         if (d <= 200){
             divisor = 5000;
         } else {
             divisor = 1000;
         }

         //calcula velocidade v necessária para chegar na bola
         float v = d/divisor;

         //Aciona o robô para ir em frente, com o spinner ligado para segurar a bola
         actuator->sendCommand(isYellow, chosenID, v, 0, 0, true, 0, false);
         usleep(100000);//sleeps for 1 second
         actuator->sendCommand(isYellow, chosenID, 0, 0, 0, true, 0, false);
     }
     while (d > 110);
}

void kickBall (Actuator *actuator, bool isYellow, int chosenID, float vk, bool isChip){
     //Teoricamente, o robô já está em posse da bola!

     actuator->sendCommand(isYellow, chosenID, 0, 0, 0, false, vk, isChip);
     usleep(1000000);//sleeps for 1 second
     actuator->sendCommand(isYellow, chosenID, 0, 0, 0, false, 0, false);
}

//Comportamentos mais complexos

void bringBallCenter (Vision *vision, Actuator *actuator, bool isYellow, int chosenID) {
    //1 - Olhar para a bola
    facePoint(vision, actuator, isYellow, chosenID, true, 0, 0, false);

    //2 - Pegar a bola
    goTo(vision, actuator, isYellow, chosenID, true, 0, 0, true);

    //3 - Olhar para o centro do campo
    //facePoint(vision, actuator, isYellow, chosenID, false, 0, 0, true);

    //4 - Ir até o centro com a bola
    goTo(vision, actuator, isYellow, chosenID, false, 0, 0, true);
    std::cout << "Bola no centro!" << std::endl;
    actuator->sendCommand(isYellow, chosenID, 0, 0, 0, false, 0, false);
    std::cout << "Velocidade deveria estar zerada!!" << std::endl;
}


//Métodos manuais

void manualMove (Actuator *actuator, bool isYellow, int chosenID, float vx, float vy, bool spinner){
    actuator->sendCommand(isYellow, chosenID, vx, vy, 0, spinner, 0, false);
    usleep(100000);//sleeps for 1 second
    actuator->sendCommand(isYellow, chosenID, 0, 0, 0, spinner, 0, false);
}

void manualRotation (Actuator *actuator, bool isYellow, int chosenID, float vw, bool spinner){
    actuator->sendCommand(isYellow, chosenID, 0, 0, vw, spinner, 0, false);
    usleep(100000);//sleeps for 1 second
    actuator->sendCommand(isYellow, chosenID, 0, 0, 0, spinner, 0, false);
}

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    Vision *vision = new Vision("224.5.23.2", 10020);
    Actuator *actuator = new Actuator("127.0.0.1", 20011);

    // Desired frequency
    int desiredFrequency = 60;

    float vk = 3; //força do chute
    float v = 3; //Velocidade de movimento manual
    float vw = 2; //Velocidade de rotação manual


    //Definindo o robô "atacante"
    bool isYellow = true;
    int chosenID = 1;
    bool isChip = false; //define se o chute é chip.

    std::cout << "Qual modo deseja usar? 'a' para automático, 'm' para manual:" << std::endl;
    char modo = '0';
    cin >> modo;
    if (modo == 'a'){
        std::cout << "O amarelo 1 ficará chutando ao gol!" << std::endl;

        while(true) {
                // TimePoint
                std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();

                // Process vision and actuator commands
                vision->processNetworkDatagrams();

                // TimePoint
                std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

                // Sleep thread
                long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
                std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
            }

    }

    else if (modo == 'm'){
        while (true){
            //selecionar time
            char team = '0';
            while (team != 'y' && team != 'b'){
                std::cout << "Qual time deseja usar? 'y' para yellow, 'b' para azul:" << std::endl;
                cin >> team;
                if (team == 'y'){
                    isYellow = true;
                } else if (team == 'b'){
                    isYellow = false;
                }
            }

            //Selecionar o robô
            chosenID = -1;
            while(!(chosenID >= 0 && chosenID <= 3)){
                std::cout << "Qual robô deseja controlar? Digite o seu ID (0 a 3):" << std::endl;
                cin >> chosenID;
            }

            //comandos:
            //Para que o comando seja tratado como um impulso, e não como uma velocidade constante,
            //fiz o programa pausar por um tempo, depois passei um novo comando zerando as velocidades após o impulso.
            std::cout << "Você já pode controlar o robô! wasd para movimentação, qe para rotação." << std::endl;
            std::cout << "z para chutar, x para chutar parabolicamente, z para ligar/desligar dribble." << std::endl;
            std::cout << "b para olhar para a bola." << std::endl;
            std::cout << "p para parar." << std::endl;
            std::cout << "l para mostrar posição do robô (Pode demorar um pouco para a vision atualizar)." << std::endl;
            char command = 's';
            bool spinner = false;

            while (command != 'p'){

                // TimePoint
                std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();

                //Roda a visão
                vision->processNetworkDatagrams();

                SSL_DetectionRobot robot = vision->getLastRobotDetection(isYellow, chosenID);

                cin >> command;

                switch (command) {
                    case 'w': manualMove(actuator, isYellow, chosenID, v, 0, spinner);
                    break;
                    case 'a': manualMove(actuator, isYellow, chosenID, 0, v, spinner);
                    break;
                    case 's': manualMove(actuator, isYellow, chosenID, -v, 0, spinner);
                    break;
                    case 'd': manualMove(actuator, isYellow, chosenID, 0, -v, spinner);
                    break;
                    case 'q': manualRotation(actuator, isYellow, chosenID, vw, spinner);
                    break;
                    case 'e': manualRotation(actuator, isYellow, chosenID, -vw, spinner);
                    break;
                    case 'z':
                        kickBall(actuator, isYellow, chosenID, vk, false);
                        spinner = false;
                    break;
                    case 'x':
                        kickBall(actuator, isYellow, chosenID, vk, true);
                        spinner = false;
                    break;
                    case 'c':
                        //Liga/desliga o spinner o spinner
                        if (spinner){
                            spinner = false;
                            std::cout << "Spinner desligado." << std::endl;
                        } else {
                            spinner = true;
                            std::cout << "Spinner ligado." << std::endl;
                        }
                        actuator->sendCommand(isYellow, chosenID, 0, 0, 0, spinner, 0, false);
                    break;
                    case 'b':
                        //olha para a bola
                        facePoint(vision, actuator, isYellow, chosenID, true, 0, 0, spinner);
                    break;
                    case 'g':
                        //atenção
                        //Quando no simulador aparece x=0.100, o vision retorna como robot.x() = 1000.
                        float x, y;
                        cin >> x;
                        cin >> y;
                        goTo(vision, actuator, isYellow, chosenID, false, x, y, spinner);
                    break;
                    case 'l':
                        vision->processNetworkDatagrams();
                        std::cout << robot.x() << " | " << robot.y() << std::endl;
                    break;
                    case '1': bringBallCenter(vision, actuator, isYellow, chosenID);
                    actuator->sendCommand(isYellow, chosenID, 0, 0, 0, spinner, 0, false);
                    break;
                }


                // TimePoint
                std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

                // Sleep thread
                long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
                std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
            }
        }

    }


    return a.exec();
}
