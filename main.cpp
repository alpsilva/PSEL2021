#include <QCoreApplication>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>
#include <qmath.h>

#include <unistd.h>

using namespace std;

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    Vision *vision = new Vision("224.0.0.1", 10002);
    Actuator *actuator = new Actuator("127.0.0.1", 20011);

    // Desired frequency
    int desiredFrequency = 60;

    float vk = 3; //força do chute
    float v = 3; //Velocidade de movimento manual
    float vw = 2; //Velocidade de rotação manual

    //Definindo o robô "atacante"
    bool isYellow = true;
    int chosenID = 3;
    bool isChip = false; //define se o chute é chip.

    auto facePoint = [](Vision *vision, Actuator *actuator, bool isYellow, int chosenID, bool pointIsBall){
        //pegando instância atual do robô selecionado
        SSL_DetectionRobot robot = vision->getLastRobotDetection(isYellow, chosenID);
        //pegando instância atual da posição da bola
        SSL_DetectionBall ball = vision->getLastBallDetection();

        float rx = robot.x();
        float ry = robot.y();
        float px;
        float py;

        //Se true, ele vira para bola. Falso, ele vira para o gol.
        if (pointIsBall){
            px = ball.x();
            py = ball.y();
        } else{
            //coordenadas do fundo do gol
            px = -0.8;
            py = 0.02;
        }


        //Angulo entre o eixo x e a bola
        float angle = atan2((ry - py), (rx - px));

        //Orientação atual do robô
        float orientation = robot.orientation();
        std::cout << "Orientação inicial do robô: " << orientation << std::endl;

        float vw = 1; //velocidade angular

        //Faz o robô girar
        if (orientation > angle){
            actuator->sendCommand(isYellow, chosenID, 0, 0, -vw, true, 0, false);
        } else{
            actuator->sendCommand(isYellow, chosenID, 0, 0, vw, true, 0, false);
        }

        float variacao = 0.01; //faixa de angulação aceitável para parar o loop
        float faixaInf = angle-variacao;
        float faixaSup = angle+variacao;


        //Robô roda enquanto a orientação dele não estiver perto o bastante da variavel angle
        while((orientation < faixaInf) || (orientation > faixaSup)){
            vision->processNetworkDatagrams();
            robot = vision->getLastRobotDetection(isYellow, chosenID);
            orientation = robot.orientation();
            //std::cout << "Angulo necessário: " << angle << std::endl;
            //std::cout << "Orientação do robô: " << orientation << std::endl;
        }

        //para de rodar
        actuator->sendCommand(isYellow, chosenID, 0, 0, 0, true, 0, false);

    };

    auto pickBall = [](Vision *vision, Actuator *actuator, bool isYellow, int chosenID){
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
         float d = qSqrt(c1 + c2);
         std::cout << "Distância para a bola: " << d << std::endl;

         //calcula velocidade v necessária para chegar na bola
         float v = d;

         //Aciona o robô para ir em frente, com o spinner ligado para segurar a bola
         actuator->sendCommand(isYellow, chosenID, v, 0, 0, true, 0, false);
         usleep(200000);//sleeps for 1 second
         actuator->sendCommand(isYellow, chosenID, 0, 0, 0, true, 0, false);
    };

    auto kickBall = [](Actuator *actuator, bool isYellow, int chosenID, float vk, bool isChip){
         //Teoricamente, o robô já está em posse da bola!

         actuator->sendCommand(isYellow, chosenID, 0, 0, 0, false, vk, isChip);
         usleep(1000000);//sleeps for 1 second
         actuator->sendCommand(isYellow, chosenID, 0, 0, 0, false, 0, false);
    };

    auto moveToPointInFront = [](Vision *vision, Actuator *actuator, bool isYellow, int chosenID, float px, float py, bool spinner) {

        vision->processNetworkDatagrams();
        SSL_DetectionRobot robot = vision->getLastRobotDetection(isYellow, chosenID);

        float rx = robot.x();
        float ry = robot.y();

        float variacao = 0.1; //Limite de erro do robô

        float vAux; //velocidade necessaria

        while (rx < px-variacao || rx > px+variacao){
            vAux = rx - px;
            vAux = vAux/100;
            actuator->sendCommand(isYellow, chosenID, vAux, 0, 0, spinner, 0, false);
            usleep(100000);//sleeps for 1 second
            actuator->sendCommand(isYellow, chosenID, 0, 0, 0, spinner, 0, false);

            vision->processNetworkDatagrams();
            SSL_DetectionRobot robot = vision->getLastRobotDetection(isYellow, chosenID);

            rx = robot.x();
            ry = robot.y();
            rx = roundf(rx * 100) / 100;
            ry = roundf(ry * 100) / 100;
            std::cout << rx << " | " << ry << std::endl;
        }

        std::cout << "=======================================================================" << std::endl;

        while (ry < py-variacao || ry > py+variacao){
            vAux = ry - py;
            vAux = vAux/100;
            actuator->sendCommand(isYellow, chosenID, 0, vAux, 0, spinner, 0, false);
            usleep(100000);//sleeps for 1 second
            actuator->sendCommand(isYellow, chosenID, 0, 0, 0, spinner, 0, false);

            vision->processNetworkDatagrams();
            SSL_DetectionRobot robot = vision->getLastRobotDetection(isYellow, chosenID);

            rx = robot.x();
            ry = robot.y();
            rx = roundf(rx * 100) / 100;
            ry = roundf(ry * 100) / 100;
            std::cout << rx << " | " << ry << std::endl;
        }
    };

    auto manualMove = [](Actuator *actuator, bool isYellow, int chosenID, float vx, float vy, bool spinner){
        actuator->sendCommand(isYellow, chosenID, vx, vy, 0, spinner, 0, false);
        usleep(100000);//sleeps for 1 second
        actuator->sendCommand(isYellow, chosenID, 0, 0, 0, spinner, 0, false);
    };

    auto manualRotation = [](Actuator *actuator, bool isYellow, int chosenID, float vw, bool spinner){
        actuator->sendCommand(isYellow, chosenID, 0, 0, vw, spinner, 0, false);
        usleep(100000);//sleeps for 1 second
        actuator->sendCommand(isYellow, chosenID, 0, 0, 0, spinner, 0, false);
    };

    while (true){
            std::cout << "Qual modo deseja usar? 'a' para automático, 'm' para manual:" << std::endl;
            char modo = '0';
            cin >> modo;
            if (modo == 'a'){
                std::cout << "O amarelo 3 ficará chutando ao gol!" << std::endl;

                while(true) {
                        // TimePoint
                        std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();

                        // Process vision and actuator commands
                        vision->processNetworkDatagrams();
                        usleep(1000000);

                        float d = 5; // distância entre o robô e a bola

                         char aux;

                        do{
                            vision->processNetworkDatagrams();
                            //pegando instância atual do robô selecionado
                            SSL_DetectionRobot robot = vision->getLastRobotDetection(isYellow, chosenID);
                            //pegando instância atual da posição da bola
                            SSL_DetectionBall ball = vision->getLastBallDetection();


                            facePoint(vision, actuator, isYellow, chosenID, true);
                            std::cout << "Olhando para a bola===================================================================" << std::endl;
                            pickBall(vision, actuator, isYellow, chosenID);
                            std::cout << "indo para a bola===================================================================" << std::endl;
                            cin >> aux;
                            float bx = ball.x();
                            float by = ball.y();
                            float rx = robot.x();
                            float ry = robot.y();


                            std::cout << "Coord bola: " << bx << " | " << by << std::endl;
                            std::cout << "Coord robô: " << rx << " | " << ry << std::endl;


                             //Calcula distância d entre a bola e o robô
                             float c1 = rx - bx;
                             float c2 = ry - by;
                             c1 = qPow(c1, 2);
                             c2 = qPow(c2, 2);
                             d = qSqrt(c1 + c2);
                        }while(d>5);

                        facePoint(vision, actuator, isYellow, chosenID, false);
                        std::cout << "olhando para o gol===================================================================" << std::endl;
                        cin >> aux;
                        kickBall(actuator, isYellow, chosenID, vk, isChip);
                        std::cout << "Chutando a bola===================================================================" << std::endl;
                        cin >> aux;
                        usleep(2000000);

                        //Faz com que a cada iteração, ele mude o tipo de chute.
                        if (isChip){
                            isChip = false;
                        } else{
                            isChip = true;
                        }

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
                    std::cout << "p para parar." << std::endl;
                    std::cout << "l para mostrar posição do robô (Pode demorar um pouco para a vision atualizar)." << std::endl;
                    char command = 's';
                    bool spinner = false;

                    while (command != 'p'){

                        //Roda a visão
                        vision->processNetworkDatagrams();

                        SSL_DetectionRobot robot = vision->getLastRobotDetection(isYellow, chosenID);

                        cin >> command;

                        if (command == 'w'){
                            manualMove(actuator, isYellow, chosenID, v, 0, spinner);
                        } else if (command == 'a') {
                            manualMove(actuator, isYellow, chosenID, 0, v, spinner);
                        } else if (command == 's') {
                            manualMove(actuator, isYellow, chosenID, -v, 0, spinner);
                        } else if (command == 'd') {
                            manualMove(actuator, isYellow, chosenID, 0, -v, spinner);
                        } else if (command == 'q'){
                            manualRotation(actuator, isYellow, chosenID, vw, spinner);
                        } else if (command == 'e'){
                            manualRotation(actuator, isYellow, chosenID, -vw, spinner);
                        } else if (command == 'z') {
                            //Faz o robô chutar!
                            kickBall(actuator, isYellow, chosenID, vk, false);
                            spinner = false;
                        } else if (command == 'x'){
                            //Faz o robô chutar parabolicamente!
                            kickBall(actuator, isYellow, chosenID, vk, true);
                            spinner = false;
                        } else if (command == 'c') {
                            //Liga/desliga o spinner o spinner
                            if (spinner){
                                spinner = false;
                                std::cout << "Spinner desligado." << std::endl;
                            } else {
                                spinner = true;
                                std::cout << "Spinner ligado." << std::endl;
                            }
                            actuator->sendCommand(isYellow, chosenID, 0, 0, 0, spinner, 0, false);
                        } else if (command == 'g'){
                            //atenção
                            //Quando no simulador aparece x=0.100, o vision retorna como robot.x() = 1000.
                            float x, y;
                            cin >> x;
                            cin >> y;
                            moveToPointInFront(vision, actuator,isYellow, chosenID, x, y, spinner);
                        } else if (command == 'l'){
                            std::cout << robot.x() << " | " << robot.y() << std::endl;
                        }
                    }
                }

            }

        }

    return a.exec();
}
