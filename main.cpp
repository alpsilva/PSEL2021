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

    auto kickBall = [](Vision *vision, Actuator *actuator, bool isYellow, int chosenID, float vk, bool isChip){
         //pegando instância atual do robô selecionado
         SSL_DetectionRobot robot = vision->getLastRobotDetection(isYellow, chosenID);
         //pegando instância atual da posição da bola
         SSL_DetectionBall ball = vision->getLastBallDetection();

         //Teoricamente, o robô já está em posse da bola!

         //Faz o robô chutar!
         actuator->sendCommand(isYellow, chosenID, 0, 0, 0, false, vk, isChip);
         usleep(1000000);//sleeps for 1 second
         actuator->sendCommand(isYellow, chosenID, 0, 0, 0, false, 0, false);
    };

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
        kickBall(vision, actuator, isYellow, chosenID, vk, isChip);
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

    return a.exec();
}
