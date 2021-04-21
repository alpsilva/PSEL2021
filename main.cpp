#include <QCoreApplication>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>
#include <qmath.h>

#include <unistd.h>

using namespace std;

float getPlayerRotateAngleTo(float px, float py, fira_message::Robot robot) {
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

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    Vision *vision = new Vision("224.0.0.1", 10002);
    Actuator *actuator = new Actuator("127.0.0.1", 20011);

    // Desired frequency
    int desiredFrequency = 60;

    float v = 100; //Velocidade de movimento manual
    float vw = 5; //Velocidade de rotação manual

    bool isYellow;
    int chosenID;

    auto facePoint = [](Vision *vision, Actuator *actuator, bool isYellow, int chosenID, bool pointIsBall, float px, float py){
        vision->processNetworkDatagrams();
        //pegando instância atual do robô selecionado
        fira_message::Robot robot = vision->getLastRobotDetection(isYellow, chosenID);
        //pegando instância atual da posição da bola
        fira_message::Ball ball = vision->getLastBallDetection();

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

        if (angle > (M_PI)){
            float resto = angle - M_PI;
            angle = -M_PI + resto;
        }

        //Orientação atual do robô
        float orientation = robot.orientation();

        float vw2; //velocidade angular dentro dessa função

        if (pointIsBall){
            vw2 = 2.5;
        } else{
            vw2 = 1;
        }

        //Faz o robô girar
        if (orientation > angle){
            actuator->sendCommand(isYellow, chosenID, vw2, -vw2);
        } else{
            actuator->sendCommand(isYellow, chosenID, -vw2, vw2);
        }

        float variacao = 0.002; //faixa de angulação aceitável para parar o loop
        float faixaInf = angle-variacao;
        float faixaSup = angle+variacao;


        //Robô roda enquanto a orientação dele não estiver perto o bastante da variavel angle
        while((orientation < faixaInf) || (orientation > faixaSup)){
            vision->processNetworkDatagrams();
            robot = vision->getLastRobotDetection(isYellow, chosenID);
            orientation = robot.orientation();
            std::cout << "Angulo necessário: " << angle << std::endl;
            std::cout << "Orientação do robô: " << orientation << std::endl;
        }

        //para de rodar
        actuator->sendCommand(isYellow, chosenID, 0, 0);
    };

    auto manualMove = [](Actuator *actuator, bool isYellow, int chosenID, float v){
        actuator->sendCommand(isYellow, chosenID, v, v);
        usleep(200000);//sleeps for 2 second
        actuator->sendCommand(isYellow, chosenID, 0, 0);
    };

    auto manualRotation = [](Actuator *actuator, bool isYellow, int chosenID, float v, bool isLeft){
        if (isLeft){
            actuator->sendCommand(isYellow, chosenID, -v, v);
        } else {
            actuator->sendCommand(isYellow, chosenID, v, -v);
        }

        usleep(200000);//sleeps for 2 second
        actuator->sendCommand(isYellow, chosenID, 0, 0);
    };

    while(true) {
        // TimePoint
        std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();

        // Process vision and actuator commands
        vision->processNetworkDatagrams();

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
            while(!(chosenID >= 0 && chosenID <= 2)){
                std::cout << "Qual robô deseja controlar? Digite o seu ID (0 a 2):" << std::endl;
                cin >> chosenID;
            }

            //comandos:
            //Para que o comando seja tratado como um impulso, e não como uma velocidade constante,
            //fiz o programa pausar por um tempo, depois passei um novo comando zerando as velocidades após o impulso.
            std::cout << "Você já pode controlar o robô! ws para movimentação, qe para rotação." << std::endl;
            std::cout << "l para mostrar posição do robô (Pode demorar um pouco para a vision atualizar)." << std::endl;
            std::cout << "b para olhar para a bola." << std::endl;
            std::cout << "x to let it rip." << std::endl;
            std::cout << "p para parar." << std::endl;

            char command = 's';
            bool beyblade = false;
            while (command != 'p'){

                //Roda a visão
                vision->processNetworkDatagrams();

                fira_message::Robot robot = vision->getLastRobotDetection(isYellow, chosenID);

                cin >> command;

                if (command == 'w'){
                    manualMove(actuator, isYellow, chosenID, -v);
                } else if (command == 's') {
                    manualMove(actuator, isYellow, chosenID, v);
                } else if (command == 'q'){
                    manualRotation(actuator, isYellow, chosenID, vw, false);
                } else if (command == 'e'){
                    manualRotation(actuator, isYellow, chosenID, vw, true);
                } else if (command == 'b'){
                    facePoint(vision, actuator, isYellow, chosenID, true, 0, 0);
                } else if (command == 'x'){
                    if (beyblade){
                        actuator->sendCommand(isYellow, chosenID, 0, 0);
                        beyblade = false;
                    } else {
                        actuator->sendCommand(isYellow, chosenID, 3 * v, -3 * v);
                        beyblade = true;
                    }
                } else if (command == 'l'){
                    std::cout << robot.x() << " | " << robot.y() << std::endl;
                    float nearest = roundf(robot.orientation() * 100) / 100;
                    std::cout << nearest << std::endl;
                }
            }
        }


        // TimePoint
        std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

        // Sleep thread
        long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }

    return a.exec();
}
