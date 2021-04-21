#include <QCoreApplication>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>
#include <unistd.h>

using namespace std;

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
                    manualMove(actuator, isYellow, chosenID, v);
                } else if (command == 's') {
                    manualMove(actuator, isYellow, chosenID, -v);
                } else if (command == 'q'){
                    manualRotation(actuator, isYellow, chosenID, vw, true);
                } else if (command == 'e'){
                    manualRotation(actuator, isYellow, chosenID, vw, false);
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
