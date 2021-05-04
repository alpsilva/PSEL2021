#include <QCoreApplication>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>
#include <qmath.h>
#include <qpair.h>

#include <unistd.h>

using namespace std;

//Funções auxiliares

float getPlayerRotateAngleTo(float rx, float ry, float ori, float tx, float ty) {
    float componentX = (tx - rx);
    float componentY = (ty - ry);
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

    angleRobotToTarget = angleOriginToTarget - ori;

    // Adjusting to rotate the minimum possible
    if(angleRobotToTarget > M_PI) angleRobotToTarget -= 2.0 * M_PI;
    if(angleRobotToTarget < -M_PI) angleRobotToTarget += 2.0 * M_PI;

    return angleRobotToTarget;
}

QPair<float, float> getVelocities (float rx, float ry, float ori, float tx, float ty) {

    float dx = (tx - rx);
    float dy = (ty - ry);

    float vx = (dx * cos(ori) + dy * sin(ori));
    float vy = (dy * cos(ori) - dx * sin(ori));

    /*
    float divisor = 10;

    if (vx <= 500 || vy <= 500){
        divisor = 100;
    } else if (vx <= 250 || vy <= 250){
        divisor = 1000;
    } else {
        divisor = 10;
    }

    vx = vx/divisor;
    vy = vy/divisor;
    */

    vx = vx/500;
    vy = vy/500;

    if (vx > 0){
        vx = sqrt(vx);
    }
    if (vy > 0){
        vy = sqrt(vy);
    }


    QPair<float, float> velocities;
    velocities.first = vx;
    velocities.second = vy;

    return velocities;
}

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    Vision *vision = new Vision("224.5.23.2", 10020);
    Actuator *actuator = new Actuator("127.0.0.1", 20011);

    // Desired frequency
    int desiredFrequency = 60;

    float vx = 0; //Velocidade de movimento eixo x
    float vy = 0; //Velocidade de movimento eixo y
    float w = 3; //Velocidade angular base
    float vw = 0; //velocidade angular variavel
    bool spinner = false; //Toggle do spinner

    float vk = 0; //Velocidade do chute
    bool isChip = false; //define se o chute é chip.


    //Definindo o robô "atacante"
    bool isYellow = true;
    int chosenID = 3;

    float angleError = 0.05; //Aproximadamente 3º


    std::cout << "1: Trazer a bola para o centro." << std::endl;
    std::cout << "2: Modo atacante." << std::endl;
    char modo = '0';
    cin >> modo;

    if (modo == '1'){
        std::cout << "O amarelo 3 ficará levando a bola para o centro!" << std::endl;

        bool hasBall = false;
        spinner = true;

        while(true) {
            // TimePoint
            std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();

            // Process vision and actuator commands
            vision->processNetworkDatagrams();

            SSL_DetectionRobot robot = vision->getLastRobotDetection(isYellow, chosenID);
            SSL_DetectionBall ball = vision->getLastBallDetection();

            float rx = robot.x();
            float ry = robot.y();
            float orientation = robot.orientation();

            float bx = ball.x();
            float by = ball.y();

            float targetx;
            float targety;

            float dx = rx - bx; //distância no eixo x entre o robô e a bola.
            float dy = ry - by; //distância no eixo y entre o robô e a bola.

            //Checa se o robô já está em posse da bola
            if (fabs(dx) <= 115 && fabs(dy) < 120){
                hasBall = true;
            } else{
                hasBall = false;
            }

            if (hasBall){
                //Leva para o centro.
                targetx = 0;
                targety = 0;
            } else {
                //Não tem a bola, vai até ela.
                targetx = bx;
                targety = by;
            }

            QPair<float, float> velocities = getVelocities(rx, ry, orientation, targetx, targety);
            vx = velocities.first;
            vy = velocities.second;

            //Angulação: é preciso que o robô rotacione até esse ponto.
            float angleRobotToObjective = getPlayerRotateAngleTo(rx, ry, orientation, targetx, targety);

            if(orientation > M_PI) orientation -= 2.0 * M_PI;
            if(orientation < -M_PI) orientation += 2.0 * M_PI;

            float angleRobotToTarget = orientation + angleRobotToObjective;

            //Se o robô já estiver perto do ângulo, não é necessário rotacioná-lo.
            if (((angleRobotToTarget - angleError) <= orientation) && (orientation <= (angleRobotToTarget + angleError))){
                vw = 0;
            } else {
                std::cout << "neededAngle: "  << angleRobotToTarget << endl;
                std::cout << "orientation: " << orientation << endl;
                if (fabs(orientation) < fabs(angleRobotToTarget)){
                    vw = w * -1;
                } else{
                    vw = w;
                }
            }

            actuator->sendCommand(isYellow, chosenID, vx, vy, vw, spinner, vk, isChip);

            // TimePoint
            std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

            // Sleep thread
            long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
            std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
        }
    }

    if (modo == '2'){
        std::cout << "O amarelo 3 ficará chutando ao gol!" << std::endl;

        bool hasBall = false;
        spinner = true;

        float goalx = -6000;
        float goaly = 0;

        float kickPosx = -4000;
        float kickPosy = 0;

        while(true) {
            // TimePoint
            std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();

            // Process vision and actuator commands
            vision->processNetworkDatagrams();

            SSL_DetectionRobot robot = vision->getLastRobotDetection(isYellow, chosenID);
            SSL_DetectionBall ball = vision->getLastBallDetection();

            float rx = robot.x();
            float ry = robot.y();
            float orientation = robot.orientation();

            float bx = ball.x();
            float by = ball.y();

            float moveTargetx;
            float moveTargety;

            float lookTargetx;
            float lookTargety;

            float dx = rx - bx; //distância no eixo x entre o robô e a bola.
            float dy = ry - by; //distância no eixo y entre o robô e a bola.

            bool isLookingToTarget = false;

            std::cout << dx << " | " << dy << endl;

            //Checa se o robô já está em posse da bola
            if (fabs(dx) <= 120 && fabs(dy) < 120){
                hasBall = true;
            } else{
                hasBall = false;
            }

            if (hasBall){
                //Leva para a posição de chute.
                moveTargetx = kickPosx;
                moveTargety = kickPosy;
                lookTargetx = goalx;
                lookTargety = goaly;
            } else {
                //Não tem a bola, vai até ela.
                moveTargetx = lookTargetx = bx;
                moveTargety = lookTargety = by;
            }

            QPair<float, float> velocities = getVelocities(rx, ry, orientation, moveTargetx, moveTargety);
            vx = velocities.first;
            vy = velocities.second;

            //Angulação: é preciso que o robô rotacione até esse ponto.
            float angleRobotToObjective = getPlayerRotateAngleTo(rx, ry, orientation, lookTargetx, lookTargety);

            if(orientation > M_PI) orientation -= 2.0 * M_PI;
            if(orientation < -M_PI) orientation += 2.0 * M_PI;

            float angleRobotToTarget = orientation + angleRobotToObjective;

            //Se o robô já estiver perto do ângulo, não é necessário rotacioná-lo.
            if (((angleRobotToTarget - angleError) <= orientation) && (orientation <= (angleRobotToTarget + angleError))){
                vw = 0;
                isLookingToTarget = true;
            } else {
                if (fabs(orientation) < fabs(angleRobotToTarget)){
                    vw = w * -1;
                } else{
                    vw = w;
                }
            }

            dx = rx - kickPosx; //distância no eixo x entre o robô e a posição de chute.
            dy = ry - kickPosy; //distância no eixo y entre o robô e a posição de chute.

            //Checa se o robô tem a bola, se está na posição de chute e se está olhando para o gol. Caso positivo, chuta. (ChipShot é alternado).
            if (hasBall && ((fabs(dx) <= 100) && (fabs(dy) <= 100)) && isLookingToTarget){
                vk = 5;
                if(isChip){
                    isChip = false;
                } else {
                    isChip = true;
                }
            }

            actuator->sendCommand(isYellow, chosenID, vx, vy, vw, spinner, vk, isChip);

            //"desliga" o chute
            vk = 0;

            // TimePoint
            std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

            // Sleep thread
            long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
            std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
        }

    }

    return a.exec();
}
