#include <QCoreApplication>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>
#include <qmath.h>

#include <unistd.h>

using namespace std;

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

float getVx (float rx, float ry, float ori, float tx, float ty) {
    float dx = (tx - rx);
    float dy = (ty - ry);

    //Função usada no omni
    float vx = (dx * cos(ori) + dy * sin(ori));
    //float vy = (dy * cos(ori) - dx * sin(ori));

    //vx = vx/500;
    //vy = vy/500;

    if (vx > 0){
        vx = sqrt(vx);
    }
    /*
    if (vy > 0){
        vy = sqrt(vy);
    }
    */

    return vx;
}

QPair<float, float> getWheelVelocities (float vx, float angle, float r, float l) {

    float vl = (2*vx) - (l*angle)/(2*r);
    float vr = (2*vx) + (l*angle)/(2*r);

    QPair<float, float> wheelVelocities;
    wheelVelocities.first = vl;
    wheelVelocities.second = vr;

    return wheelVelocities;
}


/*
float calculateProportion(float angle, float vx){
    float v = 3/pow(angle+1, 2);
    return v * vx;
}
*/


int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    Vision *vision = new Vision("224.0.0.1", 10002);
    Actuator *actuator = new Actuator("127.0.0.1", 20011);

    // Desired frequency
    int desiredFrequency = 60;

    float raio = 0; //Raio das rodas
    float distEntreRodas = 0; //Distância entre as rodas

    float vl; //Velocidade da roda esquerda
    float vr; //Velocidade da roda direita

    bool isYellow = true;
    int chosenID = 0;

    float angleError = 0.05; //aprox. 3º

    //Teste para fazer o robô ir até o centro
    float moveTargetx = 0;
    float moveTargety = 0;

    bool isInCenter = false;

    while(true) {
        // TimePoint
        std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();

        // Process vision and actuator commands
        vision->processNetworkDatagrams();

        fira_message::Robot robot = vision->getLastRobotDetection(isYellow, chosenID);

        float rx = robot.x();
        float ry = robot.y();
        float orientation = robot.orientation();

        fira_message::Ball ball = vision->getLastBallDetection();

        float bx = ball.x();
        float by = ball.y();


        //Angulação: é preciso que o robô rotacione até esse ponto.
        float angleRobotToObjective = getPlayerRotateAngleTo(rx, ry, orientation, moveTargetx, moveTargety);

        if(orientation > M_PI) orientation -= 2.0 * M_PI;
        if(orientation < -M_PI) orientation += 2.0 * M_PI;

        float angleRobotToTarget = orientation + angleRobotToObjective;

        float v = getVx(rx, ry, orientation, moveTargetx, moveTargety);

        QPair<float, float> wheelVelocities = getWheelVelocities(v, angleRobotToTarget, raio, distEntreRodas);

        vl = wheelVelocities.first;
        vr = wheelVelocities.second;

        actuator->sendCommand(isYellow, chosenID, vl, vr);

        // TimePoint
        std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

        // Sleep thread
        long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }

    return a.exec();
}
