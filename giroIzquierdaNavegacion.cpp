#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>

using namespace webots;

int main(int argc,char **argv){

    Robot *robot = new Robot();
    int timeStep = (int)robot->getBasicTimeStep();

    //Inicializar actuadores
    Motor *leftMotor = robot->getMotor("left wheel motor");
    Motor *rightMotor = robot->getMotor("right wheel motor");
    
    //configurar el modo de velocidad continua 
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    DistanceSensor *ps0= robot->getDistanceSensor("ps0");
    DistanceSensor *ps7= robot->getDistanceSensor("ps7");

    ps0->enable(timeStep);
    ps7->enable(timeStep);

    double cruiseSpeed = 2.0;

    while(robot->step(timeStep)!=-1){ // mientras la simulacion sigue activa

        double valDerecho = ps0->getValue();
        double valIzquierdo = ps7->getValue();

        std::cout << "Sensor Der (ps0): "<< valDerecho
                  << "Sensor Der (ps7):" << valIzquierdo <<std::endl;

        if(valDerecho > 100 || valIzquierdo > 100){
            
            leftMotor->setVelocity(-cruiseSpeed);
            rightMotor->setVelocity(cruiseSpeed);
            std::cout << "obstaculo detectado"<< std::endl;
            
        }
        else{
            leftMotor->setVelocity(cruiseSpeed);
            rightMotor->setVelocity(cruiseSpeed);

        }
    }

    delete robot;
    return 0;

}
