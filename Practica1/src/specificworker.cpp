/*
 *    Copyright (C) 2024 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

// Definición de los estados del robot
enum class State { IDLE, FORWARD, TURN, FOLLOW_WALL, SPIRAL };
State current_state = State::FORWARD;  // Estado inicial

const float MAX_ADV_SPEED = 1000.0f;  // Velocidad máxima de avance
const float DISTANCIA_MINIMA = 300.0f;  // Distancia mínima para evitar colisiones

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    return true;
}

void SpecificWorker::initialize()
{
    std::cout << "Initialize worker" << std::endl;
    if(this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
        #ifdef HIBERNATION_ENABLED
            hibernationChecker.start(500);
        #endif

        this->setPeriod(STATES::Compute, 100);
    }
}

void SpecificWorker::compute()
{
    try {
        // Obtener los datos del Lidar3D con los parámetros necesarios
        const std::string lidar_name = "myLidar";  // Nombre del lidar, ajusta según tu configuración
        float start = 0.0f;  // Posición de inicio
        float length = 1000.0f;  // Longitud de los datos a obtener
        int decimation = 1;  // Factor de decimación

        // Llamada correcta para obtener los datos del Lidar
        RoboCompLidar3D::TData lData = this->lidar3d_proxy->getLidarData(lidar_name, start, length, decimation);

        // Imprimir el número de puntos
        qDebug() << "Number of points: " << lData.points.size();  // Accede a la colección de puntos

        // Iterar sobre los puntos y procesarlos
        for (const auto& point : lData.points) {
            qDebug() << "Point (x, y, z): (" << point.x << ", " << point.y << ", " << point.z << ")";
            qDebug() << "Distance2D: " << point.distance2d;  // O cualquier otro campo que necesites
        }

        // Según el estado actual, llamamos a diferentes funciones de comportamiento
        switch(current_state) {
        case State::FORWARD:
            current_state = FORWARD_method(lData);
            break;
        case State::TURN:
            current_state = TURN_method(lData);
            break;
        case State::IDLE:
            // Estado IDLE, no hacer nada
            break;
        }
    }
    catch(const std::exception& e) {
        std::cout << e.what() << std::endl;
    }
}

// Función que maneja el avance hacia adelante
SpecificWorker::State SpecificWorker::FORWARD_method(const RoboCompLidar3D::TData &ldata)
{

}

// Función que maneja el giro del robot
SpecificWorker::State SpecificWorker::TURN_method(const RoboCompLidar3D::TData &ldata)
{
    // Verificar si hay espacio libre adelante

}

void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
}

void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
}

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}
