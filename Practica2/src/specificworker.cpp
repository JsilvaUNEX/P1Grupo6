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
#include <cppitertools/enumerate.hpp>
#include <cppitertools/range.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    std::locale::global(std::locale("C"));
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
        // Viewer
        viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
        auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
        robot_draw = r;
        viewer->setStyleSheet("background-color: lightGray;");


        // connect stop button un UI with a lambda function
        connect(pushButton_stop, &QPushButton::clicked, [this]()
        {
            try
            { omnirobot_proxy->setSpeedBase(0, 0, 0); }
            catch (const Ice::Exception &e)
            { std::cout << e << std::endl; }
            pushButton_stop->setText(pushButton_stop->isChecked() ? "Track" : "Stop");
        });
        viewer->show();

		this->setPeriod(STATES::Compute, 100);
		//this->setPeriod(STATES::Emergency, 500);

	}
}
/*void SpecificWorker::compute()
{
     //read bpearl (lower) lidar and draw
    auto ldata = read_lidar_bpearl();
    if(ldata.points.empty()) { qWarning() << __FUNCTION__ << "Empty bpearl lidar data"; return; };
    //draw_lidar(ldata.points, &viewer->scene);
    draw_lidar(ldata.points, &viewer->scene);

    auto ldata_helios = read_lidar_helios();
    if(ldata_helios.points.empty()) { qWarning() << __FUNCTION__ << "Empty helios lidar data"; return; };
    //draw_lidar(ldata.points, &viewer->scene);

    // check if there is new YOLO data in buffer
    RoboCompVisualElementsPub::TData data;
    auto [data_] = buffer.read_first();
    if(not data_.has_value()) { qWarning() << __FUNCTION__ << "Empty buffer"; return; }
    else data = data_.value();

    // get person and draw on viewer
    auto person = find_person_in_data(data.objects);
    if(not person.has_value())
    { qWarning() << __FUNCTION__ << QString::fromStdString(person.error()); return; }   // STOP THE ROBOT

    // call state machine to track person
    const auto &[adv, rot] = state_machine(person.value());

    // move the robot
    try{ omnirobot_proxy->setSpeedBase(0.f, adv, rot); }
    catch(const Ice::Exception &e){std::cout << e << std::endl;}
    lcdNumber_adv->display(adv);
    lcdNumber_rot ->display(rot);
}
*/

void SpecificWorker::compute()
{
    // Leer y procesar datos del LiDAR
    auto ldata = read_lidar_bpearl();
    if (ldata.points.empty())
    {
        qWarning() << __FUNCTION__ << "Empty bpearl lidar data";
        return;
    }
    draw_lidar(ldata.points, &viewer->scene);

    auto ldata_helios = read_lidar_helios();
    if (ldata_helios.points.empty())
    {
        qWarning() << __FUNCTION__ << "Empty helios lidar data";
        return;
    }
    draw_lidar(ldata_helios.points, &viewer->scene);

    // Verificar si hay nuevos datos de YOLO en el buffer
    auto [data_] = buffer.read_first();
    if (!data_.has_value())
    {
        // Si no hay datos, llamar a state_machine sin persona detectada (pasando un expected vacío)
        std::expected<RoboCompVisualElementsPub::TObject, std::string> no_person;
        const auto &[adv, rot] = state_machine(no_person);

        // Aplicar movimiento al robot en estado de búsqueda
        try { omnirobot_proxy->setSpeedBase(0.f, adv, rot); }
        catch (const Ice::Exception &e) { std::cout << e << std::endl; }

        lcdNumber_adv->display(adv);
        lcdNumber_rot->display(rot);

        return;
    }

    // Obtener datos de detección de personas y dibujar en el visor
    RoboCompVisualElementsPub::TData data = data_.value();
    auto person = find_person_in_data(data.objects);

    // Llamar a state_machine con datos de persona, si están disponibles
    const auto &[adv, rot] = state_machine(person);

    // Mover el robot
    try { omnirobot_proxy->setSpeedBase(0.f, adv, rot); }
    catch (const Ice::Exception &e) { std::cout << e << std::endl; }

    lcdNumber_adv->display(adv);
    lcdNumber_rot->display(rot);
}



//////////////////////////////////////////////////////////////////
/// YOUR CODE HERE
//////////////////////////////////////////////////////////////////
// Read the BPEARL lidar data and filter the points
RoboCompLidar3D::TData SpecificWorker::read_lidar_bpearl()
{
    try
    {
        auto ldata =  lidar3d1_proxy->getLidarData("bpearl", 0, 2*M_PI, 1);
        // filter points according to height and distance
        RoboCompLidar3D::TPoints  p_filter;
        std::ranges::copy_if(ldata.points, std::back_inserter(p_filter),
                             [](auto  &a){ return a.z < 500 and a.distance2d > 200;});
        return RoboCompLidar3D::TData{.points = p_filter, .period = ldata.period, .timestamp = ldata.timestamp};
    }
    catch(const Ice::Exception &e){std::cout << e << std::endl;}
    return {};
}
// Read the HELIOS lidar data and filter the points
RoboCompLidar3D::TData SpecificWorker::read_lidar_helios()
{
    try
    {
        auto ldata =  lidar3d_proxy->getLidarData("helios", 0, 2*M_PI, 1);
        // filter points according to height and distance
        RoboCompLidar3D::TPoints  p_filter;
        std::ranges::copy_if(ldata.points, std::back_inserter(p_filter),
                             [](auto  &a){ return a.z > 1300 and a.distance2d > 200;});
        return RoboCompLidar3D::TData{.points = p_filter, .period = ldata.period, .timestamp = ldata.timestamp};
    }
    catch(const Ice::Exception &e){std::cout << e << std::endl;}
    return {};
}
std::expected<RoboCompVisualElementsPub::TObject, std::string>
SpecificWorker::find_person_in_data(const std::vector<RoboCompVisualElementsPub::TObject> &objects)
{
    if(objects.empty())
        return std::unexpected("Empty objects in method <find_person_in_data>");
    if(auto p_ = std::ranges::find_if(objects, [](auto &a)
            { return a.id == 0 and std::stof(a.attributes.at("score")) > 0.6;}); p_ == std::end(objects))
        return std::unexpected("No person found in method <find_person_in_data>");
    else
    {
        draw_person(const_cast<RoboCompVisualElementsPub::TObject &>(*p_), &viewer->scene);
        return *p_;
    }
}

//////////////////////////////////////////////////////////////////
/// STATE  MACHINE
//////////////////////////////////////////////////////////////////
// State machine to track a person
SpecificWorker::RobotSpeed SpecificWorker::state_machine(std::expected<RoboCompVisualElementsPub::TObject, std::string> &person)
{
    RetVal res;

    // Si se detecta una persona en SEARCH, cambia inmediatamente a TRACK
    if (state == STATE::SEARCH && person.has_value())
    {
        state = STATE::TRACK;
    }
    // Si no se detecta la persona en otros estados, cambia a SEARCH
    else if (state != STATE::SEARCH && !person.has_value())
    {
        state = STATE::SEARCH;
    }

    // Cambia el estado y maneja cada caso
    switch (state)
    {
        case STATE::TRACK:
            res = track(person.value());
        label_state->setText("TRACK");
        break;
        case STATE::WAIT:
            res = wait(person.value());
        label_state->setText("WAIT");
        break;
        case STATE::STOP:
            res = stop();
        label_state->setText("STOP");
        break;
        case STATE::SEARCH:
            res = search();
        label_state->setText("SEARCH");
        break;
    }

    auto &[st, speed, rot] = res;
    state = st;  // Actualizar el estado con el resultado de la transición
    return {speed, rot};
}



/**
 * Analyzes the filtered points to determine whether to continue moving forward or to stop and turn.
 *
 * This method examines the central part of the `filtered_points` vector to find the minimum distance
 * point within that range. If the minimum distance is less than the width of the robot, it indicates
 * an obstacle is too close, prompting a state change to `TURN` and stopping motion. Otherwise,
 * the robot continues to move forward.
 *
 * @param filtered_points A vector of filtered points representing the robot's perception of obstacles.
 * @return A `RetVal` tuple consisting of the state (`FORWARD` or `TURN`), speed, and rotation.
 */
 // State function to track a person



SpecificWorker::RetVal SpecificWorker::track(const RoboCompVisualElementsPub::TObject &person)
{
    qDebug() << "TRACK MODE";



    // Obtener la posición de la persona (asumiendo que 'x_pos' y 'y_pos' son las coordenadas de la persona)
    float person_x = std::stof(person.attributes.at("x_pos"));
    float person_y = std::stof(person.attributes.at("y_pos"));

    // Calcular la distancia a la persona
    auto distance = std::hypot(person_x, person_y);
    lcdNumber_dist_to_person->display(distance);

    // Calcular el ángulo hacia la persona
    float angle_to_person = atan2(person_x, person_y);
    qDebug() << "Angle to person: " << angle_to_person;

    // Si la distancia es menor que el umbral mínimo, detener el robot
    if (distance < params.PERSON_MIN_DIST ) {
        omnirobot_proxy->setSpeedBase(0, 0, 0);  // Detener el robot
        return RetVal(STATE::WAIT, 0.f, 0.f);  // Estado WAIT si está demasiado cerca
    }



    // Escalar la velocidad de avance según la distancia
    // A medida que se acerca a la persona, la velocidad disminuye
    float max_velocity = params.MAX_ADV_SPEED;  // Velocidad máxima de avance
    float min_distance = params.PERSON_MIN_DIST;
    float max_distance = params.PERSON_MAX_DIST;

    // Escalar la velocidad entre 0 y max_velocity, dependiendo de la distancia a la persona
    float velocity = std::clamp((distance - min_distance) / (max_distance - min_distance) * max_velocity, 0.0f, max_velocity);

    // Escalar la velocidad de rotación según el ángulo
    // A medida que se alinea con la persona, la velocidad de rotación disminuye
    float max_rotation_velocity = params.MAX_ROT_SPEED;  // Velocidad máxima de rotación
    float min_rotation_threshold = params.MIN_ROT_THRESHOLD;  // Umbral mínimo para la rotación

    // La velocidad de rotación disminuye a medida que el error angular se reduce
    float angle_error = angle_to_person;
    float rotation_velocity = 0.0f;

    if (std::fabs(angle_error) > min_rotation_threshold) {
        rotation_velocity = std::clamp(angle_error * (params.MAX_ROT_SPEED * 1.4f) , -max_rotation_velocity , max_rotation_velocity);
    }

    // Ajustar la velocidad del robot
    omnirobot_proxy->setSpeedBase(velocity, 0, rotation_velocity);  // Avanzar y girar simultáneamente

    return RetVal(STATE::TRACK, velocity, angle_to_person);  // Continuar tracking
}

/*
SpecificWorker::RetVal SpecificWorker::track(const RoboCompVisualElementsPub::TObject &person)
{
    qDebug() << "TRACK MODE";

    float person_x = std::stof(person.attributes.at("x_pos"));
    float person_y = std::stof(person.attributes.at("y_pos"));

    auto distance = std::hypot(person_x, person_y);
    lcdNumber_dist_to_person->display(distance);
    float angle_to_person = atan2(person_x, person_y);
    qDebug() << "Angle to person: " << angle_to_person;

    if (distance < params.PERSON_MIN_DIST) {
        omnirobot_proxy->setSpeedBase(0, 0, 0);
        return RetVal(STATE::WAIT, 0.f, 0.f);
    }

    const double xset = 0.8;
    const double yset = 0.5;
    double s = -(xset * xset) / log(yset);
    auto gaussian_break = [s](float x) -> float {
        return static_cast<float>(exp(-x * x / s));
    };

    float max_rotation_velocity = params.MAX_ROT_SPEED * 0.5;
    float rotation_gain = 0.8;
    float rotation_velocity = std::clamp(angle_to_person * max_rotation_velocity * rotation_gain, -max_rotation_velocity, max_rotation_velocity);

    float initial_max_speed = 300.0f;
    float adjusted_max_adv_speed = std::min(params.MAX_ADV_SPEED, static_cast<float>(initial_max_speed + distance * 0.3));


    float advance_speed = adjusted_max_adv_speed * gaussian_break(rotation_velocity / max_rotation_velocity);

    omnirobot_proxy->setSpeedBase(advance_speed, 0, rotation_velocity);

    return RetVal(STATE::TRACK, advance_speed, rotation_velocity);
}

*/

SpecificWorker::RetVal SpecificWorker::search()
{
    qDebug() << "SEARCH MODE";

    // Leer y procesar los datos del LiDAR
    auto lidar_data = read_lidar_bpearl();
    if (lidar_data.points.empty()) {
        qWarning() << "SEARCH: No LIDAR data available";
        return RetVal(STATE::SEARCH, 0.f, -params.MAX_ROT_SPEED * 0.3);  // Gira lentamente en su lugar si no hay datos de LiDAR
    }

    // Girar sobre su eje lentamente en busca de la persona
    float slow_rotation_speed = -params.MAX_ROT_SPEED * 0.3;  // Velocidad de rotación lenta
    omnirobot_proxy->setSpeedBase(0, 0, slow_rotation_speed);  // Girar en su lugar

    return RetVal(STATE::SEARCH, 0.f, slow_rotation_speed);  // Mantiene el estado SEARCH con giro lento
}




//
SpecificWorker::RetVal SpecificWorker::wait(const RoboCompVisualElementsPub::TObject &person)
{
    //qDebug() << __FUNCTION__ ;
    // check if the person is further than a threshold
    if(std::hypot(std::stof(person.attributes.at("x_pos")), std::stof(person.attributes.at("y_pos"))) > params.PERSON_MIN_DIST + 100)
        return RetVal(STATE::TRACK, 0.f, 0.f);

    return RetVal(STATE::WAIT, 0.f, 0.f);

}
/**
 * @brief Determines the robot's behavior when following a wall.
 *
 * This method analyzes the filtered points to determine the robot's behavior when following a wall.
 * It first checks if the robot is about to crash into an obstacle, in which case it stops and changes
 * the state to TURN. If no obstacle is detected, it then calculates the distance to the wall on the
 * robot's side and computes the necessary speed and rotation to maintain a safe distance from the wall.
 *
 * @param filtered_points A vector containing points with distance information used for making navigation decisions.
 * @returns A tuple containing the next state (WALL), and speed values.
 */
 //
SpecificWorker::RetVal SpecificWorker::stop()
{
    //qDebug() << __FUNCTION__ ;
    // Check the status of the pushButton_stop
    if(not pushButton_stop->isChecked())
        return RetVal(STATE::TRACK, 0.f, 0.f);

    return RetVal (STATE::STOP, 0.f, 0.f);
}
/**
 * Draws LIDAR points onto a QGraphicsScene.
 *
 * This method clears any existing graphical items from the scene, then iterates over the filtered
 * LIDAR points to add new items. Each LIDAR point is represented as a colored rectangle. The point
 * with the minimum distance is highlighted in red, while the other points are drawn in green.
 *
 * @param filtered_points A collection of filtered points to be drawn, each containing the coordinates
 *                        and distance.
 * @param scene A pointer to the QGraphicsScene where the points will be drawn.
 */
void SpecificWorker::draw_lidar(auto &filtered_points, QGraphicsScene *scene)
{
    static std::vector<QGraphicsItem*> items;   // store items so they can be shown between iterations

    // remove all items drawn in the previous iteration
    for(auto i: items)
    {
        scene->removeItem(i);
        delete i;
    }
    items.clear();

    auto color = QColor(Qt::darkGreen);
    auto brush = QBrush(QColor(Qt::darkGreen));
    for(const auto &p : filtered_points)
    {
        auto item = scene->addRect(-50, -50, 100, 100, color, brush);
        item->setPos(p.x, p.y);
        items.push_back(item);
    }
}
/**
 * @brief Calculates the index of the closest lidar point to the given angle.
 *
 * This method searches through the provided list of lidar points and finds the point
 * whose angle (phi value) is closest to the specified angle. If a matching point is found,
 * the index of the point in the list is returned. If no point is found that matches the condition,
 * an error message is returned.
 *
 * @param points The collection of lidar points to search through.
 * @param angle The target angle to find the closest matching point.
 * @return std::expected<int, string> containing the index of the closest lidar point if found,
 * or an error message if no such point exists.
 */
void SpecificWorker::draw_person(RoboCompVisualElementsPub::TObject &person, QGraphicsScene *scene) const
{
    static std::vector<QGraphicsItem*> items;
    // remove all items drawn in the previous iteration
    for(auto i: items)
    {
        scene->removeItem(i);
        delete i;
    }
    items.clear();

    // draw a circle around the person
    float radius = 300;
    auto person_draw = scene->addEllipse(-radius, -radius, radius*2, radius*2, QPen(Qt::magenta, 30));
    person_draw->setPos(std::stof(person.attributes["x_pos"]), std::stof(person.attributes["y_pos"]));
    items.push_back(person_draw);

    // draw a radius inside the ellipse to indicate the person's orientation
    auto x = std::stof(person.attributes.at("x_pos"));
    auto y = std::stof(person.attributes.at("y_pos"));
    auto angle = std::stof(person.attributes.at("orientation")) + M_PI;
    auto item_radius = scene->addLine(QLineF(QPointF(x, y),
                                                                    QPointF( x - radius * sin(angle),y + radius * cos(angle))),
                                                         QPen(Qt::magenta, 20));
    items.push_back(item_radius);

    // draw a line from the robot to the person circle but ending on the circunference. The end point is the exterior of the circle
    // I need a line from the robot to the person x,y but it has to be 300mm shorter
    auto len = std::hypot(x, y);
    auto item_line = scene->addLine(QLineF(QPointF(0.f, 0.f),
                                                                   QPointF((len -radius) *x/len, (len - radius)*y/len )),
                                                           QPen(Qt::magenta, 20));
    items.push_back(item_line);
}
std::expected<int, string> SpecificWorker::closest_lidar_index_to_given_angle(const auto &points, float angle)
{
    // search for the point in points whose phi value is closest to angle
    auto res = std::ranges::find_if(points, [angle](auto &a){ return a.phi > angle;});
    if(res != std::end(points))
        return std::distance(std::begin(points), res);
    else
        return std::unexpected("No closest value found in method <closest_lidar_index_to_given_angle>");
}
void SpecificWorker::draw_path_to_person(const auto &points, QGraphicsScene *scene)
{
    if(points.empty())
        return;

    // remove all items drawn in the previous iteration
    static std::vector<QGraphicsItem*> items;
    for(auto i: items)
    {
        scene->removeItem(i);
        delete i;
    }
    items.clear();

    // draw the path as a series of lines with dots in between
    for (auto i : iter::range(0UL, points.size() - 1))
    {
        auto line = scene->addLine(QLineF(QPointF(points[i].x(), points[i].y()), QPointF(points[i+1].x(), points[i+1].y())),
                                   QPen(Qt::blue, 40));
        items.push_back(line);
        auto dot = scene->addEllipse(-30, -30, 60, 60, QPen(Qt::darkBlue, 40));
        dot->setPos(points[i].x(), points[i].y());
       items.push_back(dot);
    }
}

//////////////////////////////////////////////////////////////////
/// SUBSCRIPTIONS (runs in a different thread)
//////////////////////////////////////////////////////////////////
//SUBSCRIPTION to setVisualObjects method from VisualElementsPub interface. This is called in a different thread.
void SpecificWorker::VisualElementsPub_setVisualObjects(RoboCompVisualElementsPub::TData data)
{
    // std::cout << "VisualElements_setVisualObjects" << std::endl;
    //    for(auto object : data.objects)
    //        std::cout << "Object type: " << object.id << std::endl;
    //    qDebug() << "Size: " << data.objects.size();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    buffer.put<0>(std::move(data), timestamp); // inserts the laser data value to the queue 0.
}

//////////////////////////////////////////////////////////////////
/// AUXILIARY FUNCTIONS
//////////////////////////////////////////////////////////////////
void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
	//computeCODE
	//
	//if (SUCCESSFUL)
    //  emmit goToRestore()
}
//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
	//computeCODE
	//Restore emergency component

}
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}



/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)
// this->lidar3d_proxy->getLidarDataArrayProyectedInImage(...)
// this->lidar3d_proxy->getLidarDataProyectedInImage(...)
// this->lidar3d_proxy->getLidarDataWithThreshold2d(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// this->omnirobot_proxy->correctOdometer(...)
// this->omnirobot_proxy->getBasePose(...)
// this->omnirobot_proxy->getBaseState(...)
// this->omnirobot_proxy->resetOdometer(...)
// this->omnirobot_proxy->setOdometer(...)
// this->omnirobot_proxy->setOdometerPose(...)
// this->omnirobot_proxy->setSpeedBase(...)
// this->omnirobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

/**************************************/
// From the RoboCompVisualElements you can call this methods:
// this->visualelements_proxy->getVisualObjects(...)
// this->visualelements_proxy->setVisualObjects(...)

/**************************************/
// From the RoboCompVisualElements you can use this types:
// RoboCompVisualElements::TRoi
// RoboCompVisualElements::TObject
// RoboCompVisualElements::TObjects

// Instantiate the random number generator and distribution
//    static std::mt19937 gen(rd());
//    static std::uniform_int_distribution<int> dist(0, 1);
//    static bool first_time = true;
//    static int sign = 1;