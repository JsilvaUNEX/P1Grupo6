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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
	// Uncomment if there's too many debug messages
	// but it removes the possibility to see the messages
	// shown in the console with qDebug()
//	QLoggingCategory::setFilterRules("*.debug=false\n");
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
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }
	

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
		//this->setPeriod(STATES::Emergency, 500);

		for (int i = 0; i < SIZE; ++i)
		{
			for (int j = 0; j < SIZE; ++j)
			{
				// Inicializamos cada celda
				grid[i][j].state = State::FREE; // La celda está libre al principio
				grid[i][j].item = scene->addRect(i * 100, j * 100, 100, 100, QPen(), QBrush(Qt::lightGray)); // Dibujar el rectángulo
				grid[i][j].item->setPos(i * 100, j * 100); // Posicionamos la celda en la escena
			}
		}

		std::cout << "Cuadrícula inicializada y dibujada correctamente." << std::endl;

		// Inicialización del Lidar y la visualización
		// Suponiendo que tienes una función getLidarData() que te devuelve un vector de puntos
        try
        {
            auto lidarData = lidar3d_proxy->getLidarData("bpearl", 0.0f, 2 * M_PI, 1);

            // Dibujar los puntos Lidar en la escena
            for (const auto& point : lidarData.points)
            {
                // Dibujamos cada punto en la escena (adaptar coordenadas si es necesario)
                scene->addEllipse(point.x, point.y, 5, 5, QPen(Qt::red), QBrush(Qt::red));
            }

            std::cout << "Datos Lidar dibujados correctamente en el lienzo." << std::endl;
        }
        catch (const Ice::Exception& e)
        {
            std::cerr << "Error al obtener datos del Lidar: " << e.what() << std::endl;
        }

		// Viewer
        viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
        auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
        robot_draw = r;
        viewer->setStyleSheet("background-color: lightGray;");
        this->resize(800, 700);

		std::cout << "Datos Lidar dibujados correctamente en el lienzo." << std::endl;
	}

}

void SpecificWorker::compute()
{
    std::cout << "Compute worker" << std::endl;
	//computeCODE
	//QMutexLocker locker(mutex);
	//try
	//{
	//  camera_proxy->getYImage(0,img, cState, bState);
    //    if (img.empty())
    //        emit goToEmergency()
	//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	//  searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	//  std::cout << "Error reading from Camera" << e << std::endl;
	//}
	// Leer datos LiDAR
    auto lidar_points = read_lidar_bpearl();

    if (!lidar_points.empty())
    {
        // Transformar puntos LiDAR a la cuadrícula
        transTo2DGrid(lidar_points);
    }
    else
    {
        qWarning() << "No LiDAR points available.";
    }
	
}

std::vector<Eigen::Vector2f> SpecificWorker::read_lidar_bpearl()
{
	try
	{
		auto ldata =  lidar3d_proxy->getLidarData("bpearl", 0, 2*M_PI, 1);
		// filter points according to height and distance
		std::vector<Eigen::Vector2f>  p_filter;
		for(const auto &a: ldata.points)
		{
			if(a.z < 500 and a.distance2d > 200)
				p_filter.emplace_back(a.x, a.y);
		}
		return p_filter;
	}
	catch(const Ice::Exception &e){std::cout << e << std::endl;}
	return {};
}
std::vector<Eigen::Vector2f> SpecificWorker::read_lidar_helios()
{
	try
	{

		auto ldata =  lidar3d_proxy->getLidarData("helios", 0, 2*M_PI, 2);
		// filter points according to height and distance
		std::vector<Eigen::Vector2f> p_filter;
		for(const auto &a: ldata.points)
		{
			if(a.z > 1300 and a.distance2d > 200)
				p_filter.emplace_back(a.x, a.y);
		}



		return p_filter;
	}
	catch(const Ice::Exception &e){std::cout << e << std::endl;}
	return {};
}

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
        item->setPos(p.x(), p.y());
        items.push_back(item);
    }
}

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


RoboCompGrid2D::Result SpecificWorker::Grid2D_getPaths(RoboCompGrid2D::TPoint source, RoboCompGrid2D::TPoint target)
{
#ifdef HIBERNATION_ENABLED
	hibernation = true;
#endif
//implementCODE

}

void SpecificWorker::transTo2DGrid(const std::vector<Eigen::Vector2f> &lidar_points)
{
    // Limpiar el estado actual de la cuadrícula
    for (auto &row : grid)
    {
        for (auto &cell : row)
        {
            cell.state = State::FREE; // Reiniciar todas las celdas como libres
            cell.item->setBrush(QBrush(Qt::lightGray)); // Visualización de celdas libres
        }
    }

    // Procesar puntos LiDAR
    for (const auto &point : lidar_points)
    {
        // Convertir las coordenadas del mundo a índices de cuadrícula
        auto [i, j] = fromWorldToGrid(point.x(), point.y());

        // Asegurarse de que los índices están dentro del rango de la cuadrícula
        if (i >= 0 && i < SIZE && j >= 0 && j < SIZE)
        {
            grid[i][j].state = State::OCCUPIED; // Marcar la celda como ocupada
            grid[i][j].item->setBrush(QBrush(Qt::red)); // Visualización de celdas ocupadas
        }
    }
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
// From the RoboCompGrid2D you can use this types:
// RoboCompGrid2D::TPoint
// RoboCompGrid2D::Result

