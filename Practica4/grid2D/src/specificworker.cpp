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
		// Viewer
        viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
        auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
        robot_draw = r;
        viewer->setStyleSheet("background-color: lightGray;");
        this->resize(800, 700);

		std::cout << "Datos Lidar dibujados correctamente en el lienzo." << std::endl;


		for (int i = 0; i < SIZE; ++i)
		{
			for (int j = 0; j < SIZE; ++j)
			{
				// Inicializamos cada celda
				grid[i][j].state = State::FREE; // La celda está libre al principio
				grid[i][j].item = viewer->scene.addRect(i * 100, j * 100, 100, 100, QPen(), QBrush(Qt::lightGray)); // Dibujar el rectángulo
				grid[i][j].item->setPos(i * 100, j * 100); // Posicionamos la celda en la escena
			}
		}
		std::cout << "Cuadrícula inicializada y dibujada correctamente." << std::endl;
		connect(viewer, SIGNAL(new_mouse_coordinates(QPointF)), this, SLOT(new_mouse_coordinates(QPointF)));

		this->setPeriod(STATES::Compute, 100);
		//this->setPeriod(STATES::Emergency, 500);
	}
}

void SpecificWorker::compute()
{
    std::cout << "Compute worker" << std::endl;
	//computeCODE
	//QMutexLocker locker(mutex);
	//try
	//{
	  //camera_proxy->getYImage(0,img, cState, bState);
        //if (img.empty())
          //  emit goToEmergency()
	  //memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	  //searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	  //std::cout << "Error reading from Camera" << e << std::endl;
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

void SpecificWorker::new_mouse_coordinates(QPointF punto) {
	qDebug() << "Coordenadas: " << punto;
	//Comprobar primero que funciona y luego añadir dijkstra en este metodo
}


RoboCompGrid2D::Result SpecificWorker::Grid2D_getPaths(RoboCompGrid2D::TPoint source, RoboCompGrid2D::TPoint target)
{
	// Transformar coordenadas del mundo a índices de la cuadrícula
	auto [i_start, j_start] = fromWorldToGrid(source.x, source.y);
	auto [i_end, j_end] = fromWorldToGrid(target.x, target.y);

	// Verificar que las coordenadas están dentro de los límites de la cuadrícula
	if (i_start < 0 || i_start >= SIZE || j_start < 0 || j_start >= SIZE ||
		i_end < 0 || i_end >= SIZE || j_end < 0 || j_end >= SIZE)
	{
		return { {}, QDateTime::currentMSecsSinceEpoch(), "Source or target out of bounds", false };
	}

	// Implementar el algoritmo de Dijkstra
	std::vector<std::tuple<int, int>> path = dijkstra({i_start, j_start}, {i_end, j_end});

	if (path.empty())
	{
		return { {}, QDateTime::currentMSecsSinceEpoch(), "No valid path found", false };
	}

	// Convertir el camino de índices de cuadrícula a coordenadas del mundo
	RoboCompGrid2D::TPath result_path;
	for (const auto &[i, j] : path)
	{
		auto [x, y] = fromGridToWorld(i, j);
		result_path.push_back({x, y, params.TILE_SIZE / 2}); // El radio puede ser la mitad del tamaño del tile
	}

	return {result_path, QDateTime::currentMSecsSinceEpoch(), "", true};
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
		// Parametrización de línea
		float S = std::hypot(point.x(), point.y()) / params.TILE_SIZE; // Número de pasos
		float delta = 1.0f / S;

		for (float k = 0.0f; k <= 1.0f; k += delta)
		{
			float x = k * point.x();
			float y = k * point.y();

			auto [i, j] = fromWorldToGrid(x, y);

			if (i >= 0 && i < SIZE && j >= 0 && j < SIZE)
			{
				grid[i][j].state = State::FREE; // Intermedias como libres
				grid[i][j].item->setBrush(QBrush(Qt::white));
			}
		}

		// Marcar la última celda como ocupada
		auto [i, j] = fromWorldToGrid(point.x(), point.y());
		if (i >= 0 && i < SIZE && j >= 0 && j < SIZE)
		{
			grid[i][j].state = State::OCCUPIED;
			grid[i][j].item->setBrush(QBrush(Qt::red));
		}

		//IMPORTANTE
		//Añadir que los vecinos estén ocupados y a rojo
	}
}

std::vector<std::tuple<int, int>> SpecificWorker::dijkstra(std::tuple<int, int> start, std::tuple<int, int> end)
{
    using Node = std::tuple<int, int>;
    using DistanceNode = std::pair<int, Node>;

    // Verificar si las posiciones inicial y final son válidas
    const auto &[start_i, start_j] = start;
    const auto &[end_i, end_j] = end;

    if (start_i < 0 || start_i >= SIZE || start_j < 0 || start_j >= SIZE ||
        end_i < 0 || end_i >= SIZE || end_j < 0 || end_j >= SIZE)
    {
        qWarning() << "Start or end point is out of bounds.";
        return {};
    }

    if (grid[start_i][start_j].state == State::OCCUPIED ||
        grid[end_i][end_j].state == State::OCCUPIED)
    {
        qWarning() << "Start or end point is occupied.";
        return {};
    }

    // Direcciones posibles (arriba, abajo, izquierda, derecha)
    const std::vector<std::tuple<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}};

    // Distancias mínimas desde el nodo inicial
    std::unordered_map<Node, int, TupleHash> min_distance;
    std::unordered_map<Node, Node, TupleHash> previous;

    // Inicializar todas las distancias como "infinito"
    for (int i = 0; i < SIZE; ++i)
    {
        for (int j = 0; j < SIZE; ++j)
        {
            Node node = {i, j};
            min_distance[node] = std::numeric_limits<int>::max();
        }
    }

    // La distancia al nodo inicial es 0
    Node start_node = {start_i, start_j};
    min_distance[start_node] = 0;

    // Cola de prioridad para procesar los nodos
    std::priority_queue<DistanceNode, std::vector<DistanceNode>, std::greater<>> priority_queue;
    priority_queue.push({0, start_node});

    // Algoritmo de Dijkstra
    while (!priority_queue.empty())
    {
        auto [current_distance, current_node] = priority_queue.top();
        priority_queue.pop();

        // Si llegamos al objetivo, detener
        if (current_node == std::make_tuple(end_i, end_j))
            break;

        const auto &[current_i, current_j] = current_node;

        // Explorar vecinos
        for (const auto &[di, dj] : directions)
        {
            int neighbor_i = current_i + di;
            int neighbor_j = current_j + dj;

            // Verificar si el vecino está dentro de los límites
            if (neighbor_i < 0 || neighbor_i >= SIZE || neighbor_j < 0 || neighbor_j >= SIZE)
                continue;

            Node neighbor = {neighbor_i, neighbor_j};

            // Ignorar vecinos ocupados
            if (grid[neighbor_i][neighbor_j].state == State::OCCUPIED)
                continue;

            // Calcular la nueva distancia
            int new_distance = current_distance + 1; // Costo uniforme

            if (new_distance < min_distance[neighbor])
            {
                min_distance[neighbor] = new_distance;
                previous[neighbor] = current_node;
                priority_queue.push({new_distance, neighbor});
            }
        }
    }

    // Reconstruir el camino desde el objetivo al inicio
    std::vector<Node> path;
    Node current = {end_i, end_j};

    while (current != start_node)
    {
        path.push_back(current);

        if (previous.find(current) == previous.end())
        {
            qWarning() << "No path found.";
            return {};
        }

        current = previous[current];
    }
    path.push_back(start_node);
    std::reverse(path.begin(), path.end());

    return path;
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

