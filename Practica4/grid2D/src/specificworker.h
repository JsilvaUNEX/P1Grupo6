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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#define HIBERNATION_ENABLED


#include <genericworker.h>
#include <QGraphicsRectItem>
#include <Eigen/Dense>
#include <QGraphicsView>
#include <QGraphicsScene>
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"

#include "Lidar3D.h"
#include <expected>
#include <random>
#include <doublebuffer_sync/doublebuffer_sync.h>
#include <locale>
#include <qcustomplot/qcustomplot.h>
#include <timer/timer.h>


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	RoboCompGrid2D::Result Grid2D_getPaths(RoboCompGrid2D::TPoint source, RoboCompGrid2D::TPoint target);


public slots:
	void initialize();
	void compute();
	void emergency();
	void restore();
	int startup_check();
private:
	bool startup_check_flag;

	struct Params
	{
		float ROBOT_WIDTH = 460;  // mm
		float ROBOT_LENGTH = 480;  // mm
		float MAX_ADV_SPEED = 1900; // mm/s
		float MAX_ROT_SPEED = 2; // rad/s
		float SEARCH_ROT_SPEED = 0.9; // rad/s
		float STOP_THRESHOLD = 700; // mm
		float ADVANCE_THRESHOLD = ROBOT_WIDTH * 3; // mm
		float LIDAR_FRONT_SECTION = 0.2; // rads, aprox 12 degrees
		// person
		float PERSON_MIN_DIST = 800; // mm
		int MAX_DIST_POINTS_TO_SHOW = 300; // points to show in plot
		// lidar
		std::string LIDAR_NAME_LOW = "bpearl";
		std::string LIDAR_NAME_HIGH = "helios";
		QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};
		// control track
		float acc_distance_factor = 2;
		float k1 = 1.1;  // proportional gain for the angle error;
		float k2 = 0.5; // proportional gain for derivative of the angle error;
        // grid
		float TILE_SIZE = 100; // mm
	};

	Params params;

	enum class State
	{
		FREE,     // Celda libre
		OCCUPIED, // Celda ocupada
		UNKNOWN   // Celda desconocida
	};

	struct TCell
	{
		State state;
		QGraphicsRectItem *item;
	};

	// lidar
	std::vector<Eigen::Vector2f> read_lidar_bpearl();
	std::vector<Eigen::Vector2f> read_lidar_helios();

	// draw
	AbstractGraphicViewer *viewer;
	void draw_lidar(auto &filtered_points, QGraphicsScene *scene);
	QGraphicsPolygonItem *robot_draw;

	QGraphicsView* frame;

    //GRID
	static const int SIZE = 50;  // Tamaño del grid (50x50 celdas)
	std::array<std::array<TCell, SIZE>, SIZE> grid;  // Matriz de celdas

	// Coordinates
	void transTo2DGrid(const std::vector<Eigen::Vector2f> &lidar_points);

	// Transformations
	inline std::pair<int, int> fromWorldToGrid(float x, float y) const
	{
		int i = std::clamp(static_cast<int>((x + (params.GRID_MAX_DIM.width() / 2)) / params.TILE_SIZE), 0, SIZE - 1);
		int j = std::clamp(static_cast<int>((y + (params.GRID_MAX_DIM.height() / 2)) / params.TILE_SIZE), 0, SIZE - 1);
		return {i, j};
	}

	inline std::pair<float, float> fromGridToWorld(int i, int j) const
	{
		float x = (i * params.TILE_SIZE) - (params.GRID_MAX_DIM.width() / 2);
		float y = (j * params.TILE_SIZE) - (params.GRID_MAX_DIM.height() / 2);
		return {x, y};
	}

	QGraphicsScene* scene;
	QGraphicsView* view;

    //Dijkstra (Ultima parte)

};

#endif
