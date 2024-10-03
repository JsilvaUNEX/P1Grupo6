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

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#define HIBERNATION_ENABLED

#include <genericworker.h>
#include <tuple>  // Para el uso de std::tuple
#include <vector>  // Para el uso de std::vector

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
public:
    SpecificWorker(TuplePrx tprx, bool startup_check);
    ~SpecificWorker();
    bool setParams(RoboCompCommonBehavior::ParameterList params);

    public slots:
        void initialize();
    void compute();
    void emergency();
    void restore();
    int startup_check();

private:
    bool startup_check_flag;

    // Estados del robot
    enum class State { IDLE, FORWARD, TURN, FOLLOW_WALL, SPIRAL };
    State current_state;

    // Estrategias de comportamiento del robot
    State FORWARD_method(const RoboCompLidar3D::TData &ldata);
    State TURN_method(const RoboCompLidar3D::TData &ldata);
};

#endif // SPECIFICWORKER_H
