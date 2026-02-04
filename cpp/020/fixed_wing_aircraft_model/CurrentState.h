#ifndef CURRENTSTATE_H
#define CURRENTSTATE_H 

#include <matrix/math.hpp>

class CurrentState
{

    public :
        CurrentState()=default ; 
        ~ CurrentState()=default ; 

        // get 함수 구현
            double get_north_position() const { return _north_position; }
            double get_east_position() const{ return _east_position; }
            double get_down() const{ return _down; }

            double get_roll() const{ return _roll; }
            double get_pitch() const{ return _pitch; }
            double get_yaw() const{ return _yaw; }
            double get_speed() const{ return _speed; }

            double get_p() const{ return _p; }
            double get_q() const{ return _q; }
            double get_r() const{ return _r; }
            double get_at() const{ return _at; }

            matrix::Vector<double, 11> get_total_state() const {
                        matrix::Vector<double, 11> temp_state;
                        temp_state(0) = _north_position;
                        temp_state(1) = _east_position;
                        temp_state(2) = _down;
                        temp_state(3) = _roll;
                        temp_state(4) = _pitch;
                        temp_state(5) = _yaw;
                        temp_state(6) = _speed;
                        temp_state(7) = _p;
                        temp_state(8) = _q;
                        temp_state(9) = _r;
                        temp_state(10) = _at;
                        return temp_state;
                    }
           

        //set 함수 구현
        
            void set_north_position(double north_position) { _north_position = north_position; }
            void set_east_position(double east_position)   { _east_position = east_position; }
            void set_down(double down)                 { _down = down; } 

            // 자세 정보 (Attitude) 
            void set_roll(double roll)   { _roll = roll; }
            void set_pitch(double pitch) { _pitch = pitch; }
            void set_yaw(double yaw)     { _yaw = yaw; }
            void set_speed(double speed) { _speed = speed; }    /* Airspeed */


            // 동역학 정보 (Dynamics) 
            void set_p(double p)         { _p = p; }            /* Roll Rate */
            void set_q(double q)         { _q = q; }        
            void set_r(double r)         {_r = r;  } 
            void set_at(double at)       {_at = at;} 


            void set_total_state(const matrix::Vector<double, 11>& state) {
                    _total_state = state;
                    _north_position = state(0);
                    _east_position = state(1);
                    _down = state(2);
                    _roll = state(3);
                    _pitch = state(4);
                    _yaw = state(5);
                    _speed = state(6);
                    _p = state(7);
                    _q = state(8);
                    _r = state(9);
                    _at = state(10);
                }




   private:
       // private 변수 선언 및 초기화 진행

        double _north_position=0.0;
        double _east_position=0.0;
        double _down=0.0;

        double _roll=0.0;
        double _pitch=0.0;
        double _yaw=0.0;

        double _speed=0.0;

        double _p=0.0;
        double _q=0.0;
        double _r=0.0;
        double _at=0.0;

        matrix::Vector<double, 11 > _total_state ;




};

#endif // 