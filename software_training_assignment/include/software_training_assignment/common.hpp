#ifndef COMMON_H
#define COMMON_H

struct Position {
    float x;
    float y;
    float theta;
};

extern Position init_moving_turtle_pos;
extern Position stationary_turtle_pos;
extern Position current_moving_turtle_pos;

#endif