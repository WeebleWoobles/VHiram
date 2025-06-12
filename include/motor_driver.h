#pragma once
#include "init.h"



// FOR SETTING SPEED all should be "+ " if forward, "-" if backward
// Left and Right motors are configured so the go the same direction in code, so if you want oposing one is + other is -

void init_motor();
void MOTOR_set_speed(float speed);
void MOTOR_set_left(float speed); // sets speed and dirrection
void MOTOR_set_right(float speed);
