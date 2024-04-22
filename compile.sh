#!/bin/sh
g++ main.cpp a_star.cpp environment.cpp menus.cpp robot.cpp -c -pedantic-errors
g++ main.o a_star.o environment.o menus.o robot.o -o main
