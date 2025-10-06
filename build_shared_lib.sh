#!/bin/bash
g++ -fPIC -fvisibility=hidden -ggdb -c -Wall -Wextra serialportlib.cpp
g++ -shared -o libserialportlib.so serialportlib.o
