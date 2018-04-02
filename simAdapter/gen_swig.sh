#!/bin/bash

swig3.0 -Wall -v -debug-classes -c++ -python -o $1terrainRLAdapter.cpp $1terrainRLAdapter.swig

