cmake_minimum_required(VERSION 2.6)
PROJECT(  gen3_mission_control )

INCLUDE( /opt/robocomp/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior  )
