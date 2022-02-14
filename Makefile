include Mk/libs_flags.mk

OBJS=timer.o common.o esdf_map.o point_speed3d.o point_rotation3d.o dijkstra.o reference_samples.o vel_search_graph.o motion_primitive.o prm.o topological_prm.o sst.o drone.o main.o
TARGET=main

OBJ_DIR=obj

include Mk/recipe.mk
