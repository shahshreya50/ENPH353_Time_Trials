#!/usr/bin/env python3

from move_relative import respawn_model

if __name__ == '__main__':
    # No need to init_node when gazebo is already runing
    # it runs a little faster without init_node anyway
    model_name = 'B1'
    respawn_model(model_name)