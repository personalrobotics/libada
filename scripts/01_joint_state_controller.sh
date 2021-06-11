#!/bin/bash

rosservice call /controller_manager/switch_controller "start_controllers:
- 'joint_state_controller'
strictness: 2"
