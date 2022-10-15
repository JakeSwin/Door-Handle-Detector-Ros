#!/bin/bash
rocker --home --user --x11 --privileged tiago_image:latest --devices /dev/dri --volume ~/ros_workspaces:/workspaces --network host
