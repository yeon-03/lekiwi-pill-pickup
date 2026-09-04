#!/usr/bin/env bash
pkill -f "cartographer_node " 2>/dev/null && echo "cartographer_node 종료" || echo "(없음)"
pkill -f "cartographer_occupancy_grid_node" 2>/dev/null && echo "occupancy_grid_node 종료" || echo "(없음)"
