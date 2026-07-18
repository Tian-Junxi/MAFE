#!/usr/bin/env bash
# 循环发布四个电机转速，用于测试预测节点。

while true; do
  rostopic pub -1 /motor_speeds std_msgs/Float64MultiArray "data: [221, 265, 307, 184]"
  sleep 1
  rostopic pub -1 /motor_speeds std_msgs/Float64MultiArray "data: [330, 318, 364, 304]"
  sleep 1
  rostopic pub -1 /motor_speeds std_msgs/Float64MultiArray "data: [700, 730, 760, 790]"
  sleep 1
done
