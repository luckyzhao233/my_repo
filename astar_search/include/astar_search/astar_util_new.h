/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ASTAR_UTIL_H
#define ASTAR_UTIL_H

#include "ros_autoware.h"
enum class STATUS : uint8_t
{
  NONE,
  OPEN,
  CLOSED,
  OBS
};

struct AstarNode
{
  double x, y, theta;            // Coordinate of each node
  STATUS status = STATUS::NONE;  // NONE, OPEN, CLOSED or OBS
  double gc = 0;                 // Actual cost
  double hc = 0;                 // heuristic cost
  double move_distance = 0;      // actual move distance
  bool back;                     // true if the current direction of the vehicle is back
  AstarNode* parent = NULL;      // parent node
};

struct WaveFrontNode
{
  int index_x;
  int index_y;
  double hc;

  WaveFrontNode();
  WaveFrontNode(int x, int y, double cost);
};

struct NodeUpdate
{
  double shift_x;
  double shift_y;
  double rotation;
  double step;
  int index_theta;
  bool curve;
  bool back;
};

// For open list and goal list
struct SimpleNode
{
  int index_x;
  int index_y;
  int index_theta;
  double cost;

  bool operator>(const SimpleNode& right) const
  {
    return cost > right.cost;
  }

  SimpleNode();
  SimpleNode(int x, int y, int theta, double gc, double hc);
};


inline double calcDistance(double x1, double y1, double x2, double y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

inline double modifyTheta(double theta)
{
  if (theta < 0.0)
    return theta + 2.0 * M_PI;
  if (theta >= 2.0 * M_PI)
    return theta - 2.0 * M_PI;

  return theta;
}

inline GeometryMsgsPose transformPose(const GeometryMsgsPose& pose, const TfTransform& tf)
{
  // Convert ROS pose to TF pose
  TfPose tf_pose;
  poseMsgToTF(pose, tf_pose);

  // Transform pose
  tf_pose = tf * tf_pose;

  // Convert TF pose to ROS pose
  GeometryMsgsPose ros_pose;
  poseTFToMsg(tf_pose, ros_pose);

  return ros_pose;
}

inline WaveFrontNode getWaveFrontNode(int x, int y, double cost)
{
  WaveFrontNode node(x, y, cost);

  return node;
}

inline Point calcRelativeCoordinate(GeometryMsgsPose pose, Point point)
{
  TfTransform transform;
  poseMsgToTF(pose, transform);
  transform = transform.inverse();

  point = transform * point;
  Point point_msg;
  pointTFToMsg(point, point_msg);

  return point_msg;
}

inline double calcDiffOfRadian(double a, double b)
{
  double diff = std::fmod(std::fabs(a - b), 2.0 * M_PI);
  if (diff < M_PI)
    return diff;
  else
    return 2.0 * M_PI - diff;
}

inline GeometryMsgsPose xytToPoseMsg(double x, double y, double theta)
{
  GeometryMsgsPose p;
  p.position.x = x;
  p.position.y = y;
  p.orientation = createQuaternionMsgFromYaw(theta);

  return p;
}

#endif
