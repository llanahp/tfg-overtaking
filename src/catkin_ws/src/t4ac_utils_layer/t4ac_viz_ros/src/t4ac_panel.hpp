/*
 * Copyright (c) 2020 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see https://opensource.org/licenses/MIT.
 */
#pragma once

#include <ros/ros.h>
#include <rviz/panel.h>

class QLineEdit;
class QPushButton;
class QProgressBar;
class QCheckBox;
class QComboBox;

namespace rviz 
{
  class ViewController;
  class FramePositionTrackingViewController;
}

namespace t4ac_rviz_plugins 
{
  class t4acPanel : public rviz::Panel
  {
    Q_OBJECT
    public:
      t4acPanel(QWidget *parent = 0);

    protected Q_SLOTS:
      void t4acStart();

    protected:

      void setSimulationButtonStatus(bool active);

      QPushButton *mStartButton;

      ros::Publisher mStartPublisher;

      ros::NodeHandle mNodeHandle;
  };

} // end namespace t4ac_rviz_plugins
