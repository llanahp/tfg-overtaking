/*
 * Copyright (c) 2020 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see https://opensource.org/licenses/MIT.
 */
#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPixmap>
#include <QProgressBar>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <cstdio>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Bool.h>

#include "t4ac_panel.hpp"

PLUGINLIB_EXPORT_CLASS(t4ac_rviz_plugins::t4acPanel, rviz::Panel)

namespace t4ac_rviz_plugins 
{
  t4acPanel::t4acPanel(QWidget *parent)
    :rviz::Panel(parent)
    {
      QVBoxLayout *layout = new QVBoxLayout;
      QHBoxLayout *synchronous_layout = new QHBoxLayout;
      QFormLayout *startLayout = new QFormLayout;

      QPixmap pixmapStart(":/icons/play.png");
      QIcon iconStart(pixmapStart);
      mStartButton = new QPushButton(iconStart, "");
      connect(mStartButton, SIGNAL(released()), this, SLOT(start()));

      synchronous_layout->addWidget(mStartButton);
      startLayout->addRow("Start Navigation", synchronous_layout);

      layout->addLayout(startLayout);

      setLayout(layout);
      connect(mStartButton, SIGNAL(released()), this, SLOT(t4acStart()));

      mStartPublisher = mNodeHandle.advertise<std_msgs::Bool>("/t4ac/planning/start", 10);
    }

  void t4acPanel::setSimulationButtonStatus(bool active)
  {
    if (active)
    {
      mStartButton->setDisabled(false);
      mStartButton->setToolTip("Waiting.");
    }
    else
    {
      mStartButton->setDisabled(true);
      mStartButton->setToolTip("Start Navigation.");
    }
  }

  void t4acPanel::t4acStart()
  {
    if (mStartButton)
    {
      std_msgs::Bool start;
      start.data=true;
      mStartPublisher.publish(start);
    }
  }
}
