#ifndef RVIZ_MAP_SAVE_BUTTON
#define RVIZ_MAP_SAVE_BUTTON

#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>
#include <QPushButton>

namespace rviz_map_save
{

class MapSaveButton: public rviz::Panel
{
Q_OBJECT

public:
    MapSaveButton(QWidget *parent = 0);

    //Override
    virtual void save(rviz::Config conifg) const;

    //Override
    virtual void load(const rviz::Config &Config);

public Q_SLOTS:

    void map_save();

protected:

    void map_save_service();

protected:

    QPushButton *_map_save_b;
    ros::NodeHandle _nh;
};

}

#endif
