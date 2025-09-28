#ifndef RVIZ_NAVIGATION_BUTTON
#define RVIZ_NAVIGATION_BUTTON

#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>
#include <QPushButton>

namespace rviz_navigation
{

class NavigationButton: public rviz::Panel
{
Q_OBJECT

public:
    NavigationButton(QWidget *parent = 0);

    //Override
    virtual void save(rviz::Config conifg) const;

    //Override
    virtual void load(const rviz::Config &Config);

public Q_SLOTS:

    //start navigation
    void start();


    //stop navigation
    void stop();

    //  navigaiton goals
    void clear();

protected:

    void start_nav_service();
    void stop_nav_service();
    void clear_nav_service();

protected:

    QPushButton *_start_b;
    QPushButton *_delete_b;
    QPushButton *_stop_b;
    bool _doing_navigation;
    ros::NodeHandle _nh;
};

}

#endif
