#include "rviz_navigation_button.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QString>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <boost/thread.hpp>

namespace rviz_navigation
{

NavigationButton::NavigationButton(QWidget* parent)
    :rviz::Panel(parent),
    _doing_navigation(false)
{
    QVBoxLayout *root = new QVBoxLayout;
    _start_b = new QPushButton("Start Navigation");
    _stop_b = new QPushButton("Stop Navigation");
    _delete_b = new QPushButton("Clear Goals");

    QHBoxLayout *label_layout = new QHBoxLayout;

    root->addWidget(_start_b);
    root->addWidget(_stop_b);
    root->addWidget(_delete_b);
    root->addLayout(label_layout);
    setLayout(root);

    connect( _start_b, SIGNAL(clicked()),this,SLOT(start()));
    connect( _stop_b, SIGNAL(clicked()),this,SLOT(stop()));
    connect( _delete_b, SIGNAL(clicked()),this,SLOT(clear()));
}

void NavigationButton::start()
{
    boost::thread thread(boost::bind(&NavigationButton::start_nav_service,this));
}

void NavigationButton::stop()
{
    boost::thread thread(boost::bind(&NavigationButton::stop_nav_service,this));
}

void NavigationButton::clear()
{
    boost::thread thread(boost::bind(&NavigationButton::clear_nav_service,this));
}

//Override
void NavigationButton::save(rviz::Config config) const
{
    rviz::Panel::save(config);
    config.mapSetValue("start_button",_start_b->text());
}

//Override
void NavigationButton::load(const rviz::Config& config)
{
    rviz::Panel::load(config);
    QString start_s;
    if(config.mapGetString("start_button",&start_s))
    {
        _start_b->setText(start_s);
    }
}

void NavigationButton::start_nav_service()
{
    ROS_INFO("start navigation!");
    _start_b->setText("Start ...");
    _start_b->setEnabled(false);
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response resp;
    _doing_navigation = true;
    if(!ros::service::exists("start_navigation",true))
    {
        _start_b->setEnabled(true);
        _start_b->setText("Start Navigation");
        _doing_navigation = false;
        ROS_WARN("no such service!");
        return;
    }
    bool success = ros::service::call("start_navigation",req,resp);
    if(success)
    {
        _start_b->setEnabled(true);
        _start_b->setText("Start Navigation");
        _doing_navigation = false;
    }
    else
    {
        _start_b->setEnabled(true);
        _start_b->setText("Start Navigation");
        _doing_navigation = false;  
    }
}

void NavigationButton::stop_nav_service()
{
    ROS_INFO("stop navigation!");
    _stop_b->setText("Stop ...");
    _stop_b->setEnabled(false);
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response resp;
    if(!ros::service::exists("stop_navigation",true))
    {
        _stop_b->setEnabled(true);
        _stop_b->setText("Stop Navigation");
        ROS_WARN("no such service!");
        return;
    }
    bool success = ros::service::call("stop_navigation",req,resp);
    if(success)
    {
        _stop_b->setEnabled(true);
        _stop_b->setText("Stop Navigation");
    }
    else
    {
        _stop_b->setEnabled(true);
        _stop_b->setText("Stop Navigation");
    }
}

void NavigationButton::clear_nav_service()
{
    ROS_INFO("clear goals!");
    _delete_b->setText("Clear ...");
    _delete_b->setEnabled(false);
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response resp;
    if(!ros::service::exists("clear_goals",true))
    {
        _delete_b->setEnabled(true);
        _delete_b->setText("Clear Goals");
        ROS_WARN("no such service!");
        return;
    }
    bool success = ros::service::call("clear_goals",req,resp);
    if(success)
    {
        _delete_b->setEnabled(true);
        _delete_b->setText("Clear Goals");
    }
    else
    {
        _delete_b->setEnabled(true);
        _delete_b->setText("Clear Goals");
    }
}

}
// A rviz plugin statement
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_navigation::NavigationButton,rviz::Panel )
