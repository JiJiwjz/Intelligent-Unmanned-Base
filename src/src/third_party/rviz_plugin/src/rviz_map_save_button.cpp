#include "rviz_map_save_button.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QString>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>

namespace rviz_map_save
{

MapSaveButton::MapSaveButton(QWidget* parent)
    :rviz::Panel(parent)
{
    QVBoxLayout *root = new QVBoxLayout;
    _map_save_b = new QPushButton("Save Map");

    QHBoxLayout *label_layout = new QHBoxLayout;

    root->addWidget(_map_save_b);
    root->addLayout(label_layout);
    setLayout(root);

    connect( _map_save_b, SIGNAL(clicked()),this,SLOT(map_save()));
}

void MapSaveButton::map_save()
{
    boost::thread thread(boost::bind(&MapSaveButton::map_save_service,this));
}


//Override
void MapSaveButton::save(rviz::Config config) const
{
    rviz::Panel::save(config);
    config.mapSetValue("map_save_button",_map_save_b->text());
}

//Override
void MapSaveButton::load(const rviz::Config& config)
{
    rviz::Panel::load(config);
    QString start_s;
    if(config.mapGetString("map_save_button",&start_s))
    {
        _map_save_b->setText(start_s);
    }
}

void MapSaveButton::map_save_service()
{
    ROS_INFO("start save map!");
    _map_save_b->setText("Saving ...");
    _map_save_b->setEnabled(false);
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response resp;
    if(!ros::service::exists("map_save",true))
    {
        _map_save_b->setEnabled(true);
        _map_save_b->setText("Save Map");
        ROS_WARN("no such service!");
        return;
    }
    bool success = ros::service::call("map_save",req,resp);
    if(success)
    {
        _map_save_b->setEnabled(true);
        _map_save_b->setText("Save Map");
    }
    else
    {
        _map_save_b->setEnabled(true);
        _map_save_b->setText("Save Map"); 
    }
}

}
// A rviz plugin statement
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_map_save::MapSaveButton,rviz::Panel )
