#ifndef SAMPLE_RVIZ_PLUGINS_MAIN_PANEL_HPP
#define SAMPLE_RVIZ_PLUGINS_MAIN_PANEL_HPP

#ifndef Q_MOC_RUN
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <QtWidgets>
#include "tf_listener.hpp"
#include "drone.hpp"
#endif

namespace stationui_frontend_rviz_plugin
{

class MainPanel : public rviz_common::Panel
{
    Q_OBJECT
public:
    MainPanel(QWidget *parent = nullptr);

    virtual void onInitialize();
    virtual void load(const rviz_common::Config &config);
    virtual void save(rviz_common::Config config) const;
private:
    QHBoxLayout *layout_;
    Drone *drone_panel1_;
    Drone *drone_panel2_;
    Drone *drone_panel3_;

    rclcpp::Node::SharedPtr node_;
    TfListener* tf_listener_;
};

}

#endif // SAMPLE_RVIZ_PLUGINS_MAIN_PANEL_HPP
