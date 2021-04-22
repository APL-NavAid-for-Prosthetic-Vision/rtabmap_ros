/*
*   Copyright 2021 The Johns Hopkins University
*   Applied Physics Laboratory.  All rights reserved.
*
*/
#ifndef TABLE_SEMANTICS_PANEL_H
#define TABLE_SEMANTICS_PANEL_H

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rviz/panel.h>

#include <QTableWidget>
#include <QLineEdit>
#include <QString>
#include <QVBoxLayout>

#include <map>

#include <rtabmap/core/SemanticOctoMap.h>

#include <rtabmap_ros/SemanticClassIdElement.h>
#include <rtabmap_ros/SemanticClassIdMap.h>

#include <opencv2/core/types.hpp>

#endif


namespace rtabmap_ros
{

class TableSemanticsPanel: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
    TableSemanticsPanel( QWidget* parent = 0 );
    ~TableSemanticsPanel();

    // Now we declare overrides of rviz::Panel functions for saving and
    // loading data from the config file.  Here the data is the
    // topic name.
    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

// Next come a couple of public Qt slots.
public Q_SLOTS:
    // The control area, Display, sends its output to a Qt signal
    // for ease of re-use, so here we declare a Qt slot to receive it.
    void updateTable();
    void setTable( const QString& topic );
    void updateQTableWidget();

private:
    void updateSemanticTable(const rtabmap_ros::SemanticClassIdMapConstPtr& msg);
    std::string getOccupancyType_str(int type);

protected:
    // The ROS node handle.
    ros::NodeHandle nh_;
    ros::Subscriber semantic_classid_map_sub_;
    QTableWidget* m_pTableWidget_;
    QVBoxLayout* layout_;

    QLineEdit* classID_file_path_editor_;
    QString classID_file_path_; 
    std::map< std::string, std::map<unsigned int, std::string> > modelNameIdMap_;
    std::map<unsigned int, std::pair< int, cv::Point3f> > semanticClassIdMap_;   // [label id : {OccupancyType, maskColor}]
    std::map<unsigned int, cv::Point3f> modelMaskIdColorMap_;                   // [label id : semanticMaskColor]
};

} // end of namespace 

#endif