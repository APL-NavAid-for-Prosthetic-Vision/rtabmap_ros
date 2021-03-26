/*
*   Copyright 2020 The Johns Hopkins University
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

#include <map>

//#include <opencv2/core/types.hpp>

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
//public Q_SLOTS:
    // The control area, Display, sends its output to a Qt signal
    // for ease of re-use, so here we declare a Qt slot to receive it.
    //void updateTable( std::map<unsigned int, cv::Point3f> modelMaskIdColorMap );


protected:
    QTableWidget* m_pTableWidget_;

    QLineEdit* classID_file_path_editor_;
    QString classID_file_path_;


};

} // end of namespace 

#endif