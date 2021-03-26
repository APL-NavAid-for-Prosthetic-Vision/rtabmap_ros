/*
*   Copyright 2020 The Johns Hopkins University
*   Applied Physics Laboratory.  All rights reserved.
*
*/

#include "rtabmap_ros/rviz/TableSemanticsPanel.h"

#include <rviz/panel.h>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QStringList>
#include <QLabel>
#include <QLineEdit>
#include <QHeaderView>

namespace rtabmap_ros
{

TableSemanticsPanel::TableSemanticsPanel( QWidget* parent ) :
    rviz::Panel( parent )
{
    QHBoxLayout* classId_file_layout = new QHBoxLayout;
    classId_file_layout->addWidget( new QLabel( "ClassID structure file:" ));
    classID_file_path_editor_ = new QLineEdit;
    classId_file_layout->addWidget( classID_file_path_editor_ );

    m_pTableWidget_ = new QTableWidget(10, 3, this);
    QStringList m_TableHeader;
    m_TableHeader<<"ClassID"<<"Name"<<"Color";
    m_pTableWidget_->setHorizontalHeaderLabels(m_TableHeader);
    m_pTableWidget_->verticalHeader()->setVisible(false);
    m_pTableWidget_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_pTableWidget_->setSelectionBehavior(QAbstractItemView::SelectRows);
    //m_pTableWidget_->setShowGrid(false);
    //m_pTableWidget_->setStyleSheet("QTableView {selection-background-color: red;}");

    QVBoxLayout* layout = new QVBoxLayout;
    layout->addLayout(classId_file_layout);
    layout->addWidget(m_pTableWidget_);
    setLayout(layout);
}

TableSemanticsPanel::~TableSemanticsPanel() 
{
    delete m_pTableWidget_;
}

void TableSemanticsPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  
}


void TableSemanticsPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  
}

} // end namespace 

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rtabmap_ros::TableSemanticsPanel, rviz::Panel )