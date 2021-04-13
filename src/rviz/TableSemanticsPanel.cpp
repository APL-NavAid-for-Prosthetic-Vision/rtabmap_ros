/*
*   Copyright 2021 The Johns Hopkins University
*   Applied Physics Laboratory.  All rights reserved.
*
*/

#include "rtabmap_ros/rviz/TableSemanticsPanel.h"
#include "rtabmap_ros/utils_mapping.h"

#include <rviz/panel.h>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QStringList>
#include <QString>
#include <QLabel>
#include <QLineEdit>
#include <QHeaderView>
#include <QTableWidgetItem>
#include <QVariant>
#include <QBrush>
#include <QtGlobal>

#include <rtabmap/core/SemanticOctoMap.h>

namespace rtabmap_ros
{

TableSemanticsPanel::TableSemanticsPanel( QWidget* parent ) :
    rviz::Panel( parent )
{
    QHBoxLayout* classId_file_layout = new QHBoxLayout;
    classId_file_layout->addWidget( new QLabel( "ClassID structure file:" ));
    classID_file_path_editor_ = new QLineEdit;
    classId_file_layout->addWidget( classID_file_path_editor_ );

    m_pTableWidget_ = new QTableWidget(10, 4, this);
    QStringList m_TableHeader;
    m_TableHeader<<"ClassID"<<"Name"<<"Color"<<"Type";
    m_pTableWidget_->setHorizontalHeaderLabels(m_TableHeader);
    m_pTableWidget_->verticalHeader()->setVisible(false);
    m_pTableWidget_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_pTableWidget_->setSelectionBehavior(QAbstractItemView::SelectRows);

    layout_ = new QVBoxLayout;
    layout_->addLayout(classId_file_layout);
    layout_->addWidget(m_pTableWidget_);
    setLayout(layout_);

    connect(classID_file_path_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTable() ));

	// ros subscription
	semantic_classid_map_sub_ = nh_.subscribe("semantic_panel/semantic_classid_map", 4, &TableSemanticsPanel::updateSemanticTable, this);
}

TableSemanticsPanel::~TableSemanticsPanel() 
{
    delete m_pTableWidget_;
}

void TableSemanticsPanel::save( rviz::Config config ) const
{
  	rviz::Panel::save( config );
  	config.mapSetValue( "classid_file_path", classID_file_path_ );
  
}


void TableSemanticsPanel::load( const rviz::Config& config )
{ 
  	rviz::Panel::load( config );

  	QString classIdFilePath;
  	if(config.mapGetString( "classid_file_path", &classIdFilePath ))
  	{
    	classID_file_path_editor_->setText( classIdFilePath );
    	updateTable();
  	}
}

void TableSemanticsPanel::updateTable()
{
  	setTable( classID_file_path_editor_->text() );
}

void TableSemanticsPanel::setTable( const QString& new_file )
{
	// Only take action if the name has changed.
  	if( new_file != classID_file_path_ )
  	{
		classID_file_path_ = new_file;

		// If the file path is the empty string, don't update the map
		if( classID_file_path_ != "" )
		{
			// load yaml file
			if(!utils::parseModelConfig(classID_file_path_.toStdString(), modelNameIdMap_, modelMaskIdColorMap_))
			{
				ROS_ERROR("model classid structure FAILED to load: %s", classID_file_path_.toStdString().c_str());
			}
			else
			{
				ROS_INFO("LOADED: %s",classID_file_path_.toStdString().c_str());
			}
		}
		
		// rviz::Panel defines the configChanged() signal.  Emitting it
		// tells RViz that something in this panel has changed that will
		// affect a saved config file.  Ultimately this signal can cause
		// QWidget::setWindowModified(true) to be called on the top-level
		// rviz::VisualizationFrame, which causes a little asterisk ("*")
		// to show in the window's title bar indicating unsaved changes.
		Q_EMIT configChanged();

	}

}

void TableSemanticsPanel::updateSemanticTable(const rtabmap_ros::SemanticClassIdMapConstPtr& msg)
{
	//iterate over all items in the vector message and add them to the map structure
	for(int i=0; i < msg->classIdElements.size(); i++)
	{
		rtabmap_ros::SemanticClassIdElement classIdElementMsg = msg->classIdElements.at(i);
		unsigned int labelid = classIdElementMsg.labelId;

		// look up if the table class id is already in the map and update info 
		// if not available in the map, add it.
		std::map<unsigned int, std::pair< int, cv::Point3f> >::iterator semanticClassIdMapIter = semanticClassIdMap_.find(labelid);
		if(semanticClassIdMapIter != semanticClassIdMap_.end())
		{
			// class id was found , update values
			if(semanticClassIdMapIter->second.first != classIdElementMsg.occupancyType)
				semanticClassIdMapIter->second.first = classIdElementMsg.occupancyType;

			if(semanticClassIdMapIter->second.second.x != classIdElementMsg.red ||
				semanticClassIdMapIter->second.second.y != classIdElementMsg.green ||
				semanticClassIdMapIter->second.second.z != classIdElementMsg.blue)
			{
				cv::Point3f maskColor;
				maskColor.x = classIdElementMsg.red;
				maskColor.y = classIdElementMsg.green;
				maskColor.z = classIdElementMsg.blue;

				semanticClassIdMapIter->second.second = maskColor;
			}
		}
		else
		{
			int occupancyType = classIdElementMsg.occupancyType;
			cv::Point3f maskColor;
			maskColor.x = classIdElementMsg.red;
			maskColor.y = classIdElementMsg.green;
			maskColor.z = classIdElementMsg.blue;

			// item was not found, it needs to be added to the map
			semanticClassIdMap_.insert({ labelid ,{ occupancyType , maskColor }});
		}
	}

	// update the widget table with map information
	updateQTableWidget();
}

void TableSemanticsPanel::updateQTableWidget()
{
	
	// iterate over all semantic ClassIdMap to update the qt table widget 
	for(const auto& semanticClassIdElement : semanticClassIdMap_)
	{
		bool isTableFull = false;
		int rowUsed = -1;
		unsigned int labelId = semanticClassIdElement.first;
		
		int labelIdField = 0;
		// loop over table to ensure the class id has been added
		for(int row = 0; row < m_pTableWidget_->rowCount(); ++row)
		{
			QTableWidgetItem* widgetItemLabelId = m_pTableWidget_->item(row, labelIdField);
			if(!widgetItemLabelId)
			{
				QTableWidgetItem* labeIdItem = new QTableWidgetItem(Qt::DisplayRole);
				labeIdItem->setData(Qt::DisplayRole, QVariant(labelId));
				m_pTableWidget_->setItem(row, labelIdField, labeIdItem);

				rowUsed = row;
				break;
			}
			else 
			{
				unsigned int itemValue = widgetItemLabelId->data(Qt::DisplayRole).value<unsigned int>();
				if (itemValue == labelId)
				{	
					rowUsed = row;
					break;
				}
				else if(row + 1 == m_pTableWidget_->rowCount())
				{
					isTableFull = true;
				}
			}
		}
		
		//  if table is full, add a new row  with the new class id
		if(isTableFull)
		{
			// adding row to the end of the table
			m_pTableWidget_->insertRow(m_pTableWidget_->rowCount());
			rowUsed = m_pTableWidget_->rowCount();
			QTableWidgetItem* labeIdItem = new QTableWidgetItem(Qt::DisplayRole);
			labeIdItem->setData(Qt::DisplayRole, QVariant(labelId));
			m_pTableWidget_->setItem(rowUsed, labelIdField, labeIdItem);
		}

		if (rowUsed >= 0) 
		{
			// update the data in the row if it has changed.
			// DATA field (3): type {string}
			int typeField = 3;
			QString occupancyTypeQStr((getOccupancyType_str(semanticClassIdElement.second.first)).c_str());
			QTableWidgetItem* widgetItemType = m_pTableWidget_->item(rowUsed, typeField);
			if(!widgetItemType)
			{
				QTableWidgetItem* typeItem = new QTableWidgetItem(Qt::DisplayRole);
				typeItem->setData(Qt::DisplayRole, QVariant(occupancyTypeQStr));
				m_pTableWidget_->setItem(rowUsed, typeField, typeItem);
			}
			else
			{
				// check if this item has changed
				QString itemValue = widgetItemType->data(Qt::DisplayRole).value<QString>();
				if(occupancyTypeQStr != itemValue)
				{
					widgetItemType->setData(Qt::DisplayRole, QVariant(occupancyTypeQStr));
				}
			}

			// update the data in the row if it has changed.
			// DATA field (2) Color: type {QRect}
			int colorField = 2;
			cv::Point3f maskColor = semanticClassIdElement.second.second;
			//ROS_INFO("      color: R:%f G:%f B:%f", maskColor.x, maskColor.y, maskColor.z);
			QColor mColor;
			mColor.setRgbF(maskColor.x/255.0, maskColor.y/255.0, maskColor.z/255.0);
			QTableWidgetItem* widgetItemColor = m_pTableWidget_->item(rowUsed, colorField);
			if(!widgetItemColor)
			{
				QTableWidgetItem* maskColorItem = new QTableWidgetItem(Qt::DisplayRole);		
				QBrush brush(mColor, Qt::SolidPattern);
				maskColorItem->setBackground(brush);
				m_pTableWidget_->setItem(rowUsed, colorField, maskColorItem);
			}
			else
			{
				// check if this item has changed
				QBrush itemColorBrush = widgetItemColor->background();
				QColor itemColor = itemColorBrush.color();
				if(mColor != itemColor)
				{
					QBrush brush(itemColor, Qt::SolidPattern);
					widgetItemColor->setBackground(brush);
				}	
			}

			// update the data in the row if it has changed.
			// DATA field (1) NAME: type {string}
			int nameField = 1;
			QString nameQString;
			std::map< std::string, std::map<unsigned int, std::string> >::iterator modelNameIdMapIter = modelNameIdMap_.find(occupancyTypeQStr.toStdString());
			if(modelNameIdMapIter != modelNameIdMap_.end())
			{
				std::map<unsigned int, std::string> nameIdMap = modelNameIdMapIter->second;
				std::map<unsigned int, std::string>::iterator nameIdMapIter = nameIdMap.find(labelId);
				if(nameIdMapIter != nameIdMap.end())
				{
					nameQString = QString(nameIdMapIter->second.c_str());
				}
				else
				{
					ROS_WARN("occupany type does not have class ID");
				}
			}

			QTableWidgetItem* widgetItemNamePtr = m_pTableWidget_->item(rowUsed, nameField);
			if(!widgetItemNamePtr)
			{
				QTableWidgetItem* nameItem = new QTableWidgetItem(Qt::DisplayRole);
				nameItem->setData(Qt::DisplayRole, QVariant(nameQString));
				m_pTableWidget_->setItem(rowUsed, nameField, nameItem);
			}
			else
			{
				// check if this item corresponds to the labelId
				QString nameValue = widgetItemNamePtr->data(Qt::DisplayRole).value<QString>();
				if(!nameQString.isEmpty() && nameQString != nameValue)
				{
					widgetItemNamePtr->setData(Qt::DisplayRole, QVariant(nameQString));
				}
			}
		}
		else 
		{
			ROS_ERROR("class ID was not in the table!! THIS SHOULD NOT BE THE CASE");
		}

	}

	// update() is a QWidget function which schedules this widget to be
	// updated the next time through the main event loop.
	m_pTableWidget_->update();
}

std::string TableSemanticsPanel::getOccupancyType_str(int type)
{
	std::string typeStr;

	if(type == rtabmap::RtabmapAPLColorOcTreeNode::OccupancyType::kTypeStatic) 
	{
		typeStr = "static";
	}
	else if(type == rtabmap::RtabmapAPLColorOcTreeNode::OccupancyType::kTypeMovable) 
	{
		typeStr = "movable";
	}
	else if(type == rtabmap::RtabmapAPLColorOcTreeNode::OccupancyType::kTypeDynamic) 
	{
		typeStr = "dynamic";
	}
	else if(type == rtabmap::RtabmapAPLColorOcTreeNode::OccupancyType::kTypeEmpty) 
	{
		typeStr = "empty";
	}
	else 
	{
		typeStr = "Unknown";	
	}

	return typeStr;

}

} // end namespace 

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rtabmap_ros::TableSemanticsPanel, rviz::Panel )