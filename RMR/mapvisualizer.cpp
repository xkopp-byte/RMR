#include "mapvisualizer.h"
#include <QPainter>
#include <QPaintEvent>

MapVisualizer::MapVisualizer(Mapping* mapping, QWidget *parent)
    : QMainWindow(parent), mapping_(mapping)
{
    setWindowTitle("Map Visualization");
    setGeometry(100, 100, 800, 900);
    
    connect(mapping_, SIGNAL(mapUpdated()), this, SLOT(onMapUpdated()));
}

MapVisualizer::~MapVisualizer()
{
}

void MapVisualizer::onMapUpdated()
{
#ifndef DISABLE_MAPPING
    cv::Mat grid = mapping_->getGridImage();
    if (!grid.empty()) {
        display_image_ = QImage(grid.data, grid.cols, grid.rows, 
                                static_cast<int>(grid.step), QImage::Format_Grayscale8).copy();
    }
#endif
    update();
}

void MapVisualizer::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.fillRect(rect(), Qt::black);
    
    if (display_image_.isNull()) {
        painter.setPen(Qt::white);
        painter.drawText(rect(), Qt::AlignCenter, "Waiting for map...");
        return;
    }
    
    QRect drawRect = rect().adjusted(10, 10, -10, -10);
    painter.drawImage(drawRect, display_image_);
}