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

    const QRect canvas = rect().adjusted(10, 10, -10, -10);

    const double sx = static_cast<double>(canvas.width()) / display_image_.width();
    const double sy = static_cast<double>(canvas.height()) / display_image_.height();
    const double s = std::min(sx, sy);  // fit whole image

    const int w = static_cast<int>(display_image_.width() * s);
    const int h = static_cast<int>(display_image_.height() * s);

    // Bottom-left anchor in window coordinates
    const int x = canvas.left();
    const int y = canvas.bottom() - h + 1;

    const QRect target(x, y, w, h);
    painter.drawImage(target, display_image_);
}