#ifndef MAPVISUALIZER_H
#define MAPVISUALIZER_H

#include <QMainWindow>
#include <QImage>
#include "mapping.h"

class MapVisualizer : public QMainWindow {
    Q_OBJECT

public:
    explicit MapVisualizer(Mapping* mapping, QWidget *parent = nullptr);
    ~MapVisualizer();

private slots:
    void onMapUpdated();

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    Mapping* mapping_;
    QImage display_image_;
};

#endif // MAPVISUALIZER_H