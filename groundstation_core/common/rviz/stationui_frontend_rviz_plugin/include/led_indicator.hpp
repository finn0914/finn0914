// Adapted from https://github.com/melanholly/qt5-led-indicator-widget

#ifndef LEDINDICATOR_H
#define LEDINDICATOR_H

#ifndef Q_MOC_RUN
#include <QWidget>
#endif

namespace stationui_frontend_rviz_plugin
{

class LedIndicator: public QWidget {
    Q_OBJECT
  public:
    LedIndicator(QWidget *parent = 0);
    void setState(bool state);
    void toggle();
    void setOnColor(QColor onColor);
    void setOffColor(QColor offColor);
    void setOnPattern(Qt::BrushStyle onPattern);
    void setOffPattern(Qt::BrushStyle offPattern);
    void setLedSize(int size);

  public slots:
    void switchLedIndicator();
  protected:
    void paintEvent(QPaintEvent *);
  private:
    bool lit;
    QColor ledOnColor;
    QColor ledOffColor;
    Qt::BrushStyle ledOnPattern;
    Qt::BrushStyle ledOffPattern;
    int ledSize;
};

}

#endif // LEDINDICATOR_H