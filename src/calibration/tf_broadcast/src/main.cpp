#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "../include/tf_broadcast.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    if (QT_VERSION >= QT_VERSION_CHECK(5, 6, 0))
        QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling, true);
    QApplication a(argc, argv);
    TfBroadcast w;
    w.show();
    return a.exec();
}