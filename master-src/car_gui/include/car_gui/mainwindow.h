/******************************************************************************
 * 
 *  file: mainwindow.h
 *  author: Ruifan Wang
 *  description: This file contains the declaration of the mainwindow class.
 * 
*******************************************************************************/
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QWidget>
#include <QDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QDebug>
#include <QPainter>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QElapsedTimer>
#include <QLCDNumber>
#include <QTimer>
#include <QTime>
#include <QCoreApplication>
#include <QEventLoop>
#include <QPushButton>
#include <QComboBox>
#include <QMatrix>


#include<cmath>

#define SMALL_CIRCLE_RADIUS 30
#define BIG_CIRCLE_RADIUS 90

/******************************************************************************
 * 
 *  class: MainWindow
 *  description: This class is the mainwindow of the GUI. It contains the
 *              declaration of the mainwindow class. Including the declaration
 *             of the functions and variables.
 * 
*******************************************************************************/
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void controlCar(int* flag, float *vx, float *vz);

private:
    /*********Widget*********/
    
    //Label
    QLabel *labelError;
    QLabel *labelVx;
    QLabel *labelVz;

    //Button
    QPushButton *btnStart;
    QPushButton *btnSave;

    //Combo Box
    QComboBox *speedBox;

    //Pixmap
    QPixmap car_Pixmap;
    QPixmap bigCircle_Pixmap;
    QPixmap smallCircle_Pixmap;

    //Point
    QPoint BigCir_xy;
    QPoint SmallCir_xy;
    QPoint MapRemov_Old;

    //LCD
    QLCDNumber *lcdDisplayVx;
    QLCDNumber *lcdDisplayVz;

    /*********Function*********/

    //Init function
    void labelInit();
    void buttonInit();
    void lcdInit();
    void rockerInit();
    void openampInit();
    void timerInit();
    void comboInit();
    void pixmapInit();

    //Delay function
    void delay_ms(int ms);

    //Paint function
    void paintEvent(QPaintEvent* event);

    //Mouse function
    void mouseMoveEvent(QMouseEvent*);
    void mousePressEvent(QMouseEvent*);
    void mouseReleaseEvent(QMouseEvent*);

    /*********Variable*********/

    //Bool flag
    bool MousePressFlag;
    bool startFlag = false, controlFlag = false;
    //Rocker position
    int x = 0, y = 0;
    //Max speed
    float maxSpeed = 0.5;
    //Car angle
    float angle = 0.0;
    
private slots:
    //Start slot
    void pushButtonStart_Clicked();
    //End slot
    void pushButtonEnd_Clicked();
    //Save slot
    void pushButtonSave_Clicked();
};
#endif // MAINWINDOW_H
